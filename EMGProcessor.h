/*
 * EMGProcessor - per-channel sEMG conditioning, calibration and normalisation.
 *
 * Copyright 2017, OYMotion Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */

#ifndef _EMGPROCESSOR_H
#define _EMGPROCESSOR_H

#include <stdint.h>

#include "EMGFilters.h"

// Where a channel is in its calibration sequence.
//
// A channel must pass through REST and MVC once per wearer before activation()
// means anything. Muscle bulk, subcutaneous fat and electrode placement change
// the amplitude of the same gesture by an order of magnitude between people,
// so a hard-coded threshold cannot work across wearers.
enum EMGCalibrationState {
    EMG_CAL_UNCALIBRATED = 0, // no reference levels yet
    EMG_CAL_SETTLING,         // filters still ringing after start/reset
    EMG_CAL_REST,             // sampling the resting noise floor
    EMG_CAL_REST_DONE,        // rest captured, waiting for the MVC step
    EMG_CAL_MVC,              // sampling the maximum voluntary contraction
    EMG_CAL_READY             // normalised output is valid
};

// Reference levels for one channel and one wearer, in ADC counts.
//
// Persist this (EEPROM, flash, host) to skip recalibration on the next boot
// for the same person and the same electrode placement.
struct EMGCalibration {
    float restLevel;  // mean envelope while relaxed
    float restStdDev; // envelope standard deviation while relaxed
    float mvcLevel;   // envelope at maximum voluntary contraction
    bool  valid;

    EMGCalibration()
        : restLevel(0.0f), restStdDev(0.0f), mvcLevel(0.0f), valid(false) {}
};

// Tuning knobs. The defaults suit a DFRobot SEN0240 (OYMotion Gravity analog
// sEMG) on a 10-bit ADC and rarely need changing.
struct EMGConfig {
    // Envelope smoothing. 100ms is the usual compromise between responsiveness
    // and ripple for gesture detection.
    float envelopeTauMs;

    // Slower envelope used only to score the MVC peak. Averaging over ~250ms
    // stops a single motion artefact from setting an unreachable ceiling.
    float mvcEnvelopeTauMs;

    // Activation threshold expressed in standard deviations above the resting
    // mean. 3 sigma gives roughly one false onset per thousand resting samples.
    float onsetSigma;

    // Absolute floor for the threshold offset, in ADC counts. Protects against
    // a degenerate rest recording where the standard deviation is ~0.
    float minThresholdOffset;

    // Time constant of the resting-baseline tracker. This is the integral gain
    // of the auto-adjust: larger tau tracks drift more slowly but is less
    // likely to be pulled by residual muscle tone.
    float baselineTrackTauS;

    // Time constant for the MVC ceiling to decay back down when the wearer
    // stops reaching previous peaks. Set to 0 to freeze the ceiling.
    float mvcDecayTauS;

    // Smallest usable rest-to-MVC span, in ADC counts. Guards the normalisation
    // divide when a channel is disconnected or the MVC step was skipped.
    float minSpan;

    // Time constant of the DC-bias tracker. The SEN0240 idles at 1.5V rather
    // than at ground, and this removes that offset before filtering so the
    // highpass does not have to swing through it at startup.
    float dcBiasTauS;

    EMGConfig()
        : envelopeTauMs(100.0f),
          mvcEnvelopeTauMs(250.0f),
          onsetSigma(3.0f),
          minThresholdOffset(1.0f),
          baselineTrackTauS(5.0f),
          mvcDecayTauS(60.0f),
          minSpan(4.0f),
          dcBiasTauS(1.0f) {}
};

// One sEMG channel: filtering, rectification, envelope detection, calibration
// and normalisation.
//
// Hold one instance per sensor. Instances share no state, so three of them
// process three electrodes independently.
class EMGProcessor {
  public:
    EMGProcessor();

    // \brief Configures the channel. Call once from setup().
    void begin(SAMPLE_FREQUENCY sampleFreq, NOTCH_FREQUENCY notchFreq);

    // \brief Overrides the tuning defaults. Call after begin().
    void setConfig(const EMGConfig &config);

    // \brief Feeds one ADC reading. Must be called at exactly sampleFreq;
    //        jitter shifts the notch and the cutoffs off target.
    // \return activation(), for convenient chaining.
    float update(int rawAdc);

    // \brief Begins the resting-noise measurement. The wearer must keep the
    //        muscle relaxed for the whole window.
    void startRestCalibration(uint16_t durationMs = 3000);

    // \brief Begins the maximum-contraction measurement. The wearer contracts
    //        as hard as they can for the whole window.
    // \remark Rest calibration must have completed first.
    void startMvcCalibration(uint16_t durationMs = 5000);

    // \brief Aborts an in-progress step and returns to the previous state.
    void cancelCalibration();

    EMGCalibrationState state() const { return m_state; }

    // \brief Progress through the current timed step, 0..100. Returns 100 when
    //        no timed step is running.
    uint8_t progressPercent() const;

    bool isCalibrated() const { return m_cal.valid; }

    // Raw ADC reading from the last update().
    float raw() const { return m_raw; }

    // Bandpass-filtered, DC-free signal in ADC counts. Swings both ways.
    float filtered() const { return m_filtered; }

    // Smoothed rectified envelope in ADC counts. Always >= 0.
    float envelope() const { return m_envelope; }

    // Muscle effort as a fraction of this wearer's own maximum, 0.0 to 1.0.
    // This is the output to compare across channels and across people.
    float activation() const;

    // True while the envelope exceeds the adaptive onset threshold.
    bool isActive() const { return m_active; }

    // Current onset threshold in ADC counts.
    float threshold() const;

    // Estimated DC bias of the ADC input in counts.
    float dcBias() const { return m_dcBias; }

    EMGCalibration calibration() const { return m_cal; }

    // \brief Restores previously stored reference levels and jumps to
    //        EMG_CAL_READY. Rejects a calibration whose span is unusable.
    bool setCalibration(const EMGCalibration &cal);

    // \brief Clears filter memory and envelopes but keeps the calibration.
    void reset();

    // \brief Clears everything, including the calibration.
    void resetAll();

  private:
    void  applySampleRate();
    float alphaFromTau(float tauSeconds) const;
    void  trackBaseline();

    EMGFilters m_filters;
    EMGConfig  m_config;

    EMGCalibrationState m_state;
    EMGCalibration      m_cal;

    float m_sampleFreqHz;
    float m_raw;
    float m_filtered;
    float m_envelope;
    float m_mvcEnvelope;
    float m_dcBias;
    bool  m_dcBiasPrimed;
    bool  m_active;

    // Pre-computed smoothing gains.
    float m_alphaEnvelope;
    float m_alphaMvcEnvelope;
    float m_alphaBaseline;
    float m_alphaMvcDecay;
    float m_alphaDcBias;

    // Timed-step bookkeeping, counted in samples so the class stays free of
    // any Arduino timing dependency.
    uint32_t m_stepSamples;
    uint32_t m_stepTarget;

    // Streaming mean and variance (Welford) for the rest window.
    uint32_t m_restCount;
    float    m_restMean;
    float    m_restM2;

    // Peak tracking for the MVC window.
    float m_mvcPeak;

    uint32_t m_settleSamples;
};

#endif
