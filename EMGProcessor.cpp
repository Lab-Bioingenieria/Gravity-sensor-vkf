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

#include "EMGProcessor.h"

#include <math.h>

EMGProcessor::EMGProcessor()
    : m_state(EMG_CAL_UNCALIBRATED),
      m_sampleFreqHz(1000.0f),
      m_raw(0.0f),
      m_filtered(0.0f),
      m_envelope(0.0f),
      m_mvcEnvelope(0.0f),
      m_dcBias(0.0f),
      m_dcBiasPrimed(false),
      m_active(false),
      m_alphaEnvelope(0.0f),
      m_alphaMvcEnvelope(0.0f),
      m_alphaBaseline(0.0f),
      m_alphaMvcDecay(0.0f),
      m_alphaDcBias(0.0f),
      m_stepSamples(0),
      m_stepTarget(0),
      m_restCount(0),
      m_restMean(0.0f),
      m_restM2(0.0f),
      m_mvcPeak(0.0f),
      m_settleSamples(0) {}

// Discrete gain of a first-order lag with the given time constant, so that the
// step response reaches 63% after tauSeconds at the current sample rate.
float EMGProcessor::alphaFromTau(float tauSeconds) const {
    if (tauSeconds <= 0.0f) {
        return 1.0f;
    }
    return 1.0f - expf(-1.0f / (tauSeconds * m_sampleFreqHz));
}

void EMGProcessor::applySampleRate() {
    m_alphaEnvelope    = alphaFromTau(m_config.envelopeTauMs / 1000.0f);
    m_alphaMvcEnvelope = alphaFromTau(m_config.mvcEnvelopeTauMs / 1000.0f);
    m_alphaBaseline    = alphaFromTau(m_config.baselineTrackTauS);
    m_alphaDcBias      = alphaFromTau(m_config.dcBiasTauS);
    m_alphaMvcDecay    = (m_config.mvcDecayTauS > 0.0f)
                             ? alphaFromTau(m_config.mvcDecayTauS)
                             : 0.0f;
}

void EMGProcessor::begin(SAMPLE_FREQUENCY sampleFreq,
                         NOTCH_FREQUENCY  notchFreq) {
    m_filters.init(sampleFreq, notchFreq, true, true, true);
    m_sampleFreqHz = (float)sampleFreq;
    applySampleRate();
    resetAll();
}

void EMGProcessor::setConfig(const EMGConfig &config) {
    m_config = config;
    applySampleRate();
}

void EMGProcessor::reset() {
    m_filters.reset();
    m_filtered      = 0.0f;
    m_envelope      = 0.0f;
    m_mvcEnvelope   = 0.0f;
    m_active        = false;
    m_dcBiasPrimed  = false;
    m_settleSamples = 0;

    if (m_cal.valid) {
        m_state = EMG_CAL_SETTLING;
    } else {
        m_state = EMG_CAL_UNCALIBRATED;
    }
}

void EMGProcessor::resetAll() {
    m_cal = EMGCalibration();
    reset();
    m_state       = EMG_CAL_UNCALIBRATED;
    m_stepSamples = 0;
    m_stepTarget  = 0;
    m_restCount   = 0;
    m_restMean    = 0.0f;
    m_restM2      = 0.0f;
    m_mvcPeak     = 0.0f;
}

void EMGProcessor::startRestCalibration(uint16_t durationMs) {
    m_restCount   = 0;
    m_restMean    = 0.0f;
    m_restM2      = 0.0f;
    m_stepSamples = 0;
    m_stepTarget  = (uint32_t)((durationMs / 1000.0f) * m_sampleFreqHz);
    if (m_stepTarget == 0) {
        m_stepTarget = 1;
    }
    m_state = EMG_CAL_REST;
}

void EMGProcessor::startMvcCalibration(uint16_t durationMs) {
    // Without a noise floor there is nothing to normalise against, so refuse
    // rather than silently produce a meaningless span.
    if (m_state != EMG_CAL_REST_DONE && m_state != EMG_CAL_READY) {
        return;
    }

    m_mvcPeak     = 0.0f;
    m_stepSamples = 0;
    m_stepTarget  = (uint32_t)((durationMs / 1000.0f) * m_sampleFreqHz);
    if (m_stepTarget == 0) {
        m_stepTarget = 1;
    }
    m_state = EMG_CAL_MVC;
}

void EMGProcessor::cancelCalibration() {
    m_stepSamples = 0;
    m_stepTarget  = 0;

    if (m_cal.valid) {
        m_state = EMG_CAL_READY;
    } else if (m_state == EMG_CAL_MVC) {
        m_state = EMG_CAL_REST_DONE;
    } else {
        m_state = EMG_CAL_UNCALIBRATED;
    }
}

uint8_t EMGProcessor::progressPercent() const {
    if (m_stepTarget == 0) {
        return 100;
    }
    if (m_stepSamples >= m_stepTarget) {
        return 100;
    }
    return (uint8_t)((m_stepSamples * 100UL) / m_stepTarget);
}

float EMGProcessor::threshold() const {
    if (!m_cal.valid) {
        return 0.0f;
    }

    float offset = m_config.onsetSigma * m_cal.restStdDev;
    if (offset < m_config.minThresholdOffset) {
        offset = m_config.minThresholdOffset;
    }
    return m_cal.restLevel + offset;
}

float EMGProcessor::activation() const {
    if (!m_cal.valid) {
        return 0.0f;
    }

    float span = m_cal.mvcLevel - m_cal.restLevel;
    if (span < m_config.minSpan) {
        span = m_config.minSpan;
    }

    const float value = (m_envelope - m_cal.restLevel) / span;
    if (value <= 0.0f) {
        return 0.0f;
    }
    if (value >= 1.0f) {
        return 1.0f;
    }
    return value;
}

bool EMGProcessor::setCalibration(const EMGCalibration &cal) {
    if (!cal.valid) {
        return false;
    }
    if ((cal.mvcLevel - cal.restLevel) < m_config.minSpan) {
        return false;
    }

    m_cal   = cal;
    m_state = EMG_CAL_SETTLING;
    return true;
}

// Auto-adjust. Once calibrated, the resting level drifts as electrode gel
// settles, skin impedance falls with perspiration and the limb changes posture.
//
// The correction is an integral-only update: the error between the observed
// envelope and the stored rest level is integrated with gain m_alphaBaseline.
// A proportional term is deliberately absent - it would let a single noisy
// sample displace the reference, which is the opposite of what a baseline
// estimator needs.
//
// The update is gated on the channel being inactive. An ungated tracker chases
// the contraction it is supposed to measure and erases the very signal it is
// normalising.
void EMGProcessor::trackBaseline() {
    if (!m_active) {
        m_cal.restLevel += m_alphaBaseline * (m_envelope - m_cal.restLevel);
    }

    // The ceiling attacks immediately on a new peak so a stronger contraction
    // than the calibration one does not saturate at 1.0 forever, and decays
    // slowly so a tired wearer regains usable resolution.
    if (m_mvcEnvelope > m_cal.mvcLevel) {
        m_cal.mvcLevel = m_mvcEnvelope;
    } else if (m_alphaMvcDecay > 0.0f) {
        const float floorLevel = m_cal.restLevel + m_config.minSpan;
        m_cal.mvcLevel -= m_alphaMvcDecay * (m_cal.mvcLevel - floorLevel);
        if (m_cal.mvcLevel < floorLevel) {
            m_cal.mvcLevel = floorLevel;
        }
    }
}

float EMGProcessor::update(int rawAdc) {
    m_raw = (float)rawAdc;

    // Seeding the DC estimate from the first sample rather than from zero
    // avoids a large startup step through the highpass. The SEN0240 idles near
    // mid-supply, so a zero-seeded estimate would inject a step of several
    // hundred counts.
    if (!m_dcBiasPrimed) {
        m_dcBias       = m_raw;
        m_dcBiasPrimed = true;
    } else {
        m_dcBias += m_alphaDcBias * (m_raw - m_dcBias);
    }

    m_filtered = m_filters.updateFloat(m_raw - m_dcBias);

    const float rectified = fabsf(m_filtered);
    m_envelope += m_alphaEnvelope * (rectified - m_envelope);
    m_mvcEnvelope += m_alphaMvcEnvelope * (rectified - m_mvcEnvelope);

    // Ignore the filter's own ringing after a start or reset; feeding it into
    // the statistics would inflate the noise floor.
    if (m_settleSamples < (uint32_t)m_filters.settlingSamples()) {
        m_settleSamples++;
        if (m_state == EMG_CAL_SETTLING &&
            m_settleSamples >= (uint32_t)m_filters.settlingSamples()) {
            m_state = m_cal.valid ? EMG_CAL_READY : EMG_CAL_UNCALIBRATED;
        }
        return 0.0f;
    }

    switch (m_state) {
    case EMG_CAL_SETTLING:
        m_state = m_cal.valid ? EMG_CAL_READY : EMG_CAL_UNCALIBRATED;
        break;

    case EMG_CAL_REST: {
        // Welford's online algorithm: mean and variance in constant memory,
        // which matters when three channels share 2KB of SRAM.
        m_restCount++;
        const float delta = m_envelope - m_restMean;
        m_restMean += delta / (float)m_restCount;
        m_restM2 += delta * (m_envelope - m_restMean);

        m_stepSamples++;
        if (m_stepSamples >= m_stepTarget) {
            m_cal.restLevel = m_restMean;
            m_cal.restStdDev =
                (m_restCount > 1) ? sqrtf(m_restM2 / (float)(m_restCount - 1))
                                  : 0.0f;
            m_stepTarget = 0;
            m_state      = EMG_CAL_REST_DONE;
        }
        break;
    }

    case EMG_CAL_MVC:
        if (m_mvcEnvelope > m_mvcPeak) {
            m_mvcPeak = m_mvcEnvelope;
        }

        m_stepSamples++;
        if (m_stepSamples >= m_stepTarget) {
            m_cal.mvcLevel = m_mvcPeak;
            m_stepTarget   = 0;

            // A span this small means the electrodes are off, the wearer did
            // not contract, or the channel is disconnected. Better to report
            // uncalibrated than to hand back normalised noise.
            if ((m_cal.mvcLevel - m_cal.restLevel) >= m_config.minSpan) {
                m_cal.valid = true;
                m_state     = EMG_CAL_READY;
            } else {
                m_cal.valid = false;
                m_state     = EMG_CAL_REST_DONE;
            }
        }
        break;

    case EMG_CAL_READY:
        m_active = m_envelope > threshold();
        trackBaseline();
        break;

    case EMG_CAL_UNCALIBRATED:
    case EMG_CAL_REST_DONE:
    default:
        break;
    }

    return activation();
}
