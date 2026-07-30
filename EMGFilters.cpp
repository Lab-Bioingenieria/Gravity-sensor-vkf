/*
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

#include "EMGFilters.h"

#include <math.h>

// coefficients of transfer function of LPF
// coef[sampleFreqInd][order]
static const float lpf_numerator_coef[2][3] = {{0.3913f, 0.7827f, 0.3913f},
                                               {0.1311f, 0.2622f, 0.1311f}};
static const float lpf_denominator_coef[2][3] = {{1.0000f, 0.3695f, 0.1958f},
                                                 {1.0000f, -0.7478f, 0.2722f}};
// coefficients of transfer function of HPF
static const float hpf_numerator_coef[2][3] = {{0.8371f, -1.6742f, 0.8371f},
                                               {0.9150f, -1.8299f, 0.9150f}};
static const float hpf_denominator_coef[2][3] = {{1.0000f, -1.6475f, 0.7009f},
                                                 {1.0000f, -1.8227f, 0.8372f}};
// coefficients of transfer function of anti-hum filter
// Laid out as two cascaded biquads: [b0 b1 b2 | b0' b1' b2'] and
// [1 a1 a2 | 1 a1' a2'].
// coef[sampleFreqInd][order] for 50Hz
static const float ahf_numerator_coef_50Hz[2][6] = {
    {0.9522f, -1.5407f, 0.9522f, 0.8158f, -0.8045f, 0.0855f},
    {0.5869f, -1.1146f, 0.5869f, 1.0499f, -2.0000f, 1.0499f}};
static const float ahf_denominator_coef_50Hz[2][6] = {
    {1.0000f, -1.5395f, 0.9056f, 1.0000f, -1.1187f, 0.3129f},
    {1.0000f, -1.8844f, 0.9893f, 1.0000f, -1.8991f, 0.9892f}};
static const float ahf_output_gain_coef_50Hz[2] = {1.3422f, 1.4399f};
// coef[sampleFreqInd][order] for 60Hz
static const float ahf_numerator_coef_60Hz[2][6] = {
    {0.9528f, -1.3891f, 0.9528f, 0.8272f, -0.7225f, 0.0264f},
    {0.5824f, -1.0810f, 0.5824f, 1.0736f, -2.0000f, 1.0736f}};
static const float ahf_denominator_coef_60Hz[2][6] = {
    {1.0000f, -1.3880f, 0.9066f, 1.0000f, -0.9739f, 0.2371f},
    {1.0000f, -1.8407f, 0.9894f, 1.0000f, -1.8584f, 0.9891f}};
static const float ahf_output_gain_coef_60Hz[2] = {1.3430f, 1.4206f};

static int sampleFreqIndex(int sampleFreq) {
    if (sampleFreq == SAMPLE_FREQ_500HZ) {
        return 0;
    }
    if (sampleFreq == SAMPLE_FREQ_1000HZ) {
        return 1;
    }
    return -1;
}

void FILTER_2nd::reset() {
    m_states[0] = 0.0f;
    m_states[1] = 0.0f;
}

void FILTER_2nd::setCoefficients(const float *numerator,
                                 const float *denominator) {
    // Normalising by den[0] lets update() skip the division and guarantees the
    // recursion is well defined even if a coefficient table is edited later.
    const float a0 = denominator[0];
    if (a0 == 0.0f) {
        m_configured = false;
        return;
    }

    for (int i = 0; i < 3; i++) {
        m_num[i] = numerator[i] / a0;
        m_den[i] = denominator[i] / a0;
    }
    m_den[0]     = 1.0f;
    m_configured = true;
}

void FILTER_2nd::init(FILTER_TYPE ftype, int sampleFreq) {
    reset();

    // An unsupported rate must leave the section in a defined pass-through
    // state; the previous version left num/den uninitialised.
    m_configured = false;
    m_num[0]     = 1.0f;
    m_num[1]     = 0.0f;
    m_num[2]     = 0.0f;
    m_den[0]     = 1.0f;
    m_den[1]     = 0.0f;
    m_den[2]     = 0.0f;

    const int idx = sampleFreqIndex(sampleFreq);
    if (idx < 0) {
        return;
    }

    if (ftype == FILTER_TYPE_LOWPASS) {
        // 2nd order butterworth lowpass, cutoff frequency 150Hz
        setCoefficients(lpf_numerator_coef[idx], lpf_denominator_coef[idx]);
    } else if (ftype == FILTER_TYPE_HIGHPASS) {
        // 2nd order butterworth highpass, cutoff frequency 20Hz
        setCoefficients(hpf_numerator_coef[idx], hpf_denominator_coef[idx]);
    }
}

float FILTER_2nd::update(float input) {
    const float tmp = input - m_den[1] * m_states[0] - m_den[2] * m_states[1];
    const float output =
        m_num[0] * tmp + m_num[1] * m_states[0] + m_num[2] * m_states[1];

    m_states[1] = m_states[0];
    m_states[0] = tmp;

    return output;
}

void FILTER_4th::reset() {
    m_stage1.reset();
    m_stage2.reset();
}

void FILTER_4th::init(int sampleFreq, int humFreq) {
    m_gain       = 1.0f;
    m_configured = false;

    static const float passthroughNum[3] = {1.0f, 0.0f, 0.0f};
    static const float passthroughDen[3] = {1.0f, 0.0f, 0.0f};
    m_stage1.setCoefficients(passthroughNum, passthroughDen);
    m_stage2.setCoefficients(passthroughNum, passthroughDen);
    reset();

    const int idx = sampleFreqIndex(sampleFreq);
    if (idx < 0) {
        return;
    }

    const float *num;
    const float *den;
    if (humFreq == NOTCH_FREQ_50HZ) {
        num    = ahf_numerator_coef_50Hz[idx];
        den    = ahf_denominator_coef_50Hz[idx];
        m_gain = ahf_output_gain_coef_50Hz[idx];
    } else if (humFreq == NOTCH_FREQ_60HZ) {
        num    = ahf_numerator_coef_60Hz[idx];
        den    = ahf_denominator_coef_60Hz[idx];
        m_gain = ahf_output_gain_coef_60Hz[idx];
    } else {
        return;
    }

    m_stage1.setCoefficients(&num[0], &den[0]);
    m_stage2.setCoefficients(&num[3], &den[3]);
    m_configured = m_stage1.isConfigured() && m_stage2.isConfigured();
}

float FILTER_4th::update(float input) {
    return m_gain * m_stage2.update(m_stage1.update(input));
}

void EMGFilters::init(SAMPLE_FREQUENCY sampleFreq,
                      NOTCH_FREQUENCY  notchFreq,
                      bool             enableNotchFilter,
                      bool             enableLowpassFilter,
                      bool             enableHighpassFilter) {
    m_sampleFreq    = sampleFreq;
    m_notchFreq     = notchFreq;
    m_bypassEnabled = true;
    if (((sampleFreq == SAMPLE_FREQ_500HZ) ||
         (sampleFreq == SAMPLE_FREQ_1000HZ)) &&
        ((notchFreq == NOTCH_FREQ_50HZ) || (notchFreq == NOTCH_FREQ_60HZ))) {
        m_bypassEnabled = false;
    }

    m_lpf.init(FILTER_TYPE_LOWPASS, m_sampleFreq);
    m_hpf.init(FILTER_TYPE_HIGHPASS, m_sampleFreq);
    m_ahf.init(m_sampleFreq, m_notchFreq);

    m_notchFilterEnabled    = enableNotchFilter;
    m_lowpassFilterEnabled  = enableLowpassFilter;
    m_highpassFilterEnabled = enableHighpassFilter;
}

void EMGFilters::reset() {
    m_lpf.reset();
    m_hpf.reset();
    m_ahf.reset();
}

int EMGFilters::settlingSamples() const {
    // The 20Hz highpass dominates settling; ~5 time constants of a 20Hz
    // corner is about 40ms.
    return (m_sampleFreq == SAMPLE_FREQ_1000HZ) ? 40 : 20;
}

float EMGFilters::updateFloat(float inputValue) {
    if (m_bypassEnabled) {
        return inputValue;
    }

    // The cascade stays in float end to end. Rounding between sections, as the
    // previous int-typed pipeline did, injected quantisation noise three times
    // per sample and biased every stage towards zero.
    float output = inputValue;

    if (m_notchFilterEnabled) {
        output = m_ahf.update(output);
    }

    if (m_lowpassFilterEnabled) {
        output = m_lpf.update(output);
    }

    if (m_highpassFilterEnabled) {
        output = m_hpf.update(output);
    }

    return output;
}

int EMGFilters::update(int inputValue) {
    const float output = updateFloat((float)inputValue);
    return (int)((output >= 0.0f) ? (output + 0.5f) : (output - 0.5f));
}
