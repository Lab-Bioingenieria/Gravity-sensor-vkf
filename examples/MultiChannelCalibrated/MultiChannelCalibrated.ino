/*
 * Three DFRobot SEN0240 (Gravity analog sEMG) channels with guided
 * per-wearer calibration and normalised output.
 *
 * Why calibration is not optional: muscle bulk, subcutaneous fat and exactly
 * where the electrode lands relative to the motor point change the amplitude
 * of the same gesture by roughly an order of magnitude between people. A
 * threshold hard-coded for one wearer is meaningless for the next. This sketch
 * measures each channel's own resting floor and own maximum, then reports
 * effort as a fraction of that wearer's maximum, which is comparable across
 * channels and across people.
 *
 * Wiring
 *   SEN0240 #1 signal -> A0
 *   SEN0240 #2 signal -> A1
 *   SEN0240 #3 signal -> A2
 *   All sensors share VCC and GND with the board.
 *
 * IMPORTANT: Please unplug any mains power adapter when using sEMG sensors.
 * Only battery powered systems may be connected directly or indirectly to a
 * person wearing sEMG electrodes.
 *
 * Serial commands (115200 baud)
 *   c  restart calibration, e.g. for a new wearer
 *   d  toggle between normalised output and raw diagnostics
 */

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "EMGFilters.h"
#include "EMGProcessor.h"

// ---------------------------------------------------------------- configuration

const uint8_t SENSOR_PINS[] = {A0, A1, A2};
#define CHANNEL_COUNT (sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0]))

// Sample rate.
//
// The SEN0240 delivers 20-500Hz, so 1000Hz is the rate that captures it
// without aliasing and is what DFRobot specifies. Sampling at 500Hz folds
// everything above 250Hz back into the band; it is still usable because most
// sEMG power sits below 150Hz, but it is a compromise, not a free choice.
//
// Three channels of float filtering do not fit in 1ms on an ATmega328 (Uno,
// Nano). On those boards drop to SAMPLE_FREQ_500HZ, or use one channel. A
// 32-bit board (ESP32, Teensy, SAMD) runs three channels at 1000Hz with room
// to spare. Watch the overrun counter below - it tells you the truth.
#if defined(__AVR__)
const SAMPLE_FREQUENCY SAMPLE_RATE = SAMPLE_FREQ_500HZ;
#else
const SAMPLE_FREQUENCY SAMPLE_RATE = SAMPLE_FREQ_1000HZ;
#endif

// Mains frequency. Use NOTCH_FREQ_60HZ in the Americas and most of Japan.
const NOTCH_FREQUENCY MAINS_FREQ = NOTCH_FREQ_50HZ;

const unsigned long SAMPLE_INTERVAL_US = 1000000UL / (unsigned long)SAMPLE_RATE;

const uint16_t REST_WINDOW_MS = 3000;
const uint16_t MVC_WINDOW_MS  = 4000;

// Report to the serial line 20 times a second. Printing is far slower than
// sampling, so it must never sit inside the sampling path.
const unsigned long REPORT_INTERVAL_US = 50000UL;

// ---------------------------------------------------------------- state

EMGProcessor channels[CHANNEL_COUNT];

enum SetupPhase {
    PHASE_WARMUP,
    PHASE_PROMPT_REST,
    PHASE_REST,
    PHASE_PROMPT_MVC,
    PHASE_MVC,
    PHASE_RUN
};

SetupPhase phase       = PHASE_WARMUP;
uint8_t    mvcChannel  = 0;
bool       diagnostics = false;

unsigned long nextSampleAt  = 0;
unsigned long nextReportAt  = 0;
unsigned long warmupSamples = 0;
unsigned long overruns      = 0;

// ---------------------------------------------------------------- helpers

static void beginCalibration() {
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
        channels[i].resetAll();
    }
    mvcChannel    = 0;
    warmupSamples = 0;
    phase         = PHASE_WARMUP;
}

static void sampleAllChannels() {
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
        channels[i].update(analogRead(SENSOR_PINS[i]));
    }
}

static void printBar(float value) {
    const uint8_t filled = (uint8_t)(value * 20.0f + 0.5f);
    Serial.print('[');
    for (uint8_t i = 0; i < 20; i++) {
        Serial.print(i < filled ? '#' : '.');
    }
    Serial.print(']');
}

// ---------------------------------------------------------------- setup

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait briefly for native-USB boards; never block a UART board.
    }

    for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
        channels[i].begin(SAMPLE_RATE, MAINS_FREQ);
    }

    Serial.println();
    Serial.println(F("sEMG multi-channel, normalised output"));
    Serial.print(F("Channels: "));
    Serial.print((int)CHANNEL_COUNT);
    Serial.print(F("   Sample rate: "));
    Serial.print((int)SAMPLE_RATE);
    Serial.print(F("Hz   Notch: "));
    Serial.print((int)MAINS_FREQ);
    Serial.println(F("Hz"));

    nextSampleAt = micros();
    nextReportAt = micros();
    beginCalibration();
}

// ---------------------------------------------------------------- loop

void loop() {
    // Sampling first, and on a strict schedule. Every filter coefficient in
    // this library is derived for one exact sample rate; sampling late detunes
    // the notch and both cutoffs, and no amount of downstream smoothing
    // recovers from that.
    const unsigned long now = micros();

    // Signed comparison so the 70-minute micros() rollover is handled by
    // ordinary modular arithmetic instead of producing a huge positive delta.
    if ((long)(now - nextSampleAt) >= 0) {
        nextSampleAt += SAMPLE_INTERVAL_US;

        sampleAllChannels();

        // If the deadline for the next sample has already passed, this board
        // cannot keep up at this rate and the filter output is not trustworthy.
        if ((long)(micros() - nextSampleAt) >= 0) {
            overruns++;
            nextSampleAt = micros() + SAMPLE_INTERVAL_US;
        }

        if (phase == PHASE_WARMUP) {
            warmupSamples++;
        }
    }

    // Everything below runs between samples and must stay off the hot path.

    if (Serial.available()) {
        const int command = Serial.read();
        if (command == 'c' || command == 'C') {
            Serial.println();
            Serial.println(F("Restarting calibration."));
            beginCalibration();
        } else if (command == 'd' || command == 'D') {
            diagnostics = !diagnostics;
        }
    }

    switch (phase) {
    case PHASE_WARMUP:
        // Let the highpass ring down and the DC estimate settle before
        // measuring anything.
        if (warmupSamples > (unsigned long)SAMPLE_RATE / 2) {
            phase = PHASE_PROMPT_REST;
        }
        break;

    case PHASE_PROMPT_REST:
        Serial.println();
        Serial.println(F("STEP 1/2  Relax every muscle and hold still."));
        Serial.println(F("          Measuring the resting noise floor..."));
        for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
            channels[i].startRestCalibration(REST_WINDOW_MS);
        }
        phase = PHASE_REST;
        break;

    case PHASE_REST:
        if (channels[0].state() == EMG_CAL_REST_DONE) {
            Serial.println(F("          Rest captured."));
            phase = PHASE_PROMPT_MVC;
        }
        break;

    case PHASE_PROMPT_MVC:
        Serial.println();
        Serial.print(F("STEP 2/2  Contract the muscle on channel "));
        Serial.print((int)mvcChannel);
        Serial.println(F(" as hard as you can."));
        Serial.println(F("          Hold until this step reports done."));
        channels[mvcChannel].startMvcCalibration(MVC_WINDOW_MS);
        phase = PHASE_MVC;
        break;

    case PHASE_MVC:
        if (channels[mvcChannel].state() != EMG_CAL_MVC) {
            if (channels[mvcChannel].isCalibrated()) {
                const EMGCalibration cal = channels[mvcChannel].calibration();
                Serial.print(F("          Channel "));
                Serial.print((int)mvcChannel);
                Serial.print(F(" done. rest="));
                Serial.print(cal.restLevel, 2);
                Serial.print(F("  mvc="));
                Serial.print(cal.mvcLevel, 2);
                Serial.print(F("  onset="));
                Serial.println(channels[mvcChannel].threshold(), 2);

                mvcChannel++;
                phase = (mvcChannel < CHANNEL_COUNT) ? PHASE_PROMPT_MVC
                                                     : PHASE_RUN;
                if (phase == PHASE_RUN) {
                    Serial.println();
                    Serial.println(F("Calibrated. 'c' recalibrates for a new "
                                     "wearer, 'd' toggles diagnostics."));
                    Serial.println();
                }
            } else {
                // Rest and peak were indistinguishable: electrodes are off,
                // the channel is unplugged, or the wearer did not contract.
                Serial.print(F("          Channel "));
                Serial.print((int)mvcChannel);
                Serial.println(F(" FAILED: no contraction detected. Check the "
                                 "electrodes, then retrying."));
                phase = PHASE_PROMPT_MVC;
            }
        }
        break;

    case PHASE_RUN:
        if ((long)(micros() - nextReportAt) >= 0) {
            nextReportAt += REPORT_INTERVAL_US;

            if (diagnostics) {
                for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
                    Serial.print(channels[i].filtered(), 1);
                    Serial.print(' ');
                    Serial.print(channels[i].envelope(), 1);
                    Serial.print(' ');
                }
                Serial.print(F(" overruns="));
                Serial.println(overruns);
            } else {
                for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
                    Serial.print(F("ch"));
                    Serial.print((int)i);
                    Serial.print(' ');
                    printBar(channels[i].activation());
                    Serial.print(channels[i].isActive() ? F(" ON  ")
                                                        : F(" off "));
                }
                Serial.println();
            }
        }
        break;
    }
}
