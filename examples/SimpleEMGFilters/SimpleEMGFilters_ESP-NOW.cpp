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
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "EMGFilters.h"

// -----------------------------------------------------------------------------
// Librerías necesarias para ESP-NOW
// -----------------------------------------------------------------------------
#include <WiFi.h>
#include <esp_now.h>

#define _DEBUG      0

#define SensorInputPin 1 // GPIO1, pin ADC seguro en la ESP32-C3 Super Mini

// -----------------------------------------------------------------------------
// MAC del ESP32-S3 receptor
// Cambia estos valores por la MAC que muestra tu ESP32-S3.
// Ejemplo: 34:85:18:AB:CD:EF
// -----------------------------------------------------------------------------
uint8_t MAC_S3[] = {
    0xEC,
    0xE3,
    0x34,
    0x1A,
    0xD2,
    0xBC
};

// Define the `CALIBRATE` macro as 1 to calibrate the baseline value
// of input sEMG signals.
//
// After wiring the sEMG sensors to the Arduino board, wear the
// sEMG sensors. Relax your muscles for a few seconds, you
// will be able to see a series of squared sEMG signals values get printed on
// your serial terminal. Choose the maximal one as the baseline by setting the
// `baseline` variable.
//
// After calibriting, change the `CALIBRATE` macro to 0, and rebuild this
// project. The `envelope`, which is the squared sEMG signal data, will be
// printed to the serial line. The developer can plot it using the Arduino
// SerialPlotter.
//
// Note:
//      After calibration, Any squared value of sEMG sigal below the
//      baseline will be treated as zero.
//
//      It is recommended that you do calibration every time you wear
//      the sEMG sensor.
//
//      El ADC de la ESP32-C3 es de 12 bits (0 a 4095), no de 10 bits como el
//      Arduino Uno (0 a 1023), asi que este baseline hay que recalibrarlo en
//      esta placa siguiendo el mismo procedimiento.
//
// This manual, read-it-off-the-serial-monitor procedure does not scale: it has
// to be redone by hand for every wearer and every electrode placement, and it
// gives you a raw threshold rather than a comparable measure of effort. See
// the MultiChannelCalibrated example for the automatic equivalent, which also
// normalises the output so one number means the same thing for everyone.
#define CALIBRATE 0

long baseline = 1400;

EMGFilters myFilter;

// Set the input frequency.
//
// The filters work only with fixed sample frequency of
// `SAMPLE_FREQ_500HZ` or `SAMPLE_FREQ_1000HZ`.
// Inputs at other sample rates will bypass
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;

// Time interval between samples, in microseconds.
//
// Keep this an `unsigned long`, matching micros(). Widening the timestamps to
// 64 bits does not avoid the micros() rollover, it breaks the wraparound
// handling: at rollover a 64-bit subtraction yields a huge positive number
// instead of the correct small delta that 32-bit modular arithmetic gives.
const unsigned long interval = 1000000UL / (unsigned long)sampleRate;

unsigned long nextSampleAt = 0;

// Set the frequency of power line hum to filter out.
//
// For countries with 60Hz power line, change to "NOTCH_FREQ_60HZ"
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_60HZ;


void setup() {
    /* add setup code here */
    myFilter.init(sampleRate, humFreq, true, true, true);

    // open serial
    Serial.begin(115200);

    // -------------------------------------------------------------------------
    // Inicializar WiFi en modo estación para utilizar ESP-NOW
    // -------------------------------------------------------------------------
    WiFi.mode(WIFI_STA);

    // -------------------------------------------------------------------------
    // Inicializar ESP-NOW
    // -------------------------------------------------------------------------
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error inicializando ESP-NOW");
        return;
    }

    // -------------------------------------------------------------------------
    // Configurar el ESP32-S3 como dispositivo receptor
    // -------------------------------------------------------------------------
    esp_now_peer_info_t peerInfo = {};

    memcpy(peerInfo.peer_addr, MAC_S3, 6);

    // 0 = utilizar el canal WiFi actual
    peerInfo.channel = 0;

    // No utilizamos cifrado
    peerInfo.encrypt = false;

    // Agregar el S3 como peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Error agregando ESP32-S3");
        return;
    }

    Serial.println("ESP-NOW iniciado");

    nextSampleAt = micros();
}

void loop() {
    // Sample on an absolute schedule rather than by sleeping for whatever is
    // left after the work. The signed comparison keeps this correct across
    // the micros() rollover that happens every ~70 minutes.
    if ((long)(micros() - nextSampleAt) < 0) {
        return;
    }
    nextSampleAt += interval;

#if _DEBUG
    unsigned long timeStamp = micros();
#endif

    int data = analogRead(SensorInputPin);

    // filter processing
    int dataAfterFilter = myFilter.update(data);

    // Get envelope by squaring the input.
    //
    // `int` is 16 bits on AVR, so sq() overflows for anything past +/-181
    // counts and the envelope wraps to nonsense. A filtered sEMG burst passes
    // that easily, so accumulate in 32 bits.
    long envelope = (long)dataAfterFilter * (long)dataAfterFilter;

    if (CALIBRATE) {
        //Serial.print("Squared Data: ");
        Serial.println(envelope);
    }
    else {
        // Any value below the `baseline` value will be treated as zero
        if (envelope < baseline) {
            dataAfterFilter = 0;
            envelope = 0;
        }

        // ---------------------------------------------------------------------
        // Se mantiene la impresión original por Serial
        // ---------------------------------------------------------------------
        Serial.println(envelope);

        // ---------------------------------------------------------------------
        // Enviar el mismo valor `envelope` por ESP-NOW
        // ---------------------------------------------------------------------
        esp_now_send(
            MAC_S3,
            (uint8_t *)&envelope,
            sizeof(envelope)
        );
    }

#if _DEBUG
    Serial.print("Filters cost time: ");
    Serial.println(micros() - timeStamp);
#endif
}
