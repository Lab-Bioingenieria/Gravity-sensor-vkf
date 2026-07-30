# EMG Filters

Signal conditioning, envelope detection and per-wearer calibration for analog
sEMG sensors such as the [DFRobot SEN0240][sen0240] (Gravity analog sEMG by
OYMotion).

Two layers, use whichever fits:

- **`EMGFilters`** — the raw filter chain. Anti-hum notch at 50 or 60 Hz, a
  150 Hz lowpass and a 20 Hz highpass, each independently switchable.
- **`EMGProcessor`** — one complete channel: filtering, rectification, envelope
  detection, automatic per-wearer calibration and normalised output. This is
  what you want for gesture detection or for more than one sensor.

Only sample rates of 500 Hz and 1000 Hz are supported. The coefficients are
precomputed per rate; there is no runtime design step.

## Why calibration is not optional

Muscle bulk, subcutaneous fat and exactly where the electrode lands relative to
the motor point change the amplitude of the same gesture by roughly an order of
magnitude between people. Electrode impedance also drifts within a single
session as the skin perspires and the gel settles.

A threshold hard-coded for one wearer is meaningless for the next. `EMGProcessor`
measures each channel's own resting floor and own maximum, then reports effort
as a fraction of that wearer's maximum. That number — `activation()`, in the
range 0.0 to 1.0 — is comparable across channels and across people.

The calibration is a two-step sequence, run once per wearer:

1. **Rest** — the wearer relaxes. The processor measures the mean and standard
   deviation of the resting envelope. The onset threshold is placed a
   configurable number of standard deviations above that mean.
2. **MVC** (maximum voluntary contraction) — the wearer contracts as hard as
   they can. The processor records the peak of a 250 ms-smoothed envelope,
   which is the standard definition and is robust to a single motion artefact.

Save the resulting `EMGCalibration` and restore it with `setCalibration()` to
skip the sequence on the next boot for the same person and placement.

## Multiple sensors

Instantiate one `EMGProcessor` per sensor:

```cpp
EMGProcessor channels[3];

void setup() {
    for (uint8_t i = 0; i < 3; i++) {
        channels[i].begin(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);
    }
}
```

Each instance owns its filter state. See the `MultiChannelCalibrated` example
for a complete three-sensor sketch with a guided calibration sequence over the
serial line and a `c` command to recalibrate for a new wearer.

## The auto-adjust, and why it is not a PI controller

Once calibrated, the resting level still drifts. `EMGProcessor` corrects for
this continuously, but the correction is deliberately **integral-only**:

```
restLevel += Ki * (envelope - restLevel)    // only while the channel is idle
```

A PI or RST regulator is the wrong shape for this problem. Those are tools for
driving a *plant* to a setpoint through an *actuator*, and there is no actuator
here — nothing is being commanded, only estimated. What the problem actually
calls for is adaptive estimation of a slowly-varying statistic, and that is a
leaky integrator. A proportional term would let a single noisy sample displace
the reference, which is precisely what a baseline estimator must not do.

The two details that matter more than the controller topology:

- **The update is gated on the channel being idle.** An ungated tracker chases
  the contraction it is supposed to be measuring and erases its own signal.
- **The time constant is seconds, not milliseconds** (`baselineTrackTauS`,
  default 5 s). Drift is slow; the tracker must be slower than the signal.

The MVC ceiling adapts too: it attacks immediately on a new peak, so a
contraction stronger than the calibration one does not clip at 1.0 forever, and
decays slowly (`mvcDecayTauS`, default 60 s) so a fatiguing wearer keeps usable
resolution. Set `mvcDecayTauS` to 0 to freeze the ceiling.

## Choosing a sample rate

The SEN0240 outputs 20–500 Hz, and DFRobot specifies an ADC of at least 1 kHz.

| | 500 Hz | 1000 Hz |
|---|---|---|
| Hum rejection | ~74 dB, one exact null | ~19 dB, flat across 49–51 Hz |
| Aliasing | content above 250 Hz folds back | none within the sensor band |
| Cost | half the CPU | full |

The two rates ship genuinely different notch designs. 500 Hz places a single
deep null exactly on the mains frequency; 1000 Hz places two nulls straddling it,
trading depth for a stopband that keeps working when the mains frequency wanders.
Neither is strictly better — pick 1000 Hz unless you are CPU-bound.

**Sampling must be on time.** Every coefficient is derived for one exact rate,
so a late sample detunes the notch and both cutoffs, and nothing downstream
recovers from that. Use an absolute schedule:

```cpp
if ((long)(micros() - nextSampleAt) >= 0) {   // signed: survives rollover
    nextSampleAt += SAMPLE_INTERVAL_US;
    processor.update(analogRead(pin));
}
```

## Board capability

Three channels of float filtering do **not** fit in 1 ms on an ATmega328
(Uno, Nano). On those boards use 500 Hz, or fewer channels. A 32-bit board
(ESP32, Teensy, SAMD) handles three channels at 1000 Hz comfortably. The
`MultiChannelCalibrated` example counts overruns and prints them under `d` —
trust that counter over any estimate, including this one.

## Safety

Unplug any mains power adapter when using sEMG sensors. Only battery-powered
systems should be connected, directly or indirectly, to a person wearing
electrodes.

## Tests

The core is plain C++ with no Arduino dependency, so it is testable on the host:

```
make -C test
```

The suite covers notch depth and passband flatness, bandpass shape, channel
independence, bypass behaviour on invalid configuration, and the full
calibration and drift-tracking behaviour.

## Changes in 2.0.0

Fixes, each covered by a regression test:

- **Notch coefficients for 50 Hz at 500 Hz sampling were corrupt.** A missing
  comma made `1.0000, -1.1187` parse as the subtraction `1.0000 - 1.1187`,
  leaving five initialisers where six were needed and zero-filling the sixth.
  The null at 50 Hz survived, but the second section's poles did not: the
  passband tilted by about 25 dB, cutting 14 dB at 20 Hz where most sEMG power
  sits while amplifying 240 Hz noise by 10 dB.
- **All filter state was file-scope global.** Every `EMGFilters` instance shared
  one set of state variables, so `EMGFilters myFilter[3]` interleaved three
  channels into a single filter and returned noise. Multi-sensor setups could
  not work. State is now per-instance.
- **The cascade ran through `int`.** Each stage's output was truncated before
  feeding the next, quantising three times per sample and biasing every stage
  towards zero. The chain is now float end to end; `update(int)` still returns
  an int but rounds once, at the end.
- **Unsupported configurations left coefficients uninitialised**, risking a
  divide by a zero denominator. Sections now initialise to a defined
  pass-through state.
- **Examples could hang for the age of the universe.** `delay((interval -
  timeElapsed) / 1000)` underflows when a loop iteration runs long, and unsigned
  underflow produces a delay of roughly 584,000 years. The division by 1000 also
  truncated every sub-millisecond remainder to `delay(0)`, so the nominal sample
  rate was never actually held.
- **`sq()` overflowed in `SimpleEMGFilters`.** `int` is 16 bits on AVR, so the
  squared envelope wrapped past ±181 counts.
- **64-bit timestamps broke rollover handling** rather than fixing it. `micros()`
  returns 32 bits; widening the arithmetic turns the correct small wrapped delta
  into a huge positive one.

Added:

- `EMGProcessor` — per-channel envelope detection, two-step calibration,
  normalised `activation()` output, gated baseline tracking, DC-bias removal
  and persistable calibration.
- `MultiChannelCalibrated` example — three sensors, guided calibration,
  overrun reporting.
- Host test suite, `library.properties` and `keywords.txt`.

The `EMGFilters` API is unchanged and existing sketches still compile.

[sen0240]: https://www.dfrobot.com/product-1661.html
