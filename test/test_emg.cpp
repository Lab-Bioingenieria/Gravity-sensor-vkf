// Host-side regression tests for EMGFilters and EMGProcessor.
//
// Build and run with `make -C test`. No Arduino toolchain required; the
// library core is plain C++ on purpose so it can be tested off-target.

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "../EMGFilters.h"
#include "../EMGProcessor.h"

static int g_failures = 0;
static int g_checks   = 0;

static void check(bool condition, const char *what) {
    g_checks++;
    if (!condition) {
        g_failures++;
        std::printf("  FAIL: %s\n", what);
    }
}

static void checkRange(double value,
                       double low,
                       double high,
                       const char *what) {
    g_checks++;
    if (!(value >= low && value <= high)) {
        g_failures++;
        std::printf("  FAIL: %s (got %.4f, expected %.4f..%.4f)\n",
                    what,
                    value,
                    low,
                    high);
    }
}

// Steady-state gain of a filter at one frequency, measured as the half
// peak-to-peak of the output over the final cycles of a long sine burst.
static double measureGain(EMGFilters &f, double freq, double fs) {
    const int settle = (int)(fs * 2.0);
    const int window = (int)(fs * 1.0);

    for (int n = 0; n < settle; n++) {
        f.updateFloat((float)std::sin(2.0 * M_PI * freq * n / fs));
    }

    double lo = 1e30, hi = -1e30;
    for (int n = 0; n < window; n++) {
        const double y =
            f.updateFloat((float)std::sin(2.0 * M_PI * freq * (settle + n) / fs));
        if (y < lo) lo = y;
        if (y > hi) hi = y;
    }
    return (hi - lo) / 2.0;
}

// --------------------------------------------------------------------------

// The 50Hz/500Hz denominator table used to read
// `{1.0000, -1.5395, 0.9056, 1.0000 - 1.1187, 0.3129}`. The missing comma made
// that a subtraction, leaving five initialisers and a zero-filled sixth. The
// notch itself survived (the zeros make the null) but the second section's
// poles were destroyed, tilting the passband by ~25dB: -14dB at 20Hz where
// most sEMG power sits, +10dB at 240Hz where only noise sits.
static void test_notch_passband_is_flat() {
    std::printf("test_notch_passband_is_flat\n");

    const double fs = 500.0;
    const double probes[] = {20.0, 40.0, 80.0, 120.0, 160.0, 200.0, 240.0};

    for (size_t i = 0; i < sizeof(probes) / sizeof(probes[0]); i++) {
        EMGFilters f;
        // Notch only: the lowpass and highpass would mask the tilt.
        f.init(SAMPLE_FREQ_500HZ, NOTCH_FREQ_50HZ, true, false, false);

        char label[64];
        std::snprintf(label, sizeof(label), "notch gain flat at %.0fHz",
                      probes[i]);
        checkRange(measureGain(f, probes[i], fs), 0.70, 1.60, label);
    }
}

// The two sample rates ship genuinely different notch designs, and the
// difference matters when choosing a rate:
//
//   fs=500   one exact null on the hum frequency          -> about -74dB
//   fs=1000  two nulls straddling it (49.3Hz and 50.6Hz)  -> about -19dB,
//            but flat across the whole 49..51Hz band, so it keeps working
//            when the mains frequency wanders.
//
// Expecting 500Hz-class depth from the 1000Hz design is a misreading of the
// coefficient tables, not a defect.
static void test_notch_rejects_hum() {
    std::printf("test_notch_rejects_hum\n");

    {
        EMGFilters f;
        f.init(SAMPLE_FREQ_500HZ, NOTCH_FREQ_50HZ, true, false, false);
        check(measureGain(f, 50.0, 500.0) < 0.01, "50Hz deep null at fs=500");
    }
    {
        EMGFilters f;
        f.init(SAMPLE_FREQ_500HZ, NOTCH_FREQ_60HZ, true, false, false);
        check(measureGain(f, 60.0, 500.0) < 0.01, "60Hz deep null at fs=500");
    }
    {
        EMGFilters f;
        f.init(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ, true, false, false);
        check(measureGain(f, 50.0, 1000.0) < 0.15, "50Hz rejected at fs=1000");
    }
    {
        EMGFilters f;
        f.init(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_60HZ, true, false, false);
        check(measureGain(f, 60.0, 1000.0) < 0.15, "60Hz rejected at fs=1000");
    }

    // The wide-stopband property of the 1000Hz design is the point of it, so
    // pin it: rejection must hold across a mains excursion of +/-1Hz.
    const double offsets[] = {49.0, 49.5, 50.0, 50.5, 51.0};
    for (size_t i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        EMGFilters f;
        f.init(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ, true, false, false);

        char label[64];
        std::snprintf(label, sizeof(label), "stopband holds at %.1fHz",
                      offsets[i]);
        check(measureGain(f, offsets[i], 1000.0) < 0.15, label);
    }
}

static void test_bandpass_shape() {
    std::printf("test_bandpass_shape\n");

    EMGFilters f;
    f.init(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ, false, true, true);

    const double passband = measureGain(f, 100.0, 1000.0);

    EMGFilters low;
    low.init(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ, false, true, true);
    const double below = measureGain(low, 5.0, 1000.0);

    EMGFilters high;
    high.init(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ, false, true, true);
    const double above = measureGain(high, 400.0, 1000.0);

    check(below < passband * 0.25, "5Hz attenuated below passband");
    check(above < passband * 0.25, "400Hz attenuated below passband");
}

// Every filter section used to be a file-scope global shared by all instances.
// `EMGFilters myFilter[3]` therefore ran three channels through one set of
// state variables, interleaving them into noise. This is the bug that makes
// multi-sensor setups fail.
static void test_instances_are_independent() {
    std::printf("test_instances_are_independent\n");

    const int N = 400;

    // Reference: channel A alone, nothing else touching the library.
    EMGFilters solo;
    solo.init(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);
    double reference[N];
    for (int n = 0; n < N; n++) {
        reference[n] = solo.updateFloat((float)std::sin(2.0 * M_PI * 90.0 * n / 1000.0));
    }

    // Now the same channel A interleaved with two very different neighbours.
    EMGFilters ch[3];
    for (int i = 0; i < 3; i++) {
        ch[i].init(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);
    }

    double maxDelta = 0.0;
    for (int n = 0; n < N; n++) {
        const double a = ch[0].updateFloat((float)std::sin(2.0 * M_PI * 90.0 * n / 1000.0));
        ch[1].updateFloat(400.0f);
        ch[2].updateFloat((float)(-350.0 * std::sin(2.0 * M_PI * 17.0 * n / 1000.0)));

        const double d = std::fabs(a - reference[n]);
        if (d > maxDelta) maxDelta = d;
    }

    check(maxDelta < 1e-6, "channel 0 output unaffected by channels 1 and 2");
}

static void test_invalid_config_bypasses() {
    std::printf("test_invalid_config_bypasses\n");

    EMGFilters f;
    f.init((SAMPLE_FREQUENCY)250, NOTCH_FREQ_50HZ);
    check(f.isBypassed(), "unsupported sample rate reports bypass");

    // Must pass through cleanly rather than divide by an uninitialised
    // denominator, which the previous version did.
    bool clean = true;
    for (int n = 0; n < 200; n++) {
        const float y = f.updateFloat(123.0f);
        if (!std::isfinite(y) || std::fabs(y - 123.0f) > 1e-6f) clean = false;
    }
    check(clean, "bypassed filter passes input through unchanged");
}

static void test_no_intermediate_rounding() {
    std::printf("test_no_intermediate_rounding\n");

    // A small-amplitude signal is where int truncation between stages used to
    // destroy the most information. The float path must preserve a non-trivial
    // response instead of collapsing towards zero.
    EMGFilters f;
    f.init(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);

    double energy = 0.0;
    for (int n = 0; n < 2000; n++) {
        const double y = f.updateFloat((float)(1.5 * std::sin(2.0 * M_PI * 90.0 * n / 1000.0)));
        if (n > 1000) energy += y * y;
    }
    check(energy > 0.1, "sub-count amplitude survives the cascade");
}

// --------------------------------------------------------------------------

// Deterministic broadband noise, scaled to a contraction amplitude, riding on
// the SEN0240's mid-supply DC offset.
struct SyntheticEmg {
    unsigned long seed;
    double        dcBias;

    SyntheticEmg() : seed(12345UL), dcBias(307.0) {}

    double next(double amplitude, double humAmplitude, int n, double fs) {
        seed = seed * 1103515245UL + 12345UL;
        const double white = ((double)((seed >> 16) & 0x7FFF) / 16383.5) - 1.0;
        const double hum = humAmplitude * std::sin(2.0 * M_PI * 50.0 * n / fs);
        return dcBias + amplitude * white + hum;
    }
};

static void runFor(EMGProcessor &p,
                   SyntheticEmg &src,
                   double        amplitude,
                   double        hum,
                   int           samples,
                   int          &clock,
                   double        fs) {
    for (int i = 0; i < samples; i++) {
        p.update((int)std::lround(src.next(amplitude, hum, clock, fs)));
        clock++;
    }
}

static void test_calibration_normalises_across_amplitudes() {
    std::printf("test_calibration_normalises_across_amplitudes\n");

    const double fs = 1000.0;
    EMGProcessor p;
    p.begin(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);

    SyntheticEmg src;
    int clock = 0;

    check(!p.isCalibrated(), "starts uncalibrated");
    check(p.activation() == 0.0f, "uncalibrated activation is zero");

    // Settle, then measure the resting floor.
    runFor(p, src, 2.0, 20.0, 500, clock, fs);
    p.startRestCalibration(2000);
    runFor(p, src, 2.0, 20.0, 2000, clock, fs);
    check(p.state() == EMG_CAL_REST_DONE, "rest step completes");

    // Maximum voluntary contraction.
    p.startMvcCalibration(2000);
    runFor(p, src, 120.0, 20.0, 2000, clock, fs);
    check(p.state() == EMG_CAL_READY, "MVC step completes");
    check(p.isCalibrated(), "calibration is valid after both steps");

    // Rest again: should read near zero and report inactive.
    runFor(p, src, 2.0, 20.0, 1000, clock, fs);
    checkRange(p.activation(), 0.0, 0.10, "rest normalises near 0");
    check(!p.isActive(), "rest is not active");

    // Full contraction: should read near one.
    runFor(p, src, 120.0, 20.0, 1000, clock, fs);
    checkRange(p.activation(), 0.75, 1.00, "MVC normalises near 1");
    check(p.isActive(), "MVC is active");

    // Half effort should land in the middle, not saturate either end.
    runFor(p, src, 60.0, 20.0, 1000, clock, fs);
    checkRange(p.activation(), 0.25, 0.75, "half effort lands mid-scale");
}

// The whole point of normalisation: the same relative effort must produce the
// same activation for two people whose raw signal amplitudes differ by 8x.
// Normalisation only collapses amplitude differences while both wearers sit
// comfortably above the ADC noise floor. A SEN0240 on a 5V 10-bit ADC swings
// about +/-307 counts at full scale, so 60 and 300 counts of MVC span a
// realistic weak-to-strong range. Push the weak end down towards a few counts
// and quantisation noise inflates that wearer's own MVC reference, which
// compresses their mid-scale readings - physics, not a defect.
static void test_normalisation_is_wearer_independent() {
    std::printf("test_normalisation_is_wearer_independent\n");

    const double fs         = 1000.0;
    const double wearers[2] = {60.0, 300.0};
    float        readings[2];

    for (int w = 0; w < 2; w++) {
        EMGProcessor p;
        p.begin(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);

        SyntheticEmg src;
        int clock = 0;
        const double mvc = wearers[w];

        runFor(p, src, mvc * 0.02, 20.0, 500, clock, fs);
        p.startRestCalibration(2000);
        runFor(p, src, mvc * 0.02, 20.0, 2000, clock, fs);
        p.startMvcCalibration(2000);
        runFor(p, src, mvc, 20.0, 2000, clock, fs);

        // Both wearers now contract at 50% of their own maximum.
        runFor(p, src, mvc * 0.5, 20.0, 1000, clock, fs);
        readings[w] = p.activation();
    }

    std::printf("    weak wearer %.3f, strong wearer %.3f\n",
                readings[0], readings[1]);
    check(std::fabs(readings[0] - readings[1]) < 0.15,
          "8x amplitude difference collapses to the same activation");
}

static void test_mvc_requires_rest_first() {
    std::printf("test_mvc_requires_rest_first\n");

    EMGProcessor p;
    p.begin(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);
    p.startMvcCalibration(1000);
    check(p.state() != EMG_CAL_MVC, "MVC step refused before rest step");
}

static void test_degenerate_mvc_is_rejected() {
    std::printf("test_degenerate_mvc_is_rejected\n");

    const double fs = 1000.0;
    EMGProcessor p;
    p.begin(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);

    SyntheticEmg src;
    int clock = 0;

    runFor(p, src, 2.0, 5.0, 500, clock, fs);
    p.startRestCalibration(1000);
    runFor(p, src, 2.0, 5.0, 1000, clock, fs);

    // Wearer never contracts: span stays at noise level.
    p.startMvcCalibration(1000);
    runFor(p, src, 2.0, 5.0, 1000, clock, fs);

    check(!p.isCalibrated(), "no-contraction MVC is rejected");
    check(p.activation() == 0.0f, "rejected calibration yields zero activation");
}

// The baseline auto-adjust must absorb slow drift while the muscle is at rest,
// and must NOT follow the signal during a contraction.
static void test_baseline_tracker_follows_drift_but_not_contraction() {
    std::printf("test_baseline_tracker_follows_drift_but_not_contraction\n");

    const double fs = 1000.0;
    EMGProcessor p;
    p.begin(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);

    SyntheticEmg src;
    int clock = 0;

    runFor(p, src, 3.0, 20.0, 500, clock, fs);
    p.startRestCalibration(2000);
    runFor(p, src, 3.0, 20.0, 2000, clock, fs);
    p.startMvcCalibration(2000);
    runFor(p, src, 150.0, 20.0, 2000, clock, fs);
    check(p.isCalibrated(), "calibrated for drift test");

    // Resting noise triples, as it does when the skin-electrode interface
    // changes. The tracker should absorb it and stop reporting activity.
    runFor(p, src, 9.0, 20.0, 500, clock, fs);
    runFor(p, src, 9.0, 20.0, 20000, clock, fs);
    checkRange(p.activation(), 0.0, 0.12,
               "baseline absorbs a 3x rise in resting noise");

    // Now hold a strong contraction for 20s. If the tracker were ungated it
    // would climb to the contraction level and the reading would collapse.
    const float before = p.activation();
    runFor(p, src, 150.0, 20.0, 20000, clock, fs);
    const float after = p.activation();

    std::printf("    activation at contraction start %.3f, after 20s %.3f\n",
                before, after);
    check(after > 0.5f, "sustained contraction is not erased by the tracker");
}

static void test_saved_calibration_round_trips() {
    std::printf("test_saved_calibration_round_trips\n");

    const double fs = 1000.0;
    EMGCalibration saved;

    {
        EMGProcessor p;
        p.begin(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);
        SyntheticEmg src;
        int clock = 0;

        runFor(p, src, 3.0, 20.0, 500, clock, fs);
        p.startRestCalibration(2000);
        runFor(p, src, 3.0, 20.0, 2000, clock, fs);
        p.startMvcCalibration(2000);
        runFor(p, src, 120.0, 20.0, 2000, clock, fs);
        saved = p.calibration();
    }

    EMGProcessor p2;
    p2.begin(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);
    check(p2.setCalibration(saved), "stored calibration is accepted");

    SyntheticEmg src;
    int clock = 0;
    runFor(p2, src, 120.0, 20.0, 1500, clock, fs);
    checkRange(p2.activation(), 0.60, 1.00,
               "restored calibration reproduces a high reading at MVC");

    EMGCalibration junk;
    junk.valid     = true;
    junk.restLevel = 10.0f;
    junk.mvcLevel  = 10.5f;
    EMGProcessor p3;
    p3.begin(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);
    check(!p3.setCalibration(junk), "unusable span is rejected on restore");
}

static void test_dc_bias_is_removed_without_startup_kick() {
    std::printf("test_dc_bias_is_removed_without_startup_kick\n");

    EMGProcessor p;
    p.begin(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ);

    // SEN0240 idles at 1.5V; on a 5V 10-bit ADC that is ~307 counts.
    double worst = 0.0;
    for (int n = 0; n < 50; n++) {
        p.update(307);
        const double y = std::fabs(p.filtered());
        if (y > worst) worst = y;
    }

    check(worst < 5.0, "constant mid-supply input produces no large transient");
    checkRange(p.dcBias(), 300.0, 315.0, "DC bias estimate tracks the input");
}

// --------------------------------------------------------------------------

int main() {
    test_notch_passband_is_flat();
    test_notch_rejects_hum();
    test_bandpass_shape();
    test_instances_are_independent();
    test_invalid_config_bypasses();
    test_no_intermediate_rounding();
    test_calibration_normalises_across_amplitudes();
    test_normalisation_is_wearer_independent();
    test_mvc_requires_rest_first();
    test_degenerate_mvc_is_rejected();
    test_baseline_tracker_follows_drift_but_not_contraction();
    test_saved_calibration_round_trips();
    test_dc_bias_is_removed_without_startup_kick();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
