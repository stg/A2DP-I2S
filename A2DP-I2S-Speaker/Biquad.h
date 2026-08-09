#ifndef __BIQUAD_H__
#define __BIQUAD_H__

typedef enum {
    FILTER_NONE,          // Gain filter (gain = 0 for unity-gain identity filter)
    FILTER_LOWPASS_1P,    // Low-pass   1st order (1 pole)
    FILTER_LOWPASS_1P1Z,  // Low-pass   1st order (1 pole 1 zero)
    FILTER_LOWPASS,       // Low-pass   2nd order
    FILTER_HIGHPASS_1P,   // High-pass  1st order (1 pole)
    FILTER_HIGHPASS_1P1Z, // High-pass  1st order (1 pole 1 zero)
    FILTER_HIGHPASS,      // High-pass  2nd order
    FILTER_LOWSHELF_1ST,  // Low-shelf  1st order
    FILTER_LOWSHELF,      // Low-shelf  2nd order
    FILTER_HIGHSHELF_1ST, // High-shelf 1st order
    FILTER_HIGHSHELF,     // High-shelf 2nd order
    FILTER_ALLPASS_1ST,   // All-pass   1st order
    FILTER_ALLPASS,       // All-pass   2nd order
    FILTER_BANDPASS,      // Band-pass  2nd order
    FILTER_NOTCH,         // Notch      2nd order
    FILTER_PEAK,          // Peak       2nd order
} filter_type;

typedef struct {
    union {
    float coef[5];
        struct {
            float b[3];
            float a[2];
        };
    };
} biquad_filter;

typedef struct {
    float x[2];
    float y[2];
} biquad_history;

typedef struct {
    union {
        int32_t coef[5];
        struct {
            int32_t b[3];
            int32_t a[2];
        };
    };
} biquad_filter_q;

typedef struct {
    int32_t x[2];
    int32_t y[2];
} biquad_history_q;

// Biquad generation and processing
biquad_filter biquadMake(filter_type type, float Fs, float Fc, float Q = 1, float gain = 0);
float biquadProcess(biquad_history &hist, biquad_filter &coef, float x);

// Fixed-point version
biquad_filter_q biquadQ(biquad_filter *coef, uint8_t Q);
int32_t biquadProcessQ(biquad_history_q *hist, biquad_filter_q *coef, int32_t x);

// Conversion of floating-point<>fixed-point
int32_t floatQ(float f, uint8_t Q);
float qFloat(int32_t i, uint8_t Q);

#endif