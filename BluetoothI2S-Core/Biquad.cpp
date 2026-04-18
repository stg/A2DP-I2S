// Biquad supplies creation and processing of biquad filters, as well as data formatting for fixed-point DSPs

#include <math.h>
#include "Biquad.h"

// 32-bit signed multiply -> 32-bit result, add 32-bit (ARM: SMMLAR)
// floating point equivalent: return c + a * b
static inline __attribute__((always_inline)) int32_t smmlar(int32_t a, int32_t b, int32_t c) {
  int32_t result;
#if defined(__ARMCC_VERSION) && (__CORTEX_M >= 0x04U)
  __asm{ smmlar result, a, b, c }
#elif defined(__GNUC__) && defined(__arm__) && (__CORTEX_M >= 0x04U)
  __asm("smmlar %0, %1, %2, %3":"=r"(result):"r"(a),"r"(b),"r"(c));
#else
  result = c + ((((int64_t)a * b) + 0x80000000) >> 32);
#endif
  return result;
}

// Convert float to Qn fixed-point representation
int32_t floatQ(float f, uint8_t Q) {
    const float s = ldexpf(f, Q);
    return (s <= (float)INT32_MIN) ? INT32_MIN :
           (s >= (float)INT32_MAX) ? INT32_MAX :
           (int32_t)lrintf(s);
}

// Convert fixed-point Qn representation to float
float qFloat(int32_t i, uint8_t Q) {
    return ldexpf((float)i, -(int)Q);
}

// Design a biquad filter
biquad_filter biquadMake(filter_type type, float Fs, float Fc, float Q, float gain) {
    biquad_filter filter;
    float *a = filter.a, *b = filter.b;
    float norm;
    float Vabs = powf(10.0f, fabsf(gain) / 20.0f);
    float V = powf(10.0f, gain / 20.0f);
    float Wc = (float)M_PI * Fc / Fs;
    float K = tanf(Wc);

    switch (type) {
        case FILTER_LOWPASS_1P: {
            float r = expf(-2.0f * Wc);
            b[0] = 1.0f - r;
            b[1] = 0.0f;
            b[2] = 0.0f;
            a[0] = r;
            a[1] = 0.0f;
        } break;

        case FILTER_HIGHPASS_1P: {
            float r = expf(-2.0f * Wc);
            b[0] = r;
            b[1] = -r;
            b[2] = 0.0f;
            a[0] = r;
            a[1] = 0.0f;
        } break;

        case FILTER_LOWPASS_1P1Z:
            norm = 1.0f / (1.0f / K + 1.0f);
            b[0] = norm;
            b[1] = norm;
            b[2] = 0.0f;
            a[0] = -(1.0f - 1.0f / K) * norm;
            a[1] = 0.0f;
            break;

        case FILTER_HIGHPASS_1P1Z:
            norm = 1.0f / (K + 1.0f);
            b[0] = norm;
            b[1] = -norm;
            b[2] = 0.0f;
            a[0] = -(K - 1.0f) * norm;
            a[1] = 0.0f;
            break;

        case FILTER_LOWPASS:
            norm = 1.0f / (1.0f + K / Q + K * K);
            b[0] = K * K * norm;
            b[1] = 2.0f * b[0];
            b[2] = b[0];
            a[0] = -2.0f * (K * K - 1.0f) * norm;
            a[1] = -(1.0f - K / Q + K * K) * norm;
            break;

        case FILTER_HIGHPASS:
            norm = 1.0f / (1.0f + K / Q + K * K);
            b[0] = norm;
            b[1] = -2.0f * b[0];
            b[2] = b[0];
            a[0] = -2.0f * (K * K - 1.0f) * norm;
            a[1] = -(1.0f - K / Q + K * K) * norm;
            break;

        case FILTER_BANDPASS:
            norm = 1.0f / (1.0f + K / Q + K * K);
            b[0] = K / Q * norm;
            b[1] = 0.0f;
            b[2] = -b[0];
            a[0] = -2.0f * (K * K - 1.0f) * norm;
            a[1] = -(1.0f - K / Q + K * K) * norm;
            break;

        case FILTER_NOTCH:
            norm = 1.0f / (1.0f + K / Q + K * K);
            b[0] = (1.0f + K * K) * norm;
            b[1] = 2.0f * (K * K - 1.0f) * norm;
            b[2] = b[0];
            a[0] = -b[1];
            a[1] = -(1.0f - K / Q + K * K) * norm;
            break;

        case FILTER_PEAK: {
            float num_kq = (gain >= 0.0f) ? (Vabs * K / Q) : (K / Q);
            float den_kq = (gain >= 0.0f) ? (K / Q) : (Vabs * K / Q);
            norm = 1.0f / (1.0f + den_kq + K * K);
            b[0] = (1.0f + num_kq + K * K) * norm;
            b[1] = 2.0f * (K * K - 1.0f) * norm;
            b[2] = (1.0f - num_kq + K * K) * norm;
            a[0] = -b[1];
            a[1] = -(1.0f - den_kq + K * K) * norm;
        } break;

        case FILTER_LOWSHELF: {
            float num_k  = (gain >= 0.0f) ? (sqrtf(2.0f * Vabs) * K) : ((float)M_SQRT2 * K);
            float den_k  = (gain >= 0.0f) ? ((float)M_SQRT2 * K) : (sqrtf(2.0f * Vabs) * K);
            float num_k2 = (gain >= 0.0f) ? (Vabs * K * K) : (K * K);
            float den_k2 = (gain >= 0.0f) ? (K * K) : (Vabs * K * K);
            norm = 1.0f / (1.0f + den_k + den_k2);
            b[0] = (1.0f + num_k + num_k2) * norm;
            b[1] = 2.0f * (num_k2 - 1.0f) * norm;
            b[2] = (1.0f - num_k + num_k2) * norm;
            a[0] = -2.0f * (den_k2 - 1.0f) * norm;
            a[1] = -(1.0f - den_k + den_k2) * norm;
        } break;

        case FILTER_HIGHSHELF: {
            float num_c = (gain >= 0.0f) ? Vabs : 1.0f;
            float den_c = (gain >= 0.0f) ? 1.0f : Vabs;
            float num_k = (gain >= 0.0f) ? (sqrtf(2.0f * Vabs) * K) : ((float)M_SQRT2 * K);
            float den_k = (gain >= 0.0f) ? ((float)M_SQRT2 * K) : (sqrtf(2.0f * Vabs) * K);
            norm = 1.0f / (den_c + den_k + K * K);
            b[0] = (num_c + num_k + K * K) * norm;
            b[1] = 2.0f * (K * K - num_c) * norm;
            b[2] = (num_c - num_k + K * K) * norm;
            a[0] = -2.0f * (K * K - den_c) * norm;
            a[1] = -(den_c - den_k + K * K) * norm;
        } break;

        case FILTER_LOWSHELF_1ST: {
            float num_k = (gain >= 0.0f) ? (K * Vabs) : K;
            float den_k = (gain >= 0.0f) ? K : (K * Vabs);
            norm = 1.0f / (den_k + 1.0f);
            b[0] = (num_k + 1.0f) * norm;
            b[1] = (num_k - 1.0f) * norm;
            b[2] = 0.0f;
            a[0] = -(den_k - 1.0f) * norm;
            a[1] = 0.0f;
        } break;

        case FILTER_HIGHSHELF_1ST: {
            float num_c = (gain >= 0.0f) ? Vabs : 1.0f;
            float den_c = (gain >= 0.0f) ? 1.0f : Vabs;
            norm = 1.0f / (K + den_c);
            b[0] = (K + num_c) * norm;
            b[1] = (K - num_c) * norm;
            b[2] = 0.0f;
            a[0] = -(K - den_c) * norm;
            a[1] = 0.0f;
        } break;

        case FILTER_ALLPASS:
            norm = 1.0f / (1.0f + K / Q + K * K);
            b[0] = (1.0f - K / Q + K * K) * norm;
            b[1] = 2.0f * (K * K - 1.0f) * norm;
            b[2] = 1.0f;
            a[0] = -b[1];
            a[1] = -b[0];
            break;

        case FILTER_ALLPASS_1ST:
            b[0] = (1.0f - K) / (1.0f + K);
            b[1] = -1.0f;
            b[2] = 0.0f;
            a[0] = b[0];
            a[1] = 0.0f;
            break;

        case FILTER_NONE:
        default:
            b[0] = V;
            b[1] = 0.0f;
            b[2] = 0.0f;
            a[0] = 0.0f;
            a[1] = 0.0f;
            break;
    }

    return filter;
}


// Process sample x through biquad filter defined by coef, with internal state hist
float biquadProcess(biquad_history &hist, biquad_filter &coef, float x) {
    float y = coef.b[0] * x
            + coef.b[1] * hist.x[0]
            + coef.b[2] * hist.x[1]
            + coef.a[0] * hist.y[0]
            + coef.a[1] * hist.y[1];
    hist.x[1] = hist.x[0]; hist.x[0] = x;
    hist.y[1] = hist.y[0]; hist.y[0] = y;
    return y;
}

// Same as above but for fixed-point
int32_t biquadProcessQ(biquad_history_q *hist, biquad_filter_q *coef, int32_t x) {
    int32_t y = (
        smmlar(coef->b[0], x,
        smmlar(coef->b[1], hist->x[0],
        smmlar(coef->b[2], hist->x[1],
        smmlar(coef->a[0], hist->y[0],
        smmlar(coef->a[1], hist->y[1],
    0)))))) << 2;
    
    hist->x[1] = hist->x[0]; hist->x[0] = x;
    hist->y[1] = hist->y[0]; hist->y[0] = y;
    return y;
}

// Convert a biquad filter to Qn fixed-point representation
biquad_filter_q biquadQ(biquad_filter *coef, uint8_t Q) {
    biquad_filter_q filter;
    for(uint8_t n = 0; n < 5; n++)
        filter.coef[n] = floatQ(coef->coef[n], Q);
    return filter;
}
