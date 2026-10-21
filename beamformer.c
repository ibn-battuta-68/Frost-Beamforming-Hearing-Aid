/* Fixed-point Frost beamforming and Newton-Schulz matrix inversion. */
#include "beamformer.h"

#define MATRIX_MAX_N FROST_TAPS
#define MATRIX_ONE FROST_ONE
#include <limits.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static bool multiply(const int32_t *a, const int32_t *b, int32_t *out, unsigned n) {
    for (unsigned i = 0; i < n; ++i) {
        for (unsigned j = 0; j < n; ++j) {
            int64_t sum = 0;
            for (unsigned k = 0; k < n; ++k) {
                int64_t product = (int64_t)a[i*n+k] * b[k*n+j];
                if ((product > 0 && sum > INT64_MAX-product) ||
                    (product < 0 && sum < INT64_MIN-product)) return false;
                sum += product;
            }
            sum /= MATRIX_ONE;
            if (sum > INT32_MAX || sum < INT32_MIN) return false;
            out[i*n+j] = (int32_t)sum;
        }
    }
    return true;
}

static bool matrix_inverse_spd(const int32_t *a, int32_t *inverse, unsigned n) {
    if (!a || !inverse || !n || n > MATRIX_MAX_N || a == inverse) return false;
    /* Static workspace avoids a large embedded stack. Initialization only;
     * callers must serialize calls. Inverse and input may not overlap. */
    static int32_t product[MATRIX_MAX_N*MATRIX_MAX_N];
    static int32_t correction[MATRIX_MAX_N*MATRIX_MAX_N];
    static int32_t next[MATRIX_MAX_N*MATRIX_MAX_N];
    int64_t norm = 0;
    for (unsigned i = 0; i < n; ++i) {
        if (a[i*n+i] <= 0) return false;
        int64_t row = 0;
        for (unsigned j = 0; j < n; ++j) {
            if (a[i*n+j] != a[j*n+i]) return false;
            row += llabs((int64_t)a[i*n+j]);
        }
        if (row > norm) norm = row;
    }
    /* X0 = I / (2||A||inf). For SPD A this places eigenvalues of AX0 in
     * (0, 1/2], so Newton-Schulz X <- X(2I-AX) converges. The conservative
     * seed also exercises refinement for the default diagonal Gram matrix. */
    int64_t seed = (int64_t)MATRIX_ONE*MATRIX_ONE / (2*norm);
    if (seed <= 0 || seed > INT32_MAX) return false;
    memset(inverse, 0, n*n*sizeof(*inverse));
    for (unsigned i = 0; i < n; ++i) inverse[i*n+i] = (int32_t)seed;
    for (unsigned iteration = 0; iteration < 32; ++iteration) {
        if (!multiply(a, inverse, product, n)) return false;
        int64_t worst = 0;
        for (unsigned i = 0; i < n; ++i) {
            int64_t row = 0;
            for (unsigned j = 0; j < n; ++j) {
                int64_t identity = i == j ? MATRIX_ONE : 0;
                row += llabs(identity-product[i*n+j]);
                int64_t value = 2*identity-product[i*n+j];
                if (value > INT32_MAX || value < INT32_MIN) return false;
                correction[i*n+j] = (int32_t)value;
            }
            if (row > worst) worst = row;
        }
        if (worst <= 128) return true; /* infinity residual <= 2^-13 */
        if (!multiply(inverse, correction, next, n)) return false;
        memcpy(inverse, next, n*n*sizeof(*inverse));
    }
    return false;
}

static void reset_weights(frost_t *s) {
    memset(s->weights, 0, sizeof(s->weights));
    for (unsigned k = 0; k < FROST_MICS; ++k)
        s->weights[0][k] = FROST_ONE/FROST_MICS;
}

bool frost_init(frost_t *s) {
    if (!s) return false;
    memset(s, 0, sizeof(*s));
    /* C selects the four microphone weights at each tap, so C^T C = 4I.
     * Compute its inverse at startup, then exploit the diagonal structure. */
    static int32_t gram[FROST_TAPS*FROST_TAPS];
    static int32_t inverse[FROST_TAPS*FROST_TAPS];
    memset(gram, 0, sizeof(gram));
    for (unsigned j = 0; j < FROST_TAPS; ++j)
        gram[j*FROST_TAPS+j] = FROST_MICS*FROST_ONE;
    if (!matrix_inverse_spd(gram, inverse, FROST_TAPS)) return false;
    for (unsigned j = 0; j < FROST_TAPS; ++j)
        s->inverse_diagonal[j] = inverse[j*FROST_TAPS+j];
    reset_weights(s);
    return true;
}

int16_t frost_process(frost_t *s, const int16_t mics[FROST_MICS], bool adapt) {
    s->head = (s->head + FROST_TAPS-1) % FROST_TAPS;
    for (unsigned k = 0; k < FROST_MICS; ++k)
        s->history[s->head][k] = (int16_t)(mics[k]/8);
    int64_t acc = 0;
    int64_t energy = 65536; /* regularization for silence / low-level signals */
    for (unsigned j = 0; j < FROST_TAPS; ++j) {
        const int16_t *x = s->history[(s->head+j)%FROST_TAPS];
        for (unsigned k = 0; k < FROST_MICS; ++k) {
            acc += (int64_t)s->weights[j][k]*x[k];
            energy += (int32_t)x[k]*x[k];
        }
    }
    int32_t y = (int32_t)(acc/FROST_ONE);
    if (adapt) {
        /* Normalized LMS with mu=1/8. One reciprocal division per sample;
         * gain is Q30, updates are Q20. No float in the processing path. */
        int64_t gain = ((int64_t)y*INT64_C(1073741824))/(8*energy);
        bool bounded = true;
        for (unsigned j = 0; j < FROST_TAPS; ++j) {
            const int16_t *x = s->history[(s->head+j)%FROST_TAPS];
            int64_t sum = 0;
            for (unsigned k = 0; k < FROST_MICS; ++k) {
                s->weights[j][k] -= (int32_t)(gain*x[k]/1024);
                sum += s->weights[j][k];
            }
            int32_t target = j == 0 ? FROST_ONE : 0;
            int32_t correction = (int32_t)((sum-target)*s->inverse_diagonal[j]/FROST_ONE);
            int64_t projected_sum = 0;
            for (unsigned k = 0; k < FROST_MICS; ++k) {
                s->weights[j][k] -= correction;
                projected_sum += s->weights[j][k];
            }
            /* Redistribute integer remainder so C^T w = f exactly, including
             * negative sums; avoids cumulative fixed-point constraint drift. */
            int32_t remainder = (int32_t)(target-projected_sum);
            for (unsigned k = 0; k < FROST_MICS; ++k) {
                int32_t share = remainder/(int32_t)(FROST_MICS-k);
                s->weights[j][k] += share;
                remainder -= share;
                if (s->weights[j][k] > 2*FROST_ONE ||
                    s->weights[j][k] < -2*FROST_ONE) bounded = false;
            }
        }
        if (!bounded) { reset_weights(s); ++s->resets; }
    }
    int32_t pcm = y*8;
    if (pcm > 32767) { pcm = 32767; ++s->clipped; }
    if (pcm < -32768) { pcm = -32768; ++s->clipped; }
    return (int16_t)pcm;
}
