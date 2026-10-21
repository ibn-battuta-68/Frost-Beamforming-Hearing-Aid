#ifndef BEAMFORMER_H
#define BEAMFORMER_H
#include <stdbool.h>
#include <stdint.h>
#define FROST_MICS 4
#define FROST_TAPS 16
#define FROST_Q 20
#define FROST_ONE INT32_C(1048576)
#define FROST_SAMPLE_RATE 8000
typedef struct {
    int32_t weights[FROST_TAPS][FROST_MICS]; /* tap-first, Q20 */
    int16_t history[FROST_TAPS][FROST_MICS]; /* PCM / 8 */
    int32_t inverse_diagonal[FROST_TAPS];
    unsigned head;
    uint32_t resets;
    uint32_t clipped;
} frost_t;
bool frost_init(frost_t *state);
int16_t frost_process(frost_t *state, const int16_t mics[FROST_MICS], bool adapt);
#endif
