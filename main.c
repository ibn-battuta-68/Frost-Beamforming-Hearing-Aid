/* RP2040 audio pipeline: synchronized PIO input -> dual DMA -> Frost -> USB.
 * Uses the Pico SDK; i2s_input.pio.h is generated from i2s_input.pio.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "i2s_input.pio.h"
#include "beamformer.h"

#define AUDIO_FRAMES 32 /* 4 ms at 8 kHz */
#define AUDIO_WORDS (2*AUDIO_FRAMES)
#define AUDIO_BUFFERS 3
#define AUDIO_BCLK_PIN 16 /* also fixed in i2s_input.pio */
#define AUDIO_WS_PIN 17   /* also fixed in i2s_input.pio */
#define AUDIO_SD_A_PIN 18
#define AUDIO_SD_B_PIN 19
#define AUDIO_TIMING_PIN 20
#define USB_QUEUE_BLOCKS 8

typedef struct {
    const uint32_t *a;
    const uint32_t *b;
    unsigned slot_a, slot_b;
    uint32_t completed_us;
} audio_block_t;

enum { FREE, FILLING, READY, READING };
static uint32_t samples[2][AUDIO_BUFFERS][AUDIO_WORDS];
static volatile unsigned ownership[2][AUDIO_BUFFERS];
static uint32_t sequences[2][AUDIO_BUFFERS], completed[2][AUDIO_BUFFERS];
static unsigned active[2];
static uint32_t produced[2], consumed;
static int channels[2];
static volatile bool failed;
static PIO audio_pio = pio0;

static void stop_capture(void) {
    failed = true;
    pio_set_sm_mask_enabled(audio_pio, 7u, false);
}

static void dma_irq(void) {
    for (unsigned lane = 0; lane < 2; ++lane) {
        uint32_t mask = 1u << (unsigned)channels[lane];
        if (!(dma_hw->ints0 & mask)) continue;
        dma_hw->ints0 = mask;
        unsigned done = active[lane];
        sequences[lane][done] = produced[lane]++;
        completed[lane][done] = time_us_32();
        __dmb(); /* publish DMA-written sample data before READY */
        ownership[lane][done] = READY;
        unsigned next;
        for (next = 0; next < AUDIO_BUFFERS; ++next)
            if (ownership[lane][next] == FREE) break;
        if (next == AUDIO_BUFFERS) { stop_capture(); return; }
        ownership[lane][next] = FILLING;
        active[lane] = next;
        dma_channel_set_write_addr((uint)channels[lane], samples[lane][next], false);
        dma_channel_set_trans_count((uint)channels[lane], AUDIO_WORDS, true);
    }
}

static void audio_input_init(void) {
    for (unsigned sm = 0; sm < 3; ++sm) pio_sm_claim(audio_pio, sm);
    uint clocks = pio_add_program(audio_pio, &i2s_clocks_program);
    uint receive = pio_add_program(audio_pio, &i2s_receive_program);
    pio_gpio_init(audio_pio, AUDIO_BCLK_PIN);
    pio_gpio_init(audio_pio, AUDIO_WS_PIN);
    pio_sm_config c = i2s_clocks_program_get_default_config(clocks);
    sm_config_set_sideset_pins(&c, AUDIO_BCLK_PIN);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys)/(128.0f*FROST_SAMPLE_RATE));
    pio_sm_set_consecutive_pindirs(audio_pio, 0, AUDIO_BCLK_PIN, 2, true);
    pio_sm_set_pins_with_mask(audio_pio, 0, 0, (1u<<AUDIO_BCLK_PIN)|(1u<<AUDIO_WS_PIN));
    pio_sm_init(audio_pio, 0, clocks, &c);
    for (unsigned lane = 0; lane < 2; ++lane) {
        uint sm = lane+1;
        uint pin = lane == 0 ? AUDIO_SD_A_PIN : AUDIO_SD_B_PIN;
        pio_gpio_init(audio_pio, pin);
        gpio_pull_down(pin);
        pio_sm_set_consecutive_pindirs(audio_pio, sm, pin, 1, false);
        c = i2s_receive_program_get_default_config(receive);
        sm_config_set_in_pins(&c, pin);
        sm_config_set_in_shift(&c, false, true, 32);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
        pio_sm_init(audio_pio, sm, receive, &c);
        channels[lane] = dma_claim_unused_channel(true);
        dma_channel_config d = dma_channel_get_default_config((uint)channels[lane]);
        channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
        channel_config_set_read_increment(&d, false);
        channel_config_set_write_increment(&d, true);
        channel_config_set_dreq(&d, pio_get_dreq(audio_pio, sm, false));
        ownership[lane][0] = FILLING;
        dma_channel_configure((uint)channels[lane], &d, samples[lane][0],
                              &audio_pio->rxf[sm], AUDIO_WORDS, false);
        dma_channel_set_irq0_enabled((uint)channels[lane], true);
    }
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_start_channel_mask((1u<<(uint)channels[0]) | (1u<<(uint)channels[1]));
    audio_pio->fdebug = (1u<<1)|(1u<<2); /* clear RXSTALL */
    pio_enable_sm_mask_in_sync(audio_pio, 7u);
}

static bool audio_input_acquire(audio_block_t *block) {
    uint32_t saved = save_and_disable_interrupts();
    unsigned found[2] = {AUDIO_BUFFERS, AUDIO_BUFFERS};
    for (unsigned lane = 0; lane < 2; ++lane)
        for (unsigned slot = 0; slot < AUDIO_BUFFERS; ++slot)
            if (ownership[lane][slot] == READY && sequences[lane][slot] == consumed)
                found[lane] = slot;
    bool available = !failed && found[0] < AUDIO_BUFFERS && found[1] < AUDIO_BUFFERS;
    if (available) {
        block->slot_a = found[0]; block->slot_b = found[1];
        block->a = samples[0][found[0]]; block->b = samples[1][found[1]];
        ++consumed;
        block->completed_us = completed[0][found[0]];
        ownership[0][found[0]] = READING; ownership[1][found[1]] = READING;
    }
    restore_interrupts(saved);
    return available;
}

static void audio_input_release(const audio_block_t *block) {
    uint32_t saved = save_and_disable_interrupts();
    ownership[0][block->slot_a] = FREE;
    ownership[1][block->slot_b] = FREE;
    restore_interrupts(saved);
}

static bool audio_input_failed(void) {
    if (audio_pio->fdebug & ((1u<<1)|(1u<<2))) stop_capture();
    return failed;
}

/* USB writes can block, so core 1 consumes copies from a bounded queue. */
typedef struct { int16_t pcm[AUDIO_FRAMES]; } output_block_t;
static queue_t usb_queue;
static frost_t beamformer;

/* Inspect these counters with a debugger while profiling on hardware. */
static volatile uint32_t process_us, latency_us, deadline_misses, usb_drops;

static void usb_worker(void) {
    output_block_t output;
    while (true) {
        if (!stdio_usb_connected()) { sleep_ms(1); continue; }
        queue_remove_blocking(&usb_queue, &output);
        fwrite(output.pcm, sizeof(int16_t), AUDIO_FRAMES, stdout);
        fflush(stdout);
    }
}

int main(void) {
    /* SDK initialization stays on core 0, with its default alarm pool. */
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);
    gpio_init(AUDIO_TIMING_PIN);
    gpio_set_dir(AUDIO_TIMING_PIN, GPIO_OUT);
    if (!frost_init(&beamformer)) panic("Constraint inverse failed");
    queue_init(&usb_queue, sizeof(output_block_t), USB_QUEUE_BLOCKS);
    multicore_launch_core1(usb_worker);
    audio_input_init();

    while (true) {
        if (audio_input_failed()) panic("Audio capture overrun");
        audio_block_t block;
        if (!audio_input_acquire(&block)) { tight_loop_contents(); continue; }
        output_block_t output;
        uint32_t start = time_us_32();
        gpio_put(AUDIO_TIMING_PIN, 1);
        for (unsigned i = 0; i < AUDIO_FRAMES; ++i) {
            /* I2S words start at the MSB; retain the top 16 PCM bits. */
            int16_t mics[4] = {
                (int16_t)(block.a[2*i] >> 16), (int16_t)(block.a[2*i+1] >> 16),
                (int16_t)(block.b[2*i] >> 16), (int16_t)(block.b[2*i+1] >> 16)};
            output.pcm[i] = frost_process(&beamformer, mics, true);
        }
        gpio_put(AUDIO_TIMING_PIN, 0);
        process_us = time_us_32()-start;
        /* Device-side estimate; excludes conversion, USB and playback delay. */
        latency_us = time_us_32()-block.completed_us +
                     AUDIO_FRAMES*1000000u/FROST_SAMPLE_RATE;
        if (process_us >= AUDIO_FRAMES*1000000u/FROST_SAMPLE_RATE ||
            latency_us >= 15000u) ++deadline_misses;
        audio_input_release(&block);
        if (!queue_try_add(&usb_queue, &output)) ++usb_drops;
    }
}
