#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"

#if LOGIC_ANALYZER
#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"

static inline uint bits_packed_per_word(uint pin_count) {
    // If the number of pins to be sampled divides the shift register size, we
    // can use the full SR and FIFO width, and push when the input shift count
    // exactly reaches 32. If not, we have to push earlier, so we use the FIFO
    // a little less efficiently.
    const uint SHIFT_REG_WIDTH = 32;
    return SHIFT_REG_WIDTH - (SHIFT_REG_WIDTH % pin_count);
}

void la_init(PIO pio, uint sm, uint pin_base, uint pin_count, float div) {
    // Load a program to capture n pins. This is just a single `in pins, n`
    // instruction with a wrap.
    uint16_t capture_prog_instr = pio_encode_in(pio_pins, pin_count);
    struct pio_program capture_prog = {
            .instructions = &capture_prog_instr,
            .length = 1,
            .origin = -1
    };
    uint offset = pio_add_program(pio, &capture_prog);

    // Configure state machine to loop over this `in` instruction forever,
    // with autopush enabled.
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, pin_base);
    sm_config_set_wrap(&c, offset, offset);
    sm_config_set_clkdiv(&c, div);
    // Note that we may push at a < 32 bit threshold if pin_count does not
    // divide 32. We are using shift-to-right, so the sample data ends up
    // left-justified in the FIFO in this case, with some zeroes at the LSBs.
    sm_config_set_in_shift(&c, true, true, bits_packed_per_word(pin_count));
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c);
}

void la_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words,
                        uint trigger_pin, bool trigger_level) {
    pio_sm_set_enabled(pio, sm, false);
    // Need to clear _input shift counter_, as well as FIFO, because there may be
    // partial ISR contents left over from a previous run. sm_restart does this.
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(dma_chan, &c,
        capture_buf,        // Destination pointer
        &pio->rxf[sm],      // Source pointer
        capture_size_words, // Number of transfers
        true                // Start immediately
    );

    pio_sm_exec(pio, sm, pio_encode_wait_gpio(trigger_level, trigger_pin));
    pio_sm_set_enabled(pio, sm, true);
}

void la_reset(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words) {
    pio_sm_set_enabled(pio, sm, false);
    // Need to clear _input shift counter_, as well as FIFO, because there may be
    // partial ISR contents left over from a previous run. sm_restart does this.
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(dma_chan, &c,
        capture_buf,        // Destination pointer
        &pio->rxf[sm],      // Source pointer
        capture_size_words, // Number of transfers
        true                // Start immediately
    );

    pio_sm_set_enabled(pio, sm, true);
}

void la_print_capture_buf(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples) {
    // Display the capture buffer in text form, like this:
    // 00: __--__--__--__--__--__--
    // 01: ____----____----____----
    printf("Capture:\n");
    // Each FIFO record may be only partially filled with bits, depending on
    // whether pin_count is a factor of 32.
    uint record_size_bits = bits_packed_per_word(pin_count);
    for (int pin = 0; pin < pin_count; ++pin) {
        printf("%02d: ", pin + pin_base);
        for (int sample = 0; sample < n_samples; ++sample) {
            uint bit_index = pin + sample * pin_count;
            uint word_index = bit_index / record_size_bits;
            // Data is left-justified in each FIFO entry, hence the (32 - record_size_bits) offset
            uint word_mask = 1u << (bit_index % record_size_bits + 32 - record_size_bits);
            printf(buf[word_index] & word_mask ? "-" : "_");
        }
        printf("\n");
    }
}

void la_print_vertical(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples) {
    for (int pin = 0; pin < pin_count; ++pin) {
        uint bit_index = pin + pin_base;
        printf("[%2d]", bit_index);
    }
    printf("\n");

    // Each FIFO record may be only partially filled with bits, depending on
    // whether pin_count is a factor of 32.
    uint record_size_bits = bits_packed_per_word(pin_count);
    for (int sample = 0; sample < n_samples; ++sample) {
        for (int pin = 0; pin < pin_count; ++pin) {
            //printf("%02d: ", pin + pin_base);
            uint bit_index = pin + sample * pin_count;
            uint word_index = bit_index / record_size_bits;
            // Data is left-justified in each FIFO entry, hence the (32 - record_size_bits) offset
            uint word_mask = 1u << (bit_index % record_size_bits + 32 - record_size_bits);
            printf(buf[word_index] & word_mask ? "[ |]" : "[| ]");
        }
        printf("\n");
    }
}

uint la_calc_buf_size_words(uint samples, uint pins) {
    // We're going to capture into a u32 buffer, for best DMA efficiency. Need
    // to be careful of rounding in case the number of pins being sampled
    // isn't a power of 2.
    uint total_sample_bits = samples * pins;
    total_sample_bits += bits_packed_per_word(pins) - 1;
    uint buf_size_words = total_sample_bits / bits_packed_per_word(pins);

    return buf_size_words;
}
#endif

void do_tests();

int main() {
    stdio_init_all();

    // Set up the gpclk generator
    clocks_hw->clk[clk_gpout0].ctrl = (CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS << CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_LSB) |
                                 CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS;
    clocks_hw->clk[clk_gpout0].div = 100 << CLOCKS_CLK_GPOUT0_DIV_INT_LSB;
    // Set gpio pin to gpclock function
    gpio_set_function(21, GPIO_FUNC_GPCK);

    // wait for USB connection:
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }

    puts("sys: set 266Mhz...");
    set_sys_clock_khz(266000, true);

    do_tests();

    printf("done\n");
    stdio_flush();
    sleep_ms(250);

    // wait for keypress and then reboot:
    getchar();
    reset_usb_boot(0, 0);

    return 0;
}

void do_tests() {
    // generate clock pulse on GPIO 28 line at 3.58MHz (NTSC):
    gpio_set_function(28, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(28);
    pwm_set_wrap(slice_num, 1);
    pwm_set_gpio_level(28, 1);
    pwm_set_clkdiv(slice_num, 266.f / 3.58f * 0.5f);
    pwm_set_enabled(slice_num, true);

#if LOGIC_ANALYZER
#define CAPTURE_N_SAMPLES 256
#define CAPTURE_PIN_COUNT 32
#define CAPTURE_PIN_BASE 0

    uint      la_buf_size_words = la_calc_buf_size_words(CAPTURE_N_SAMPLES, CAPTURE_PIN_COUNT);
    uint32_t *la_capture_buf = malloc(la_buf_size_words * sizeof(uint32_t));
    hard_assert(la_capture_buf);

    PIO  la_pio = pio1;
    uint la_sm = pio_claim_unused_sm(la_pio, true);
    uint la_dma_chan = dma_claim_unused_channel(true);

    la_init(la_pio, la_sm, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, 1.f);
#endif

    while (true) {
#if LOGIC_ANALYZER
        // la_arm(la_pio, la_sm, la_dma_chan, la_capture_buf, la_buf_size_words, PCS_B_PSRAM_CE, false);
        la_reset(la_pio, la_sm, la_dma_chan, la_capture_buf, la_buf_size_words);
#endif

#if LOGIC_ANALYZER
        dma_channel_wait_for_finish_blocking(la_dma_chan);

        // la_print_capture_buf(la_capture_buf, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CAPTURE_N_SAMPLES);
        la_print_vertical(la_capture_buf, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CAPTURE_N_SAMPLES);
#endif
    }
}
