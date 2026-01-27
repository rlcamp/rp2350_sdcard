/* proof of concept data logger to test sd card code */
#ifndef PROGNAME
#define PROGNAME "pico_sdcard"
#endif

/* lower level pico-sdk includes */
#include "hardware/xosc.h"
#include "hardware/structs/rosc.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

/* so that we can reset into bootloader on command */
#include "pico/bootrom.h"

/* for SEV and WFE */
#include "RP2350.h"

/* other open source or project-agnostic includes by authors */
#include "rp2350_sdcard.h"
#include "cooperative_fatfs.h"
#include "rp2350_cooperative_uart.h"

/* third party includes */
#include "ff.h"

/* c standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "version.h"

extern void yield(void);

volatile unsigned char verbose = 0;

__attribute((weak))
uint32_t get_fattime(void) {
    /* dummy fattime function, replace with something filled by an rtc */
    struct tm out = { .tm_year = 2025 };
    return ((out.tm_year - 80) << 25U |
            (out.tm_mon + 1) << 21U |
            out.tm_mday << 16U |
            out.tm_hour << 11U |
            out.tm_min << 5U |
            out.tm_sec >> 1U);
}

static int fputs_to_open_file(FIL * fp, const char * string) {
    const size_t len = strlen(string);
    FRESULT fres;
    UINT write_count;
    if ((fres = f_write(fp, string, len, &write_count))) {
        dprintf(2, "error: %s: f_write(): %d\r\n", __func__, fres);
        return -1;
    }
    if (write_count < len) {
        dprintf(2, "error: %s: short write\r\n", __func__);
        return -1;
    }
    return 0;
}

void run_from_xosc(void) {
    clock_configure_undivided(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, XOSC_MHZ * MHZ);
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF, 0, XOSC_MHZ * MHZ);

    clock_stop(clk_usb);
    clock_stop(clk_adc);
    clock_stop(clk_hstx);

    clock_configure_undivided(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, XOSC_MHZ * MHZ);

    /* disable PLLs */
    pll_deinit(pll_sys);
    pll_deinit(pll_usb);

    /* disable rosc and wait for it to be stopped */
    rosc_hw->ctrl = (rosc_hw->ctrl & ~ROSC_CTRL_ENABLE_BITS) | (ROSC_CTRL_ENABLE_VALUE_DISABLE << ROSC_CTRL_ENABLE_LSB);
    while (rosc_hw->status & ROSC_STATUS_STABLE_BITS);
}

void lower_power_sleep_ms(const unsigned delay_ms) {
    /* get a timer, enable interrupt for alarm, but leave it disabled in nvic */
    const unsigned alarm_num = timer_hardware_alarm_claim_unused(timer_hw, true);
    hw_set_bits(&timer_hw->inte, 1U << alarm_num);
    irq_set_enabled(hardware_alarm_get_irq_num(alarm_num), false);

    /* arm timer */
    timer_hw->alarm[alarm_num] = timer_hw->timerawl + delay_ms * 1000;

    /* run other tasks or low power sleep until alarm interrupt */
    while (!(timer_hw->intr & (1U << alarm_num)))
        yield();

    /* acknowledge and clear the interrupt in both timer and nvic */
    hw_clear_bits(&timer_hw->intr, 1U << alarm_num);
    irq_clear(hardware_alarm_get_irq_num(alarm_num));

    /* cleanup */
    hw_clear_bits(&timer_hw->inte, 1U << alarm_num);
    timer_hardware_alarm_unclaim(timer_hw, alarm_num);
}

/* allow an enable line to be shared between peripherals */
static volatile unsigned char enable_line_users = 0;

void enable_line_request(void) {
    /* cooperative lock is necessary so that we can yield within the init path */
    static volatile unsigned char locked = 0;
    while (locked) yield();
    locked = 1;

    if (!(enable_line_users++)) {
        gpio_init(22);
        gpio_set_dir(22, GPIO_OUT);
        gpio_put(22, 1);

        /* wait for inrush current, must also be > 1 ms for sdmmc */
        lower_power_sleep_ms(50);
    }

    locked = 0;
    __SEV();
}

void enable_line_release(void) {
    if (!(--enable_line_users)) {
        gpio_put(22, 0);
        gpio_deinit(22);
    }
}

static int touch(const char * path) {
    if (-1 == card_request()) return -1;

    do {
        /* large thing on stack */
        FIL * fp = &(FIL) { };
        FRESULT fres;

        if ((fres = f_open(fp, path, FA_CREATE_ALWAYS | FA_WRITE))) {
            dprintf(2, "%s: f_open(\"%s\"): %d\r\n", __func__, path, fres);
            break;
        }

        /* this will usually return immediately, occasionally it will internally yield() */
        if (-1 == fputs_to_open_file(fp, "hello\n")) break;

        if ((fres = f_close(fp))) {
            dprintf(2, "%s: f_close(\"%s\"): %d\r\n", __func__, path, fres);
            break;
        }

        dprintf(2, "%s: successfully wrote to %s\r\n", __func__, path);

        card_release();
        return 0;
    } while(0);

    card_release();
    return -1;
}

int main(void) {
    run_from_xosc();

    scb_hw->scr |= M33_SCR_SLEEPDEEP_BITS;

    /* TODO: figure out why power consumption is much higher when not recording */

    /* turn off clocks to a bunch of stuff we aren't (yet) using */
    clocks_hw->wake_en1 = (CLOCKS_WAKE_EN1_BITS &
                           ~(CLOCKS_WAKE_EN1_CLK_USB_BITS|
                             CLOCKS_WAKE_EN1_CLK_SYS_USBCTRL_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_UART1_BITS |
                             CLOCKS_WAKE_EN1_CLK_PERI_UART1_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_TRNG_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_SPI1_BITS |
                             CLOCKS_WAKE_EN1_CLK_PERI_SPI1_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_SPI0_BITS |
                             CLOCKS_WAKE_EN1_CLK_PERI_SPI0_BITS));

    clocks_hw->wake_en0 = (CLOCKS_WAKE_EN0_BITS &
                           ~(CLOCKS_WAKE_EN0_CLK_SYS_SHA256_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PWM_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PLL_USB_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PLL_SYS_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PIO2_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PIO1_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PIO0_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_JTAG_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_I2C1_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_I2C0_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_HSTX_BITS |
                             CLOCKS_WAKE_EN0_CLK_HSTX_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_ADC_BITS |
                             CLOCKS_WAKE_EN0_CLK_ADC_BITS));

    /* make sure we don't clock anything in sleep that wasn't clocked in wake */
    clocks_hw->sleep_en1 = clocks_hw->wake_en1;
    clocks_hw->sleep_en0 = clocks_hw->wake_en0;

    /* enable sevonpend so that we don't need a nearly empty isr */
    scb_hw->scr |= M33_SCR_SEVONPEND_BITS;

    cooperative_uart_init();

    dprintf(2, "\r\n%s: built from %s\r\n", PROGNAME, GIT_VERSION);

    /* loop on characters from uart */
    while (1) {
        const char * line = get_line_from_uart();

        /* if we got a complete line... */
        if (line) {
            const unsigned long long uptime_now = timer_time_us_64(timer_hw);
            dprintf(2, "%% %s\r\n", line);

            if (line == strstr(line, "ls "))
                ls(line + 3);
            else if (!strcmp(line, "ls"))
                ls(NULL);
            else if (line == strstr(line, "cat "))
                cat(line + 4);
            else if (line == strstr(line, "touch "))
                touch(line + 6);

            else if (!strcmp(line, "flash")) {
                dprintf(2, "%s: resetting into bootloader\r\n", PROGNAME);
                uart_tx_wait_blocking_with_yield();
                rom_reset_usb_boot_extra(-1, 0, false);
            }

            else if (!strcmp(line, "uptime"))
                dprintf(2, "%s: uptime %lu\r\n", PROGNAME, (unsigned long)(uptime_now / 1000000ULL));

            else if (line == strstr(line, "verbose "))
                verbose = strtoul(line + 8, NULL, 10);
        }

        yield();
    }
}
