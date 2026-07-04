#include "rp2350_cooperative_uart.h"

#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "RP2350.h"

/* use the same macro name that pico stdio would use */
#ifndef PICO_DEFAULT_UART_BAUD_RATE
#define PICO_DEFAULT_UART_BAUD_RATE 115200
#endif

unsigned cooperative_uart_current_baudrate = PICO_DEFAULT_UART_BAUD_RATE;

__attribute((weak)) void yield(void) { }
__attribute((weak)) void * current_task(void) { return NULL; }

/* ring buffer that holds individual bytes between interrupt and main thread. note the size
 of this buffer should probably be a function of baud rate and max expected delay between
 calls to uart0_next_byte() from the main thread */
static unsigned char rx_ring[128];
static size_t rx_ring_filled = 0, rx_ring_drained = 0;

static unsigned char tx_ring[128];
static size_t tx_ring_filled = 0, tx_ring_drained = 0;

void UART0_IRQ_Handler(void) {
    /* if we can move bytes into the tx fifo, move as many as we can */
    while (!(uart0_hw->fr & UART_UARTFR_TXFF_BITS) && tx_ring_filled != tx_ring_drained)
        uart0_hw->dr = tx_ring[tx_ring_drained++ % sizeof(tx_ring)];

    /* unconditionally clear the fifo-became-empty condition */
    uart0_hw->icr = 1U << UART_UARTICR_TXIC_LSB;

    /* we might have also gotten here because there are bytes in the rx fifo */
    while (!(uart0_hw->fr & UART_UARTFR_RXFE_BITS)) {
        /* consume the byte from the uart peripheral */
        const unsigned char byte = uart0_hw->dr;

        rx_ring[rx_ring_filled++ % sizeof(rx_ring)] = byte;
    }
}

void uart_write_with_yield(const void * bytes, const size_t count) {
    const unsigned char * cursor = bytes, * stop = cursor + count;
    hw_set_bits(&uart0_hw->imsc, 1U << UART_UARTIMSC_TXIM_LSB);
    while (cursor != stop) {
        __DSB();
        const size_t slots_available = sizeof(tx_ring) - (tx_ring_filled - *(volatile size_t *)&tx_ring_drained);
        if (!slots_available) {
            yield();
            continue;
        }

        const size_t count_remaining = stop - cursor;
        const size_t bytes_to_send_now = count_remaining < slots_available ? count_remaining : slots_available;

        /* copy a bunch of bytes into the ring buffer before enabling the interrupt */
        for (size_t ibyte = 0; ibyte < bytes_to_send_now; ibyte++)
            tx_ring[tx_ring_filled++ % sizeof(tx_ring)] = cursor[ibyte];

        cursor += bytes_to_send_now;
        __DSB();
        irq_set_pending(UART_IRQ_NUM(uart0));
    }
}

void uart_tx_wait_blocking_with_yield(void) {
    /* block in busy yield until all bytes in the tx fifo have been transmitted */
    while (uart0_hw->fr & UART_UARTFR_BUSY_BITS) {
        /* since we are not waiting for an interrupt-accompanied condition, we must
         inhibit the wfe call within yield, but can still allow yield to do other things */
        __sev();
        yield();
    }
}

/* silence compiler warning about no previous prototype */
extern int _write(int fd, void * bytes, int len);

/* write() is aliased to this, higher level functions in the printf family invoke this */
int _write(int fd, void * bytes, int len) {
    (void)fd; /* just send stderr and stdout to same ep */

    /* writing to the uart acquires a lock that is only released by writing "\n", such that
     whole lines of text will be emitted atomically even when multiple tasks are emitting
     them, even if they are doing so using multiple calls to this function. tasks waiting
     for the lock will be looping on yield(), a function that round robins the tasks and
     calls __WFE() once per loop over all tasks */
    static volatile uintptr_t task_holding_lock = 0;

    /* get a unique and nonzero identifier for the current task */
    const uintptr_t me = (uintptr_t)current_task() + 1U;

    /* if we do not own the lock, it is either not locked or locked by another thread. the
     fast path is that we already own the lock from a previous call */
    if (task_holding_lock != me) {
        while (task_holding_lock) yield();
        task_holding_lock = me;
    }

    /* this can also internally call yield */
    uart_write_with_yield(bytes, len);

    if (((char *)bytes)[len - 1] == '\n') {
        /* release the lock */
        task_holding_lock = 0;

        /* since we are changing a condition that other tasks might be waiting on, outside
         of an interrupt context, we need to make sure they get a chance to see it before
         the processor actually sleeps again */
        __SEV();
    }
    return len;
}

/* do an end run around newlib _write_r */
__attribute__ ((alias("_write"))) int write(int fd, void * bytes, int len);


/* maintains line buffer, returns either NULL or a complete line */
const char * get_line_from_uart(void) {
    static char linebuf[83] = { 0 };
    static size_t ilinebuf = 0;

    __DSB();
    /* if any new bytes from uart, and not already a complete line... */
    for (size_t filled_now; rx_ring_drained != (filled_now = *(volatile size_t *)&rx_ring_filled);) {
        /* if reader fell behind, fast forward over the missed bytes and set lockout */
        if (filled_now - rx_ring_drained > sizeof(rx_ring)) {
            rx_ring_drained = filled_now - sizeof(rx_ring);
            ilinebuf = sizeof(linebuf);
            continue;
        }

        /* consume a byte from the upstream ring buffer filled by the interrupt handler */
        const unsigned char byte = rx_ring[rx_ring_drained++ % sizeof(rx_ring)];

        /* if we are in lockout, we ignore this byte, and may drop out of lockout */
        if (ilinebuf == sizeof(linebuf)) {
            if ('\r' == byte || '\n' == byte) ilinebuf = 0;
            continue;
        }

        linebuf[ilinebuf++] = byte;

        /* we treat either line ending identically, and ignore duplicates/empty lines */
        if ('\r' == byte || '\n' == byte) {
            /* overwrite whichever line ending we got with a zero termination */
            linebuf[ilinebuf - 1] = '\0';

            /* reset cursor */
            ilinebuf = 0;

            /* if the line ending in this newline was nonempty, return it */
            if (linebuf[0] != '\0') return linebuf;
        }
    }

    return NULL;
}

void cooperative_uart_init(void) {
    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));
    gpio_pull_up(1);

    uart_init(uart0, cooperative_uart_current_baudrate);
    irq_set_enabled(UART_IRQ_NUM(uart0), true);

    /* wake up when either fifo reaches 1/8 full, or when fifo is nonempty and a timeout elapses */
    hw_set_bits(&uart0_hw->imsc, 1U << UART_UARTIMSC_RXIM_LSB | 1U << UART_UARTIMSC_RTIM_LSB);
    hw_write_masked(&uart0_hw->ifls, 0 << UART_UARTIFLS_RXIFLSEL_LSB, 0);
}
