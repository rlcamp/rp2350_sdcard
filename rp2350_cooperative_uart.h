#include <stddef.h>

extern unsigned cooperative_uart_current_baudrate;

void uart_write_with_yield(const void * bytes, const size_t count);

void uart_tx_wait_blocking_with_yield(void);

const char * get_line_from_uart(void);

void cooperative_uart_init(void);

int write(int fd, void * bytes, int len);
