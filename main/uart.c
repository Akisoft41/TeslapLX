// Copyright 2020 Pascal Akermann
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"


ssize_t _uart_read(void *cookie, char *buf, size_t size)
{
    return uart_read_bytes((uart_port_t)cookie, (uint8_t*)buf, size, portMAX_DELAY);
}

ssize_t _uart_write(void *cookie, const char *buf, size_t size)
{
    return uart_write_bytes((uart_port_t)cookie, buf, size);
}

static const cookie_io_functions_t  uart_cookie_func = {
    .read  = _uart_read,
    .write = _uart_write,
    .seek  = NULL,
    .close = NULL
};

FILE *uart_fopen(uart_port_t port, const char *mode)
{
    return fopencookie((void*)port, mode, uart_cookie_func);
}

