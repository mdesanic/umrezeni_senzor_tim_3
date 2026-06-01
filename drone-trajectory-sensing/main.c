/*******************************************************************************
* File Name: main.c
*
* Description:
*   UART primjer bez DMA-a.
*
* PuTTY setup:
*   Connection type: Serial
*   Serial line: COMx
*   Speed: 115200
*
*   Connection -> Serial:
*   Data bits: 8
*   Stop bits: 1
*   Parity: None
*   Flow control: None
*
* Napomena:
*   COMx se provjeri u Windows Device Manageru:
*   Device Manager -> Ports (COM & LPT)
*******************************************************************************/

#if defined(CY_USING_HAL)
#include "cyhal.h"
#endif

#include "cybsp.h"
#include <stdint.h>
#include <stddef.h>

cyhal_uart_t uart_obj;

/* UART init: 115200 baud, 8N1 */
static cy_rslt_t uart_init(void)
{
    cyhal_uart_cfg_t uart_cfg = {
        .data_bits      = 8,
        .stop_bits      = 1,
        .parity         = CYHAL_UART_PARITY_NONE,
        .rx_buffer      = NULL,
        .rx_buffer_size = 0,
    };

    cy_rslt_t result = cyhal_uart_init(&uart_obj,
                                       CYBSP_DEBUG_UART_TX,
                                       CYBSP_DEBUG_UART_RX,
                                       NC,
                                       NC,
                                       NULL,
                                       &uart_cfg);

    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    return cyhal_uart_set_baud(&uart_obj, 115200, NULL);
}

/* Slanje raw podataka */
static cy_rslt_t uart_send_data(uint8_t *data, size_t length)
{
    return cyhal_uart_write(&uart_obj, data, &length);
}

/* Slanje teksta */
static cy_rslt_t uart_send_text(const char *text)
{
    size_t length = 0;

    while (text[length] != '\0')
    {
        length++;
    }

    return cyhal_uart_write(&uart_obj, (void *)text, &length);
}

int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    result = uart_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    uart_send_text("\r\nUART started without DMA.\r\n");

    uint32_t counter = 0;

    for (;;)
    {
        /*
         * Primjer 1: slanje običnog teksta u PuTTY.
         * Kolege ovdje mogu ubaciti svoje vrijednosti/senzorske podatke.
         */
        uart_send_text("Hello from Infineon board\r\n");

        /*
         * Primjer 2: slanje raw buffera.
         * Ovo neće uvijek izgledati čitljivo u PuTTY-u jer nisu svi bajtovi ASCII znakovi.
         */
        uint8_t data[] = {
            'D', 'A', 'T', 'A', ':', ' ',
            '1', '2', '3', '4',
            '\r', '\n'
        };

        uart_send_data(data, sizeof(data));

        /*
         * Primjer 3: vrlo jednostavan brojač bez sprintf-a.
         * Za pravi formatted output može se kasnije dodati sprintf/printf.
         */
        uart_send_text("Loop counter tick\r\n");

        counter++;

        /*
         * Pauza da se poruke ne šalju prebrzo.
         * 1000 ms = 1 sekunda
         */
        cyhal_system_delay_ms(1000);
    }
}

/* [] END OF FILE */