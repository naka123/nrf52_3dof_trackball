/*
Copyright 2019 Sekigon

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "mouse_sensor"
#include "nrf_nvic.h"


#define READ(addr) (addr)
#define WRITE(addr) (0x80 | addr)

#define REG_PID1 0x00
#define REG_PID2 0x01
#define REG_STAT 0x02
#define REG_X 0x03
#define REG_Y 0x04
#define REG_CONFIG 0x06
#define REG_IMG_QUALITY 0x07

typedef int (*spi_paw3204_t)(const dev_pins pins, uint8_t *p_tx_buffer, size_t tx_length, uint8_t *p_rx_buffer, size_t rx_length, uint8_t cs_pin);

int spi_soft_half_duplex(const dev_pins pins, uint8_t *p_tx_buffer, size_t tx_length, uint8_t *p_rx_buffer, size_t rx_length, uint8_t cs_pin) {
    if (tx_length != 2 || rx_length != 2) {
        p_rx_buffer[1] = 0xFF;
        return 1;
    }

    taskENTER_CRITICAL();
    uint8_t is_nested;
    sd_nvic_critical_region_enter(&is_nested);

    writePin(pins.pin_sdio, readPin(pins.pin_sdio));
    setPinOutput(pins.pin_sdio);

    for (int8_t idx = 7; idx >= 0; idx--) {
        writePinLow(pins.pin_sclk);
        writePin(pins.pin_sdio, (p_tx_buffer[0] >> idx) & 1);
        writePinHigh(pins.pin_sclk);
    }

    _delay_us(5);
    setPinInputHigh(pins.pin_sdio);

    p_rx_buffer[1] = 0;
    for (int8_t idx = 7; idx >= 0; idx--) {
        writePinLow(pins.pin_sclk);
        writePinHigh(pins.pin_sclk);
        p_rx_buffer[1] |= readPin(pins.pin_sdio) << idx;
    }

    sd_nvic_critical_region_exit(is_nested);
    taskEXIT_CRITICAL();

    return 0;
}

// spi_paw3204_t spi_paw3204 = spim_start;
spi_paw3204_t spi_paw3204 = spi_soft_half_duplex;

uint8_t read_pid_paw3204(const dev_pins pins) {
    uint8_t snd[] = {READ(REG_PID1), 0xFF};
    uint8_t rcv[] = {0xFF, 0xFF};

    spi_paw3204(pins, snd, sizeof(snd), rcv, sizeof(rcv), 0xFF);

    return rcv[1];
}

// set IO pins
void init_paw3204(const dev_pins pins) {
    setPinOutput(pins.pin_sclk);
    setPinInputHigh(pins.pin_sdio);
}


void set_dpi_paw3204(const dev_pins pins, uint8_t bits) {
    uint8_t snd[] = {WRITE(REG_CONFIG), 0b00000000 | (bits & 0b11)};
    uint8_t rcv[] = {0xFF, 0xFF};

    spi_paw3204(pins, snd, sizeof(snd), rcv, sizeof(rcv), 0xFF);
}

int read_paw3204_status(const dev_pins pins, uint8_t *stat) {
    {
        uint8_t snd[] = {READ(REG_STAT), 0xFF};
        uint8_t rcv[] = {0xFF, 0xFF};

        spi_paw3204(pins, snd, sizeof(snd), rcv, sizeof(rcv), 0xFF);
        *stat = rcv[1];
    }
    return 1;
}

int read_paw3204_data(const dev_pins pins, uint8_t *qua, int8_t *x, int8_t *y) {
    {
        uint8_t snd[] = {READ(REG_IMG_QUALITY), 0xFF};
        uint8_t rcv[] = {0xFF, 0xFF};

        spi_paw3204(pins, snd, sizeof(snd), rcv, sizeof(rcv), 0xFF);
        *qua = rcv[1];
    }
    {
        uint8_t snd[] = {READ(REG_X), 0xFF};
        uint8_t rcv[] = {0xFF, 0xFF};

        spi_paw3204(pins, snd, sizeof(snd), rcv, sizeof(rcv), 0xFF);
        *x = *((int8_t *)(rcv + 1));
    }
    {
        uint8_t snd[] = {READ(REG_Y), 0xFF};
        uint8_t rcv[] = {0xFF, 0xFF};

        spi_paw3204(pins, snd, sizeof(snd), rcv, sizeof(rcv), 0xFF);
        *y = *((int8_t *)(rcv + 1));
    }
    return 1;
}

void read_all_paw3204(const dev_pins pins, paw3204_all_reg *dat) {
    for (uint8_t idx = 0; idx < sizeof(paw3204_all_reg); idx++) {
        uint8_t snd[] = {READ(idx), 0xFF};
        uint8_t rcv[] = {0xFF, 0xFF};

        spi_paw3204(pins, snd, sizeof(snd), rcv, sizeof(rcv), 0xFF);
        dat->reg[idx] = rcv[1];
    }
}
