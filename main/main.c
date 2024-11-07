/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "hardware/gpio.h"

// int main() {
//     stdio_init_all();
//     while (true) {
//         printf("Hello, world!\n");
//         sleep_ms(1000);
//     }
// }


#include "pico/stdlib.h"
#include "ili9341.h"
#include "gfx.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#define READ_BIT 0x80

typedef struct {
    int32_t t_fine;
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t dig_H1, dig_H3;
    int8_t dig_H6;
    int16_t dig_H2, dig_H4, dig_H5;
} bme280_calib_data;

int32_t compensate_temp(int32_t adc_T, bme280_calib_data *calib) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t) calib->dig_T1 << 1))) * ((int32_t) calib->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t) calib->dig_T1)) * ((adc_T >> 4) - ((int32_t) calib->dig_T1))) >> 12) *
            ((int32_t) calib->dig_T3)) >> 14;

    calib->t_fine = var1 + var2;
    T = (calib->t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t compensate_pressure(int32_t adc_P, bme280_calib_data *calib) {
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t) calib->t_fine) >> 1) - (int32_t) 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t) calib->dig_P6);
    var2 = var2 + ((var1 * ((int32_t) calib->dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t) calib->dig_P4) << 16);
    var1 = (((calib->dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
            ((((int32_t) calib->dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t) calib->dig_P1)) >> 15);
    if (var1 == 0)
        return 0;

    p = (((uint32_t) (((int32_t) 1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t) var1);
    else
        p = (p / (uint32_t) var1) * 2;

    var1 = (((int32_t) calib->dig_P9) * ((int32_t) (((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t) (p >> 2)) * ((int32_t) calib->dig_P8)) >> 13;
    p = (uint32_t) ((int32_t) p + ((var1 + var2 + calib->dig_P7) >> 4));

    return p;
}

uint32_t compensate_humidity(int32_t adc_H, bme280_calib_data *calib) {
    int32_t v_x1_u32r;
    v_x1_u32r = (calib->t_fine - ((int32_t) 76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t) calib->dig_H4) << 20) - (((int32_t) calib->dig_H5) * v_x1_u32r)) +
                   ((int32_t) 16384)) >> 15) * (((((((v_x1_u32r * ((int32_t) calib->dig_H6)) >> 10) *
            (((v_x1_u32r * ((int32_t) calib->dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
                                                 ((int32_t) calib->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t) calib->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t) (v_x1_u32r >> 12);
}

void read_compensation_parameters(bme280_calib_data *calib) {
    uint8_t buffer[26];
    read_registers(0x88, buffer, 26);

    calib->dig_T1 = buffer[0] | (buffer[1] << 8);
    calib->dig_T2 = buffer[2] | (buffer[3] << 8);
    calib->dig_T3 = buffer[4] | (buffer[5] << 8);
    calib->dig_P1 = buffer[6] | (buffer[7] << 8);
    calib->dig_P2 = buffer[8] | (buffer[9] << 8);
    calib->dig_P3 = buffer[10] | (buffer[11] << 8);
    calib->dig_P4 = buffer[12] | (buffer[13] << 8);
    calib->dig_P5 = buffer[14] | (buffer[15] << 8);
    calib->dig_P6 = buffer[16] | (buffer[17] << 8);
    calib->dig_P7 = buffer[18] | (buffer[19] << 8);
    calib->dig_P8 = buffer[20] | (buffer[21] << 8);
    calib->dig_P9 = buffer[22] | (buffer[23] << 8);
    calib->dig_H1 = buffer[25];
    
    read_registers(0xE1, buffer, 8);
    calib->dig_H2 = buffer[0] | (buffer[1] << 8);
    calib->dig_H3 = buffer[2];
    calib->dig_H4 = buffer[3] << 4 | (buffer[4] & 0xf);
    calib->dig_H5 = (buffer[4] >> 4) | (buffer[5] << 4);
    calib->dig_H6 = buffer[6];
}

int main() {
    stdio_init_all();
    if (!defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || 
        !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)) {
        puts("Default SPI pins were not defined");
        return -1;
    }

    printf("Hello, bme280! Reading raw data from registers via SPI...\n");

    spi_init(spi_default, 500 * 1000);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

    uint8_t id;
    read_registers(0xD0, &id, 1);
    printf("Chip ID is 0x%x\n", id);

    bme280_calib_data calib;
    read_compensation_parameters(&calib);

    write_register(0xF2, 0x1);
    write_register(0xF4, 0x27);

    int32_t humidity, pressure, temperature;
    LCD_initDisplay();
    LCD_setRotation(1);

    while (1) {
        uint8_t buffer[8];
        read_registers(0xF7, buffer, 8);

        int32_t adc_P = buffer[0] << 12 | buffer[1] << 4 | buffer[2] >> 4;
        int32_t adc_T = buffer[3] << 12 | buffer[4] << 4 | buffer[5] >> 4;
        int32_t adc_H = buffer[6] << 8 | buffer[7];

        temperature = compensate_temp(adc_T, &calib);
        pressure = compensate_pressure(adc_P, &calib);
        humidity = compensate_humidity(adc_H, &calib);

        LCD_clearScreen();
        char bufferDisplay[20];
        snprintf(bufferDisplay, sizeof(bufferDisplay), "Temp: %.2f C", temperature / 100.0);
        LCD_print(bufferDisplay, 10, 10);

        snprintf(bufferDisplay, sizeof(bufferDisplay), "Pressure: %.2f hPa", pressure / 100.0);
        LCD_print(bufferDisplay, 10, 30);

        snprintf(bufferDisplay, sizeof(bufferDisplay), "Humidity: %.2f%%", humidity / 1024.0);
        LCD_print(bufferDisplay, 10, 50);

        sleep_ms(2000);
    }
}
