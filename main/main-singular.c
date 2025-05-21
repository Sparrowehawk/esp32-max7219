#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define ESP_HOST VSPI_HOST
#define MOSI_PIN GPIO_NUM_23
#define SCLK_PIN GPIO_NUM_18
#define CS_PIN GPIO_NUM_5

#define SPI_TAG "spi_protocol"

void spi_init(void);
void spi_write_data(uint8_t addr, uint8_t data);
uint8_t data_transpose(uint8_t input);
void max7219_init(void);

esp_err_t ret;
spi_device_handle_t spi;

void spi_init(void)
{
    gpio_set_direction(CS_PIN, GPIO_MODE_OUTPUT);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = -1,
        .sclk_io_num = SCLK_PIN,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .max_transfer_sz = 32};

    spi_device_interface_config_t dev_cfg = {
        .mode = 0,
        .clock_speed_hz = 1000000,
        .spics_io_num = -1,
        .queue_size = 8};

    ret = spi_bus_initialize(ESP_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(ESP_HOST, &dev_cfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void spi_write_data(uint8_t addr, uint8_t data)
{
    spi_transaction_t trans_desc = {
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {addr, data},
        .length = 16,
    };

    gpio_set_level(CS_PIN, 0);

    printf("Writing '%x' data at %x\n", data, addr);
    ret = spi_device_polling_transmit(spi, &trans_desc);
    if (ret != ESP_OK)
    {
        ESP_LOGE(SPI_TAG, "SPI write operation failed\n");
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(CS_PIN, 1);
    printf("Writing operation complete\n");
}

void max7219_init(void)
{

    spi_write_data(0x09, 0x00);

    // Turn brightness to 50%
    uint8_t addr = 0x0A;
    uint8_t data = 0x07;
    spi_write_data(addr, data);

    // turn on
    addr = 0x0C;
    data = 0x01;
    spi_write_data(addr, data);

    // test mode, data = 1
    // data mode, data = 0
    addr = 0x0F;
    data = 0x00;
    spi_write_data(addr, data);

    addr = 0x0B;
    data = 0x07;
    spi_write_data(addr, data);
}

// If VCC, GND, Din, SCLK and CS is plugged directly into your breadboard, you are in col ordering
// Therefore, address 0x08 displaying 00010000 will display the 4th LED down from the rightmost column

void max7219_diagonal(void)
{
    // Binary works better to display data as each 1 corresponds to a LED
    uint8_t addr = 0x01;
    uint8_t data = 0b00000001;
    spi_write_data(addr, data);

    addr = 0x02;
    data = 0b00000010;
    spi_write_data(addr, data);

    addr = 0x03;
    data = 0b00000100;
    spi_write_data(addr, data);

    addr = 0x04;
    data = 0b00001000;
    spi_write_data(addr, data);

    addr = 0x05;
    data = 0b00010000;
    spi_write_data(addr, data);

    addr = 0x06;
    data = 0b00100000;
    spi_write_data(addr, data);

    addr = 0x07;
    data = 0b01000000;
    spi_write_data(addr, data);

    addr = 0x08;
    data = 0b10000000;
    spi_write_data(addr, data);
}

int app_main(void)
{
    spi_init();
    max7219_init();

    max7219_diagonal();

    return 0;
}