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

#define NUM_DISPLAYS 3

#define SPI_TAG "spi_protocol"

void spi_init(void);
void spi_write_data(uint8_t addr, uint8_t data);
void spi_write_multi(uint8_t addr, uint8_t *data, int num_devices);
void max7219_init(void);
void example_display(void);

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
        .max_transfer_sz = NUM_DISPLAYS * 2};

    spi_device_interface_config_t dev_cfg = {
        .mode = 0,
        .clock_speed_hz = 1000000,
        .spics_io_num = -1,
        .queue_size = 8};

    ret = spi_bus_initialize(ESP_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(ESP_HOST, &dev_cfg, &spi);
    ESP_ERROR_CHECK(ret);

    // Set CS high initially
    gpio_set_level(CS_PIN, 1);
}

void spi_write_multi(uint8_t addr, uint8_t *data, int num_devices)
{
    uint8_t tx_buf[NUM_DISPLAYS * 2];

    // Build data stream for all devices; send last device data first (MSB shifted out first)
    for (int i = 0; i < num_devices; i++)
    {
        int dev_idx = num_devices - 1 - i; // reverse order for daisy chain
        tx_buf[i * 2] = addr;
        tx_buf[i * 2 + 1] = data[dev_idx];
    }

    spi_transaction_t trans_desc = {
        .length = 16 * num_devices, // total bits to send
        .tx_buffer = tx_buf,
    };

    gpio_set_level(CS_PIN, 0); // CS low to start SPI transaction

    ret = spi_device_polling_transmit(spi, &trans_desc);
    if (ret != ESP_OK)
    {
        ESP_LOGE(SPI_TAG, "SPI multi write failed");
    }

    gpio_set_level(CS_PIN, 1); // CS high to end SPI transaction
}

void max7219_init(void)
{
    // Initialization commands for MAX7219
    uint8_t init_cmds[] = {0x09, 0x0A, 0x0B, 0x0C, 0x0F};
    uint8_t init_vals[] = {0x00, 0x07, 0x07, 0x01, 0x00};

    for (int i = 0; i < sizeof(init_cmds); i++)
    {
        uint8_t data[NUM_DISPLAYS];
        for (int d = 0; d < NUM_DISPLAYS; d++)
        {
            data[d] = init_vals[i];
        }
        spi_write_multi(init_cmds[i], data, NUM_DISPLAYS);
        vTaskDelay(pdMS_TO_TICKS(10)); // small delay after each command
    }
}

void straight_line_display(void)
{
    uint8_t data[NUM_DISPLAYS] = {0b00000001, 0b00000010, 0b00000100};
    for (uint8_t row = 1; row < 9; row++)
    {
        spi_write_multi(row, data, NUM_DISPLAYS);

        printf("Row %02d data for all displays: ", row);
        for (int i = 0; i < NUM_DISPLAYS; i++)
        {
            printf("0x%02X ", data[i]);
        }
        printf("\n");

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void diagonal_line_display(void)
{
    uint8_t diagonal_line[8] = {
        0b00000001,
        0b00000010,
        0b00000100,
        0b00001000,
        0b00010000,
        0b00100000,
        0b01000000,
        0b10000000};

    uint8_t data[NUM_DISPLAYS]; // data to send for all displays

    for (uint8_t row = 1; row <= 8; row++)
    {
        // Fill the data array with the pattern for each display
        for (int d = 0; d < NUM_DISPLAYS; d++)
        {
            data[d] = diagonal_line[row - 1]; // same pattern on all displays
        }

        spi_write_multi(row, data, NUM_DISPLAYS);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

uint8_t reverse_bits(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

void inv_diagonal_line_disp(void)
{

    uint8_t diagonal_line[8] = {
        0b00000001,
        0b00000010,
        0b00000100,
        0b00001000,
        0b00010000,
        0b00100000,
        0b01000000,
        0b10000000};

    uint8_t data[NUM_DISPLAYS];

    for (uint8_t row = 1; row <= 8; row++)
    {
        for (int d = 0; d < NUM_DISPLAYS; d++)
        {
            if (d == 1)
            {
                // For display 2 (index 1), flip the bits
                data[d] = reverse_bits(diagonal_line[row - 1]);
            }
            else
            {
                data[d] = diagonal_line[row - 1];
            }
        }

        spi_write_multi(row, data, NUM_DISPLAYS);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

int app_main(void)
{
    spi_init();
    max7219_init();

    inv_diagonal_line_disp();

    return 0;
}