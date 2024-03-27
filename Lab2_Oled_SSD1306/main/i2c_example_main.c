/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "LOGO.h"
#include "SSD1306.h"
#include "font8x8_basic.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO GPIO_NUM_22 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_21 /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(0)  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

#define ESP_ERR (__STATUX__) if ()
SemaphoreHandle_t print_mux = NULL;
static const char *TAG = "OLED";

void ssd1306_init();
void task_ssd1306_display_text(const void *arg_text, uint8_t _page, uint8_t _seg);
void task_ssd1306_display_clear();
esp_err_t task_ssd1306_display_location(uint8_t _page, uint8_t _seg);
esp_err_t task_ssd1306_display_image(uint8_t *images, uint8_t _page, uint8_t _seg, int _size);

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 * @note cannot use master read slave on esp32c3 because there is only one i2c controller on esp32c3
 */
static esp_err_t __attribute__((unused)) i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
    {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 * @note cannot use master write slave on esp32c3 because there is only one i2c controller on esp32c3
 */
static esp_err_t __attribute__((unused)) i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK)
    {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void ssd1306_init()
{
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_MEMORY_ADDR_MODE, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_ADDR_MODE, true);
    // set lower and upper column register address 0b upper = 0000, lower 0000,
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, 0x10, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
    i2c_master_write_byte(cmd, 0x14, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP_1, true); // reverse left-right mapping
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE_0, true); // reverse up-bottom mapping

    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);
    i2c_master_write_byte(cmd, OLED_CMD_DEACTIVE_SCROLL, true); // 2E
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);  // A6
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);      // AF

    // i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);
    // i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);
    // i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        ESP_LOGI(TAG, "OLED configured successfully");
    }
    else
    {
        ESP_LOGE(TAG, "OLED configuration failed. code: 0x%.2X", espRc);
    }
    i2c_cmd_link_delete(cmd);
    return;
}

void task_ssd1306_display_text(const void *arg_text, uint8_t _page, uint8_t _seg)
{
    char *text = (char *)arg_text;
    uint8_t text_len = strlen(text);

    uint8_t image[8];

    if (task_ssd1306_display_location(_page, _seg) == ESP_OK)
    {
        ESP_LOGI(TAG, "Printinng.....");
        for (uint8_t i = 0; i < text_len; i++)
        {
            memcpy(image,font8x8_basic_tr[(uint8_t)text[i]],8);
            task_ssd1306_display_image(image, _page, _seg,sizeof(image));
            _seg = _seg + 8;
        }
    }
    return;
}

esp_err_t task_ssd1306_display_location(uint8_t _page, uint8_t _seg)
{
    i2c_cmd_handle_t cmd;

    esp_err_t status = 0;

    uint8_t lowColumnSeg = _seg & 0x0F;
    uint8_t highColumnSeg = (_seg >> 4) & 0x0F;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, 0x00 | lowColumnSeg, true);  // reset column - choose column --> 0
    i2c_master_write_byte(cmd, 0x10 | highColumnSeg, true); // reset line - choose line --> 0
    i2c_master_write_byte(cmd, 0xB0 | _page, true);         // reset page

    i2c_master_stop(cmd);
    
    status = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (status == ESP_OK)
    {
        ESP_LOGI(TAG, "Pointer Located");
        return status;
    }
    else
    {
        ESP_LOGI(TAG, "Pointer not located");
        ESP_LOGI(TAG, "ERROR Code : %d ", status);
    }
    i2c_cmd_link_delete(cmd);
    return status;
}

esp_err_t task_ssd1306_display_image(uint8_t *images, uint8_t _page, uint8_t _seg, int _size)
{
    // ESP_LOGI(TAG, "Size : %d" , _size);
    esp_err_t status = 0;

    i2c_cmd_handle_t cmd;
    
    if (task_ssd1306_display_location(_page, _seg) == ESP_OK)
    {
        ESP_LOGI(TAG, "Displaying.....");
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
        i2c_master_write(cmd, images , _size, true);

        i2c_master_stop(cmd);
        status = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (status == ESP_OK)   
        {
            ESP_LOGI(TAG, "Image Displayed");
            return status;
        }
        else                    
        {
            ESP_LOGI(TAG, "Image Displaying error!!");
        }
    }
    return status;
}

void task_ssd1306_display_logo (uint8_t * logo)
{
    uint8_t _page = 0;
    int index = 0;

    for (; _page < 8 ; _page++)
    {
        
        index = _page*128;
        ESP_LOGI(TAG , "index : %d, Page : %d,  ", index , _page);
        task_ssd1306_display_image(&logo[index],_page,0x00,128);
         
    }
    return;
}

void task_ssd1306_display_clear()
{
    esp_err_t status ;

    uint8_t clear[128];
    for (uint8_t i = 0; i < 128; i++)
    {
        clear[i] = 0x00;
    }

    for (uint8_t _page = 0; _page < 8; _page++)
    {
        status = task_ssd1306_display_image(clear, _page , 0x00 , sizeof(clear));
        if ( status == ESP_OK ) 
        {
            ESP_LOGI(TAG, "Cleared!");
        }
        
    }
    return;
}

void app_main(void)
{
    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());

    ssd1306_init();
    while (1)
    {
        task_ssd1306_display_clear();
        task_ssd1306_display_text("21520152\n", 0, 32);
        task_ssd1306_display_text("21522773\n", 2, 32);
        task_ssd1306_display_text("21520457\n", 4, 32);
        task_ssd1306_display_text("21522426\n", 6, 32);
        vTaskDelay(5000/portTICK_PERIOD_MS);
        task_ssd1306_display_clear();
        task_ssd1306_display_logo(uit_logo_map);
        vTaskDelay(5000/portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "done.....");
    }
}
