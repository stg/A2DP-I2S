// I2C supports the application with multi-threaded sharing of the I2C bus

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/semphr.h>
#include "I2C.h"

// I²C
#define I2C_I2C I2C_NUM_0
#define I2C_SCL GPIO_NUM_23
#define I2C_SDA GPIO_NUM_22
#define I2C_RATE 400000     // hz
#define I2C_TIMEOUT 0.0005  // sec
#define I2C_RETRY_TIMEOUT 50000 // µS
#define I2C_QUEUE_SIZE 16

// I2C transaction
typedef struct  {
  uint8_t  addr;
  uint16_t reg;
  bool     reg16;
  uint8_t* data;
  uint8_t  length;
  SemaphoreHandle_t done;
} i2c_transaction;

// I2C queue
static QueueHandle_t i2c_queue;
static SemaphoreHandle_t i2c_mutex;

// Export I2C driver
I2C_Driver I2C;

// I2C configuration
static const i2c_config_t conf = {
  .mode = I2C_MODE_MASTER,
  .sda_io_num = I2C_SDA,
  .scl_io_num = I2C_SCL,
  .sda_pullup_en = GPIO_PULLUP_DISABLE,
  .scl_pullup_en = GPIO_PULLUP_DISABLE,
  .master { .clk_speed = I2C_RATE }
};

// Bus wedge clear routine
static void i2c_bus_clear(void) {
    gpio_reset_pin(I2C_SCL);
    gpio_reset_pin(I2C_SDA);
    gpio_set_direction(I2C_SCL, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction(I2C_SDA, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(I2C_SCL, 1);
    gpio_set_level(I2C_SDA, 1);
    if(gpio_get_level(I2C_SCL) && gpio_get_level(I2C_SDA)) {
        vTaskDelay(pdMS_TO_TICKS(5));
        i2c_param_config(I2C_I2C, &conf); // restore pins
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    for(int i = 0; i < 9; i++) {
        gpio_set_level(I2C_SCL, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(I2C_SCL, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if(gpio_get_level(I2C_SDA) == 1) {
        gpio_set_level(I2C_SDA, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(I2C_SCL, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(I2C_SDA, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    i2c_param_config(I2C_I2C, &conf); // this resets the pins
}

void i2c_task(void *dummy) {
  i2c_transaction *t;
  while(true) {
    if (xQueueReceive(i2c_queue, &t, portMAX_DELAY)) {
      
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, (t->addr << 1) | I2C_MASTER_WRITE, true);
      if(t->reg16) i2c_master_write_byte(cmd, t->reg >> 8, true);
      i2c_master_write_byte(cmd, t->reg, true);

      if(t->addr & 0x80) {
        
        // read
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (t->addr << 1) | I2C_MASTER_READ, true);
        if(t->length > 1) i2c_master_read(cmd, t->data, t->length - 1, I2C_MASTER_ACK);
        if(t->length > 0) i2c_master_read_byte(cmd, t->data + t->length - 1, I2C_MASTER_NACK);

      } else {
        
        // write
        if(t->length > 0) i2c_master_write(cmd, t->data, t->length, true);

      }

      i2c_master_stop(cmd);
      esp_err_t ret;
      uint32_t t0 = esp_timer_get_time();
      do {
        ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
        if(ret != ESP_OK) i2c_bus_clear();
      } while(ret != ESP_OK && (esp_timer_get_time() - t0) < I2C_RETRY_TIMEOUT);
      i2c_cmd_link_delete(cmd);

      // done
      xSemaphoreGive(t->done);
    }
  }
}

void I2C_Driver::write(uint8_t addr, uint16_t reg, uint8_t * data, uint8_t length, bool reg16) {
  i2c_transaction t = { (uint8_t)((addr & 0x7F) | 0x00), reg, reg16, data, length, xSemaphoreCreateBinary() };
  i2c_transaction * pt = &t;
  xQueueSend(i2c_queue, &pt, portMAX_DELAY);
  xSemaphoreTake(t.done, portMAX_DELAY);
  vSemaphoreDelete(t.done);
}

void I2C_Driver::read(uint8_t addr, uint16_t reg, uint8_t * data, uint8_t length, bool reg16) {
  i2c_transaction t = { (uint8_t)((addr & 0x7F) | 0x80), reg, reg16, data, length, xSemaphoreCreateBinary() };
  i2c_transaction * pt = &t;
  xQueueSend(i2c_queue, &pt, portMAX_DELAY);
  xSemaphoreTake(t.done, portMAX_DELAY);
  vSemaphoreDelete(t.done);
}

void I2C_Driver::init() {
  ESP_ERROR_CHECK(i2c_param_config(I2C_I2C, &conf));
  ESP_ERROR_CHECK(i2c_set_timeout(I2C_I2C, APB_CLK_FREQ * I2C_TIMEOUT));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_I2C, conf.mode, 0, 0, 0));

  i2c_mutex = xSemaphoreCreateRecursiveMutex();
  i2c_queue = xQueueCreate(I2C_QUEUE_SIZE, sizeof(i2c_transaction *));
  xTaskCreate(i2c_task, "I2C Task", 1024, NULL, 2, NULL);

}

void I2C_Driver::acquire() {
  xSemaphoreTakeRecursive(i2c_mutex, portMAX_DELAY);
}

void I2C_Driver::release() {
  xSemaphoreGiveRecursive(i2c_mutex);
}

void I2C_Driver::write(uint8_t addr, uint8_t reg, uint8_t data) {
  write(addr, reg, &data, 1);
}

void I2C_Driver::write16(uint8_t addr, uint16_t reg, uint16_t data) {
  uint8_t u8[2] = { (uint8_t)(data >> 8), (uint8_t)data };
  write(addr, reg, u8, 2, true);
}

uint16_t I2C_Driver::read16(uint8_t addr, uint16_t reg) {
  uint8_t u8[2] = { 0, 0 };
  read(addr, reg, u8, 2, true);
  return (u8[0] << 8) | u8[1];
}

uint8_t I2C_Driver::read(uint8_t addr, uint8_t reg) {
  uint8_t data;
  read(addr, reg, &data, 1);
  return data;
}
