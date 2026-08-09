#include <stdint.h>
#include <stdbool.h>

class I2C_Driver {
public:
  I2C_Driver() {}

  void     init();
  void     acquire();
  void     release();
  void     write(uint8_t addr, uint16_t reg, uint8_t * data, uint8_t length, bool reg16 = false);
  void     write(uint8_t addr, uint8_t reg, uint8_t data);
  void     write16(uint8_t addr, uint16_t reg, uint16_t data);
  void     read(uint8_t addr, uint16_t reg, uint8_t * data, uint8_t length, bool reg16 = false);
  uint8_t  read(uint8_t addr, uint8_t reg);
  uint16_t read16(uint8_t addr, uint16_t reg);
};

extern I2C_Driver I2C;
