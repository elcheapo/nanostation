/*
 * i2c_keyboard.h
 *
 *  Created on: 20 juin 2012
 *      Author: florrain
 */

#ifndef I2C_KEYBOARD_H_
#define I2C_KEYBOARD_H_

class I2c_Keyboard
{
  private:
    uint8_t _i2c_address;
    volatile uint8_t column[4];
//    uint8_t state;
//    uint8_t last_key;
//    uint8_t last_star_line;

  public:

    I2c_Keyboard(uint8_t i2c_address);
    uint8_t scan(void);
    inline uint8_t get_column(uint8_t column_id);
    int8_t get_key(void);
    int8_t get_key_debounced(uint8_t & last);
    uint8_t get_star_line(void);
    int8_t get_star_line_debounced(uint8_t & last);
};

inline uint8_t I2c_Keyboard::get_column(uint8_t column_id) {return column[column_id & 0x03] & 0x0f;};

extern I2c_Keyboard kbd1;
extern I2c_Keyboard kbd2;
extern I2c_Keyboard kbd3;
// extern I2c_Keyboard * kbd[3];

#endif /* I2C_KEYBOARD_H_ */
