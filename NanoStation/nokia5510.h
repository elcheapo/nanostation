/*
 * nokia5510.h
 *
 *  Created on: Feb 19, 2011
 *      Author: francois
 */

#ifndef NOKIA5510_H_
#define NOKIA5510_H_

#include "Print.h"

/*
This Code has extra features
including a XY positioning function on Display
and a Line Draw function on Nokia 3310 LCD
It is modded from the original
http://www.arduino.cc/playground/Code/PCD8544
*/

#define NUM_LINES 6
#define NUM_COL 12

void lcd_reset(void);

class Nokia5510 : public Print {
  private:
    uint8_t _CS_pin_mask;
    uint8_t led_line;
    uint8_t lcd_update;
    uint8_t pseudo_led_value[10];
    uint8_t current_x;
    uint8_t current_y;
    uint8_t lcd_text[NUM_LINES][NUM_COL];
    void LcdWrite(uint8_t dc, uint8_t data);
    void gotoXY(uint8_t x, uint8_t y);
    void lcd_print(uint8_t character);


  public:
    Nokia5510(uint8_t pin_cs_mmask, uint8_t led);
    void begin(void);
    void end(void);
    size_t write(uint8_t);
    void update(void);
    void pseudo_led(uint8_t led,uint8_t on_off);
    void clear(void);
    void low_level_clear(void);
    void clearline(void);
    uint8_t get_pseudo_led (uint8_t no);
    void go(uint8_t x, uint8_t y);
    void normal(void);
    void inverse(void);
    void bar(uint8_t col, uint8_t heigh, uint8_t max);
    void bar(uint8_t col, uint8_t heigh);
    void menu(const __FlashStringHelper *line1, \
    		const __FlashStringHelper *line2, \
    		const __FlashStringHelper *line3, \
    		const __FlashStringHelper *line4, \
    		const __FlashStringHelper *line5, \
    		const __FlashStringHelper *line6);

};

extern Nokia5510 lcd1;

#define LED_ON 1
#define LED_OFF 0

void lcd_reset(void);

#endif /* NOKIA5510_H_ */
