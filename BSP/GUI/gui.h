#ifndef INC_2022_PROVINCE_A_GUI_H
#define INC_2022_PROVINCE_A_GUI_H

#include "ssd1306.h"
#include "fonts.h"
#include "i2c.h"

#define SCREEN_I2C_HANDLE &hi2c2

void gui_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void gui_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t fill);
void gui_draw_triangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3);
void gui_task_selection(char *taskName, uint8_t conform);
void gui_show_variables(char *var1, int16_t value1, char *var2, int16_t value2, char *var3, int16_t value3);
void gui_control_servo(uint16_t servo1_degree, uint16_t servo1_max_degree, uint16_t servo2_degree, uint16_t servo2_max_degree);

#endif //INC_2022_PROVINCE_A_GUI_H
