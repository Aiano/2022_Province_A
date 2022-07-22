#include <stdlib.h>
#include "gui.h"

void gui_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
    if (x1 == x2) { // slope == infinite
        if (y2 < y1) { // Make sure that y1 <= y2
            uint8_t temp = y2;
            y2 = y1;
            y1 = temp;
        }
        for (uint8_t y0 = y1; y0 <= y2; y0++) {
            ssd1306_DrawPixel(x1, y0, White);
        }
        return;
    }

    if (x2 < x1) { // Make sure that x1 <= x2
        // Swap two variable in-place
        uint8_t temp = x2;
        x2 = x1;
        x1 = temp;

        temp = y2;
        y2 = y1;
        y1 = temp;
    }


    for (uint8_t x0 = x1; x0 <= x2; x0++) { // iterate each pixel of the line
        uint8_t y0 = (y2 - y1) * (x0 - x1) / (x2 - x1) + y1;
        ssd1306_DrawPixel(x0, y0, White);
    }
    return;
}

void gui_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t fill) {
    // Clock wise from left top
    if(fill){
        for(int i = y1; i <= y2; i++){
            for(int j = x1; j <= x2; j++){
                ssd1306_DrawPixel(j, i, White);
            }
        }
        return;
    }

    gui_draw_line(x1, y1, x2, y1); // Line1
    gui_draw_line(x2, y1, x2, y2); // Line2
    gui_draw_line(x2, y2, x1, y2); // Line3
    gui_draw_line(x1, y2, x1, y1); // Line4
    return;
}

void gui_draw_triangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3) {
    // Clock wise from left top

    gui_draw_line(x1, y1, x2, y2); // Line1
    gui_draw_line(x2, y2, x3, y3); // Line2
    gui_draw_line(x3, y3, x1, y1); // Line3
}

void gui_task_selection(char *taskName, uint8_t confirm) {
    if (confirm == 0) {
        ssd1306_Fill(Black);
        gui_draw_rectangle(15, 5, 112, 60, 0);
        gui_draw_triangle(11, 15, 11, 50, 0, 32);
        gui_draw_triangle(116, 15, 116, 50, 127, 32);
        ssd1306_SetCursor(20, 20);
        ssd1306_WriteString(taskName, Font_16x26, White);
    } else {
        ssd1306_Fill(White);
        ssd1306_SetCursor(20, 20);
        ssd1306_WriteString(taskName, Font_16x26, Black);
    }

    ssd1306_UpdateScreen(SCREEN_I2C_HANDLE);
}

void gui_show_variables(char *var1, int16_t value1, char *var2, int16_t value2, char *var3, int16_t value3) {
    ssd1306_Fill(Black);
    gui_draw_rectangle(2, 2, 125, 61, 0);

    char str[10]; // value buffer

    // first line
    if(var1 != NULL) {
        ssd1306_SetCursor(4, 4);
        ssd1306_WriteString(var1, Font_7x10, White);
        ssd1306_WriteChar(':', Font_7x10, White);
        itoa(value1, str, 10);
        ssd1306_WriteString(str, Font_7x10, White);
    }

    // second line
    if(var2 != NULL) {
        ssd1306_SetCursor(4, 16);
        ssd1306_WriteString(var2, Font_7x10, White);
        ssd1306_WriteChar(':', Font_7x10, White);
        itoa(value2, str, 10);
        ssd1306_WriteString(str, Font_7x10, White);
    }

    // third line
    if(var3 != NULL) {
        ssd1306_SetCursor(4, 28);
        ssd1306_WriteString(var3, Font_7x10, White);
        ssd1306_WriteChar(':', Font_7x10, White);
        itoa(value3, str, 10);
        ssd1306_WriteString(str, Font_7x10, White);
    }

    ssd1306_UpdateScreen(SCREEN_I2C_HANDLE);
    return;
}

void gui_control_servo(uint16_t servo1_degree, uint16_t servo1_max_degree, uint16_t servo2_degree, uint16_t servo2_max_degree) {
    ssd1306_Fill(Black);

    char str[5];

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("S1:", Font_16x26, White);
    itoa(servo1_degree, str, 10);
    ssd1306_WriteString(str, Font_11x18, White);
    ssd1306_WriteChar('/',Font_11x18, White);
    itoa(servo1_max_degree, str, 10);
    ssd1306_WriteString(str, Font_11x18, White);
    gui_draw_rectangle(48, 18, 48 + 80 * servo1_degree/servo1_max_degree, 20, 1);

    ssd1306_SetCursor(2, 30);
    ssd1306_WriteString("S2:", Font_16x26, White);
    itoa(servo2_degree, str, 10);
    ssd1306_WriteString(str, Font_11x18, White);
    ssd1306_WriteChar('/',Font_11x18, White);
    itoa(servo2_max_degree, str, 10);
    ssd1306_WriteString(str, Font_11x18, White);
    gui_draw_rectangle(48, 48, 48 + 80 * servo2_degree/servo2_max_degree, 50, 1);

    ssd1306_UpdateScreen(SCREEN_I2C_HANDLE);
}


