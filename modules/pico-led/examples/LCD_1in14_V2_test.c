/*****************************************************************************
* | File      	:   LCD_1in14_test.c
* | Function    :   test Demo 
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2021-03-16
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
******************************************************************************/
#include "LCD_Test.h"
#include "LCD_1in14_V2.h"

/* set address */
bool reserved_addr(uint8_t addr) {
return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}


int LCD_1in14_V2_test(void)
{
    DEV_Delay_ms(100);
    printf("LCD_1in14_test Demo\r\n");
    if(DEV_Module_Init()!=0){
        return -1;
    }
    DEV_SET_PWM(50);
    /* LCD Init */
    printf("1.14inch LCD demo...\r\n");
    LCD_1IN14_V2_Init(HORIZONTAL);
    LCD_1IN14_V2_Clear(WHITE);
    
    //LCD_SetBacklight(1023);
    UDOUBLE Imagesize = LCD_1IN14_V2_HEIGHT*LCD_1IN14_V2_WIDTH*2;
    UWORD *BlackImage;
    if((BlackImage = (UWORD *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        exit(0);
    }
    // /*1.Create a new image cache named IMAGE_RGB and fill it with white*/
    Paint_NewImage((UBYTE *)BlackImage,LCD_1IN14_V2.WIDTH,LCD_1IN14_V2.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_Clear(WHITE);
    Paint_SetRotate(ROTATE_0);
    Paint_Clear(WHITE);
    
    // /* GUI */
    printf("drawing...\r\n");
    // /*2.Drawing on the image*/
#if 1
    Paint_DrawPoint(2,1, BLACK, DOT_PIXEL_1X1,  DOT_FILL_RIGHTUP);//240 240
    Paint_DrawPoint(2,6, BLACK, DOT_PIXEL_2X2,  DOT_FILL_RIGHTUP);
    Paint_DrawPoint(2,11, BLACK, DOT_PIXEL_3X3, DOT_FILL_RIGHTUP);
    Paint_DrawPoint(2,16, BLACK, DOT_PIXEL_4X4, DOT_FILL_RIGHTUP);
    Paint_DrawPoint(2,21, BLACK, DOT_PIXEL_5X5, DOT_FILL_RIGHTUP);
    Paint_DrawLine( 10,  5, 40, 35, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine( 10, 35, 40,  5, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);

    Paint_DrawLine( 80,  20, 110, 20, CYAN, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawLine( 95,   5,  95, 35, CYAN, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

    Paint_DrawRectangle(10, 5, 40, 35, RED, DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
    Paint_DrawRectangle(45, 5, 75, 35, BLUE, DOT_PIXEL_2X2,DRAW_FILL_FULL);

    Paint_DrawCircle(95, 20, 15, GREEN, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawCircle(130, 20, 15, GREEN, DOT_PIXEL_1X1, DRAW_FILL_FULL);


    Paint_DrawNum (50, 40 ,9.87654321, &Font20,3,  WHITE,  BLACK);
    Paint_DrawString_EN(1, 40, "ABC", &Font20, 0x000f, 0xfff0);
    Paint_DrawString_CN(1,60, "��ӭʹ��",  &Font24CN, WHITE, BLUE);
    Paint_DrawString_EN(1, 100, "Pico-LCD-1.14", &Font16, RED, WHITE); 

    // /*3.Refresh the picture in RAM to LCD*/
    LCD_1IN14_V2_Display(BlackImage);
    DEV_Delay_ms(2000);
    DEV_SET_PWM(10);
#endif
#if 1
     
     Paint_DrawImage(gImage_1inch14_1,0,0,240,135);
     LCD_1IN14_V2_Display(BlackImage);
     DEV_Delay_ms(2000);
     
#endif
    uint8_t keyA = 15; 
    uint8_t keyB = 17; 

    uint8_t up = 2;
	uint8_t dowm = 18;
	uint8_t left = 16;
	uint8_t right = 20;
	uint8_t ctrl = 3;
   

    SET_Infrared_PIN(keyA);    
    SET_Infrared_PIN(keyB);
		 
	SET_Infrared_PIN(up);
    SET_Infrared_PIN(dowm);
    SET_Infrared_PIN(left);
    SET_Infrared_PIN(right);
    SET_Infrared_PIN(ctrl);
    
    Paint_Clear(WHITE);
    Paint_DrawRectangle(208,12,229,32, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
    Paint_DrawRectangle(208,103,229,123, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
    Paint_DrawRectangle(37,35,58,55, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
    Paint_DrawRectangle(37,85,58,105, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
    Paint_DrawRectangle(12,60,33,80, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
    Paint_DrawRectangle(62,60,83,80, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
    Paint_DrawRectangle(37,60,58,80, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
    LCD_1IN14_V2_Display(BlackImage);
    
    
    while(1){
        if(DEV_Digital_Read(keyA ) == 0){
            Paint_DrawRectangle(208,12,228,32, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(208,12,228,32,BlackImage);
            printf("gpio =%d\r\n",keyA);
        }
        else{
            Paint_DrawRectangle(208,12,228,32, WHITE, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(208,12,228,32,BlackImage);
        }
            
        if(DEV_Digital_Read(keyB ) == 0){
            Paint_DrawRectangle(208,103,228,123, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(208,103,228,123,BlackImage);
            printf("gpio =%d\r\n",keyB);
        }
        else{
            Paint_DrawRectangle(208,103,228,123, WHITE, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(208,103,228,123,BlackImage);
        }
        
        if(DEV_Digital_Read(up ) == 0){
            Paint_DrawRectangle(37,35,57,55, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(37,35,57,55,BlackImage);
            printf("gpio =%d\r\n",up);
        }
        else{
            Paint_DrawRectangle(37,35,57,55, WHITE, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(37,35,57,55,BlackImage);
        }

        if(DEV_Digital_Read(dowm ) == 0){
            Paint_DrawRectangle(37,85,57,105, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(37,85,57,105,BlackImage);
            printf("gpio =%d\r\n",dowm);
        }
        else{
            Paint_DrawRectangle(37,85,57,105, WHITE, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(37,85,57,105,BlackImage);
        }
        
        if(DEV_Digital_Read(left ) == 0){
            Paint_DrawRectangle(12,60,32,80, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(12,60,32,80,BlackImage);
            printf("gpio =%d\r\n",left);
        }
        else{
            Paint_DrawRectangle(12,60,32,80, WHITE, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(12,60,32,80,BlackImage);
        }
            
        if(DEV_Digital_Read(right ) == 0){
            Paint_DrawRectangle(62,60,82,80, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(62,60,82,80,BlackImage);
            printf("gpio =%d\r\n",right);
        }
        else{
            Paint_DrawRectangle(62,60,82,80, WHITE, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(62,60,82,80,BlackImage);
        }
        
        if(DEV_Digital_Read(ctrl ) == 0){
            Paint_DrawRectangle(37,60,57,80, 0xF800, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(37,60,57,80,BlackImage);
            printf("gpio =%d\r\n",ctrl);
        }
        else{
            Paint_DrawRectangle(37,60,57,80, WHITE, DOT_PIXEL_2X2,DRAW_FILL_FULL);
            LCD_1IN14_V2_DisplayWindows(37,60,57,80,BlackImage);
        }
    }

    /* Module Exit */
    free(BlackImage);
    BlackImage = NULL;
    
    
    DEV_Module_Exit();
    return 0;
}
