#include "pico-led.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "./examples/LCD_Test.h"
#include "./lib/LCD/LCD_0in96.h"
#include "pico/stdlib.h"

#ifdef __cplusplus
}
#endif

static uint16_t magic = 42;

Module::Module() {

}

void Module::HardwareRead(picoled_ins_t *ins, picoled_outs_t *outs) {}
void Module::HardwareWrite(picoled_ins_t *ins, picoled_outs_t *outs) {}

UDOUBLE Imagesize = LCD_0IN96_HEIGHT * LCD_0IN96_WIDTH * 2;
static UWORD *BlackImage;

void Module::initHardware() {
	DEV_Delay_ms(100);
    printf("LCD_0in96_test Demo\r\n");
    if(DEV_Module_Init() != 0){
        return;
    }

	/* LCD Init */
    printf("0.96inch LCD demo...\r\n");
    LCD_0IN96_Init(HORIZONTAL);
    LCD_0IN96_Clear(WHITE);
    DEV_Delay_ms(1000);

    if((BlackImage = (UWORD *) malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        exit(0);
    }

    // /*1.Create a new image cache named IMAGE_RGB and fill it with white*/
    Paint_NewImage((UBYTE *)BlackImage, LCD_0IN96.WIDTH, LCD_0IN96.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_Clear(WHITE);
    Paint_SetRotate(ROTATE_0);
    Paint_Clear(WHITE);
}

bool reserved_addr(uint8_t addr) {
	return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

volatile bool filled = false;

int64_t alarm_callback(alarm_id_t id, void *user_data) {
	uint64_t t = time_us_64();
    // Serial.printf("Timer %d fired!\n", (int) id);
	filled = !filled;

    // Can return a value here in us to fire in the future
    return 500000;
}

bool repeating_timer_callback(struct repeating_timer *t) {
	filled = !filled;
	return true;
}

void Module::process(float msDelta) {

	static bool setup = false;
	static struct repeating_timer timer;

	if (!setup) {

		// Paint_DrawCircle(95, 20, 15, RED,DOT_PIXEL_1X1, DRAW_FILL_EMPTY);

		/*3.Refresh the picture in RAM to LCD*/
		// LCD_0IN96_Display(BlackImage);
		// DEV_Delay_ms(3000);
		// DEV_SET_PWM(100); //Set the backlight max:100

		// Paint_DrawImage(gImage_0inch96_1, 0, 0, 160, 80);
		// LCD_0IN96_Display(BlackImage);
		// DEV_Delay_ms(3000);

		add_alarm_in_ms(500, alarm_callback, NULL, false);
		// add_repeating_timer_ms(-500, repeating_timer_callback, NULL, &timer);

		Paint_Clear(WHITE);
		LCD_0IN96_Display(BlackImage);

		setup = true;
	} else {

		Paint_Clear(WHITE);

		Paint_DrawCircle(80, 40, 15, RED, DOT_PIXEL_2X2, filled ? DRAW_FILL_FULL : DRAW_FILL_EMPTY);

		LCD_0IN96_Display(BlackImage);
	}

	// Technically you should do this stuff at some point
	/* Module Exit */
    // free(BlackImage);
    // BlackImage = NULL;

    // DEV_Module_Exit();
}
