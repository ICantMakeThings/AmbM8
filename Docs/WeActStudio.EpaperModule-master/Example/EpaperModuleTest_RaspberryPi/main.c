#include "epaper.h"
#include "bmp.h"
#include <signal.h>
#include <wiringPi.h>

void Handler(int signo)
{
    // System Exit
    printf("\r\nHandler:exit %d\r\n", signo);
    epd_io_deinit();
    exit(0);
}

struct
{
    uint8_t image_bw[50 * 300];
    uint8_t image_red[50 * 300];
} display_buf;

void init_154bw(void)
{
    uint8_t text[20];

    epd_set_panel(EPD154, 200, 200);

    epd_io_init();

    epd_paint_newimage(display_buf.image_bw, EPD_W, EPD_H, EPD_ROTATE_180, EPD_COLOR_WHITE);
    epd_paint_selectimage(display_buf.image_bw);
    epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_showPicture((EPD_H - 200) / 2,(EPD_W - 64) / 2,200,64,gImage_5,EPD_COLOR_WHITE);	
    sprintf((char *)&text, "Full Refresh");
    epd_paint_showString(10, 10, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    epd_init();
    epd_displayBW(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(2000);

    epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_showPicture((EPD_H - 200) / 2,(EPD_W - 64) / 2,200,64,gImage_5,EPD_COLOR_WHITE);	
    sprintf((char *)&text, "Fast Refresh");
    epd_paint_showString(10, 10, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    epd_init_fast();
    epd_displayBW_fast(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(2000);

    epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_showString(10, 0, (uint8_t *)&"1.54 Inch", EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
	epd_paint_showString(10, 23, (uint8_t *)&"Epaper Module", EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
	epd_paint_showString(10, 48, (uint8_t *)&"Designed By WeAct Studio", EPD_FONT_SIZE12x6, EPD_COLOR_BLACK);
	epd_paint_showString(10, 60, (uint8_t *)&"with 200 x 200 resolution", EPD_FONT_SIZE12x6, EPD_COLOR_BLACK);
	epd_paint_showPicture((EPD_H - 200) / 2,130,200,64,gImage_5,EPD_COLOR_WHITE);
    sprintf((char *)&text, "Partial Mode");
    epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    epd_init_partial();
    epd_displayBW_partial(display_buf.image_bw);

    delay(1000);
    for (uint32_t i = 123; i < 8 * 123; i += 123)
    {
        sprintf((char *)&text, ">> Num=%d     ", i);
        epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

        epd_displayBW_partial(display_buf.image_bw);

        delay(100);
    }

    sprintf((char *)&text, ">> Hello World.");
    epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_displayBW_partial(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(1000);

    epd_init();
    epd_displayBW(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);
}

void init_213bw(void)
{
    uint8_t text[20];

    epd_set_panel(EPD213_219, 122, 250);

    epd_io_init();

    epd_paint_newimage(display_buf.image_bw, EPD_W, EPD_H, EPD_ROTATE_180, EPD_COLOR_WHITE);
    epd_paint_selectimage(display_buf.image_bw);

    epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_showPicture((EPD_H - 250) / 2, (EPD_W - 122) / 2, 250, 122, gImage_4, EPD_COLOR_WHITE);
    sprintf((char *)&text, "Full Refresh");
    epd_paint_showString(10, 10, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    epd_init();
    epd_displayBW(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(2000);

    epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_showPicture((EPD_H - 250) / 2, (EPD_W - 122) / 2, 250, 122, gImage_4, EPD_COLOR_WHITE);
    sprintf((char *)&text, "Fast Refresh");
    epd_paint_showString(10, 10, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    epd_init_fast();
    epd_displayBW_fast(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(2000);

    epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_showString(10, 0, (uint8_t *)&"2.13 Epaper Module", EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_paint_showString(10, 50, (uint8_t *)&"with 250 x 122 resolution", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
    epd_paint_showString(10, 29, (uint8_t *)&"Designed By WeAct Studio", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
    epd_paint_drawRectangle(10, EPD_W - 20, EPD_H - 10, EPD_W - 6, EPD_COLOR_BLACK, 1);
    sprintf((char *)&text, "Partial Refresh");
    epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    epd_init_partial();
    epd_displayBW_partial(display_buf.image_bw);

    delay(1000);
    for (uint32_t i = 123; i < 8 * 123; i += 123)
    {
        sprintf((char *)&text, ">> Num=%d     ", i);
        epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

        epd_displayBW_partial(display_buf.image_bw);

        delay(100);
    }

    sprintf((char *)&text, ">> Hello World.");
    epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_displayBW_partial(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(1000);

    epd_init();
    epd_displayBW(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);
}

void init_213bwr(void)
{
    uint8_t text[20];

    epd_set_panel(EPD213_219, 122, 250);

    epd_io_init();

    epd_paint_newimage(display_buf.image_bw, EPD_W, EPD_H, EPD_ROTATE_180, EPD_COLOR_WHITE);

    epd_paint_selectimage(display_buf.image_bw);
    epd_paint_clear(EPD_COLOR_WHITE);

    epd_paint_selectimage(display_buf.image_red);
    epd_paint_clear(EPD_COLOR_WHITE);

    epd_paint_showPicture((EPD_H - 250) / 2, (EPD_W - 122) / 2, 250, 122, gImage_4, EPD_COLOR_WHITE);

    epd_init();
    epd_display(display_buf.image_bw, display_buf.image_red);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(3000);

    epd_paint_selectimage(display_buf.image_bw);
    epd_paint_clear(EPD_COLOR_WHITE);

    epd_paint_showString(10, 0, (uint8_t *)&"2.13 Epaper Module", EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_paint_showString(10, 50, (uint8_t *)&"with 250 x 122 resolution", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
    epd_paint_showString(10, 29, (uint8_t *)&"Designed By WeAct Studio", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);

    epd_paint_selectimage(display_buf.image_red);
    epd_paint_clear(EPD_COLOR_WHITE);

    sprintf((char *)&text, ">> Hello World.");
    epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_RED);
    epd_paint_drawRectangle(10, 103, EPD_H - 10, 116, EPD_COLOR_RED, 1);

    epd_init();
    epd_display(display_buf.image_bw, display_buf.image_red);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);
}

void init_290bw(void)
{
    uint8_t text[20];

    epd_set_panel(EPD213_219, 128, 296);

    epd_io_init();

    epd_paint_newimage(display_buf.image_bw, EPD_W, EPD_H, EPD_ROTATE_180, EPD_COLOR_WHITE);
    epd_paint_selectimage(display_buf.image_bw);

    epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_showPicture((EPD_H - 250) / 2, (EPD_W - 122) / 2, 250, 122, gImage_4, EPD_COLOR_WHITE);
    sprintf((char *)&text, "Full Refresh");
    epd_paint_showString(10, 10, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    epd_init();
    epd_displayBW(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(2000);

    epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_showPicture((EPD_H - 250) / 2, (EPD_W - 122) / 2, 250, 122, gImage_4, EPD_COLOR_WHITE);
    sprintf((char *)&text, "Fast Refresh");
    epd_paint_showString(10, 10, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    epd_init_fast();
    epd_displayBW_fast(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(2000);

    epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_showString(10, 0, (uint8_t *)&"2.9 Epaper Module", EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_paint_showString(10, 50, (uint8_t *)&"with 296 x 128 resolution", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
    epd_paint_showString(10, 29, (uint8_t *)&"Designed By WeAct Studio", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
    epd_paint_drawRectangle(10, EPD_W - 20, EPD_H - 10, EPD_W - 6, EPD_COLOR_BLACK, 1);
    sprintf((char *)&text, "Partial Refresh");
    epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    epd_init_partial();
    epd_displayBW_partial(display_buf.image_bw);

    delay(1000);
    for (uint32_t i = 123; i < 8 * 123; i += 123)
    {
        sprintf((char *)&text, ">> Num=%d     ", i);
        epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

        epd_displayBW_partial(display_buf.image_bw);

        delay(100);
    }

    sprintf((char *)&text, ">> Hello World.");
    epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_displayBW_partial(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(1000);

    epd_init();
    epd_displayBW(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);
}

void init_290bwr(void)
{
    uint8_t text[20];

    epd_set_panel(EPD213_219, 128, 296);

    epd_io_init();

    epd_paint_newimage(display_buf.image_bw, EPD_W, EPD_H, EPD_ROTATE_180, EPD_COLOR_WHITE);

    epd_paint_selectimage(display_buf.image_bw);
    epd_paint_clear(EPD_COLOR_WHITE);

    epd_paint_selectimage(display_buf.image_red);
    epd_paint_clear(EPD_COLOR_WHITE);

    epd_paint_showPicture((EPD_H - 250) / 2, (EPD_W - 122) / 2, 250, 122, gImage_4, EPD_COLOR_WHITE);

    epd_init();
    epd_display(display_buf.image_bw, display_buf.image_red);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(3000);

    epd_paint_selectimage(display_buf.image_bw);
    epd_paint_clear(EPD_COLOR_WHITE);

    epd_paint_showString(10, 0, (uint8_t *)&"2.9 Epaper Module", EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_paint_showString(10, 50, (uint8_t *)&"with 296 x 128 resolution", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
    epd_paint_showString(10, 29, (uint8_t *)&"Designed By WeAct Studio", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);

    epd_paint_selectimage(display_buf.image_red);
    epd_paint_clear(EPD_COLOR_WHITE);

    sprintf((char *)&text, ">> Hello World.");
    epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_RED);
    epd_paint_drawRectangle(10, 103, EPD_H - 10, 116, EPD_COLOR_RED, 1);

    epd_init();
    epd_display(display_buf.image_bw, display_buf.image_red);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);
}

void init_370bw(void)
{
    uint8_t text[20];

    epd_set_panel(EPD370_UC8253, 240, 416);

    epd_io_init();

    epd_paint_newimage(display_buf.image_bw, EPD_W, EPD_H, EPD_ROTATE_180, EPD_COLOR_WHITE);

    epd_paint_selectimage(display_buf.image_bw);
    epd_paint_clear(EPD_COLOR_WHITE);

    // Full refresh
    epd_paint_showString(10, 0, (uint8_t *)&"Full Refresh",
                         EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_paint_drawCircle(300, 100, 25, EPD_COLOR_BLACK, 0);
    epd_paint_drawCircle(300, 150, 25, EPD_COLOR_BLACK, 1);

    epd_paint_showPicture(10, 50, 250, 122, gImage_4, EPD_COLOR_BLACK);

    epd_init();
    epd_displayBW(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(3000);

    // Fast refresh
    epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_showString(10, 0, (uint8_t *)&"Fast Refresh",
                         EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_paint_drawCircle(300, 100, 25, EPD_COLOR_BLACK, 0);
    epd_paint_drawCircle(300, 150, 25, EPD_COLOR_BLACK, 1);

    epd_paint_showPicture(10, 50, 250, 122, gImage_4, EPD_COLOR_BLACK);

    epd_init_fast();
    epd_displayBW_fast(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    delay(3000);

    // Partial refresh
    epd_paint_clear(EPD_COLOR_WHITE);

    epd_init_fast();
    epd_displayBW_fast(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    epd_paint_showString(10, 0, (uint8_t *)&"Partial Refresh",
                         EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    for (uint32_t i = 123; i < 8 * 123; i += 123)
    {
        sprintf((char *)&text, ">> Num=%d     ", i);
        epd_paint_showString(10, 71, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
        epd_init_partial();
        epd_displayBW(display_buf.image_bw);
        epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);
        delay(100);
    }

    delay(3000);

    // Full refresh
    epd_paint_clear(EPD_COLOR_WHITE);

    epd_paint_showString(10, 0, (uint8_t *)&"3.7 Inch Epaper Module",
                         EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_paint_showString(10, 48, (uint8_t *)&"with 240 x 416 resolution",
                         EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);

    epd_paint_showString(10, 29, (uint8_t *)&"Designed By WeAct Studio",
                         EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);

    sprintf((char *)&text, ">> Hello World.");
    epd_paint_showString(10, 66, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    epd_paint_drawRectangle(10, 93, EPD_H - 10, 102, EPD_COLOR_BLACK, 1);
    epd_paint_showString(10, 104, (uint8_t *)&"weactstudio.taobao.com",
                         EPD_FONT_SIZE8x6, EPD_COLOR_BLACK);
    epd_paint_showString(10, 114, (uint8_t *)&"weactstudio.aliexpress.com",
                         EPD_FONT_SIZE8x6, EPD_COLOR_BLACK);
    epd_init();
    epd_displayBW(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);
}

void init_420bw(void)
{
    uint8_t text[20];

    epd_set_panel(EPD420, 400, 300);

    epd_io_init();

    epd_paint_newimage(display_buf.image_bw, EPD_W, EPD_H, EPD_ROTATE_270, EPD_COLOR_WHITE);
    epd_paint_selectimage(display_buf.image_bw);

    // Fast Refresh
    epd_paint_clear(EPD_COLOR_WHITE);
    epd_paint_showString(10, 0, (uint8_t *)&"4.2 Inch Epaper Module",
                         EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_paint_showString(10, 48, (uint8_t *)&"with 400 x 300 resolution",
                         EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
    epd_paint_showString(10, 29, (uint8_t *)&"Designed By WeAct Studio",
                         EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
    sprintf((char *)&text, "Fast Refresh");
    epd_paint_showString(10, 66, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

    epd_paint_drawRectangle(10, 93, EPD_H - 10, 102, EPD_COLOR_BLACK, 1);
    epd_paint_showString(10, 104, (uint8_t *)&"weactstudio.taobao.com",
                         EPD_FONT_SIZE8x6, EPD_COLOR_BLACK);
    epd_paint_showString(10, 114, (uint8_t *)&"weactstudio.aliexpress.com",
                         EPD_FONT_SIZE8x6, EPD_COLOR_BLACK);

    epd_init_fast();
    epd_displayBW_fast(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);
    delay(1000);

    // Partial Refresh
    sprintf((char *)&text, "Partial Refresh");
    epd_paint_showString(10, 66, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_init_partial();
    epd_displayBW_partial(display_buf.image_bw);
    delay(1000);
    for (uint32_t i = 123; i < 8 * 123; i += 123)
    {
        sprintf((char *)&text, ">> Num=%d     ", i);
        epd_paint_showString(10, 66, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);

        epd_displayBW_partial(display_buf.image_bw);

        delay(100);
    }

    sprintf((char *)&text, ">> Hello World.");
    epd_paint_showString(10, 66, text, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_displayBW_partial(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

    // Full Refresh
    epd_paint_drawCircle(300, 150 + 122 / 3, 25, EPD_COLOR_BLACK, 0);
    epd_paint_drawCircle(300, 200 + 122 / 3, 25, EPD_COLOR_BLACK, 1);

    epd_paint_showPicture(10, 150, 250, 122, gImage_4, EPD_COLOR_BLACK);

    epd_init();
    epd_displayBW(display_buf.image_bw);
    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);
}

void init_420bwr(void)
{
    uint8_t text[20];

    epd_set_panel(EPD420, 400, 300);

    epd_io_init();

    epd_init();

    epd_paint_newimage(display_buf.image_bw, EPD_W, EPD_H, EPD_ROTATE_270, EPD_COLOR_WHITE);

    epd_paint_selectimage(display_buf.image_bw);
    epd_paint_clear(EPD_COLOR_WHITE);

    epd_paint_showString(10, 0, (uint8_t *)&"4.2 Inch Epaper Module",
                         EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_paint_showString(10, 48, (uint8_t *)&"with 400 x 300 resolution",
                         EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);

    epd_paint_showString(10, 29, (uint8_t *)&"Designed By WeAct Studio",
                         EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);

    epd_paint_drawCircle(300, 150 + 122 / 3, 25, EPD_COLOR_BLACK, 0);
    epd_paint_drawCircle(300, 200 + 122 / 3, 25, EPD_COLOR_BLACK, 1);

    epd_paint_showPicture(10, 150, 250, 122, gImage_4, EPD_COLOR_BLACK);

    epd_paint_selectimage(display_buf.image_red);
    epd_paint_clear(EPD_COLOR_WHITE);

    sprintf((char *)&text, ">> Hello World.");
    epd_paint_showString(10, 66, text, EPD_FONT_SIZE24x12, EPD_COLOR_RED);

    epd_paint_drawRectangle(10, 93, EPD_H - 10, 102, EPD_COLOR_RED, 1);
    epd_paint_showString(10, 104, (uint8_t *)&"weactstudio.taobao.com",
                         EPD_FONT_SIZE8x6, EPD_COLOR_RED);
    epd_paint_showString(10, 114, (uint8_t *)&"weactstudio.aliexpress.com",
                         EPD_FONT_SIZE8x6, EPD_COLOR_RED);

    epd_paint_showPicture(10, 150, 250, 122, gImage_4, EPD_COLOR_WHITE);

    epd_display(display_buf.image_bw, display_buf.image_red);

    epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);
}

int main(void)
{
    // Exception handling:ctrl + c
    signal(SIGINT, Handler);

    printf("Hello World!\n");
    printf("Hello WeAct Studio!\n");

#if defined(EPD154_BW) 
    init_154bw();  
#elif defined(EPD213_BW) 
    init_213bw();
#elif defined(EPD213_BWR) 
    init_213bwr();
#elif defined(EPD290_BW)
    init_290bw();
#elif defined(EPD290_BWR)
    init_290bwr();
#elif defined(EPD370_BW)
    init_370bw();
#elif defined(EPD420_BW)
    init_420bw();
#elif defined(EPD420_BWR)
    init_420bwr();
#else
#error "EPD_TYPE is not defined, example: make EPD_TYPE=EPD420_BWR "
#endif

    printf("epd io deinit.\n");
    epd_io_deinit();
    return 0;
}