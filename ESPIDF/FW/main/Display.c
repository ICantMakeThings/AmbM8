// 1.54" 200x200 WEACT SSD1681 e-paper driver + UI & keyh handler

#include "Display.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define W 200
#define H 200
#define FB_BYTES (W * H / 8)
#define PIN_MOSI GPIO_NUM_16
#define PIN_SCLK GPIO_NUM_17
#define PIN_CS GPIO_NUM_18
#define PIN_RST GPIO_NUM_19
#define PIN_BUSY GPIO_NUM_20
#define PIN_DC GPIO_NUM_21
#define KEY_LEFT GPIO_NUM_48
#define KEY_RIGHT GPIO_NUM_52
#define KEY_UP GPIO_NUM_49
#define KEY_DOWN GPIO_NUM_50
#define KEY_PLAY GPIO_NUM_38
#define KEY_REC GPIO_NUM_29
#define KEY_MONITOR GPIO_NUM_37
#define KEY_NEXT GPIO_NUM_10
#define KEY_PREVIOUS GPIO_NUM_3
#define ENCODER_SW GPIO_NUM_2
#define ENCODER_A GPIO_NUM_53
#define ENCODER_B GPIO_NUM_54
static const char *TAG = "epaper";
static spi_device_handle_t s_spi;
static uint8_t s_fb[FB_BYTES];
static display_mode_t s_mode;
static display_io_t s_io;
static display_audio_stats_t s_stats;
static display_usb_stats_t s_usb_stats;
static uint8_t s_levels[6];
static float s_output_dbfs[2] = {-120.0f, -120.0f};
static uint8_t s_gain_channel;
static int8_t s_gain_db;
static volatile uint8_t s_home_volume;
static uint64_t s_total, s_free;
static bool s_mounted;
static uint8_t s_menu;
static volatile uint8_t s_spectrum_source;
static float s_spectrum_dbfs[16] = {
    -120,-120,-120,-120,-120,-120,-120,-120,
    -120,-120,-120,-120,-120,-120,-120,-120
};
static int8_t s_spectrum_sensitivity_db = 6;
static volatile bool s_dirty = true;
static display_standalone_key_cb_t s_standalone_key_cb;
static display_control_cb_t s_control_cb;
static char s_standalone_path[32] = "/";
static char s_standalone_entries[8][32];
static uint8_t s_standalone_count, s_standalone_selected, s_standalone_volume = 80;
static bool s_standalone_playing, s_standalone_paused;
static display_standalone_page_t s_standalone_page = DISPLAY_STANDALONE_HOME;
static char s_standalone_track[32], s_standalone_status[32] = "SD CARD STARTING";
static char s_standalone_format[12] = "WAV";
static uint32_t s_standalone_rate, s_standalone_bitrate;
static uint8_t s_standalone_bits, s_standalone_channels;
static uint64_t s_standalone_file_size;
static uint32_t s_standalone_elapsed_ms, s_standalone_duration_ms;
static bool s_standalone_duration_estimated;
static bool s_monitor_on;
static bool s_reduced_motion;
static uint8_t s_monitor_yaw;
static float s_monitor_wxyz_db[4] = {-120, -120, -120, -120};
static float s_monitor_out_db[2] = {-120, -120};
static bool s_monitor_clip;

typedef struct {
    display_settings_menu_t menu;
    uint8_t selected;
    uint8_t count;
    char items[8][48];
    char title[24];
    char hint[40];
} settings_view_t;
static settings_view_t s_settings;
static bool s_tap_active;
static uint8_t s_tap_levels[4];
static uint8_t s_tap_count;
static char s_tap_labels[4][4];
static char s_rec_labels[4][8] = {"I1", "I2", "I3", "I4"};
static uint8_t s_rec_channels = 4;
static bool s_recording_active;

// SPI
static void tx(bool data, uint8_t v) {
    gpio_set_level(PIN_DC, data);
    spi_transaction_t t = {.length=8, .tx_buffer=&v};
    spi_device_transmit(s_spi, &t);
}
static void cmd(uint8_t v) { tx(false, v); }
static void dat(uint8_t v) { tx(true, v); }
static void wait_busy(void) { for (int i=0; i<500 && gpio_get_level(PIN_BUSY); ++i) vTaskDelay(pdMS_TO_TICKS(10)); }
static void reset_panel(void) { gpio_set_level(PIN_RST, 0); vTaskDelay(pdMS_TO_TICKS(10)); gpio_set_level(PIN_RST, 1); vTaskDelay(pdMS_TO_TICKS(10)); wait_busy(); }

// Framebuffer thingy that buffers
static void pixel(int x, int y, bool black) {
    if (x<0 || x>=W || y<0 || y>=H) return;
    int px = W - 1 - y;
    int py = x;
    x = px; y = py;
    uint16_t i=(uint16_t)y*(W/8)+(x>>3); uint8_t mask=(uint8_t)(0x80u>>(x&7));
    if (black) s_fb[i] &= (uint8_t)~mask; else s_fb[i] |= mask;
}
static void fill_rect(int x,int y,int w,int h,bool black) { for(int yy=y;yy<y+h;yy++) for(int xx=x;xx<x+w;xx++) pixel(xx,yy,black); }
static void line(int x0,int y0,int x1,int y1,bool black) {
    int dx=x1>x0?x1-x0:x0-x1, sx=x0<x1?1:-1, dy=-(y1>y0?y1-y0:y0-y1), sy=y0<y1?1:-1, e=dx+dy;
    for (;;) { pixel(x0,y0,black); if(x0==x1&&y0==y1) break; int e2=2*e; if(e2>=dy){e+=dy;x0+=sx;} if(e2<=dx){e+=dx;y0+=sy;} }
}

static const uint8_t glyph[][5] = {
 {0,0,0,0,0},{0,0,0x5f,0,0},{0,7,0,7,0},{0x14,0x7f,0x14,0x7f,0x14},
 {0x24,0x2a,0x7f,0x2a,0x12},{0x23,0x13,8,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0,5,3,0,0},
 {0,0x1c,0x22,0x41,0},{0,0x41,0x22,0x1c,0},{8,0x2a,0x1c,0x2a,8},{8,8,0x3e,8,8},{0,0x50,0x30,0,0},{8,8,8,8,8},{0,0x60,0x60,0,0},
 {0x20,0x10,8,4,2},{0x3e,0x51,0x49,0x45,0x3e},{0,0x42,0x7f,0x40,0},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},
 {0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3c,0x4a,0x49,0x49,0x30},{1,0x71,9,5,3},{0x36,0x49,0x49,0x49,0x36},{6,0x49,0x49,0x29,0x1e},
 {0x7e,0x11,0x11,0x11,0x7e},{0x7f,0x49,0x49,0x49,0x36},{0x3e,0x41,0x41,0x41,0x22},{0x7f,0x41,0x41,0x22,0x1c},{0x7f,0x49,0x49,0x49,0x41},{0x7f,9,9,9,1},
 {0x3e,0x41,0x49,0x49,0x7a},{0x7f,8,8,8,0x7f},{0,0x41,0x7f,0x41,0},{0x20,0x40,0x41,0x3f,1},{0x7f,8,0x14,0x22,0x41},{0x7f,0x40,0x40,0x40,0x40},{0x7f,2,0x0c,2,0x7f},{0x7f,4,8,0x10,0x7f},
 {0x3e,0x41,0x41,0x41,0x3e},{0x7f,9,9,9,6},{0x3e,0x41,0x51,0x21,0x5e},{0x7f,9,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},{1,1,0x7f,1,1},{0x3f,0x40,0x40,0x40,0x3f},{0x1f,0x20,0x40,0x20,0x1f},{0x3f,0x40,0x38,0x40,0x3f},{0x63,0x14,8,0x14,0x63},{7,8,0x70,8,7},{0x61,0x51,0x49,0x45,0x43}
};
// 5x7 font renderer when it bitmaps the bitmapping below
static void text(int x,int y,const char *s,int scale) {
    for(;*s;s++,x+=6*scale){ char c=*s; int ix=c==' '?0:(c>='0'&&c<='9'?c-'0'+16:(c>='A'&&c<='Z'?c-'A'+26: (c=='!'?1:c=='+'?11:c=='-'?13:c=='.'?14:c=='/'?15:0)));
        for(int col=0;col<5;col++) for(int row=0;row<7;row++) if(glyph[ix][col]&(1<<row)) fill_rect(x+col*scale,y+row*scale,scale,scale,true);
    }
}
static void draw_triangle(int x,int y) { for(int r=0;r<12;r++){ line(x-r,y+r,x+r,y+r,true); } text(x-2,y+4,"!",1); }
// draws the active page in specific mode in frame yk?
static void draw_main(void) {
    memset(s_fb,0xff,sizeof(s_fb));
    text(8,7,s_mode==DISPLAY_MODE_STORAGE?"STORAGE":s_mode==DISPLAY_MODE_STANDALONE?"STANDALONE":"UAC2",2);
    const char *right=s_mode==DISPLAY_MODE_UAC2?"96KHZ/24-BIT":s_mode==DISPLAY_MODE_STANDALONE?(s_standalone_playing?(s_standalone_paused?"PAUSED":"PLAYING"):"READY"):s_io==DISPLAY_IO_UNMOUNTED?"UNMOUNTED":s_io==DISPLAY_IO_WRITING?"WRITING":s_io==DISPLAY_IO_READING?"READING":"READY";
    text(W-6-(int)strlen(right)*6,9,right,1);
    if(s_stats.clipping && s_mode==DISPLAY_MODE_UAC2) draw_triangle(49,4);
    line(0,28,199,28,true);
    if(s_mode==DISPLAY_MODE_STANDALONE && s_standalone_page==DISPLAY_STANDALONE_INFO) {
        char row[40], time[16];
        text(8,34,"TRACK INFO",2);
        snprintf(row,sizeof(row),"FORMAT %s",s_standalone_format); text(8,56,row,1);
        snprintf(row,sizeof(row),"RATE %luHZ",(unsigned long)s_standalone_rate); text(8,73,row,1);
        snprintf(row,sizeof(row),"BITRATE %luKBPS",(unsigned long)(s_standalone_bitrate/1000)); text(8,90,row,1);
        snprintf(row,sizeof(row),"%uBIT %s",(unsigned)s_standalone_bits,s_standalone_channels==1?"MONO":"STEREO"); text(8,107,row,1);
        if(s_standalone_file_size >= 1024U*1024U)
            snprintf(row,sizeof(row),"FILE %llu.%lluMB",(unsigned long long)(s_standalone_file_size/(1024U*1024U)),(unsigned long long)((s_standalone_file_size%(1024U*1024U))*10/(1024U*1024U)));
        else snprintf(row,sizeof(row),"FILE %lluKB",(unsigned long long)(s_standalone_file_size/1024U));
        text(8,124,row,1);
        uint32_t elapsed=s_standalone_elapsed_ms/1000U, total=s_standalone_duration_ms/1000U;
        snprintf(time,sizeof(time),"%02lu-%02lu",(unsigned long)(elapsed/60U),(unsigned long)(elapsed%60U));
        snprintf(row,sizeof(row),"ELAPSED %s",time); text(8,141,row,1);
        snprintf(time,sizeof(time),"%02lu-%02lu",(unsigned long)(total/60U),(unsigned long)(total%60U));
        snprintf(row,sizeof(row),"LEFT %s%s",s_standalone_duration_estimated?"~":"",time); text(8,158,row,1);
        int progress=s_standalone_duration_ms?(int)((uint64_t)s_standalone_elapsed_ms*184/s_standalone_duration_ms):0;
        if(progress>184)progress=184;
        line(8,179,192,179,true); line(8,184,192,184,true);
        line(8,179,8,184,true); line(192,179,192,184,true);
        if(progress>0) fill_rect(9,180,progress,4,true);
        text(8,191,"L/R PAGES",1);
        return;
    }
    if(s_mode==DISPLAY_MODE_STANDALONE && s_standalone_page==DISPLAY_STANDALONE_LISTEN) {
        text(8,33,"LISTEN",2);
        text(78,37,s_standalone_path,1);
        for(uint8_t i=0;i<s_standalone_count && i<8;i++) {
            int y=52+(int)i*14;
            if(i==s_standalone_selected) fill_rect(5,y-2,190,12,true);
            if(i==s_standalone_selected) {
                char row[32]; snprintf(row,sizeof(row),"%s",s_standalone_entries[i]);
                for(int c=0;row[c] && c<31;c++) {
                    char ch=row[c]; int ix=ch==' '?0:(ch>='0'&&ch<='9'?ch-'0'+16:(ch>='A'&&ch<='Z'?ch-'A'+26:(ch=='!'?1:ch=='-'?13:ch=='.'?14:ch=='/'?15:0)));
                    for(int col=0;col<5;col++) for(int rr=0;rr<7;rr++) if(glyph[ix][col]&(1<<rr)) fill_rect(8+c*6+col,y+rr,1,1,false);
                }
            } else text(8,y,s_standalone_entries[i],1);
        }
        line(5,139,195,139,true);
        text(8,142,s_standalone_track[0]?s_standalone_track:s_standalone_status,1);
        text(8,155,s_standalone_playing?(s_standalone_paused?"PREV   PLAY   NEXT":"PREV  PAUSE  NEXT"):"PREV   PLAY   NEXT",1);
        int progress=s_standalone_duration_ms?(int)((uint64_t)s_standalone_elapsed_ms*184/s_standalone_duration_ms):0;
        if(progress>184)progress=184;
        line(8,169,192,169,true); line(8,175,192,175,true);
        line(8,169,8,175,true); line(192,169,192,175,true);
        if(progress>0) fill_rect(9,170,progress,5,true);
        uint32_t elapsed=s_standalone_elapsed_ms/1000U;
        uint32_t remaining=s_standalone_duration_ms>s_standalone_elapsed_ms?(s_standalone_duration_ms-s_standalone_elapsed_ms)/1000U:0;
        char times[24];
        snprintf(times,sizeof(times),"%02lu-%02lu LEFT %s%02lu-%02lu",(unsigned long)(elapsed/60U),(unsigned long)(elapsed%60U),s_standalone_duration_estimated?"~":"",(unsigned long)(remaining/60U),(unsigned long)(remaining%60U));
        text(8,180,times,1);
        char vol[16]; snprintf(vol,sizeof(vol),"VOL %u%%",s_standalone_volume); text(144,192,vol,1);
        text(8,192,"HOLD LEFT BACK",1);
        return;
    }
    if(s_mode==DISPLAY_MODE_STANDALONE && s_standalone_page==DISPLAY_STANDALONE_EQ) {
        static const char *source_names[6]={"O1","O2","I1","I2","I3","I4"};
        char title[24];
        snprintf(title,sizeof(title),"%s SPECTRUM",source_names[s_spectrum_source%6]);
        text(8,34,title,2);
        const int plot_x=16, plot_y=58, plot_w=142, plot_h=104;
        for(int i=0;i<=4;i++) {
            int y=plot_y+i*(plot_h/4);
            for(int x=plot_x;x<plot_x+plot_w;x+=6) line(x,y,x+2,y,true);
            int line_db=-(i*25)-s_spectrum_sensitivity_db;
            char label[12];
            if(line_db==0) snprintf(label,sizeof(label),"0DBFS");
            else snprintf(label,sizeof(label),"-%dDBFS",-line_db);
            text(162,y-3,label,1);
        }
        for(int i=0;i<16;i++) {
            float db=s_spectrum_dbfs[i]+s_spectrum_sensitivity_db;
            if(db < -100.0f) db=-100.0f;
            if(db > 0.0f) db=0.0f;
            int h=(int)((db+100.0f)*(plot_h/100.0f));
            fill_rect(plot_x+i*9,plot_y+plot_h-h,6,h,true);
        }
        text(8,166,"20HZ",1); text(82,166,"1KHZ",1); text(135,166,"20KHZ",1);
        text(8,176,s_spectrum_source<2?"OUTPUT SPECTRUM":"PCM1865 4CH INPUT",1);
        text(8,188,"UP/DOWN SOURCE  L/R PAGES",1);
        return;
    }
    if(s_mode==DISPLAY_MODE_STANDALONE && s_standalone_page==DISPLAY_STANDALONE_MONITOR) {
        text(8,34,"LIVE MONITOR",2);
        char st[28];
        snprintf(st,sizeof(st),"%s  YAW %d DEG",s_monitor_on?"ON":"OFF",(int)(s_monitor_yaw&3)*90);
        text(8,56,st,1);
        if(s_monitor_clip) text(150,56,"CLIP",1);
        static const char *bn[4]={"W","X","Y","Z"};
        for(int i=0;i<4;i++){
            int x=20+i*45;
            text(x+8,70,bn[i],1);
            line(x,84,x,164,true); line(x+20,84,x+20,164,true);
            line(x,84,x+20,84,true); line(x,164,x+20,164,true);
            float db=s_monitor_wxyz_db[i];
            if(db<-60.0f)db=-60.0f;
            if(db>0.0f)db=0.0f;
            int h=(int)((db+60.0f)*80.0f/60.0f);
            if(h>0)fill_rect(x+2,162-h,18,h,true);
            char lb[8];
            if(db<=-59.5f) snprintf(lb,sizeof(lb),"-INF");
            else snprintf(lb,sizeof(lb),"%d",(int)lroundf(db));
            text(x+2,166,lb,1);
        }
        char o[36];
        snprintf(o,sizeof(o),"OUT L%d R%d DB",(int)lroundf(s_monitor_out_db[0]),(int)lroundf(s_monitor_out_db[1]));
        text(8,178,o,1);
        text(8,190,"HOLD KEY2 TOGGLE ENC=YAW",1);
        return;
    }
    if(s_mode==DISPLAY_MODE_STANDALONE && s_standalone_page==DISPLAY_STANDALONE_SETTINGS) {
        text(8,34,s_settings.title,2);
        line(0,58,199,58,true);
        if(s_tap_active) {
            for(int i=0;i<s_tap_count;i++) {
                int y=72+i*28;
                text(8,y,s_tap_labels[i],2);
                line(40,y,196,y,true);
                line(40,y+16,196,y+16,true);
                line(40,y,40,y+16,true);
                line(196,y,196,y+16,true);
                int v=s_tap_levels[i];
                if(v>100) v=100;
                fill_rect(42,y+2,(int)((152*(uint32_t)v)/100),13,true);
            }
            text(8,190,s_settings.hint,1);
            return;
        }
        for(int i=0;i<s_settings.count && i<8;i++) {
            int y=64+i*18;
            text(8,y,s_settings.items[i],1);
            if(i==s_settings.selected) fill_rect(2,y,4,13,true);
        }
        text(8,190,s_settings.hint,1);
        return;
    }
    if(s_menu==1) {
        char b[40]; text(8,34,"AUDIO STATS",2);
        snprintf(b,sizeof(b),"USB %lu PKT/S",(unsigned long)s_usb_stats.usb_packets_per_sec); text(8,52,b,1);
        snprintf(b,sizeof(b),"%lu B/S",(unsigned long)s_usb_stats.usb_bytes_per_sec); text(8,67,b,1);
        snprintf(b,sizeof(b),"FIFO %lu B/S",(unsigned long)s_usb_stats.fifo_bytes_per_sec); text(8,82,b,1);
        snprintf(b,sizeof(b),"TASK %lu PKT/S",(unsigned long)s_usb_stats.task_packets_per_sec); text(8,97,b,1);
        snprintf(b,sizeof(b),"Q HI %lu/%lu DROP %lu",(unsigned long)s_usb_stats.queue_high_water,(unsigned long)s_usb_stats.queue_depth,(unsigned long)s_usb_stats.drops); text(8,112,b,1);
        snprintf(b,sizeof(b),"RX %lu CALL %lu FR/S",(unsigned long)s_stats.rx_calls_per_sec,(unsigned long)s_stats.rx_frames_per_sec); text(8,127,b,1);
        snprintf(b,sizeof(b),"LEN %lu-%lu GAP %luUS",(unsigned long)s_stats.rx_len_min,(unsigned long)s_stats.rx_len_max,(unsigned long)s_stats.rx_gap_max_us); text(8,142,b,1);
        snprintf(b,sizeof(b),"QUEUE %lu ASRC %+ld",(unsigned long)s_stats.queue_frames,(long)s_stats.asrc_ppm); text(8,157,b,1);
        snprintf(b,sizeof(b),"UND %lu I2S %luUS",(unsigned long)s_stats.underruns,(unsigned long)s_stats.i2s_write_max_us); text(8,172,b,1);
        snprintf(b,sizeof(b),"I2S ERR %lu",(unsigned long)s_stats.i2s_write_errors); text(8,187,b,1);
        return;
    }
    if(s_menu==2) {
        static const char *source_names[6]={"O1","O2","I1","I2","I3","I4"};
        char title[24];
        snprintf(title,sizeof(title),"%s SPECTRUM",source_names[s_spectrum_source%6]);
        text(8,34,title,2);
        const int plot_x=16, plot_y=58, plot_w=142, plot_h=104;
        for(int i=0;i<=4;i++){
            int y=plot_y+i*(plot_h/4);
            for(int x=plot_x;x<plot_x+plot_w;x+=6) line(x,y,x+2,y,true);
            int line_db=-(i*25)-s_spectrum_sensitivity_db;
            char label[12];
            if(line_db==0) snprintf(label,sizeof(label),"0DBFS");
            else snprintf(label,sizeof(label),"-%dDBFS",-line_db);
            text(160,y-3,label,1);
        }
        for(int i=0;i<16;i++){
            float db=s_spectrum_dbfs[i];
            db += s_spectrum_sensitivity_db;
            if(db < -100.0f) db=-100.0f;
            if(db > 0.0f) db=0.0f;
            int h=(int)((db+100.0f)*(plot_h/100.0f));
            int x=plot_x+i*9;
            fill_rect(x,plot_y+plot_h-h,6,h,true);
        }
        text(8,166,"20HZ",1); text(82,166,"1KHZ",1); text(135,166,"20KHZ",1);
        text(8,176,"PCM5100A DR100 SNR100",1);
        char sensitivity[24];
        snprintf(sensitivity,sizeof(sensitivity),"THD -90DB SENS +%dDB",(int)s_spectrum_sensitivity_db);
        text(8,188,sensitivity,1);
        return;
    }
    if(s_mode==DISPLAY_MODE_STORAGE) {
        char b[32]; uint64_t total=s_total/1000000000ULL, freegb=s_free/1000000000ULL;
        snprintf(b,sizeof(b),"%lluGB TOTAL",(unsigned long long)total);
        text(8,72,b,1); text(8,92,s_mounted?"FREE SPACE HOST":"CARD NOT READY",1);
        text(8,122,s_mounted?"MANAGED BY PC":"CHECK SD CARD",1); (void)freegb; return;
    }
    if (s_mode == DISPLAY_MODE_STANDALONE &&
        (s_recording_active || s_standalone_page == DISPLAY_STANDALONE_RECORDING)) {
        text(8, 34, "RECORDING", 2);
        line(0, 58, 199, 58, true);

        text(8, 64, s_standalone_status, 1);
        if (s_monitor_on) text(174, 34, "MON", 1);

        const char *ch_names[4] = {
            s_rec_labels[0], s_rec_labels[1], s_rec_labels[2], s_rec_labels[3]
        };
        uint8_t rec_ch = s_rec_channels;
        if (rec_ch < 1) rec_ch = 1;
        if (rec_ch > 4) rec_ch = 4;
        for (int i = 0; i < rec_ch; i++) {
            int x = 20 + i * 45;
            text(x, 80, ch_names[i], 1);

            line(x, 95, x, 165, true);
            line(x + 20, 95, x + 20, 165, true);
            line(x, 95, x + 20, 95, true);
            line(x, 165, x + 20, 165, true);
            int v = s_levels[i];
            if (v > 100) v = 100;
            int bar_h = (v * 70) / 100;
            fill_rect(x + 2, 163 - bar_h, 18, bar_h, true);

            line(x, 109, x + 20, 109, true);
        }

        text(8, 172, s_standalone_track[0] ? s_standalone_track : "NO FILE", 1);

        text(8, 188, "REC: START/STOP  HOLD LEFT: EXIT", 1);
        return;
    }
    if(s_mode==DISPLAY_MODE_STANDALONE) text(8,30,"HOME",1);
    const char *names[6]={"I1","I2","I3","I4","O1","O2"};
    for(int i=0;i<6;i++){int x=14+i*32; text(x,39,names[i],1); int v=s_levels[i]; if(v>100)v=100;
        line(x+5,62,x+5,169,true); line(x+1,169,x+9,169,true); line(x+1,62,x+9,62,true);
        int y=168-(v*100/100); fill_rect(x+3,y,5,169-y,true);
        if(i>=4){char db[8]; if(s_output_dbfs[i-4] <= -100.0f) snprintf(db,sizeof(db),"-INF"); else snprintf(db,sizeof(db),"%d",(int)lroundf(s_output_dbfs[i-4])); text(x-7,171,db,1);}}
    if(s_mode==DISPLAY_MODE_STANDALONE) {
        text(8,177,"INPUTS PCM1865 4CH",1);
        text(8,188,s_standalone_playing?s_standalone_track:"L/R PAGES",1);
        char vol[16]; snprintf(vol,sizeof(vol),"VOL %u%%",s_standalone_volume); text(144,177,vol,1);
    } else {

        for(int x=8;x<198;x+=6) line(x,89,x+3,89,true);

        static const char *gain_names[7]={"OUT","I1","I2","I3","I4","O1","O2"};
        char gain_status[32];
        snprintf(gain_status,sizeof(gain_status),"%s %+dDB CLIP 80%%",gain_names[s_gain_channel%7],(int)s_gain_db);
        text(8,188,gain_status,1);

        char main_vol[16]; snprintf(main_vol,sizeof(main_vol),"VOL %u%%",s_home_volume);
        text(144,188,main_vol,1);
    }
}
// E-paper RAM window diffing with partial refresh cuz fast, like 2hz
static void set_full_ram_area(void) {
    cmd(0x11); dat(0x03);
    cmd(0x44); dat(0); dat((W/8)-1);
    cmd(0x45); dat(0); dat(0); dat(H-1); dat(0);
    cmd(0x4E); dat(0);
    cmd(0x4F); dat(0); dat(0);
}
static void transfer_fb(uint8_t ram_cmd, const uint8_t *fb) {
    set_full_ram_area();
    cmd(ram_cmd);
    gpio_set_level(PIN_DC, 1);
    for(int offset=0;offset<FB_BYTES;){
        size_t n=FB_BYTES-offset;
        if(n>64)n=64;
        spi_transaction_t t={.length=n*8,.tx_buffer=fb+offset};
        ESP_ERROR_CHECK(spi_device_transmit(s_spi,&t));
        offset+=(int)n;
    }
}
static uint8_t s_previous_fb[FB_BYTES];
static bool fb_equal(const uint8_t *a, const uint8_t *b) {
    for (int i = 0; i < FB_BYTES; i++) if (a[i] != b[i]) return false;
    return true;
}
static void refresh(bool full) {
    if(full) {
        reset_panel();
        cmd(0x12); vTaskDelay(pdMS_TO_TICKS(10)); wait_busy();
        cmd(0x01); dat(0xC7); dat(0x00); dat(0x00);
        cmd(0x3C); dat(0x05);
        cmd(0x18); dat(0x80);
        memset(s_previous_fb, 0xff, sizeof(s_previous_fb));
        transfer_fb(0x26, s_previous_fb);
        transfer_fb(0x24, s_fb);
        cmd(0x22); dat(0xF7); cmd(0x20); wait_busy();
        memcpy(s_previous_fb, s_fb, sizeof(s_fb));
        return;
    }
    transfer_fb(0x24, s_fb);
    cmd(0x22); dat(0xFC); cmd(0x20); wait_busy();

    transfer_fb(0x26, s_fb);
    memcpy(s_previous_fb, s_fb, sizeof(s_fb));
}
// debounce (like clicks vs holding button)
static void key_task(void *arg){
    (void)arg;
    const gpio_num_t pins[9]={KEY_LEFT,KEY_RIGHT,KEY_UP,KEY_DOWN,KEY_PLAY,KEY_REC,KEY_NEXT,KEY_PREVIOUS,KEY_MONITOR};
    int stable[10], candidate[10], samples[10]={0};
    int64_t pressed_at[10]={0};
    bool long_action[10]={false};
    int enc_prev=(gpio_get_level(ENCODER_A)<<1)|gpio_get_level(ENCODER_B);
    int enc_accum=0;
    int64_t click_deadline=0;
    bool click_pending=false;

    bool side_seek_fired[2]={false,false};
    int64_t side_last_repeat_us[2]={0,0};
    for(int i=0;i<9;i++) stable[i]=candidate[i]=gpio_get_level(pins[i]);
    stable[9]=candidate[9]=gpio_get_level(ENCODER_SW);
    for(;;){
        int64_t now=esp_timer_get_time();
        for(int i=0;i<10;i++){
            if(i==9) {
                int level=gpio_get_level(ENCODER_SW);
                if(level!=candidate[i]){candidate[i]=level;samples[i]=1;}
                else if(samples[i]<3)samples[i]++;
                if(samples[i]>=3 && stable[i]!=candidate[i]) {
                    stable[i]=candidate[i];
                    if(!stable[i]) { pressed_at[i]=now; long_action[i]=false; }
                    else if(!long_action[i]) {
                        if(click_pending && now<click_deadline) {
                            click_pending=false;
                            if(s_control_cb)s_control_cb(DISPLAY_CONTROL_ENCODER_DOUBLE,0);
                        } else { click_pending=true; click_deadline=now+350000; }
                    }
                }
                continue;
            }
            int level=gpio_get_level(pins[i]);
            if(level!=candidate[i]){candidate[i]=level;samples[i]=1;}
            else if(samples[i]<3)samples[i]++;
            if(samples[i]>=3 && stable[i]!=candidate[i]){
                stable[i]=candidate[i];
                if(!stable[i]){
                    pressed_at[i]=now;
                    long_action[i]=false;
                    if(i>=4 && s_control_cb) {
                        if(pins[i]==KEY_PLAY)s_control_cb(DISPLAY_CONTROL_PLAY_PAUSE,1);
                        else if(pins[i]==KEY_REC)s_control_cb(DISPLAY_CONTROL_REC,1);
                        else if(pins[i]==KEY_NEXT && s_mode!=DISPLAY_MODE_STANDALONE)s_control_cb(DISPLAY_CONTROL_NEXT,1);
                        else if(pins[i]==KEY_PREVIOUS && s_mode!=DISPLAY_MODE_STANDALONE)s_control_cb(DISPLAY_CONTROL_PREVIOUS,1);

                    } else if((pins[i]==KEY_LEFT || pins[i]==KEY_RIGHT) && s_mode!=DISPLAY_MODE_STANDALONE){
                        bool forward=pins[i]==KEY_RIGHT;
                        s_menu=forward?(uint8_t)((s_menu+1)%3):(uint8_t)((s_menu+2)%3);
                        s_dirty=true;
                        const char *menu_name=s_menu==0?"main screen":s_menu==1?"audio stats":"spectrum";
                        ESP_LOGI(TAG,"GPIO %d pressed; showing %s",(int)pins[i],menu_name);
                    }
                }else if(s_menu==2 && (pins[i]==KEY_UP || pins[i]==KEY_DOWN) && !long_action[i]){
                    int next=s_spectrum_sensitivity_db+(pins[i]==KEY_UP?3:-3);
                    if(next<0)next=0;
                    if(next>24)next=24;
                    if(next!=s_spectrum_sensitivity_db){
                        s_spectrum_sensitivity_db=(int8_t)next;
                        nvs_handle_t nvs;
                        esp_err_t err=nvs_open("display",NVS_READWRITE,&nvs);
                        if(err==ESP_OK){
                            err=nvs_set_i8(nvs,"spec_sens",s_spectrum_sensitivity_db);
                            if(err==ESP_OK)err=nvs_commit(nvs);
                            nvs_close(nvs);
                        }
                        if(err!=ESP_OK)ESP_LOGW(TAG,"Could not save spectrum sensitivity: %s",esp_err_to_name(err));
                        s_dirty=true;
                        ESP_LOGI(TAG,"Spectrum sensitivity %+d dB",(int)s_spectrum_sensitivity_db);
                    }
                } else if(s_mode==DISPLAY_MODE_STANDALONE && s_standalone_page==DISPLAY_STANDALONE_EQ &&
                          (pins[i]==KEY_UP || pins[i]==KEY_DOWN)) {
                    int next=s_spectrum_sensitivity_db+(pins[i]==KEY_UP?3:-3);
                    if(next<0)next=0;
                    if(next>24)next=24;
                    if(next!=s_spectrum_sensitivity_db) {
                        s_spectrum_sensitivity_db=(int8_t)next;
                        nvs_handle_t nvs;
                        esp_err_t err=nvs_open("display",NVS_READWRITE,&nvs);
                        if(err==ESP_OK) {
                            err=nvs_set_i8(nvs,"spec_sens",s_spectrum_sensitivity_db);
                            if(err==ESP_OK)err=nvs_commit(nvs);
                            nvs_close(nvs);
                        }
                        s_dirty=true;
                    }
                } else if(s_mode==DISPLAY_MODE_STANDALONE && s_standalone_key_cb) {
                    if(!long_action[i]) {
                        display_key_t key = pins[i]==KEY_UP?DISPLAY_KEY_UP:pins[i]==KEY_DOWN?DISPLAY_KEY_DOWN:pins[i]==KEY_LEFT?DISPLAY_KEY_LEFT:DISPLAY_KEY_RIGHT;
                        s_standalone_key_cb(key, false);
                        s_dirty=true;
                    }
                }
                if(stable[i] && (i==6 || i==7) && s_control_cb) {
                    if(s_mode==DISPLAY_MODE_STANDALONE) {

                        if(!side_seek_fired[i==6?0:1])
                            s_control_cb(i==6?DISPLAY_CONTROL_NEXT:DISPLAY_CONTROL_PREVIOUS,0);
                        side_seek_fired[i==6?0:1]=false;
                    }

                }
                if(stable[i] && i==4 && s_control_cb)s_control_cb(DISPLAY_CONTROL_PLAY_PAUSE,0);
                if(stable[i] && i==5 && s_control_cb)s_control_cb(DISPLAY_CONTROL_REC,0);
            }
        }
        if(s_mode==DISPLAY_MODE_STANDALONE) {
            for(int i=0;i<2;i++) {
                int k=i==0?0:1;
                if(stable[k]==0 && !long_action[k] && now-pressed_at[k]>=500000) {
                    long_action[k]=true;
                    if(s_standalone_key_cb) s_standalone_key_cb(k==0?DISPLAY_KEY_LEFT:DISPLAY_KEY_RIGHT,true);
                    s_dirty=true;
                }
            }
        }

        if(s_mode==DISPLAY_MODE_STANDALONE && stable[8]==0 && !long_action[8] && now-pressed_at[8]>=600000) {
            long_action[8]=true;
            if(s_control_cb)s_control_cb(DISPLAY_CONTROL_MONITOR,1);
            s_dirty=true;
        }

        if(s_mode==DISPLAY_MODE_STANDALONE && s_control_cb) {
            for(int k=0;k<2;k++) {
                int i=k==0?6:7;
                if(stable[i]==0 && now-pressed_at[i]>=700000) {
                    if(!side_seek_fired[k]) {
                        side_seek_fired[k]=true;
                        side_last_repeat_us[k]=now;
                        s_control_cb(k==0?DISPLAY_CONTROL_SEEK_FWD:DISPLAY_CONTROL_SEEK_BACK,1);
                    } else if(now-side_last_repeat_us[k]>=1000000) {
                        side_last_repeat_us[k]=now;
                        s_control_cb(k==0?DISPLAY_CONTROL_SEEK_FWD:DISPLAY_CONTROL_SEEK_BACK,1);
                    }
                }
            }
        }
        if(stable[9]==0 && !long_action[9] && now-pressed_at[9]>=500000) {
            long_action[9]=true; click_pending=false;
            if(s_control_cb)s_control_cb(DISPLAY_CONTROL_ENCODER_HOLD,0);
        }
        if(click_pending && now>=click_deadline) {
            click_pending=false;
            if(s_control_cb)s_control_cb(DISPLAY_CONTROL_ENCODER_CLICK,0);
        }
        int enc_now=(gpio_get_level(ENCODER_A)<<1)|gpio_get_level(ENCODER_B);
        static const int8_t transition[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
        enc_accum+=transition[(enc_prev<<2)|enc_now]; enc_prev=enc_now;

        while(enc_accum>=4 || enc_accum<=-4) {
            int direction=enc_accum>0?1:-1;
            enc_accum+=direction*-4;
            if(s_control_cb)s_control_cb(DISPLAY_CONTROL_ENCODER_STEP,direction);
        }
        if(s_menu==2){
            for(int i=0;i<4;i++){
                if(stable[i]==0 && !long_action[i] && (pins[i]==KEY_UP || pins[i]==KEY_DOWN) && now-pressed_at[i]>=500000){
                    uint8_t source=display_spectrum_source();
                    source=(uint8_t)((source+(pins[i]==KEY_UP?1:5))%6);
                    display_select_spectrum_source(source);
                    long_action[i]=true;
                    static const char *source_names[6]={"O1","O2","I1","I2","I3","I4"};
                    ESP_LOGI(TAG,"Spectrum source: %s",source_names[source]);
                }
            }
        }
        if(s_mode==DISPLAY_MODE_STANDALONE && s_standalone_page==DISPLAY_STANDALONE_EQ) {
            for(int i=0;i<4;i++) {
                if(stable[i]==0 && !long_action[i] && (pins[i]==KEY_UP || pins[i]==KEY_DOWN) &&
                   now-pressed_at[i]>=500000) {
                    uint8_t source=display_spectrum_source();
                    source=(uint8_t)((source+(pins[i]==KEY_UP?1:5))%6);
                    display_select_spectrum_source(source);
                    long_action[i]=true;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
//2hz refresh
static void display_task(void *arg)
{
    (void)arg;
    bool first=true;
    TickType_t last=0;
    for(;;) {
        TickType_t now=xTaskGetTickCount();
        bool mic_active=s_recording_active || s_monitor_on;
        if(s_dirty && !(s_reduced_motion && mic_active) &&
           (first || now-last>=pdMS_TO_TICKS(500))) {
            s_dirty=false;
            draw_main();
            last=now;
            if(first || !fb_equal(s_previous_fb,s_fb)) refresh(first);
            first=false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
// NVS prefs and init
void display_start(display_mode_t mode) {
    s_mode=mode; memset(s_fb,0xff,sizeof(s_fb));
    nvs_handle_t nvs;
    if(nvs_open("display",NVS_READWRITE,&nvs)==ESP_OK){
        int8_t saved;
        if(nvs_get_i8(nvs,"spec_sens",&saved)==ESP_OK){
            if(saved<0) saved=0;
            if(saved>24) saved=24;
            s_spectrum_sensitivity_db=saved;
        }
        saved=0;
        if(nvs_get_i8(nvs,"red_motion",&saved)==ESP_OK)
            s_reduced_motion=saved!=0;
        nvs_close(nvs);
    }

    gpio_config_t out={.pin_bit_mask=(1ULL<<PIN_RST)|(1ULL<<PIN_DC),.mode=GPIO_MODE_OUTPUT}; ESP_ERROR_CHECK(gpio_config(&out));
    gpio_config_t in={.pin_bit_mask=1ULL<<PIN_BUSY,.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE}; ESP_ERROR_CHECK(gpio_config(&in));
    gpio_config_t keys={.pin_bit_mask=(1ULL<<KEY_LEFT)|(1ULL<<KEY_RIGHT)|(1ULL<<KEY_UP)|(1ULL<<KEY_DOWN)|(1ULL<<KEY_PLAY)|(1ULL<<KEY_REC)|(1ULL<<KEY_NEXT)|(1ULL<<KEY_PREVIOUS)|(1ULL<<KEY_MONITOR)|(1ULL<<ENCODER_SW),.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE}; ESP_ERROR_CHECK(gpio_config(&keys));
    gpio_config_t encoder={.pin_bit_mask=(1ULL<<ENCODER_A)|(1ULL<<ENCODER_B),.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE}; ESP_ERROR_CHECK(gpio_config(&encoder));
    spi_bus_config_t bus={.mosi_io_num=PIN_MOSI,.miso_io_num=-1,.sclk_io_num=PIN_SCLK,.quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=64}; ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST,&bus,SPI_DMA_DISABLED));
    spi_device_interface_config_t dev={.clock_speed_hz=4*1000*1000,.mode=0,.spics_io_num=PIN_CS,.queue_size=1}; ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST,&dev,&s_spi));
    ESP_LOGI(TAG,"Starting 200x200 SSD1681 display");
    configASSERT(xTaskCreate(display_task,"epaper",4096,NULL,3,NULL)==pdPASS);
    configASSERT(xTaskCreate(key_task,"display_keys",2048,NULL,2,NULL)==pdPASS);
}

void display_set_mode(display_mode_t mode){s_mode=mode;s_menu=0;s_dirty=true;}
void display_set_io(display_io_t state){s_io=state;s_dirty=true;}
void display_set_storage(uint64_t total,uint64_t free_bytes,bool mounted){s_total=total;s_free=free_bytes;s_mounted=mounted;s_dirty=true;}
void display_set_audio_stats(const display_audio_stats_t *s){if(s) s_stats=*s;s_dirty=true;}
void display_set_usb_stats(const display_usb_stats_t *s){if(s) s_usb_stats=*s;s_dirty=true;}
void display_set_clipping(bool clipping){s_stats.clipping=clipping;s_dirty=true;}
void display_set_audio_levels(const uint8_t levels[6]){if(levels)memcpy(s_levels,levels,6);s_dirty=true;}

void display_set_spectrum(const float dbfs[16]){
    if(dbfs)memcpy(s_spectrum_dbfs,dbfs,sizeof(s_spectrum_dbfs));
    if(display_spectrum_active())s_dirty=true;
}
bool display_spectrum_active(void){return s_menu==2 || (s_mode==DISPLAY_MODE_STANDALONE && s_standalone_page==DISPLAY_STANDALONE_EQ);}
void display_select_spectrum_source(uint8_t source){
    s_spectrum_source=source%6;
    memset(s_spectrum_dbfs,0,sizeof(s_spectrum_dbfs));
    for(int i=0;i<16;i++)s_spectrum_dbfs[i]=-120.0f;
    s_dirty=true;
}
uint8_t display_spectrum_source(void){return s_spectrum_source;}
// meter setting
void display_set_output_meter(float o1_dbfs,float o2_dbfs,bool clipping){
    s_output_dbfs[0]=o1_dbfs; s_output_dbfs[1]=o2_dbfs;
    for(int i=0;i<2;i++){
        float db=s_output_dbfs[i];
        if (db < -60.0f) {
            db = -60.0f;
        }
        if (db > 0.0f) {
            db = 0.0f;
        }
        s_levels[4+i]=(uint8_t)((db+60.0f)*100.0f/60.0f);
    }
    s_stats.clipping=clipping; s_dirty=true;
}
void display_set_channel_gain(uint8_t channel,int8_t gain_db){

    s_gain_channel=channel%7; s_gain_db=gain_db; s_dirty=true;
}

void display_set_main_volume(uint8_t percent){
    if(percent>100)percent=100;
    s_home_volume=percent; s_dirty=true;
}
bool display_advanced_page(void){return s_menu==1;}
void display_set_standalone_key_callback(display_standalone_key_cb_t callback){s_standalone_key_cb=callback;}
void display_set_control_callback(display_control_cb_t callback){s_control_cb=callback;}
// state setters / menu thing
void display_set_standalone_view(const char *path,const char *const *entries,uint8_t count,uint8_t selected,bool playing,uint8_t volume,display_standalone_page_t page,bool paused,const char *track,const char *status){
    snprintf(s_standalone_path,sizeof(s_standalone_path),"%s",path?path:"/");
    s_standalone_count=count>8?8:count; s_standalone_selected=selected; s_standalone_playing=playing; s_standalone_volume=volume; s_standalone_page=page; s_standalone_paused=paused;
    snprintf(s_standalone_track,sizeof(s_standalone_track),"%s",track?track:"");
    if(status) snprintf(s_standalone_status,sizeof(s_standalone_status),"%s",status);
    for(uint8_t i=0;i<s_standalone_count;i++) snprintf(s_standalone_entries[i],sizeof(s_standalone_entries[i]),"%s",entries[i]);
    s_dirty=true;
}
void display_set_standalone_track_info(const char *format,uint32_t sample_rate,uint32_t bitrate,uint8_t bits,uint8_t channels,uint64_t file_size,uint32_t elapsed_ms,uint32_t duration_ms,bool estimated){
    snprintf(s_standalone_format,sizeof(s_standalone_format),"%s",format?format:"-");
    s_standalone_rate=sample_rate; s_standalone_bitrate=bitrate; s_standalone_bits=bits; s_standalone_channels=channels;
    s_standalone_file_size=file_size; s_standalone_elapsed_ms=elapsed_ms; s_standalone_duration_ms=duration_ms; s_standalone_duration_estimated=estimated;
    if(s_standalone_page==DISPLAY_STANDALONE_INFO || s_standalone_page==DISPLAY_STANDALONE_LISTEN)s_dirty=true;
}

void display_set_recording_track(const char *filename)
{
    snprintf(s_standalone_track, sizeof(s_standalone_track), "%s", filename ? filename : "");
    s_dirty = true;
}

void display_set_recording_active(bool active)
{
    s_recording_active = active;
    s_dirty = true;
}

// this settign is to fix the tiny audable beeps caused by display refresh
bool display_set_reduced_motion(bool enabled)
{
    s_reduced_motion=enabled;
    nvs_handle_t nvs;
    esp_err_t err=nvs_open("display",NVS_READWRITE,&nvs);
    if(err==ESP_OK) {
        err=nvs_set_i8(nvs,"red_motion",enabled?1:0);
        if(err==ESP_OK) err=nvs_commit(nvs);
        nvs_close(nvs);
    }
    if(err!=ESP_OK) ESP_LOGW(TAG,"Could not save reduced screen motion: %s",esp_err_to_name(err));
    s_dirty=true;
    return err==ESP_OK;
}

bool display_reduced_motion(void)
{
    return s_reduced_motion;
}

void display_set_standalone_status(const char *status)
{
    snprintf(s_standalone_status, sizeof(s_standalone_status), "%s", status ? status : "");
    s_dirty = true;
}

void display_set_monitor_view(bool on, uint8_t yaw, const float wxyz_db[4],
                              const float out_db[2], bool clip)
{
    s_monitor_on = on;
    s_monitor_yaw = yaw;
    if (wxyz_db) memcpy(s_monitor_wxyz_db, wxyz_db, sizeof(s_monitor_wxyz_db));
    if (out_db) memcpy(s_monitor_out_db, out_db, sizeof(s_monitor_out_db));
    s_monitor_clip = clip;
    s_dirty = true;
}

void display_set_settings_view(display_settings_menu_t menu, uint8_t selected,
                               uint8_t count, const char *const *items,
                               const char *title, const char *hint)
{
    s_settings.menu = menu;
    s_settings.selected = selected;
    s_settings.count = count > 8 ? 8 : count;
    snprintf(s_settings.title, sizeof(s_settings.title), "%s", title ? title : "");
    snprintf(s_settings.hint, sizeof(s_settings.hint), "%s", hint ? hint : "");
    for (uint8_t i = 0; i < s_settings.count; i++) {
        snprintf(s_settings.items[i], sizeof(s_settings.items[i]), "%s",
                 items[i] ? items[i] : "");
    }
    s_dirty = true;
}

void display_set_tap_view(bool on, const uint8_t levels[4],
                          const char *const labels[4], uint8_t count)
{
    s_tap_active = on;
    s_tap_count = count > 4 ? 4 : count;
    if (levels) memcpy(s_tap_levels, levels, sizeof(s_tap_levels));
    if (labels) {
        for (int i = 0; i < s_tap_count; i++) {
            snprintf(s_tap_labels[i], sizeof(s_tap_labels[i]), "%s",
                     labels[i] ? labels[i] : "");
        }
    }
    s_dirty = true;
}

void display_set_recording_labels(const char *l1, const char *l2,
                                  const char *l3, const char *l4, uint8_t channels)
{
    snprintf(s_rec_labels[0], sizeof(s_rec_labels[0]), "%s", l1 ? l1 : "I1");
    snprintf(s_rec_labels[1], sizeof(s_rec_labels[1]), "%s", l2 ? l2 : "");
    snprintf(s_rec_labels[2], sizeof(s_rec_labels[2]), "%s", l3 ? l3 : "");
    snprintf(s_rec_labels[3], sizeof(s_rec_labels[3]), "%s", l4 ? l4 : "");
    s_rec_channels = channels;
    s_dirty = true;
}
