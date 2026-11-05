// Standalone main file

#include "Standalone.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <math.h>
#include <sys/stat.h>

#include "Display.h"
#include "EqApo.h"
#include "InputsPreset.h"
#include "PCM1865.h"
#include "Recording.h"
#include "StatusLeds.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_default_configs.h"
#include "led_strip.h"
#include "decoder/impl/esp_mp3_dec.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_ldo_regulator.h"
#include "esp_timer.h"
#include "nvs.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

#define SD_MOUNT_POINT "/sdcard"
#define ENTRY_MAX 64
#define ENTRY_NAME 256
#define DISPLAY_ROWS 6
#define AUDIO_FRAMES 512
#define SPECTRUM_N 4096
#define SPECTRUM_BANDS 16
#define MP3_INPUT_BYTES 8192
#define MP3_OUTPUT_BYTES 8192
#define I2S_BCLK GPIO_NUM_4
#define I2S_WS GPIO_NUM_11
#define I2S_DOUT GPIO_NUM_5

#define MON_DAC_RATE 96000
#define MON_BLOCK 256
#define MON_BCK GPIO_NUM_13
#define MON_WS GPIO_NUM_14
#define MON_DIN GPIO_NUM_12
#define MON_DMA_DESC 6
#define MON_DMA_FRAME 80
#define MON_ITD 64
#define MON_MASTER_GAIN 0.5f
#define MON_Z_MIX 0.25f
#define EQ_FILE_PATH SD_MOUNT_POINT "/eq.txt"
#define RESET_CONFIRM_WINDOW_MS 5000UL

static const char *TAG = "standalone";
typedef struct { char name[ENTRY_NAME]; bool directory; } file_entry_t;
typedef enum { CMD_UP, CMD_DOWN, CMD_LEFT, CMD_RIGHT, CMD_STOP, CMD_REC_TOGGLE, CMD_ENCODER_STEP, CMD_MONITOR_WAKE, CMD_MONITOR_TOGGLE, CMD_RESET_CONFIRM,
               CMD_PLAY_PAUSE, CMD_SKIP_FWD, CMD_SKIP_BACK, CMD_SEEK_FWD, CMD_SEEK_BACK, CMD_ENCODER_CLICK } command_key_t;
typedef struct { command_key_t key; bool long_press; } key_command_t;

static void monitor_set(bool on);
static void monitor_yaw_step(int direction);
static esp_err_t audio_init(uint32_t rate, uint32_t mclk_multiple, bool low_latency);
static void audio_output_deinit(void);
static esp_err_t monitor_output_init(bool low_latency);
static void publish_view(void);
static void cycle_page(int direction);
static void settings_reset_cancel(void);
static void tap_test_set(bool on);

static file_entry_t s_entries[ENTRY_MAX];
static char s_path[256] = SD_MOUNT_POINT;
static char s_track_path[520];
static uint8_t s_entry_count, s_selected, s_volume = 80;
static volatile bool s_playing;
static bool s_mounted;
static bool s_request_play;
static volatile bool s_stop_requested;
static volatile bool s_paused;
static bool s_player_focus;
static display_standalone_page_t s_page = DISPLAY_STANDALONE_HOME;
static char s_status[32] = "INSERT SD CARD";
static QueueHandle_t s_commands;
static SemaphoreHandle_t s_player_stopped;
static volatile bool s_player_task_running;
static sdmmc_card_t *s_card;
static esp_ldo_channel_handle_t s_sd_ldo;
static i2s_chan_handle_t s_tx;
static bool s_tx_low_latency;
static led_strip_handle_t s_led;
static esp_timer_handle_t s_led_timer;
static float s_gain = 0.8f;
static int8_t s_channel_gain_db[6];
static int32_t s_spectrum_samples[SPECTRUM_N];
static float s_spectrum_window[SPECTRUM_N];
static float s_spectrum_dbfs[SPECTRUM_BANDS];
static uint32_t s_spectrum_write, s_spectrum_count, s_spectrum_samples_since_update;
static uint32_t s_spectrum_rate;
static float s_spectrum_window_sum;
static float s_bass_lowpass[2], s_bass_dc[2];
static uint32_t s_bass_rate;
static volatile float s_bass_dbfs=-120.0f;
static char s_info_format[12]="WAV PCM";
static uint32_t s_info_rate, s_info_bitrate;
static uint8_t s_info_bits, s_info_channels;
static uint64_t s_info_file_size;
static uint32_t s_info_elapsed_ms, s_info_duration_ms;
static bool s_info_duration_estimated;
static char s_resume_path[520];
static uint32_t s_resume_elapsed_ms;
static uint32_t s_resume_byte_offset;
static bool s_resume_mp3;
static bool s_resume_available;
static bool s_resume_request;

static volatile bool s_seek_request;
static volatile int32_t s_seek_delta_ms;

static bool s_volume_dirty;
static int64_t s_volume_changed_us;
#define VOLUME_SAVE_DELAY_US 400000

#define READER_CHUNK_BYTES (1024 * 32)
typedef struct {
    uint8_t *data;
    size_t len;
    uint32_t epoch;
} reader_block_t;
typedef struct {
    reader_block_t blk;
    size_t pos;
    bool have;
} reader_stream_t;

static QueueHandle_t s_reader_free, s_reader_ready;
static FILE *s_reader_file;
static volatile uint32_t s_reader_epoch;
static volatile long s_reader_seek_offset;
static volatile bool s_reader_seek_req;
static volatile bool s_reader_running;
static TaskHandle_t s_reader_task;
static SemaphoreHandle_t s_reader_stopped;
static volatile uint32_t s_reader_reads, s_reader_short, s_reader_stalls;
static uint8_t *s_reader_slot[2];

typedef enum { MON_STOP } mon_command_t;

typedef struct {
    float c, p;
    float gl, gr;
    int dl, dr;
    float buf[MON_ITD + 1];
    int wp;
} mon_speaker_t;

static mon_speaker_t s_spk[4] = {
    { 0.70710678f,  0.70710678f, 0, 0, 0, 0, {0}, 0},
    { 0.70710678f, -0.70710678f, 0, 0, 0, 0, {0}, 0},
    {-0.70710678f,  0.70710678f, 0, 0, 0, 0, {0}, 0},
    {-0.70710678f, -0.70710678f, 0, 0, 0, 0, {0}, 0},
};

static volatile bool s_monitor_on;
static i2s_chan_handle_t s_mon_rx;
static QueueHandle_t s_mon_queue;
static SemaphoreHandle_t s_mon_stopped;
static bool s_mon_task_running;
static int8_t s_monitor_yaw;
static int32_t s_mon_dma[MON_BLOCK * 4];
static int32_t s_mon_out[MON_BLOCK * 2 * 2];
static float s_mon_wxyz_db[4] = {-120, -120, -120, -120};
static float s_mon_out_db[2] = {-120, -120};
static float s_mon_meter_sq[6];
static bool s_mon_clip;
static uint8_t s_wav_input[AUDIO_FRAMES * 4 * sizeof(uint32_t)];
typedef struct {
    uint16_t frames;
    int32_t samples[MON_BLOCK * 4];
} recording_monitor_block_t;
static QueueHandle_t s_record_monitor_queue;
static SemaphoreHandle_t s_record_monitor_stopped;
static SemaphoreHandle_t s_record_monitor_mutex;
static uint32_t s_record_monitor_drops;

static bool s_mon_decode = true;
static bool s_mon_eq_on;
static bool s_wav_ambi_decode = true;
static bool s_eq_loaded;
static char s_eq_browser_hint[40] = "CLICK DIR / EQ FILE";
static char s_eq_path[256] = EQ_FILE_PATH;
static char s_eq_browser_path[256] = SD_MOUNT_POINT;
static file_entry_t s_eq_entries[ENTRY_MAX];
static uint8_t s_eq_entry_count, s_eq_selected;
static bool s_mon_meter_only;
static bool s_record_monitor_enabled;
static volatile bool s_recording_monitor_on;
static volatile bool s_record_monitor_task_running;
static bool s_recording_start_pending;
static bool s_recording_monitor_pending;
static bool s_reset_confirm;
static int64_t s_reset_deadline_us;
static int s_settings_menu = SETTINGS_MENU_ROOT;
static int s_settings_sel;
static char s_eq_status[24] = "EQ.TXT NOT LOADED";
static char s_set_rows[8][44];
static const char *s_set_ptrs[8];

// Track file filters
static bool ends_with_wav(const char *name)
{
    size_t n = strlen(name);
    return n >= 4 && name[n-4] == '.' && tolower((unsigned char)name[n-3]) == 'w' &&
           tolower((unsigned char)name[n-2]) == 'a' && tolower((unsigned char)name[n-1]) == 'v';
}

static bool ends_with_mp3(const char *name)
{
    size_t n=strlen(name);
    return n>=4 && name[n-4]=='.' && tolower((unsigned char)name[n-3])=='m' &&
           tolower((unsigned char)name[n-2])=='p' && tolower((unsigned char)name[n-1])=='3';
}

static bool supported_audio(const char *name)
{
    return ends_with_wav(name) || ends_with_mp3(name);
}

// track info (format, rate, kbps, elapsed/total)
static void publish_track_info(void)
{
    display_set_standalone_track_info(s_info_format,s_info_rate,s_info_bitrate,s_info_bits,
                                      s_info_channels,s_info_file_size,s_info_elapsed_ms,
                                      s_info_duration_ms,s_info_duration_estimated);
}

// add sample energy to 16-band spectrum
static void standalone_spectrum_add_sample(uint8_t source, int32_t sample, uint32_t rate)
{
    if(!display_spectrum_active() || source!=display_spectrum_source()) return;
    if(s_spectrum_rate!=rate) {
        s_spectrum_rate=rate;
        s_spectrum_write=0;
        s_spectrum_count=0;
        s_spectrum_samples_since_update=0;
    }
    s_spectrum_samples[s_spectrum_write]=sample;
    s_spectrum_write=(s_spectrum_write+1)%SPECTRUM_N;
    if(s_spectrum_count<SPECTRUM_N) s_spectrum_count++;
    if(++s_spectrum_samples_since_update<rate/2) return;
    s_spectrum_samples_since_update=0;
    if(s_spectrum_count<SPECTRUM_N) return;
    static bool window_ready;
    if(!window_ready) {
        for(int i=0;i<SPECTRUM_N;i++) {
            s_spectrum_window[i]=0.5f-0.5f*cosf(6.28318530718f*(float)i/(float)(SPECTRUM_N-1));
            s_spectrum_window_sum+=s_spectrum_window[i];
        }
        window_ready=true;
    }
    for(int band=0;band<SPECTRUM_BANDS;band++) {
        float frequency=20.0f*powf(1000.0f,(float)band/(float)(SPECTRUM_BANDS-1));
        float coeff=2.0f*cosf(6.28318530718f*frequency/(float)rate), q1=0.0f, q2=0.0f;
        for(int i=0;i<SPECTRUM_N;i++) {
            uint32_t index=(s_spectrum_write+(uint32_t)i)%SPECTRUM_N;
            float value=(float)s_spectrum_samples[index]/8388607.0f;
            float q0=value*s_spectrum_window[i]+coeff*q1-q2;
            q2=q1; q1=q0;
        }
        float power=q1*q1+q2*q2-coeff*q1*q2;
        if(power<0.0f) power=0.0f;
        float amplitude=s_spectrum_window_sum>0.0f?2.0f*sqrtf(power)/s_spectrum_window_sum:0.0f;
        s_spectrum_dbfs[band]=amplitude>1.0e-6f?20.0f*log10f(amplitude):-120.0f;
    }
    display_set_spectrum(s_spectrum_dbfs);
}

static bool s_led_on, s_led_in_flight;

// led blinks to 808's (crudly)
static void standalone_led_timer_callback(void *arg)
{
    (void)arg;
    float bass_db=s_bass_dbfs;
    bool active=s_playing && !s_paused && bass_db>=-40.0f;
    if(!active && !s_led_on && !s_led_in_flight) return;
    if(s_led_in_flight) {
        if(led_strip_refresh_async_done(s_led)!=ESP_OK) return;
        s_led_in_flight=false;
    }
    s_led_on=active?!s_led_on:false;
    uint8_t red=0, blue=0;
    if(s_led_on) {
        float level=(bass_db+40.0f)/40.0f;
        if(level<0.0f) level=0.0f;
        if(level>1.0f) level=1.0f;
        uint8_t brightness=(uint8_t)(8.0f+level*72.0f);
        red=(uint8_t)(brightness*3/4);
        blue=brightness;
    }
    led_strip_set_pixel(s_led,0,red,0,blue);
    if(led_strip_refresh_async(s_led)==ESP_OK) s_led_in_flight=true;
}

static esp_err_t standalone_led_init(void)
{
    const led_strip_config_t strip={
        .strip_gpio_num=GPIO_NUM_51,
        .max_leds=1,
        .led_model=LED_MODEL_WS2812,
        .color_component_format=LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out=false,
    };
    const led_strip_rmt_config_t rmt={
        .clk_src=RMT_CLK_SRC_DEFAULT,
        .resolution_hz=10*1000*1000,
        .flags.with_dma=false,
    };
    esp_err_t err=led_strip_new_rmt_device(&strip,&rmt,&s_led);
    if(err==ESP_OK) err=led_strip_clear(s_led);
    return err;
}

// W/X/Y/Z delay lines and per tap filter 
static inline void mon_a2b(const int32_t *f, float norm, float out[4])
{
    const uint8_t *map = inp_mic_map();
    float a=(float)(f[map[0]] >> 8)*norm, b=(float)(f[map[1]] >> 8)*norm;
    float c=(float)(f[map[2]] >> 8)*norm, d=(float)(f[map[3]] >> 8)*norm;
    out[0]=0.5f*(a+b+c+d);
    out[1]=0.5f*(a+b-c-d);
    out[2]=0.5f*(a-b+c-d);
    out[3]=0.5f*(a-b-c+d);
}

static inline void mon_push(float w, float x, float y)
{
    for(int k=0;k<4;k++) {
        mon_speaker_t *s=&s_spk[k];
        s->buf[s->wp]=0.5f*(w+s->c*x+s->p*y);
        s->wp=(s->wp+1)%(MON_ITD+1);
    }
}

static inline float mon_tap(int k, bool left)
{
    const mon_speaker_t *s=&s_spk[k];
    int d=left?s->dl:s->dr;
    int idx=(s->wp-1-d+(MON_ITD+1))%(MON_ITD+1);
    float g=left?s->gl:s->gr;
    return g*s->buf[idx];
}

static inline void mon_raw(const int32_t *f, float norm, float out[4])
{
    out[0]=norm*(float)(f[0] >> 8);
    out[1]=norm*(float)(f[1] >> 8);
    out[2]=norm*(float)(f[2] >> 8);
    out[3]=norm*(float)(f[3] >> 8);
}

// per-capsule WXYZ dB levels and clip detect 
static void monitor_meter_process(const int32_t *in, size_t frames)
{
    const input_preset_t *p=inp_preset();
    const int pch=p->channels;
    const float norm=1.0f/8388608.0f;
    const float alpha=1.0f-expf(-(float)frames/(float)p->rate/0.1f);
    static float msq[4];
    double acc[4]={0,0,0,0};
    for(size_t i=0;i<frames;i++) {
        for(int ch=0;ch<pch;ch++) {
            float v=(float)(in[i*pch+ch]>>8)*norm;
            acc[ch]+=(double)v*v;
        }
    }
    for(int ch=0;ch<pch;ch++) {
        float target=(float)(acc[ch]/(double)frames);
        msq[ch]+=(target-msq[ch])*alpha;
    }
    uint8_t levels[4];
    const char *labels[4];
    for(int ch=0;ch<pch;ch++) {
        float db=msq[ch]>1.0e-12f?10.0f*log10f(msq[ch]):-120.0f;
        if(db<-60.0f) db=-60.0f;
        if(db>0.0f) db=0.0f;
        levels[ch]=(uint8_t)((db+60.0f)*(100.0f/60.0f));
        labels[ch]=p->slot_label[ch];
    }
    display_set_tap_view(true,levels,labels,(uint8_t)pch);
}

// routes the two captured slots through the same pipeline
static bool monitor_process_stereo(const int32_t *in, size_t frames)
{
    const float norm=1.0f/8388608.0f;
    const float g=MON_MASTER_GAIN*(float)s_volume/100.0f;
    size_t out_frames=frames/2;
    if(out_frames==0) return true;
    int32_t *outp=s_mon_out;
    double accL=0,accR=0;
    bool clip=false;
    for(size_t j=0;j<out_frames;j++) {
        const int32_t *f0=&in[(2*j)*2], *f1=&in[(2*j+1)*2];
        float l=(float)((f0[0]>>8)+(f1[0]>>8))*0.5f*norm;
        float r=(float)((f0[1]>>8)+(f1[1]>>8))*0.5f*norm;
        if(s_mon_eq_on) {
            eq_apply_float(0,&l,1,MON_DAC_RATE);
            eq_apply_float(1,&r,1,MON_DAC_RATE);
        }
        l*=g; r*=g;
        if(l>0.999f||l<-0.999f||r>0.999f||r<-0.999f) clip=true;
        int32_t lo=(int32_t)(l*8388607.0f), ro=(int32_t)(r*8388607.0f);
        if(lo> 8388607)lo= 8388607;
        if(lo<-8388608)lo=-8388608;
        if(ro> 8388607)ro= 8388607;
        if(ro<-8388608)ro=-8388608;
        outp[0]=lo<<8; outp[1]=ro<<8; outp+=2;
        accL+=(double)l*l; accR+=(double)r*r;
    }
    size_t produced=out_frames;
    size_t bytes=produced*2*sizeof(int32_t);
    size_t written=0;
    if(i2s_channel_write(s_tx,s_mon_out,bytes,&written,100)!=ESP_OK || written!=bytes) return false;

    float alpha=1.0f-expf(-(float)produced/(float)MON_DAC_RATE/0.25f);
    s_mon_meter_sq[0]=s_mon_meter_sq[1]=s_mon_meter_sq[2]=s_mon_meter_sq[3]=0.0f;
    s_mon_wxyz_db[0]=s_mon_wxyz_db[1]=s_mon_wxyz_db[2]=s_mon_wxyz_db[3]=-120.0f;
    s_mon_meter_sq[4]+=((float)(accL/(double)produced)-s_mon_meter_sq[4])*alpha;
    s_mon_meter_sq[5]+=((float)(accR/(double)produced)-s_mon_meter_sq[5])*alpha;
    for(int k=0;k<2;k++) {
        s_mon_out_db[k]=s_mon_meter_sq[4+k]>1.0e-12f?10.0f*log10f(s_mon_meter_sq[4+k]):-120.0f;
    }
    if(clip) status_leds_flash_clip();
    display_set_monitor_view(true,(uint8_t)(s_monitor_yaw&3),s_mon_wxyz_db,s_mon_out_db,clip);
    return true;
}

// AGC, per-channel gain, binaural decode or raw passthru
static bool monitor_process_internal(const int32_t *in, size_t frames,
                                     bool force_four_channel, bool input_wxyz,
                                     bool publish_meters)
{
    if(s_mon_meter_only) {
        monitor_meter_process(in,frames);
        return true;
    }

    if(!force_four_channel && inp_preset()->channels==2) {
        return monitor_process_stereo(in,frames);
    }

    static const float rot_c[4]={1.0f,0.0f,-1.0f,0.0f};
    static const float rot_s[4]={0.0f,1.0f,0.0f,-1.0f};
    const float rc=rot_c[s_monitor_yaw&3], rs=rot_s[s_monitor_yaw&3];
    const float norm=1.0f/8388608.0f;
    const float vol=(float)s_volume/100.0f;
    const bool raw=force_four_channel ? !s_wav_ambi_decode : !s_mon_decode;
    double acc[6]={0,0,0,0,0,0};
    bool clip=false;
    int32_t *outp=s_mon_out;

    float b0[4], b1[4], raw0[4], raw1[4];
    for(size_t i=0;i<frames;i++) {
        if(input_wxyz) {
            for(int ch=0;ch<4;ch++) b0[ch]=(float)(in[i*4+ch]>>8)*norm;
        } else {
            mon_a2b(&in[i*4],norm,b0);
        }
        mon_raw(&in[i*4],norm,raw0);
        bool has_next=i+1<frames;
        if(input_wxyz) {
            for(int ch=0;ch<4;ch++)
                b1[ch]=(float)(in[(has_next?i+1:i)*4+ch]>>8)*norm;
        } else {
            mon_a2b(&in[(has_next?i+1:i)*4],norm,b1);
        }
        mon_raw(&in[(has_next?i+1:i)*4],norm,raw1);
        for(int half=0;half<2;half++) {
            float w,x,y,z,rw[4];
            if(half==0) {
                w=b0[0]; x=b0[1]; y=b0[2]; z=b0[3];
                for(int k=0;k<4;k++) rw[k]=raw0[k];
            } else {
                w=0.5f*(b0[0]+b1[0]); x=0.5f*(b0[1]+b1[1]);
                y=0.5f*(b0[2]+b1[2]); z=0.5f*(b0[3]+b1[3]);
                for(int k=0;k<4;k++) rw[k]=0.5f*(raw0[k]+raw1[k]);
            }
            float xr=x*rc-y*rs, yr=x*rs+y*rc;
            float l, r;
            if(raw) {

                l=0.5f*(rw[0]+rw[1]);
                r=0.5f*(rw[2]+rw[3]);
            } else {
                mon_push(w,xr,yr);
                l=(mon_tap(0,true)+mon_tap(1,true)+mon_tap(2,true)+mon_tap(3,true))+MON_Z_MIX*z;
                r=(mon_tap(0,false)+mon_tap(1,false)+mon_tap(2,false)+mon_tap(3,false))+MON_Z_MIX*z;
            }

            if(s_mon_eq_on) {
                eq_apply_float(0,&l,1,MON_DAC_RATE);
                eq_apply_float(1,&r,1,MON_DAC_RATE);
            }
            float g=MON_MASTER_GAIN*vol;
            l*=g; r*=g;
            if(l>0.999f||l<-0.999f||r>0.999f||r<-0.999f) clip=true;
            int32_t lo=(int32_t)(l*8388607.0f), ro=(int32_t)(r*8388607.0f);
            if(lo> 8388607)lo= 8388607;
            if(lo<-8388608)lo=-8388608;
            if(ro> 8388607)ro= 8388607;
            if(ro<-8388608)ro=-8388608;
            outp[0]=lo<<8; outp[1]=ro<<8; outp+=2;
            acc[0]+=(double)w*w; acc[1]+=(double)xr*xr;
            acc[2]+=(double)yr*yr; acc[3]+=(double)z*z;
            acc[4]+=(double)l*l; acc[5]+=(double)r*r;
        }
    }

    size_t produced=frames*2;
    size_t bytes=produced*2*sizeof(int32_t);
    size_t written=0;
    if(i2s_channel_write(s_tx,s_mon_out,bytes,&written,100)!=ESP_OK || written!=bytes) return false;

    float alpha=1.0f-expf(-(float)produced/(float)MON_DAC_RATE/0.25f);
    for(int k=0;k<6;k++) {
        float msq=(float)(acc[k]/(double)produced);
        s_mon_meter_sq[k]+=(msq-s_mon_meter_sq[k])*alpha;
    }
    for(int k=0;k<4;k++) {
        s_mon_wxyz_db[k]=s_mon_meter_sq[k]>1.0e-12f?10.0f*log10f(s_mon_meter_sq[k]):-120.0f;
    }
    for(int k=0;k<2;k++) {
        s_mon_out_db[k]=s_mon_meter_sq[4+k]>1.0e-12f?10.0f*log10f(s_mon_meter_sq[4+k]):-120.0f;
    }
    if(clip) status_leds_flash_clip();
    if(publish_meters)
        display_set_monitor_view(true,(uint8_t)(s_monitor_yaw&3),
                                 s_mon_wxyz_db,s_mon_out_db,clip);
    return true;
}

// picks the 4ch or 2ch processor
static bool monitor_process(const int32_t *in, size_t frames)
{
    return monitor_process_internal(in,frames,false,false,true);
}

// streams the recording monitor mix while capturing 
static void recording_monitor_task(void *arg)
{
    (void)arg;
    recording_monitor_block_t block;
    while(xQueueReceive(s_record_monitor_queue,&block,portMAX_DELAY)==pdTRUE) {
        if(!block.frames) break;
        if(!monitor_process(block.samples,block.frames)) {
            ESP_LOGE(TAG,"Recording monitor DAC write failed");
            s_recording_monitor_on=false;
            s_monitor_on=false;
            display_set_standalone_status("REC MONITOR ERROR");
            break;
        }
    }
    display_set_monitor_view(false,(uint8_t)(s_monitor_yaw&3),
                             s_mon_wxyz_db,s_mon_out_db,false);
    xSemaphoreGive(s_record_monitor_stopped);
    vTaskDelete(NULL);
}

// capture channel
static esp_err_t monitor_i2s_init(void)
{
    const input_preset_t *p=inp_preset();
    i2s_chan_config_t cc=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO,I2S_ROLE_MASTER);
    cc.dma_desc_num=MON_DMA_DESC; cc.dma_frame_num=MON_DMA_FRAME;
    cc.auto_clear_after_cb=false;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&cc,NULL,&s_mon_rx),TAG,"mon rx channel");
    i2s_tdm_config_t cfg={
        .clk_cfg=I2S_TDM_CLK_DEFAULT_CONFIG(p->rate),
        .slot_cfg=I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                 I2S_SLOT_MODE_STEREO,
                                                 p->mask),
        .gpio_cfg={.mclk=GPIO_NUM_NC,.bclk=MON_BCK,.ws=MON_WS,.dout=GPIO_NUM_NC,.din=MON_DIN},
    };
    cfg.slot_cfg.slot_bit_width=I2S_SLOT_BIT_WIDTH_32BIT;
    cfg.slot_cfg.total_slot=p->total_slots;
    cfg.slot_cfg.ws_pol=1;
    cfg.slot_cfg.bit_shift=0;
    cfg.clk_cfg.clk_src=I2S_CLK_SRC_APLL;
    cfg.clk_cfg.mclk_multiple=p->mclk_mult==256u?I2S_MCLK_MULTIPLE_256:I2S_MCLK_MULTIPLE_768;
    esp_err_t err=i2s_channel_init_tdm_mode(s_mon_rx,&cfg);
    if(err==ESP_OK) err=i2s_channel_enable(s_mon_rx);
    if(err!=ESP_OK) {
        i2s_del_channel(s_mon_rx);
        s_mon_rx=NULL;
        return err;
    }

    gpio_pullup_dis(MON_DIN);
    gpio_pulldown_en(MON_DIN);
    ESP_LOGI(TAG,"Monitor ADC I2S ready: %lu Hz TDM, %u ch (%s)",
             (unsigned long)p->rate,p->channels,p->name);
    return ESP_OK;
}

// captures ADC frames
static void monitor_task(void *arg)
{
    (void)arg;
    uint8_t blocks_since_yield=0;
    for(;;) {
        mon_command_t cmd;
        if(xQueueReceive(s_mon_queue,&cmd,0)==pdTRUE && cmd==MON_STOP) break;
        size_t got=0;
        esp_err_t err=i2s_channel_read(s_mon_rx,s_mon_dma,sizeof(s_mon_dma),&got,pdMS_TO_TICKS(50));
        if(err!=ESP_OK && err!=ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG,"Monitor I2S read failed: %s",esp_err_to_name(err));
            break;
        }
        size_t frames=got/(inp_preset()->channels*(int)sizeof(int32_t));
        if(!frames) continue;
        if(frames>MON_BLOCK) frames=MON_BLOCK;
        if(!monitor_process(s_mon_dma,frames)) {
            ESP_LOGE(TAG,"Monitor DAC write failed; stopping");
            break;
        }
        if(++blocks_since_yield==32) {
            blocks_since_yield=0;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    if(s_mon_rx) {
        (void)i2s_channel_disable(s_mon_rx);
        (void)i2s_del_channel(s_mon_rx);
        s_mon_rx=NULL;
    }
    s_mon_task_running=false;
    if(s_mon_stopped) xSemaphoreGive(s_mon_stopped);
    vTaskDelete(NULL);
}

// releases output I2S channel once nothing needs it 
static void audio_output_deinit(void)
{
    if(!s_tx) return;
    (void)i2s_channel_disable(s_tx);
    (void)i2s_del_channel(s_tx);
    s_tx=NULL;
    s_tx_low_latency=false;
}

// rotates ambisonic yaw by permuting the A-format corners
static void monitor_yaw_step(int direction)
{
    s_monitor_yaw=(int8_t)(((int)s_monitor_yaw+direction+4)%4);
    nvs_handle_t nvs;
    if(nvs_open("audio",NVS_READWRITE,&nvs)==ESP_OK) {
        if(nvs_set_i8(nvs,"mon_rot",s_monitor_yaw)==ESP_OK)(void)nvs_commit(nvs);
        nvs_close(nvs);
    }
    display_set_monitor_view(s_monitor_on,(uint8_t)(s_monitor_yaw&3),
                             s_mon_wxyz_db,s_mon_out_db,s_mon_clip);
    ESP_LOGI(TAG,"Monitor mic yaw: %d deg",(int)(s_monitor_yaw&3)*90);
}

// NVS int8 setter
static esp_err_t settings_store_i8(const char *key, int8_t value)
{
    nvs_handle_t nvs;
    esp_err_t err=nvs_open("audio",NVS_READWRITE,&nvs);
    if(err==ESP_OK) {
        err=nvs_set_i8(nvs,key,value);
        if(err==ESP_OK) err=nvs_commit(nvs);
        nvs_close(nvs);
    }
    if(err!=ESP_OK) ESP_LOGW(TAG,"Could not save setting %s: %s",key,esp_err_to_name(err));
    return err;
}

// settings menu
static void settings_publish(void)
{
    const input_preset_t *preset=inp_preset();
    const bool four_ch=preset->ambisonic_slots==4;
    const char *title="SETTINGS";
    const char *hint=s_reset_confirm?"SURE! PRESS REC":"";
    int count=5;

    switch((display_settings_menu_t)s_settings_menu) {
    case SETTINGS_MENU_MONITOR:
        title="MONITOR OPTIONS";
        count=2;
        snprintf(s_set_rows[0],sizeof(s_set_rows[0]),"%s",
                 four_ch?(s_mon_decode?"DECODING ON":"DECODING OFF"):"DECODING N/A 2CH");
        snprintf(s_set_rows[1],sizeof(s_set_rows[1]),"%s",
                 four_ch?"ORDER 1ST (WXYZ)":"ORDER - 2CH ONLY");
        break;
    case SETTINGS_MENU_INPUTS:
        title="INPUTS";
        count=(int)inp_preset_count()+1;
        for(uint8_t i=0;i<inp_preset_count();i++) {
            snprintf(s_set_rows[i],sizeof(s_set_rows[i]),"%s%s",
                     i==inp_preset_index()?"ACTIVE ":"      ",inp_preset_at(i)->name);
        }
        snprintf(s_set_rows[inp_preset_count()],sizeof(s_set_rows[0]),"INPUT LEVELS");
        hint="TAP A MIC TO FIND IT";
        break;
    case SETTINGS_MENU_RECPROC:
        title="REC PROCESSING";
        count=3;
        snprintf(s_set_rows[0],sizeof(s_set_rows[0]),"FORMAT %s",
                 four_ch?(inp_rec_wxyz()?"WXYZ":"RAW"):"RAW (2CH)");
        snprintf(s_set_rows[1],sizeof(s_set_rows[1]),"MIC MAP %s %s %s %s",
                 preset->slot_label[inp_mic_map()[0]],
                 preset->slot_label[inp_mic_map()[1]],
                 preset->slot_label[inp_mic_map()[2]],
                 preset->slot_label[inp_mic_map()[3]]);
        snprintf(s_set_rows[2],sizeof(s_set_rows[2]),"AUTO GAIN %s",
                 recording_auto_gain_enabled()?"ON":"OFF");
        break;
    case SETTINGS_MENU_MICMAP:
        title="MIC MAP";
        count=4;
        for(int i=0;i<4;i++) {
            static const char *corners[4]={"FLU","FRD","BLB","BRU"};
            snprintf(s_set_rows[i],sizeof(s_set_rows[i]),"%s %s",
                     corners[i],preset->slot_label[inp_mic_map()[i]]);
        }
        hint="CLICK - NEXT INPUT";
        break;
    case SETTINGS_MENU_RESET:
        title="FACTORY RESET";
        count=2;
        snprintf(s_set_rows[0],sizeof(s_set_rows[0]),s_reset_confirm?"PRESS REC NOW!":"ERASE ALL SETTINGS");
        snprintf(s_set_rows[1],sizeof(s_set_rows[1]),"CANCEL");
        hint="PRESS REC. 5S WINDOW";
        break;
    case SETTINGS_MENU_RECORDING:
        title="ETC OPTIONS";
        count=5;
        snprintf(s_set_rows[0],sizeof(s_set_rows[0]),"LIVE MONITOR %s",
                 s_record_monitor_enabled?"ON":"OFF");
        snprintf(s_set_rows[1],sizeof(s_set_rows[1]),"REDUCED MOTION %s",
                 display_reduced_motion()?"ON":"OFF");
        {
            const char *name=strrchr(s_eq_path,'/');
            name=name?name+1:s_eq_path;
            char short_name[24];
            snprintf(short_name,sizeof(short_name),"%.*s",20,name);
            char *extension=strrchr(short_name,'.');
            if(extension) *extension='\0';
            snprintf(s_set_rows[2],sizeof(s_set_rows[2]),"EQ %s %s",
                     s_mon_eq_on?"ON":"OFF",
                     s_eq_loaded?(*short_name?short_name:"SELECT FILE"):"NO FILE");
        }
        snprintf(s_set_rows[3],sizeof(s_set_rows[3]),"SELECT EQ FILE");
        snprintf(s_set_rows[4],sizeof(s_set_rows[4]),"4CH WAV AMBI %s",
                 s_wav_ambi_decode?"ON":"OFF");
        hint="REC TOGGLE ANY PAGE";
        break;
    case SETTINGS_MENU_EQ_FILES: {
        count=s_eq_entry_count?s_eq_entry_count:1;
        uint8_t start=0;
        if(s_eq_selected>=DISPLAY_ROWS) start=(uint8_t)(s_eq_selected-DISPLAY_ROWS+1);
        int rows=0;
        for(uint8_t i=start;i<s_eq_entry_count && rows<DISPLAY_ROWS;i++,rows++) {
            snprintf(s_set_rows[rows],sizeof(s_set_rows[rows]),"%.*s",
                     (int)sizeof(s_set_rows[rows])-2,s_eq_entries[i].name);
            if(s_eq_entries[i].directory) strcat(s_set_rows[rows],"/");
        }
        if(!s_eq_entry_count) {
            snprintf(s_set_rows[0],sizeof(s_set_rows[0]),"NO EQ FILES");
            rows=1;
        }
        char browser_title[24];
        const char *relative=s_eq_browser_path+strlen(SD_MOUNT_POINT);
        const char *folder=strrchr(relative,'/');
        folder=folder?folder+1:relative;
        if(!*folder) folder="/";
        snprintf(browser_title,sizeof(browser_title),"EQ %.*s",20,folder);
        hint=s_eq_browser_hint;
        int selected=s_eq_selected>=start?(int)(s_eq_selected-start):0;
        for(int i=0;i<rows;i++) s_set_ptrs[i]=s_set_rows[i];
        display_set_settings_view(SETTINGS_MENU_EQ_FILES,(uint8_t)selected,
                                  (uint8_t)rows,s_set_ptrs,browser_title,hint);
        return;
    }
    default:
        snprintf(s_set_rows[0],sizeof(s_set_rows[0]),"MONITOR OPTIONS");
        snprintf(s_set_rows[1],sizeof(s_set_rows[1]),"INPUTS (%s)",preset->name);
        snprintf(s_set_rows[2],sizeof(s_set_rows[2]),"REC PROCESSING");
        snprintf(s_set_rows[3],sizeof(s_set_rows[3]),"ETC OPTIONS");
        snprintf(s_set_rows[4],sizeof(s_set_rows[4]),"FACTORY RESET");
        break;
    }

    int sel=s_settings_sel;
    if(sel>=count) sel=count-1;
    for(int i=0;i<count && i<8;i++) s_set_ptrs[i]=s_set_rows[i];
    display_set_settings_view((display_settings_menu_t)s_settings_menu,(uint8_t)sel,
                              count>8?8:count,s_set_ptrs,title,hint);
}

// reselects the EQ
static bool settings_load_eq(void)
{
    if(eq_load_file(s_eq_path)) {
        s_eq_loaded=true;
        eq_set_enabled(s_mon_eq_on);
        const char *name=strrchr(s_eq_path,'/');
        snprintf(s_eq_status,sizeof(s_eq_status),"%.*s",
                 23,name?name+1:s_eq_path);
        ESP_LOGI(TAG,"Loaded EQ file %s: %d filters",s_eq_path,eq_filter_count());
        return true;
    } else {
        eq_reset();
        s_eq_loaded=false;
        snprintf(s_eq_status,sizeof(s_eq_status),"NO VALID EQ FILTERS");
        ESP_LOGW(TAG,"EQ file missing, empty, or contains no valid filters: %s",s_eq_path);
        return false;
    }
}

// EQ file browser
static bool eq_text_file(const char *name)
{
    size_t n=strlen(name);
    return n>=4 && name[n-4]=='.' &&
           tolower((unsigned char)name[n-3])=='t' &&
           tolower((unsigned char)name[n-2])=='x' &&
           tolower((unsigned char)name[n-1])=='t';
}

static void eq_browser_scan(void)
{
    s_eq_entry_count=0;
    DIR *dir=opendir(s_eq_browser_path);
    if(!dir) {
        snprintf(s_status,sizeof(s_status),"CANNOT READ EQ FOLDER");
        settings_publish();
        return;
    }
    struct dirent *item;
    while((item=readdir(dir)) && s_eq_entry_count<ENTRY_MAX) {
        if(!strcmp(item->d_name,".") || !strcmp(item->d_name,"..")) continue;
        char full[520];
        struct stat st;
        int written=snprintf(full,sizeof(full),"%s/%s",s_eq_browser_path,item->d_name);
        if(written<0 || (size_t)written>=sizeof(full) || stat(full,&st)!=0) continue;
        bool is_dir=S_ISDIR(st.st_mode);
        if(!is_dir && (!eq_text_file(item->d_name) || st.st_size==0)) continue;
        file_entry_t *entry=&s_eq_entries[s_eq_entry_count];
        size_t len=strnlen(item->d_name,sizeof(entry->name));
        if(len>=sizeof(entry->name)) continue;
        memcpy(entry->name,item->d_name,len+1);
        entry->directory=is_dir;
        s_eq_entry_count++;
    }
    closedir(dir);
    s_eq_selected=s_eq_entry_count && s_eq_selected<s_eq_entry_count?s_eq_selected:0;
    if(!s_eq_entry_count) {
        snprintf(s_status,sizeof(s_status),"NO EQ TXT FILES");
        snprintf(s_eq_browser_hint,sizeof(s_eq_browser_hint),"NO EQ TXT FILES");
    } else {
        snprintf(s_eq_browser_hint,sizeof(s_eq_browser_hint),"CLICK DIR / EQ FILE");
    }
    settings_publish();
}

// persists chosen eq & reloads filter chain 
static esp_err_t settings_store_eq_path(const char *path)
{
    nvs_handle_t nvs;
    esp_err_t err=nvs_open("audio",NVS_READWRITE,&nvs);
    if(err==ESP_OK) {
        err=nvs_set_str(nvs,"eq_path",path);
        if(err==ESP_OK) err=nvs_commit(nvs);
        nvs_close(nvs);
    }
    if(err!=ESP_OK) ESP_LOGW(TAG,"Could not save EQ file path: %s",esp_err_to_name(err));
    return err;
}

// Silent capsule tap-meter (helper for mapping mic wiring, idk if it even works.... -.- )
static void tap_test_set(bool on)
{
    if(on) {
        if(s_monitor_on || s_playing || s_paused) {
            snprintf(s_status,sizeof(s_status),"STOP PLAY+MON FIRST");
            settings_publish();
            return;
        }
        s_mon_meter_only=true;
        monitor_set(true);
    } else if(s_mon_meter_only) {
        monitor_set(false);
    }
}

// settings navigation
static int settings_count(void)
{
    switch((display_settings_menu_t)s_settings_menu) {
    case SETTINGS_MENU_MONITOR: return 2;
    case SETTINGS_MENU_INPUTS:  return (int)inp_preset_count()+1;
    case SETTINGS_MENU_RECPROC: return 3;
    case SETTINGS_MENU_MICMAP:  return 4;
    case SETTINGS_MENU_RESET:   return 2;
    case SETTINGS_MENU_RECORDING: return 5;
    case SETTINGS_MENU_EQ_FILES: return s_eq_entry_count?s_eq_entry_count:1;
    default: return 5;
    }
}

static void settings_move(int direction)
{
    if(s_mon_meter_only) tap_test_set(false);
    settings_reset_cancel();
    int count=settings_count();
    if(!count) return;
    if(s_settings_menu==SETTINGS_MENU_EQ_FILES) {
        s_eq_selected=(uint8_t)((s_eq_selected+direction+count)%count);
        settings_publish();
        return;
    }
    s_settings_sel=(s_settings_sel+direction+count)%count;
    settings_publish();
}

static void settings_back(void)
{
    if(s_mon_meter_only) { tap_test_set(false); settings_publish(); return; }
    s_reset_confirm=false;
    if(s_settings_menu==SETTINGS_MENU_EQ_FILES) {
        if(strcmp(s_eq_browser_path,SD_MOUNT_POINT)) {
            char *slash=strrchr(s_eq_browser_path,'/');
            if(slash && slash>=s_eq_browser_path+strlen(SD_MOUNT_POINT)) *slash='\0';
            s_eq_selected=0;
            eq_browser_scan();
        } else {
            s_settings_menu=SETTINGS_MENU_RECORDING;
            s_settings_sel=3;
            settings_publish();
        }
        return;
    }
    if(s_settings_menu!=SETTINGS_MENU_ROOT) {
        s_settings_menu=(s_settings_menu==SETTINGS_MENU_MICMAP)?
            SETTINGS_MENU_RECPROC:SETTINGS_MENU_ROOT;
        s_settings_sel=0;
        settings_publish();
    } else {
        cycle_page(-1);
    }
}

// erases NVS
static void factory_reset_execute(void)
{
    nvs_handle_t nvs;
    ESP_LOGW(TAG,"FACTORY RESET: erasing settings and rebooting");
    if(nvs_open("audio",NVS_READWRITE,&nvs)==ESP_OK) {
        (void)nvs_erase_all(nvs);
        (void)nvs_commit(nvs);
        nvs_close(nvs);
    }
    if(nvs_open("display",NVS_READWRITE,&nvs)==ESP_OK) {
        (void)nvs_erase_all(nvs);
        (void)nvs_commit(nvs);
        nvs_close(nvs);
    }
    esp_restart();
}

// factory reset function 
static void factory_reset_request(void)
{
    if(!s_reset_confirm) {
        s_reset_confirm=true;
        s_reset_deadline_us=esp_timer_get_time()+
            (int64_t)RESET_CONFIRM_WINDOW_MS*1000;
        settings_publish();
        return;
    }
    if(esp_timer_get_time()>s_reset_deadline_us) {
        s_reset_confirm=false;
        snprintf(s_status,sizeof(s_status),"RESET TIMED OUT");
        settings_publish();
        return;
    }
    factory_reset_execute();
}

static void settings_reset_cancel(void)
{
    if(s_reset_confirm) {
        s_reset_confirm=false;
        settings_publish();
    }
}

static bool settings_reset_poll_expired(void)
{
    if(s_reset_confirm && esp_timer_get_time()>s_reset_deadline_us) {
        s_reset_confirm=false;
        snprintf(s_status,sizeof(s_status),"RESET TIMED OUT");
        settings_publish();
        return true;
    }
    return false;
}

// Selects highlighted entry in settings
static void settings_activate(void)
{
    if(s_mon_meter_only) tap_test_set(false);
    const input_preset_t *preset=inp_preset();
    const bool four_ch=preset->ambisonic_slots==4;
    switch((display_settings_menu_t)s_settings_menu) {
    case SETTINGS_MENU_ROOT:
        {
            static const int roots[4]={SETTINGS_MENU_MONITOR,SETTINGS_MENU_INPUTS,
                                       SETTINGS_MENU_RECPROC,SETTINGS_MENU_RECORDING};
            s_settings_menu=s_settings_sel==4?SETTINGS_MENU_RESET:roots[s_settings_sel];
        }
        s_settings_sel=0;
        settings_publish();
        return;
    case SETTINGS_MENU_MONITOR:
        if(s_settings_sel==0) {
            if(!four_ch) {
                snprintf(s_status,sizeof(s_status),"2CH RAW PASSTHRU");
            } else {
                s_mon_decode=!s_mon_decode;
                settings_store_i8("mon_dec",s_mon_decode?1:0);
                snprintf(s_status,sizeof(s_status),"%s",s_mon_decode?"BINAURAL DECODE":"RAW A-FORMAT");
                if(s_monitor_on) monitor_yaw_step(0);
            }
        } else if(s_settings_sel==1) {
            snprintf(s_status,sizeof(s_status),"%s",four_ch?"1ST ORDER FIXED":"2CH: NO AMBISONICS");
        }
        break;
    case SETTINGS_MENU_RECORDING:
        if(s_settings_sel==0) {
            bool next=!s_record_monitor_enabled;
            if(settings_store_i8("rec_mon",next?1:0)!=ESP_OK) {
                snprintf(s_status,sizeof(s_status),"SAVE FAILED");
                break;
            }
            if(recording_is_active()) {
                esp_err_t err;
                if(next) err=standalone_recording_monitor_enable();
                else {
                    standalone_recording_monitor_stop();
                    err=ESP_OK;
                }
                if(err!=ESP_OK) {
                    (void)settings_store_i8("rec_mon",s_record_monitor_enabled?1:0);
                    snprintf(s_status,sizeof(s_status),"MONITOR START FAILED");
                    settings_publish();
                    return;
                }
            }
            s_record_monitor_enabled=next;
            snprintf(s_status,sizeof(s_status),"REC MONITOR %s",next?"ON":"OFF");
        } else if(s_settings_sel==1) {
            bool enabled=!display_reduced_motion();
            if(display_set_reduced_motion(enabled))
                snprintf(s_status,sizeof(s_status),"REDUCED MOTION %s",enabled?"ON":"OFF");
            else snprintf(s_status,sizeof(s_status),"MOTION SETTING SAVE FAILED");
        } else if(s_settings_sel==2) {
            s_mon_eq_on=!s_mon_eq_on;
            if(settings_store_i8("mon_eq",s_mon_eq_on?1:0)!=ESP_OK) {
                s_mon_eq_on=!s_mon_eq_on;
                snprintf(s_status,sizeof(s_status),"SAVE FAILED");
                break;
            }
            if(s_mon_eq_on && !s_eq_loaded) settings_load_eq();
            else eq_set_enabled(s_mon_eq_on);
            snprintf(s_status,sizeof(s_status),"HEADPHONE EQ %s",s_mon_eq_on?"ON":"OFF");
        } else if(s_settings_sel==3) {
            if(!s_mounted) {
                snprintf(s_status,sizeof(s_status),"SD CARD NOT MOUNTED");
                break;
            }
            snprintf(s_eq_browser_path,sizeof(s_eq_browser_path),"%s",SD_MOUNT_POINT);
            s_eq_selected=0;
            snprintf(s_eq_browser_hint,sizeof(s_eq_browser_hint),"CLICK DIR / EQ FILE");
            s_settings_menu=SETTINGS_MENU_EQ_FILES;
            eq_browser_scan();
            return;
        } else if(s_settings_sel==4) {
            bool next=!s_wav_ambi_decode;
            if(settings_store_i8("wav_ambi",next?1:0)==ESP_OK) {
                s_wav_ambi_decode=next;
                snprintf(s_status,sizeof(s_status),"4CH WAV AMBI %s",next?"ON":"OFF");
            } else {
                snprintf(s_status,sizeof(s_status),"SAVE FAILED");
            }
        }
        break;
    case SETTINGS_MENU_INPUTS:
        if(s_settings_sel==(int)inp_preset_count()) {
            tap_test_set(true);
            return;
        }
        if((uint8_t)s_settings_sel==inp_preset_index()) return;
        if(inp_preset_save((uint8_t)s_settings_sel)==ESP_OK) {
            preset=inp_preset();
            snprintf(s_status,sizeof(s_status),"INPUTS %s",preset->name);
            if(s_monitor_on) {

                monitor_set(false);
                monitor_set(true);
            }
        } else {
            snprintf(s_status,sizeof(s_status),"PRESET SAVE FAILED");
        }
        break;
    case SETTINGS_MENU_RECPROC:
        if(s_settings_sel==0) {
            if(!four_ch) {
                snprintf(s_status,sizeof(s_status),"2CH RECORDS RAW ONLY");
            } else {
                bool next=!inp_rec_wxyz();
                if(inp_rec_wxyz_save(next)==ESP_OK) {
                    snprintf(s_status,sizeof(s_status),"RECORD - %s",next?"WXYZ":"RAW");
                } else {
                    snprintf(s_status,sizeof(s_status),"SAVE FAILED");
                }
            }
        } else if(s_settings_sel==1) {
            if(four_ch) {
                s_settings_menu=SETTINGS_MENU_MICMAP;
                s_settings_sel=0;
                settings_publish();
            } else {
                snprintf(s_status,sizeof(s_status),"MIC MAP NEEDS 4CH");
                settings_publish();
            }
            return;
        } else {
            bool next=!recording_auto_gain_enabled();
            if(settings_store_i8("rec_agc",next?1:0)!=ESP_OK) {
                snprintf(s_status,sizeof(s_status),"SAVE FAILED");
                break;
            }
            recording_set_auto_gain(next);
            snprintf(s_status,sizeof(s_status),"AUTO GAIN %s",next?"ON":"OFF");
        }
        break;
    case SETTINGS_MENU_MICMAP: {
        uint8_t map[4];
        memcpy(map,inp_mic_map(),sizeof(map));
        map[s_settings_sel%4]=(uint8_t)((map[s_settings_sel%4]+1)%4);
        if(inp_mic_map_save(map)==ESP_OK) {
            snprintf(s_status,sizeof(s_status),"MIC MAP SAVED");
        } else {
            snprintf(s_status,sizeof(s_status),"SAVE FAILED");
        }
        break;
    }
    case SETTINGS_MENU_RESET:
        if(s_settings_sel==0) { factory_reset_request(); return; }
        s_reset_confirm=false;
        s_settings_menu=SETTINGS_MENU_ROOT;
        s_settings_sel=4;
        settings_publish();
        return;
    case SETTINGS_MENU_EQ_FILES:
        if(s_eq_entry_count) {
            file_entry_t *entry=&s_eq_entries[s_eq_selected];
            char selected[520];
            int written=snprintf(selected,sizeof(selected),"%s/%s",
                                 s_eq_browser_path,entry->name);
            if(written<0 || (size_t)written>=sizeof(selected)) {
                snprintf(s_status,sizeof(s_status),"EQ PATH TOO LONG");
                break;
            }
            if(entry->directory) {
                size_t selected_len=strlen(selected);
                if(selected_len>=sizeof(s_eq_browser_path)) {
                    snprintf(s_status,sizeof(s_status),"EQ PATH TOO LONG");
                    break;
                }
                memcpy(s_eq_browser_path,selected,selected_len+1);
                s_eq_selected=0;
                eq_browser_scan();
                return;
            }
            size_t selected_len=strlen(selected);
            if(selected_len>=sizeof(s_eq_path)) {
                snprintf(s_status,sizeof(s_status),"EQ PATH TOO LONG");
                break;
            }
            if(!eq_load_file(selected)) {
                if(s_eq_loaded) (void)settings_load_eq();
                else {
                    eq_reset();
                    eq_set_enabled(false);
                }
                snprintf(s_eq_browser_hint,sizeof(s_eq_browser_hint),"NO VALID EQ FILTERS");
                snprintf(s_status,sizeof(s_status),"NO VALID EQ FILTERS");
                settings_publish();
                return;
            }
            memcpy(s_eq_path,selected,selected_len+1);
            s_eq_loaded=true;
            eq_set_enabled(s_mon_eq_on);
            const char *selected_name=strrchr(s_eq_path,'/');
            selected_name=selected_name?selected_name+1:s_eq_path;
            snprintf(s_eq_status,sizeof(s_eq_status),"%.*s",23,selected_name);
            esp_err_t save_err=settings_store_eq_path(s_eq_path);
            s_settings_menu=SETTINGS_MENU_RECORDING;
            s_settings_sel=3;
            snprintf(s_eq_browser_hint,sizeof(s_eq_browser_hint),"CLICK DIR / EQ FILE");
            if(save_err==ESP_OK) snprintf(s_status,sizeof(s_status),"EQ LOADED");
            else snprintf(s_status,sizeof(s_status),"EQ LOADED, SAVE FAILED");
        }
        break;
    }
    settings_publish();
}

// toggles the live monitor
static void monitor_set(bool on)
{
    if(recording_is_active()) {
        if(on==s_recording_monitor_on) return;
        if(on) {
            esp_err_t err=standalone_recording_monitor_enable();
            if(err!=ESP_OK) {
                snprintf(s_status,sizeof(s_status),"MON DAC ERROR");
                publish_view();
                return;
            }
        } else {
            standalone_recording_monitor_stop();
            audio_output_deinit();
        }
        if(settings_store_i8("rec_mon",on?1:0)==ESP_OK) {
            s_record_monitor_enabled=on;
        } else {
            s_record_monitor_enabled=!on;
            snprintf(s_status,sizeof(s_status),"REC OPTION SAVE FAILED");
        }
        publish_view();
        return;
    }
    if(on==s_monitor_on) return;
    if(on) {

        if(s_playing || s_paused) {
            if(s_track_path[0]) {
                snprintf(s_resume_path,sizeof(s_resume_path),"%s",s_track_path);
                s_resume_elapsed_ms=s_info_elapsed_ms;
                s_resume_mp3=!strcmp(s_info_format,"MP3");
                s_resume_byte_offset=s_resume_mp3 && s_info_bitrate
                    ? (uint32_t)((uint64_t)s_info_elapsed_ms*s_info_bitrate/8000ULL)
                    : 0;
                s_resume_available=true;
            }
            s_paused=false;
            s_playing=false;
            s_player_focus=false;
            key_command_t wake={.key=CMD_MONITOR_WAKE,.long_press=false};
            xQueueSend(s_commands,&wake,0);
        }
        if(!s_mon_meter_only) {
            if(monitor_output_init(true)!=ESP_OK) {
                snprintf(s_status,sizeof(s_status),"MON DAC ERROR");
                publish_view();
                return;
            }
        }
        if(monitor_i2s_init()!=ESP_OK) {
            snprintf(s_status,sizeof(s_status),"MON ADC ERROR");
            publish_view();
            return;
        }

        if(pcm1865_init()!=ESP_OK) {
            ESP_LOGW(TAG,"PCM1865 missing; monitor runs with digital silence");
        } else {

            if(inp_preset_apply()!=ESP_OK) {
                ESP_LOGW(TAG,"PCM1865 preset apply failed; default routing kept");
            }
        }
        s_mon_queue=xQueueCreate(4,sizeof(mon_command_t));
        s_mon_stopped=xSemaphoreCreateBinary();
        if(!s_mon_queue || !s_mon_stopped) {
            if(s_mon_queue){vQueueDelete(s_mon_queue);s_mon_queue=NULL;}
            if(s_mon_stopped){vSemaphoreDelete(s_mon_stopped);s_mon_stopped=NULL;}
            audio_output_deinit();
            if(s_mon_rx){(void)i2s_channel_disable(s_mon_rx);(void)i2s_del_channel(s_mon_rx);s_mon_rx=NULL;}
            snprintf(s_status,sizeof(s_status),"MON MEMORY ERROR");
            publish_view();
            return;
        }
        for(int k=0;k<6;k++) s_mon_meter_sq[k]=0.0f;
        s_mon_clip=false;
        s_mon_task_running=true;
        if(xTaskCreate(monitor_task,"mon_task",4096,NULL,4,NULL)!=pdPASS) {
            s_mon_task_running=false;
            vQueueDelete(s_mon_queue);s_mon_queue=NULL;
            vSemaphoreDelete(s_mon_stopped);s_mon_stopped=NULL;
            if(s_mon_rx){(void)i2s_channel_disable(s_mon_rx);(void)i2s_del_channel(s_mon_rx);s_mon_rx=NULL;}
            snprintf(s_status,sizeof(s_status),"MON TASK ERROR");
            publish_view();
            return;
        }
        (void)status_leds_start();
        if(!s_mon_meter_only) {

            if(s_mon_eq_on) settings_load_eq();
            eq_set_enabled(s_mon_eq_on);
        }
        s_monitor_on=true;
        if(s_mon_meter_only) {
            snprintf(s_status,sizeof(s_status),"TAP TEST ON");
        } else {
            s_page=DISPLAY_STANDALONE_MONITOR;
            snprintf(s_status,sizeof(s_status),"LIVE MONITOR ON");
        }
    } else {
        s_monitor_on=false;
        bool was_meter_only=s_mon_meter_only;
        s_mon_meter_only=false;
        if(s_mon_task_running && s_mon_queue) {
            mon_command_t stop=MON_STOP;
            (void)xQueueSend(s_mon_queue,&stop,pdMS_TO_TICKS(100));
            if(s_mon_stopped) (void)xSemaphoreTake(s_mon_stopped,pdMS_TO_TICKS(2000));
        }
        if(s_mon_queue){vQueueDelete(s_mon_queue);s_mon_queue=NULL;}
        if(s_mon_stopped){vSemaphoreDelete(s_mon_stopped);s_mon_stopped=NULL;}
        if(!was_meter_only) status_leds_stop();
        else {
            display_set_tap_view(false,NULL,NULL,0);
            snprintf(s_status,sizeof(s_status),"TAP TEST OFF");
        }
        if(!s_mon_meter_only) s_page=DISPLAY_STANDALONE_MONITOR;
        if(!was_meter_only) snprintf(s_status,sizeof(s_status),"LIVE MONITOR OFF");
        for(int k=0;k<4;k++) s_mon_wxyz_db[k]=-120.0f;
        for(int k=0;k<2;k++) s_mon_out_db[k]=-120.0f;
        display_set_monitor_view(false,(uint8_t)(s_monitor_yaw&3),s_mon_wxyz_db,s_mon_out_db,false);
    }
    publish_view();
}

// Reinitialises the output I2S bus 
static esp_err_t monitor_output_init(bool low_latency)
{
    const uint32_t dac_mult=inp_preset()->mclk_mult==256u?
                            I2S_MCLK_MULTIPLE_512:I2S_MCLK_MULTIPLE_384;
    esp_err_t err=audio_init(MON_DAC_RATE,dac_mult,low_latency);
    if(err!=ESP_OK) return err;
    for(int k=0;k<4;k++) {
        memset(s_spk[k].buf,0,sizeof(s_spk[k].buf));
        s_spk[k].wp=0;
        float p=s_spk[k].p;
        s_spk[k].gl=sqrtf((1.0f+p)*0.5f);
        s_spk[k].gr=sqrtf((1.0f-p)*0.5f);
        s_spk[k].dl=p>0.0f?0:(int)(-p*MON_ITD+0.5f);
        s_spk[k].dr=p>0.0f?(int)(p*MON_ITD+0.5f):0;
    }
    if(s_mon_eq_on) settings_load_eq();
    eq_set_enabled(s_mon_eq_on);
    memset(s_mon_meter_sq,0,sizeof(s_mon_meter_sq));
    s_mon_clip=false;
    return ESP_OK;
}

// Runs binaural monitor while recordin 
esp_err_t standalone_recording_monitor_enable(void)
{
    if(s_recording_monitor_on) return ESP_OK;
    if(!s_record_monitor_mutex) return ESP_ERR_INVALID_STATE;
    if(s_record_monitor_task_running) standalone_recording_monitor_stop();
    esp_err_t err=monitor_output_init(true);
    if(err!=ESP_OK) return err;
    s_record_monitor_queue=xQueueCreate(8,sizeof(recording_monitor_block_t));
    s_record_monitor_stopped=xSemaphoreCreateBinary();
    if(!s_record_monitor_queue || !s_record_monitor_stopped) {
        if(s_record_monitor_queue) {
            vQueueDelete(s_record_monitor_queue);
            s_record_monitor_queue=NULL;
        }
        if(s_record_monitor_stopped) {
            vSemaphoreDelete(s_record_monitor_stopped);
            s_record_monitor_stopped=NULL;
        }
        return ESP_ERR_NO_MEM;
    }
    s_record_monitor_drops=0;
    s_recording_monitor_on=true;
    s_monitor_on=true;
    s_record_monitor_task_running=true;
    if(xTaskCreatePinnedToCore(recording_monitor_task,"rec_monitor",6144,NULL,7,NULL,0)!=pdPASS) {
        s_recording_monitor_on=false;
        s_monitor_on=false;
        s_record_monitor_task_running=false;
        vQueueDelete(s_record_monitor_queue);
        s_record_monitor_queue=NULL;
        vSemaphoreDelete(s_record_monitor_stopped);
        s_record_monitor_stopped=NULL;
        return ESP_ERR_NO_MEM;
    }
    display_set_monitor_view(true,(uint8_t)(s_monitor_yaw&3),
                             s_mon_wxyz_db,s_mon_out_db,false);
    return ESP_OK;
}

// Shuts down recording monitor so no erros splurge out
void standalone_recording_monitor_stop(void)
{
    if(s_record_monitor_mutex) xSemaphoreTake(s_record_monitor_mutex,portMAX_DELAY);
    bool was_running=s_record_monitor_task_running;
    s_recording_monitor_on=false;
    s_monitor_on=false;
    if(was_running && s_record_monitor_queue) {
        recording_monitor_block_t stop={0};
        xQueueReset(s_record_monitor_queue);
        (void)xQueueSend(s_record_monitor_queue,&stop,0);
    }
    if(s_record_monitor_mutex) xSemaphoreGive(s_record_monitor_mutex);
    if(was_running && s_record_monitor_stopped) {
        (void)xSemaphoreTake(s_record_monitor_stopped,portMAX_DELAY);
        if(s_record_monitor_mutex) xSemaphoreTake(s_record_monitor_mutex,portMAX_DELAY);
        if(s_record_monitor_queue) {
            vQueueDelete(s_record_monitor_queue);
            s_record_monitor_queue=NULL;
        }
        vSemaphoreDelete(s_record_monitor_stopped);
        s_record_monitor_stopped=NULL;
        if(s_record_monitor_mutex) xSemaphoreGive(s_record_monitor_mutex);
    }
    audio_output_deinit();
    for(int k=0;k<4;k++) s_mon_wxyz_db[k]=-120.0f;
    for(int k=0;k<2;k++) s_mon_out_db[k]=-120.0f;
    display_set_monitor_view(false,(uint8_t)(s_monitor_yaw&3),
                             s_mon_wxyz_db,s_mon_out_db,false);
}

// Recording monitor status API used by Recording.c 
bool standalone_recording_monitor_is_enabled(void)
{
    return s_recording_monitor_on;
}

bool standalone_recording_monitor_process(const int32_t *samples,size_t frames)
{
    if(!samples) return false;
    if(!s_recording_monitor_on) return true;
    if(!s_record_monitor_mutex) return !s_recording_monitor_on;
    while(frames) {
        size_t block=frames>MON_BLOCK?MON_BLOCK:frames;
        recording_monitor_block_t queued={.frames=(uint16_t)block};
        size_t channels=inp_preset()->channels;
        memcpy(queued.samples,samples,block*channels*sizeof(samples[0]));
        xSemaphoreTake(s_record_monitor_mutex,portMAX_DELAY);
        bool enabled=s_recording_monitor_on && s_record_monitor_queue;
        BaseType_t sent=enabled?xQueueSend(s_record_monitor_queue,&queued,0):pdTRUE;
        xSemaphoreGive(s_record_monitor_mutex);
        if(enabled && sent!=pdTRUE) {
            if(++s_record_monitor_drops==1 || s_record_monitor_drops%100==0) {
                ESP_LOGW(TAG,"Recording monitor queue full; dropped %lu audio blocks",
                         (unsigned long)s_record_monitor_drops);
            }
        }
        samples+=block*channels;
        frames-=block;
    }
    return true;
}

// pushes browser/player/monitor/record view state to display 
static void publish_view(void)
{
    const char *rows[DISPLAY_ROWS];
    char labels[DISPLAY_ROWS][ENTRY_NAME+1];
    uint8_t first = s_selected >= DISPLAY_ROWS ? s_selected - DISPLAY_ROWS + 1 : 0;
    uint8_t count = 0;
    for (uint8_t i=first; i<s_entry_count && count<DISPLAY_ROWS; ++i) {
        size_t name_len = strlen(s_entries[i].name);
        memcpy(labels[count], s_entries[i].name, name_len);
        if (s_entries[i].directory) labels[count][name_len++] = '/';
        labels[count][name_len] = '\0';
        for(char *p=labels[count];*p;p++) *p=(char)toupper((unsigned char)*p);
        rows[count] = labels[count];
        count++;
    }
    uint8_t selected_row = s_selected >= first ? s_selected-first : 0;
    char short_path[32];
    const char *relative = s_path + strlen(SD_MOUNT_POINT);
    if (!*relative) relative = "/";
    size_t path_len = strnlen(relative, sizeof(short_path)-1);
    memcpy(short_path, relative, path_len);
    short_path[path_len] = '\0';
    for(char *p=short_path;*p;p++) *p=(char)toupper((unsigned char)*p);
    const char *track = strrchr(s_track_path, '/');
    char track_name[ENTRY_NAME];
    track = track ? track+1 : "";
    size_t track_len = strnlen(track, sizeof(track_name)-1);
    memcpy(track_name, track, track_len);
    track_name[track_len] = '\0';
    for(char *p=track_name;*p;p++) *p=(char)toupper((unsigned char)*p);
    display_set_standalone_view(short_path, rows, count, selected_row, s_playing,
                                s_volume, s_page, s_paused, track_name,
                                recording_is_active()?NULL:s_status);
    publish_track_info();
}

// reeds foldr listin and sorts audio ingy
static void scan_directory(void)
{
    s_entry_count = 0;
    if (!s_mounted) {
        publish_view();
        return;
    }
    DIR *dir = opendir(s_path);
    if (!dir) {
        snprintf(s_status, sizeof(s_status), "CANNOT READ FOLDER");
        publish_view();
        return;
    }
    struct dirent *item;
    while ((item = readdir(dir)) && s_entry_count < ENTRY_MAX) {
        if (!strcmp(item->d_name, ".") || !strcmp(item->d_name, "..")) continue;
        char full[520]; struct stat st;
        snprintf(full, sizeof(full), "%s/%s", s_path, item->d_name);
        if (stat(full, &st) != 0) continue;
        bool is_dir = S_ISDIR(st.st_mode);
        if (!is_dir && !supported_audio(item->d_name)) continue;
        file_entry_t *entry = &s_entries[s_entry_count++];
        size_t name_len = strnlen(item->d_name, sizeof(entry->name));
        if (name_len >= sizeof(entry->name)) { s_entry_count--; continue; }
        memcpy(entry->name, item->d_name, name_len);
        entry->name[name_len] = '\0';
        entry->directory = is_dir;
    }
    closedir(dir);
    s_selected = s_entry_count ? (s_selected < s_entry_count ? s_selected : s_entry_count-1) : 0;
    if (!s_entry_count) snprintf(s_status, sizeof(s_status), "NO AUDIO FILES");
    else snprintf(s_status, sizeof(s_status), "SELECT A TRACK");
    publish_view();
}

// builfs full path of highlighted entry
static void make_selected_path(char *out, size_t capacity)
{
    snprintf(out, capacity, "%s/%s", s_path, s_entries[s_selected].name);
}

// WAV parsing helper
static bool read_u16(FILE *f, uint16_t *v)
{
    uint8_t b[2]; if (fread(b,1,2,f)!=2) return false;
    *v=(uint16_t)b[0]|((uint16_t)b[1]<<8); return true;
}

static bool read_u32(FILE *f, uint32_t *v)
{
    uint8_t b[4]; if (fread(b,1,4,f)!=4) return false;
    *v=(uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16)|((uint32_t)b[3]<<24); return true;
}

typedef struct { uint32_t rate, data_bytes; uint16_t channels, bits; long data_offset; } wav_info_t;
static bool parse_wav(FILE *f, wav_info_t *w)
{
    char id[4]; uint32_t riff_size;
    if (fread(id,1,4,f)!=4 || memcmp(id,"RIFF",4) || !read_u32(f,&riff_size) ||
        fread(id,1,4,f)!=4 || memcmp(id,"WAVE",4)) return false;
    (void)riff_size;
    bool have_fmt=false, have_data=false; uint16_t format=0, block_align=0;
    while (!have_data) {
        uint32_t chunk_size;
        if (fread(id,1,4,f)!=4 || !read_u32(f,&chunk_size)) break;
        long payload=ftell(f);
        if (!memcmp(id,"fmt ",4)) {
            uint32_t byte_rate;
            if (chunk_size<16 || !read_u16(f,&format) || !read_u16(f,&w->channels) ||
                !read_u32(f,&w->rate) || !read_u32(f,&byte_rate) ||
                !read_u16(f,&block_align) || !read_u16(f,&w->bits)) return false;
            (void)byte_rate;
            have_fmt=true;
        } else if (!memcmp(id,"data",4)) {
            w->data_offset=payload; w->data_bytes=chunk_size; have_data=true;
        }
        if (!have_data && fseek(f,payload+(long)chunk_size+(chunk_size&1),SEEK_SET)!=0) break;
    }
    return have_fmt && have_data && format==1 &&
           (w->channels==1 || w->channels==2 || w->channels==4) &&
           (w->bits==16 || w->bits==24 || w->bits==32) &&
           w->rate>=8000 && w->rate<=96000 &&
           block_align==w->channels*(w->bits/8);
}

// Config the playback channel for the right rate/slots/latency 
static esp_err_t audio_init(uint32_t rate, uint32_t mclk_multiple, bool low_latency)
{
    if(s_tx && s_tx_low_latency!=low_latency) audio_output_deinit();
    if (s_tx) {
        ESP_RETURN_ON_ERROR(i2s_channel_disable(s_tx), TAG, "I2S disable");
        i2s_std_clk_config_t clock=I2S_STD_CLK_DEFAULT_CONFIG(rate);
        clock.clk_src=I2S_CLK_SRC_APLL; clock.mclk_multiple=mclk_multiple;
        ESP_RETURN_ON_ERROR(i2s_channel_reconfig_std_clock(s_tx,&clock), TAG, "I2S clock");
        return i2s_channel_enable(s_tx);
    }
    i2s_chan_config_t channel = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    channel.dma_desc_num=low_latency?3:6;
    channel.dma_frame_num=low_latency?256:504;
    channel.auto_clear_after_cb=true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&channel,&s_tx,NULL), TAG, "I2S channel");
    i2s_std_config_t cfg={
        .clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,I2S_SLOT_MODE_STEREO),
        .gpio_cfg={.mclk=GPIO_NUM_NC,.bclk=I2S_BCLK,.ws=I2S_WS,.dout=I2S_DOUT,.din=GPIO_NUM_NC},
    };
    cfg.clk_cfg.clk_src=I2S_CLK_SRC_APLL;
    cfg.clk_cfg.mclk_multiple=mclk_multiple;
    esp_err_t err=i2s_channel_init_std_mode(s_tx,&cfg);
    if(err!=ESP_OK) {
        (void)i2s_del_channel(s_tx);
        s_tx=NULL;
        return err;
    }
    err=i2s_channel_enable(s_tx);
    if(err!=ESP_OK) {
        (void)i2s_del_channel(s_tx);
        s_tx=NULL;
        return err;
    }
    s_tx_low_latency=low_latency;
    return ESP_OK;
}

// Sample, unpack & scale helpers for multi-bit (16-32) streams
static int32_t decode_wav_sample(const uint8_t *b, uint16_t bits)
{
    int32_t sample;
    if (bits==16) sample=(int16_t)((uint16_t)b[0]|((uint16_t)b[1]<<8));
    else if(bits==32) {
        uint32_t raw=(uint32_t)b[0]|((uint32_t)b[1]<<8)|
                     ((uint32_t)b[2]<<16)|((uint32_t)b[3]<<24);
        memcpy(&sample,&raw,sizeof(sample));
    }
    else {
        uint32_t raw=(uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16);
        if(raw&0x800000) raw|=0xff000000;
        sample=(int32_t)raw;
    }
    return sample;
}

static int32_t scale_pcm_sample(int32_t sample, float gain, int scale)
{
    double scaled=(double)sample*(double)gain*(double)scale;
    if(scaled>2147483647.0) return INT32_MAX;
    if(scaled<-2147483648.0) return INT32_MIN;
    return (int32_t)scaled;
}

static void play_selected(void);
static void reader_stop(void);
static void reader_task(void *arg);

// spawns the SD reader
static bool reader_start(FILE *f)
{
    if(s_reader_task || s_reader_free) reader_stop();
    s_reader_file=f;
    s_reader_epoch=1;
    s_reader_seek_req=false;
    s_reader_running=true;
    s_reader_short=0;
    for(int k=0;k<2;k++) s_reader_slot[k]=malloc(READER_CHUNK_BYTES);
    if(!s_reader_slot[0] || !s_reader_slot[1]) {
        for(int k=0;k<2;k++){ free(s_reader_slot[k]); s_reader_slot[k]=NULL; }
        ESP_LOGW(TAG,"No memory for WAV prefetch reader; using direct SD reads");
        return false;
    }
    s_reader_free=xQueueCreate(2,sizeof(reader_block_t));
    s_reader_ready=xQueueCreate(2,sizeof(reader_block_t));
    s_reader_stopped=xSemaphoreCreateBinary();
    if(!s_reader_free || !s_reader_ready || !s_reader_stopped) {
        reader_stop();
        return false;
    }
    for(int k=0;k<2;k++) {
        reader_block_t blk={.data=s_reader_slot[k],.len=0,.epoch=0};
        xQueueSend(s_reader_free,&blk,0);
    }
    if(xTaskCreatePinnedToCore(reader_task,"wav_reader",4096,NULL,4,&s_reader_task,0)!=pdPASS) {
        s_reader_task=NULL;
        reader_stop();
        return false;
    }
    ESP_LOGI(TAG,"WAV prefetch reader started (2 x %u KB)",READER_CHUNK_BYTES/1024);
    return true;
}

// stops & tears down the prefetch reader 
static void reader_stop(void)
{
    if(s_reader_task) {
        s_reader_running=false;
        if(s_reader_stopped) {
            if(xSemaphoreTake(s_reader_stopped,pdMS_TO_TICKS(3000))==pdTRUE) {
                s_reader_task=NULL;
            } else {

                ESP_LOGW(TAG,"WAV reader did not stop in time; buffers left allocated");
                s_reader_task=NULL;
                s_reader_free=NULL;
                s_reader_ready=NULL;
                s_reader_stopped=NULL;
                s_reader_file=NULL;
                for(int k=0;k<2;k++) s_reader_slot[k]=NULL;
                return;
            }
        }
    }
    if(s_reader_free) { vQueueDelete(s_reader_free); s_reader_free=NULL; }
    if(s_reader_ready) { vQueueDelete(s_reader_ready); s_reader_ready=NULL; }
    if(s_reader_stopped) { vSemaphoreDelete(s_reader_stopped); s_reader_stopped=NULL; }
    for(int k=0;k<2;k++) { free(s_reader_slot[k]); s_reader_slot[k]=NULL; }
    s_reader_file=NULL;
}

// Queues a byte-offset seek to apply on the reader thread
static void reader_request_seek(long offset)
{
    s_reader_epoch++;
    s_reader_seek_offset=offset;
    s_reader_seek_req=true;

    reader_block_t stale;
    while(xQueueReceive(s_reader_ready,&stale,0)==pdTRUE)
        xQueueSend(s_reader_free,&stale,0);
}

// keeps the playback stream buffer fed ahead
static void reader_task(void *arg)
{
    (void)arg;
    while(s_reader_running) {
        if(s_reader_seek_req) {
            (void)fseek(s_reader_file,s_reader_seek_offset,SEEK_SET);
            s_reader_seek_req=false;
        }
        reader_block_t blk;
        if(xQueueReceive(s_reader_free,&blk,pdMS_TO_TICKS(100))!=pdTRUE) continue;
        uint32_t my_epoch=s_reader_epoch;
        if(!s_reader_running) { xQueueSend(s_reader_free,&blk,0); break; }
        blk.len=0;
        if(!s_reader_seek_req && s_reader_epoch==my_epoch) {
            size_t got=fread(blk.data,1,READER_CHUNK_BYTES,s_reader_file);
            s_reader_reads++;
            blk.len=got; blk.epoch=my_epoch;
            if(got && got<READER_CHUNK_BYTES) s_reader_short++;
        }

        if(s_reader_epoch!=my_epoch) {
            xQueueSend(s_reader_free,&blk,0);
            continue;
        }

        while(s_reader_running && s_reader_epoch==my_epoch) {
            if(xQueueSend(s_reader_ready,&blk,pdMS_TO_TICKS(100))==pdTRUE) break;
        }
        if(!s_reader_running || s_reader_epoch!=my_epoch)
            (void)xQueueSend(s_reader_free,&blk,0);
    }
    s_reader_task=NULL;
    if(s_reader_stopped) xSemaphoreGive(s_reader_stopped);
    vTaskDelete(NULL);
}

// timed fill and copy
static bool stream_fill(reader_stream_t *st, TickType_t timeout)
{
    for(;;) {
        if(!st->have) {
            if(xQueueReceive(s_reader_ready,&st->blk,timeout)!=pdTRUE) return false;
            if(st->blk.epoch!=s_reader_epoch) {
                xQueueSend(s_reader_free,&st->blk,0);
                continue;
            }
            st->pos=0;
            st->have=true;
        }
        if(st->pos>=st->blk.len) {
            xQueueSend(s_reader_free,&st->blk,0);
            st->have=false;
            continue;
        }
        return true;
    }
}

static bool stream_copy(reader_stream_t *st, uint8_t *dst, size_t bytes, TickType_t timeout)
{
    while(bytes) {
        if(!stream_fill(st,timeout)) return false;
        size_t chunk=st->blk.len-st->pos;
        if(chunk>bytes) chunk=bytes;
        memcpy(dst,&st->blk.data[st->pos],chunk);
        st->pos+=chunk;
        dst+=chunk;
        bytes-=chunk;
    }
    return true;
}

// dirty flag and debounced NVS save
static void volume_mark_dirty(void)
{
    s_volume_dirty=true;
    s_volume_changed_us=esp_timer_get_time();
}

// Next/previous pages on side keys
static void cycle_page(int direction)
{
    if(s_mon_meter_only) tap_test_set(false);
    int page=((int)s_page+direction+7)%7;
    s_page=(display_standalone_page_t)page;
    if(s_page==DISPLAY_STANDALONE_SETTINGS) settings_publish();
}

// volume change coarse keys, fine encoder
static void change_volume(command_key_t key)
{
    int direction=key==CMD_UP?1:key==CMD_DOWN?-1:0;
    if(!direction) return;
    int step=s_volume<10 || (direction<0 && s_volume<=10)?1:5;
    int next=(int)s_volume+direction*step;
    if(direction<0 && s_volume>10 && next<10) next=10;
    if(next<0) next=0;
    if(next>100) next=100;
    if(next!=s_volume) {
        s_volume=(uint8_t)next;
        volume_mark_dirty();
    }
}

static void change_volume_fine(int direction)
{
    change_volume(direction>0?CMD_UP:CMD_DOWN);
}

static void volume_save_now(void)
{
    if(!s_volume_dirty) return;
    s_volume_dirty=false;
    nvs_handle_t nvs;
    if(nvs_open("audio",NVS_READWRITE,&nvs)==ESP_OK) {
        if(nvs_set_u8(nvs,"playvol",s_volume)==ESP_OK)(void)nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static void volume_save_poll(void)
{
    if(s_volume_dirty && esp_timer_get_time()-s_volume_changed_us>=VOLUME_SAVE_DELAY_US)
        volume_save_now();
}

// pick a file / auto resume last track
static bool selected_is_audio(void)
{
    return s_entry_count && !s_entries[s_selected].directory && supported_audio(s_entries[s_selected].name);
}

static void request_selected_track(void)
{
    if(s_monitor_on) monitor_set(false);
    if(!selected_is_audio()) return;
    char requested[520];
    make_selected_path(requested,sizeof(requested));
    s_resume_request=s_resume_available && strcmp(requested,s_resume_path)==0;
    if(!s_resume_request) s_resume_available=false;
    if(s_playing && strcmp(requested,s_track_path)==0) {
        s_player_focus=true;
        return;
    }
    s_player_focus=true;
    s_paused=false;
    s_playing=false;
    s_request_play=true;
}

static void request_resume_track(void)
{
    if(!s_resume_available) return;
    if(s_monitor_on) monitor_set(false);
    for(uint8_t i=0;i<s_entry_count;i++) {
        if(s_entries[i].directory || !supported_audio(s_entries[i].name)) continue;
        char path[520];
        snprintf(path,sizeof(path),"%s/%s",s_path,s_entries[i].name);
        if(strcmp(path,s_resume_path)==0) {
            s_selected=i;
            s_resume_request=true;
            s_request_play=true;
            s_player_focus=true;
            s_paused=false;
            return;
        }
    }
    snprintf(s_status,sizeof(s_status),"RETURN TO TRACK FOLDER");
}

static void navigate_back(void)
{
    if(strcmp(s_path,SD_MOUNT_POINT)) {
        char *slash=strrchr(s_path,'/');
        if(slash && slash>=s_path+strlen(SD_MOUNT_POINT)) *slash='\0';
        s_selected=0;
        scan_directory();
    } else cycle_page(-1);
}

// to seek.. or not to seek, thats the intents
static void request_seek(int delta_ms)
{
    s_seek_delta_ms=delta_ms;
    s_seek_request=true;
}

// stops the current playback and starts another file
static void skip_track(int direction)
{
    if(s_monitor_on) monitor_set(false);
    if(!s_entry_count) return;
    int idx=(int)s_selected;
    for(uint8_t n=0;n<s_entry_count;n++) {
        idx+=direction;
        if(idx<0) idx=(int)s_entry_count-1;
        if(idx>=(int)s_entry_count) idx=0;
        if(!s_entries[idx].directory && supported_audio(s_entries[idx].name)) {
            if(idx!=(int)s_selected) {
                s_selected=(uint8_t)idx;
                s_playing=false;
                s_paused=false;
                s_player_focus=true;
                s_request_play=true;
            }
            return;
        }
    }
}

// starts / stops the recorder
static void request_recording_toggle(void)
{
    if(recording_is_active()) {
        esp_err_t err=recording_stop();
        if(err!=ESP_OK) {
            ESP_LOGE(TAG,"Could not stop recording: %s",esp_err_to_name(err));
            snprintf(s_status,sizeof(s_status),"REC STOP FAILED");
        } else {
            snprintf(s_status,sizeof(s_status),"RECORDING SAVED");
        }
        return;
    }

    s_recording_monitor_pending=s_record_monitor_enabled || s_monitor_on;
    if(s_monitor_on) monitor_set(false);
    s_playing=false;
    s_paused=false;
    s_player_focus=false;
    s_request_play=false;
    s_recording_start_pending=true;
    snprintf(s_status,sizeof(s_status),"REC STARTING");
}

// button command dispatch 
static void process_command(key_command_t command)
{
    if(command.key==CMD_MONITOR_WAKE) return;
    if(command.key==CMD_MONITOR_TOGGLE) {
        if(recording_is_active()) monitor_set(!s_recording_monitor_on);
        else monitor_set(!s_monitor_on);
        return;
    }
    if(command.key==CMD_RESET_CONFIRM) {

        if(s_page==DISPLAY_STANDALONE_SETTINGS) factory_reset_request();
        return;
    }
    if(command.key==CMD_REC_TOGGLE) {
        request_recording_toggle();
        return;
    }
    if(s_page==DISPLAY_STANDALONE_SETTINGS) settings_reset_poll_expired();
    if(command.key==CMD_ENCODER_STEP) {
        int direction=command.long_press?1:-1;
        if(s_page==DISPLAY_STANDALONE_SETTINGS) settings_move(direction);
        else change_volume_fine(direction);
        return;
    }
    if(command.key==CMD_STOP) {
        s_stop_requested=true;
        s_playing=false;
        s_paused=false;
        s_player_focus=false;
        s_request_play=false;
        return;
    }
    if(s_player_focus && s_playing) {

        bool track_playing=s_playing && !s_paused;
        if(command.key==CMD_SEEK_FWD) { request_seek(10000); return; }
        if(command.key==CMD_SEEK_BACK) { request_seek(-10000); return; }
        if(command.key==CMD_SKIP_FWD) { skip_track(1); return; }
        if(command.key==CMD_SKIP_BACK) { skip_track(-1); return; }
        if(command.key==CMD_ENCODER_STEP) { change_volume_fine(command.long_press?1:-1); return; }
        if(command.key==CMD_ENCODER_CLICK) {
            s_paused=!s_paused;
            snprintf(s_status,sizeof(s_status),"%s",s_paused?"PAUSED":"PLAYING");
            return;
        }
        if(command.key==CMD_PLAY_PAUSE) {
            s_paused=!s_paused;
            snprintf(s_status,sizeof(s_status),"%s",s_paused?"PAUSED":"PLAYING");
            volume_save_now();
            return;
        }
        if(command.key==CMD_UP || command.key==CMD_DOWN) {

            if(s_page==DISPLAY_STANDALONE_EQ) {
                uint8_t source=display_spectrum_source();
                source=(uint8_t)((source+(command.key==CMD_UP?1:5))%6);
                display_select_spectrum_source(source);
            } else if(s_page==DISPLAY_STANDALONE_SETTINGS) {
                settings_move(command.key==CMD_UP?-1:1);
            } else if(s_page==DISPLAY_STANDALONE_MONITOR) {
                monitor_yaw_step(command.key==CMD_UP?1:-1);
            } else if(s_entry_count) {

                if(command.key==CMD_UP)
                    s_selected=(uint8_t)((s_selected+s_entry_count-1)%s_entry_count);
                else
                    s_selected=(uint8_t)((s_selected+1)%s_entry_count);
            }
            return;
        }
        if(command.long_press && command.key==CMD_LEFT) {

            if(s_page==DISPLAY_STANDALONE_LISTEN) navigate_back();
            else cycle_page(-1);
            return;
        }
        if(command.long_press && command.key==CMD_RIGHT) {
            if(s_page==DISPLAY_STANDALONE_SETTINGS) { settings_activate(); return; }
            if(s_page==DISPLAY_STANDALONE_LISTEN && s_entry_count) {
                if(s_entries[s_selected].directory) { play_selected(); return; }
                if(selected_is_audio()) {
                    char requested[520];
                    make_selected_path(requested,sizeof(requested));
                    if(strcmp(requested,s_track_path)==0) {

                        s_paused=!s_paused;
                        snprintf(s_status,sizeof(s_status),"%s",s_paused?"PAUSED":"PLAYING");
                    } else if(!track_playing && s_resume_available && strcmp(requested,s_resume_path)==0) {
                        request_resume_track();
                    } else {

                        request_selected_track();
                    }
                    return;
                }
            }

            s_paused=!s_paused;
            snprintf(s_status,sizeof(s_status),"%s",s_paused?"PAUSED":"PLAYING");
            return;
        }

        if(command.key==CMD_LEFT) {
            cycle_page(-1);
        } else if(command.key==CMD_RIGHT) {
            cycle_page(1);
        }
        return;
    }

    if(command.key==CMD_ENCODER_CLICK) {

        if(s_page==DISPLAY_STANDALONE_LISTEN && selected_is_audio()) request_selected_track();
        else cycle_page(1);
        return;
    }

    if(command.key==CMD_UP || command.key==CMD_DOWN) {
        if(s_page==DISPLAY_STANDALONE_EQ) {
            uint8_t source=display_spectrum_source();
            source=(uint8_t)((source+(command.key==CMD_UP?1:5))%6);
            display_select_spectrum_source(source);
        } else if(s_page==DISPLAY_STANDALONE_SETTINGS) {
            settings_move(command.key==CMD_UP?-1:1);
        } else if(s_page==DISPLAY_STANDALONE_MONITOR) {
            monitor_yaw_step(command.key==CMD_UP?1:-1);
        } else if(s_page==DISPLAY_STANDALONE_LISTEN && s_entry_count) {

            if(command.key==CMD_UP)
                s_selected=(uint8_t)((s_selected+s_entry_count-1)%s_entry_count);
            else
                s_selected=(uint8_t)((s_selected+1)%s_entry_count);
        } else change_volume(command.key);
        return;
    }

    if(command.long_press && command.key==CMD_LEFT) {
        if(s_page==DISPLAY_STANDALONE_SETTINGS) settings_back();
        else if(s_page==DISPLAY_STANDALONE_LISTEN) navigate_back();
        else cycle_page(-1);
        return;
    }
    if((command.long_press && command.key==CMD_RIGHT) || command.key==CMD_PLAY_PAUSE) {

        if(s_page==DISPLAY_STANDALONE_SETTINGS) settings_activate();
        else if(s_page==DISPLAY_STANDALONE_MONITOR && s_resume_available) request_resume_track();
        else if(s_page==DISPLAY_STANDALONE_LISTEN && s_entry_count) {
            if(s_entries[s_selected].directory) play_selected();
            else if(selected_is_audio()) {
                char requested[520];
                make_selected_path(requested,sizeof(requested));
                if(s_playing && strcmp(requested,s_track_path)==0) {
                    s_player_focus=true;
                    s_paused=!s_paused;
                    snprintf(s_status,sizeof(s_status),"%s",s_paused?"PAUSED":"PLAYING");
                } else if(s_resume_available && strcmp(requested,s_resume_path)==0) {
                    request_resume_track();
                } else request_selected_track();
            }
        }
        return;
    }

    if(command.key==CMD_LEFT) {
        cycle_page(-1);
    } else if(command.key==CMD_RIGHT) {
        if(s_page==DISPLAY_STANDALONE_LISTEN && selected_is_audio()) request_selected_track();
        else cycle_page(1);
    }
}

// queues & recalls commands for the player loop
void standalone_control(display_control_event_t event, int value)
{
    if(!s_commands) return;
    key_command_t command;
    switch(event) {
    case DISPLAY_CONTROL_PLAY_PAUSE:
        if(value==0) return;
        command=(key_command_t){CMD_PLAY_PAUSE,false};
        break;
    case DISPLAY_CONTROL_NEXT:
        command=(key_command_t){CMD_SKIP_FWD,false};
        break;
    case DISPLAY_CONTROL_PREVIOUS:
        command=(key_command_t){CMD_SKIP_BACK,false};
        break;
    case DISPLAY_CONTROL_SEEK_FWD:
        if(value) command=(key_command_t){CMD_SEEK_FWD,false};
        else return;
        break;
    case DISPLAY_CONTROL_SEEK_BACK:
        if(value) command=(key_command_t){CMD_SEEK_BACK,false};
        else return;
        break;
    case DISPLAY_CONTROL_MONITOR: command=(key_command_t){CMD_MONITOR_TOGGLE,false}; break;
    case DISPLAY_CONTROL_REC:
        if(value==0) return;
        if(s_page==DISPLAY_STANDALONE_SETTINGS &&
           (display_settings_menu_t)s_settings_menu==SETTINGS_MENU_RESET) {
            command=(key_command_t){CMD_RESET_CONFIRM,false};
        } else {
            command=(key_command_t){CMD_REC_TOGGLE,false};
        }
        break;
    case DISPLAY_CONTROL_ENCODER_STEP:
        command=(key_command_t){CMD_ENCODER_STEP,value>0};
        break;
    case DISPLAY_CONTROL_ENCODER_CLICK:
        if(s_page==DISPLAY_STANDALONE_SETTINGS) {

            command=(key_command_t){CMD_RIGHT,true};
            break;
        }
        command=(key_command_t){CMD_ENCODER_CLICK,false};
        break;
    case DISPLAY_CONTROL_ENCODER_DOUBLE:
        return;
    case DISPLAY_CONTROL_ENCODER_HOLD:
        if(s_page==DISPLAY_STANDALONE_SETTINGS) command=(key_command_t){CMD_LEFT,true};
        else return;
        break;
    default: return;
    }
    (void)xQueueSend(s_commands,&command,0);
}

// writes a rendered PCM block 
static bool output_pcm_block(const int32_t *samples,uint32_t frames,uint32_t rate)
{
    if(!frames) return true;
    if(s_bass_rate!=rate) {
        s_bass_rate=rate;
        s_bass_lowpass[0]=s_bass_lowpass[1]=0.0f;
        s_bass_dc[0]=s_bass_dc[1]=0.0f;
    }
    const float bass_alpha=1.0f-expf(-6.28318530718f*150.0f/(float)rate);
    const float dc_alpha=1.0f-expf(-6.28318530718f*20.0f/(float)rate);
    float peak[2]={0.0f,0.0f};
    double bass_sum_sq=0.0;
    for(uint32_t i=0;i<frames;i++) {
        for(int ch=0;ch<2;ch++) {
            float normalized=(float)samples[2*i+ch]/2147483648.0f;
            float magnitude=fabsf(normalized);
            if(magnitude>peak[ch]) peak[ch]=magnitude;
            s_bass_lowpass[ch]+=bass_alpha*(normalized-s_bass_lowpass[ch]);
            s_bass_dc[ch]+=dc_alpha*(normalized-s_bass_dc[ch]);
            float bass=s_bass_lowpass[ch]-s_bass_dc[ch];
            bass_sum_sq+=(double)bass*(double)bass;
            standalone_spectrum_add_sample((uint8_t)ch,samples[2*i+ch]/256,rate);
        }
    }
    float bass_rms=sqrtf((float)(bass_sum_sq/(double)(frames*2)));
    s_bass_dbfs=bass_rms>1.0e-6f?20.0f*log10f(bass_rms):-120.0f;
    float db_l=peak[0]>0.0f?20.0f*log10f(peak[0]):-120.0f;
    float db_r=peak[1]>0.0f?20.0f*log10f(peak[1]):-120.0f;
    display_set_output_meter(db_l,db_r,peak[0]>=0.999999f||peak[1]>=0.999999f);
    if(s_mon_eq_on) eq_apply_int32_stereo((int32_t *)samples,frames,rate);
    size_t written=0;
    size_t bytes=(size_t)frames*2*sizeof(int32_t);
    esp_err_t err=i2s_channel_write(s_tx,samples,bytes,&written,1000);
    if(err!=ESP_OK || written!=bytes) {

        ESP_LOGE(TAG,"I2S DAC write %s: %u/%u bytes (%s)",
                 written&&written<bytes?"PARTIAL":"FAILED",
                 (unsigned)written,(unsigned)bytes,esp_err_to_name(err));
        return false;
    }
    return true;
}

// watchdog diagnostics
static void playback_stall_check(const char *what,uint32_t frames,uint32_t rate,int64_t start_us)
{
    int64_t now=esp_timer_get_time();
    uint32_t expected_us=rate?(uint32_t)((uint64_t)frames*1000000ULL/rate):0;
    uint32_t actual_us=(uint32_t)(now-start_us);
    static int64_t last_log;
    if(actual_us>2*(int64_t)expected_us && actual_us>40000 && now-last_log>2000000) {
        last_log=now;
        ESP_LOGW(TAG,"%s: %u frames (%u us expected) took %u us to produce",
                 what,(unsigned)frames,(unsigned)expected_us,(unsigned)actual_us);
    }
}

// pending seek intent ting
static int32_t consume_seek_request(void)
{
    if(!s_seek_request) return 0;
    s_seek_request=false;
    int32_t delta=s_seek_delta_ms;
    s_seek_delta_ms=0;
    return delta;
}

// Applies WAV seek by byte math 
static bool wav_apply_seek(FILE *f,const wav_info_t *wav,bool use_reader,
                           uint64_t *elapsed_frames,uint64_t total_frames)
{
    int32_t delta=consume_seek_request();
    if(!delta) return false;
    int64_t target_ms=(int64_t)((*elapsed_frames)*1000LL)/(int64_t)wav->rate+delta;
    int64_t duration_ms=(int64_t)((total_frames*1000LL)/(int64_t)wav->rate);
    if(target_ms<0) target_ms=0;
    if(target_ms>duration_ms) target_ms=duration_ms;
    uint64_t target_frames=(uint64_t)target_ms*wav->rate/1000ULL;
    long offset=(long)wav->data_offset+(long)(target_frames*(uint64_t)(wav->channels*(wav->bits/8)));
    bool ok;
    if(use_reader) {
        reader_request_seek(offset);
        ok=true;
    } else {
        ok=fseek(f,offset,SEEK_SET)==0;
        if(!ok) ESP_LOGW(TAG,"WAV fseek to %ld failed",offset);
    }
    *elapsed_frames=target_frames;
    s_info_elapsed_ms=(uint32_t)target_ms;
    s_bass_lowpass[0]=s_bass_lowpass[1]=0.0f;
    s_bass_dc[0]=s_bass_dc[1]=0.0f;
    snprintf(s_status,sizeof(s_status),"%s xS",delta>0?"FWD":"REW");
    publish_track_info();
    publish_view();
    ESP_LOGI(TAG,"WAV seek %+d ms -> %u ms (offset %ld, ok=%d)",
             (int)delta,(unsigned)target_ms,offset,ok);
    return true;
}

// prefetch, volume/fades, meters, monitor toggle and key handling while playing audio 
static bool play_wav_file(const char *path, uint32_t resume_ms)
{
    FILE *f=fopen(path,"rb");
    if(!f) { snprintf(s_status,sizeof(s_status),"FILE OPEN ERROR"); return false; }
    wav_info_t wav={0};
    if(!parse_wav(f,&wav)) {
        fclose(f); snprintf(s_status,sizeof(s_status),"PCM WAV FORMAT UNSUPPORTED"); return false;
    }
    bool ambi_decode=wav.channels==4 && s_wav_ambi_decode && wav.rate==48000;
    bool input_wxyz=inp_rec_wxyz();
    uint32_t block_align=wav.channels*(wav.bits/8);
    uint64_t total_frames=block_align?wav.data_bytes/block_align:0;
    uint64_t resume_frames=(uint64_t)resume_ms*wav.rate/1000ULL;
    if(resume_frames>total_frames) resume_frames=total_frames;
    long resume_offset=wav.data_offset+(long)(resume_frames*block_align);
    esp_err_t audio_err=ambi_decode
        ?monitor_output_init(false)
        :audio_init(wav.rate,I2S_MCLK_MULTIPLE_256,false);
    if(audio_err!=ESP_OK ||
       fseek(f,resume_offset,SEEK_SET)!=0) {
        fclose(f); snprintf(s_status,sizeof(s_status),"AUDIO INIT ERROR"); return false;
    }
    struct stat file_stat={0};
    (void)stat(path,&file_stat);
    s_info_format[0]='W'; s_info_format[1]='A'; s_info_format[2]='V'; s_info_format[3]='\0';
    s_info_rate=wav.rate; s_info_bits=(uint8_t)wav.bits; s_info_channels=(uint8_t)wav.channels;
    s_info_bitrate=wav.rate*wav.channels*wav.bits;
    s_info_file_size=file_stat.st_size>0?(uint64_t)file_stat.st_size:0;
    s_info_elapsed_ms=(uint32_t)(resume_frames*1000ULL/wav.rate);
    s_info_duration_ms=wav.rate?(uint32_t)(total_frames*1000ULL/wav.rate):0;
    s_info_duration_estimated=false;
    publish_track_info();
    snprintf(s_track_path,sizeof(s_track_path),"%s",path);
    s_playing=true; s_paused=false; snprintf(s_status,sizeof(s_status),"PLAYING"); publish_view();
    s_bass_lowpass[0]=s_bass_lowpass[1]=0.0f;
    s_bass_dc[0]=s_bass_dc[1]=0.0f;
    s_bass_dbfs=-120.0f;
    s_bass_rate=wav.rate;
    uint32_t frames_left=(uint32_t)(total_frames-resume_frames);
    uint64_t elapsed_frames=resume_frames;
    int32_t out[AUDIO_FRAMES*2];
    int32_t ambi_in[MON_BLOCK*4];

    bool use_reader=ambi_decode && reader_start(f);
    reader_stream_t rstream={0};
    while(frames_left && s_playing) {
        if(s_paused) {
            key_command_t command;
            if(xQueueReceive(s_commands,&command,portMAX_DELAY)==pdTRUE) process_command(command);
            (void)wav_apply_seek(f,&wav,use_reader,&elapsed_frames,total_frames);
            volume_save_poll();
            publish_view();
            continue;
        }
        key_command_t command;
        while(xQueueReceive(s_commands,&command,0)==pdTRUE) {
            process_command(command);
            publish_view();
            if(!s_playing || s_paused) break;
        }
        if(!s_playing) break;
        if(s_paused) continue;
        (void)wav_apply_seek(f,&wav,use_reader,&elapsed_frames,total_frames);
        if(!s_playing) break;

        frames_left=total_frames>elapsed_frames?(uint32_t)(total_frames-elapsed_frames):0;
        if(!frames_left) break;
        uint32_t max_frames=ambi_decode?MON_BLOCK:AUDIO_FRAMES;
        uint32_t n=frames_left>max_frames?max_frames:frames_left;
        size_t block_bytes=(size_t)n*block_align;
        int64_t block_start=esp_timer_get_time();
        if(use_reader) {
            if(!stream_copy(&rstream,s_wav_input,block_bytes,pdMS_TO_TICKS(200))) {
                if(++s_reader_stalls>=3) {
                    ESP_LOGW(TAG,"WAV reader stalled: %lu stalls in a row at %u ms",
                             (unsigned long)s_reader_stalls,s_info_elapsed_ms);
                    break;
                }
                continue;
            }
            s_reader_stalls=0;
        } else {
            size_t got=fread(s_wav_input,1,block_bytes,f);
            if(got!=block_bytes) {

                ESP_LOGW(TAG,"WAV short read at file offset %ld: %u/%u bytes (pos %u ms)",
                         ftell(f),(unsigned)got,(unsigned)block_bytes,s_info_elapsed_ms);
                break;
            }
        }
        float gain=(s_volume/100.0f)*s_gain;
        float gain_l=gain*powf(10.0f,(float)s_channel_gain_db[4]*0.05f);
        float gain_r=gain*powf(10.0f,(float)s_channel_gain_db[5]*0.05f);
        for(uint32_t i=0;i<n;i++) {
            int32_t input[4]={0};
            for(uint16_t ch=0;ch<wav.channels;ch++) {
                const uint8_t *sample_data=&s_wav_input[(size_t)i*block_align+
                                                        (size_t)ch*(wav.bits/8)];
                int32_t raw=decode_wav_sample(sample_data,wav.bits);
                int to_q31=wav.bits==16?65536:wav.bits==24?256:1;
                input[ch]=scale_pcm_sample(raw,ambi_decode?s_gain:1.0f,to_q31);
            }
            if(wav.channels==4 && ambi_decode) {
                for(int ch=0;ch<4;ch++) ambi_in[4*i+ch]=input[ch];
            } else {
                int32_t left=wav.channels==4?(int32_t)(((int64_t)input[0]+input[1])/2):input[0];
                int32_t right=wav.channels==4?(int32_t)(((int64_t)input[2]+input[3])/2):
                              wav.channels==2?input[1]:input[0];
                out[2*i]=scale_pcm_sample(left,gain_l,1);
                out[2*i+1]=scale_pcm_sample(right,gain_r,1);
            }
        }
        bool pcm_out=true;
        if(ambi_decode) {
            pcm_out=monitor_process_internal(ambi_in,n,true,input_wxyz,false);
        } else {
            pcm_out=output_pcm_block(out,n,wav.rate);
        }
        playback_stall_check(ambi_decode?"ambi decode":"wav decode",n,wav.rate,block_start);
        if(!pcm_out) break;
        frames_left-=n;
        elapsed_frames+=n;
        s_info_elapsed_ms=(uint32_t)(elapsed_frames*1000ULL/wav.rate);
        publish_track_info();
        volume_save_poll();
    }
    if(use_reader) {
        if(s_reader_short) ESP_LOGI(TAG,"WAV reader stats: %lu chunks, %lu short EOF chunk, stalls seen",
                                    (unsigned long)s_reader_reads,(unsigned long)s_reader_short);
        reader_stop();
    }
    fclose(f);
    if(s_playing && !frames_left) {
        snprintf(s_status,sizeof(s_status),"TRACK FINISHED");
        s_player_focus=false;
        s_page=DISPLAY_STANDALONE_LISTEN;
        s_seek_request=false;
    } else if(s_playing && frames_left) {
        snprintf(s_status,sizeof(s_status),"PLAYBACK ERROR");
        s_player_focus=false;
        s_page=DISPLAY_STANDALONE_LISTEN;
    } else if(!s_playing && !s_request_play) {
        snprintf(s_status,sizeof(s_status),"STOPPED");
        s_player_focus=false;
    }
    volume_save_now();
    s_playing=false;
    s_paused=false;
    return true;
}

// polls keys during MP3 decode
static bool mp3_service_keys(void)
{
    key_command_t command;
    while(xQueueReceive(s_commands,&command,0)==pdTRUE) {
        process_command(command);
        publish_view();
        if(!s_playing || s_paused) return false;
    }
    if(s_paused && s_playing) {
        if(xQueueReceive(s_commands,&command,portMAX_DELAY)==pdTRUE) process_command(command);
        publish_view();
        return false;
    }
    return s_playing;
}

// decodes, meters, seeks and shares the same output/monitor plumbing MP3
static bool play_mp3_file(const char *path, uint32_t resume_ms)
{
    FILE *f=fopen(path,"rb");
    if(!f) { snprintf(s_status,sizeof(s_status),"FILE OPEN ERROR"); return false; }
    struct stat file_stat={0};
    (void)stat(path,&file_stat);
    uint64_t file_size=file_stat.st_size>0?(uint64_t)file_stat.st_size:0;
    uint8_t tag[10];
    long data_offset=0;
    if(fread(tag,1,sizeof(tag),f)==sizeof(tag) && memcmp(tag,"ID3",3)==0) {
        uint32_t tag_size=((uint32_t)(tag[6]&0x7f)<<21)|((uint32_t)(tag[7]&0x7f)<<14)|
                          ((uint32_t)(tag[8]&0x7f)<<7)|(uint32_t)(tag[9]&0x7f);
        data_offset=10L+(long)tag_size+((tag[5]&0x10)?10L:0L);
    }
    long mp3_offset=data_offset;
    if(resume_ms && s_resume_mp3 && s_resume_byte_offset) {
        uint64_t requested=(uint64_t)data_offset+s_resume_byte_offset;
        if(requested<(uint64_t)file_size) mp3_offset=(long)requested;
    }
    if(fseek(f,mp3_offset,SEEK_SET)!=0) { fclose(f); snprintf(s_status,sizeof(s_status),"MP3 SEEK ERROR"); return false; }
    void *decoder=NULL;
    esp_audio_err_t dec_err=esp_mp3_dec_open(NULL,0,&decoder);
    if(dec_err!=ESP_AUDIO_ERR_OK || !decoder) {
        fclose(f); snprintf(s_status,sizeof(s_status),"MP3 DECODER ERROR"); return false;
    }
    uint8_t *input=malloc(MP3_INPUT_BYTES);
    uint8_t *decoded=malloc(MP3_OUTPUT_BYTES);
    if(!input || !decoded) {
        free(input); free(decoded); (void)esp_mp3_dec_close(decoder); fclose(f);
        snprintf(s_status,sizeof(s_status),"MP3 MEMORY ERROR"); return false;
    }
    uint32_t input_len=0;
    bool eof=false, ok=true, have_info=false;
    uint64_t decoded_frames=0;
    int seek_grace=0;
    esp_audio_dec_info_t info={0};
    memcpy(s_info_format,"MP3",4);
    s_info_rate=0; s_info_bitrate=0; s_info_bits=16; s_info_channels=2;
    s_info_file_size=file_size; s_info_elapsed_ms=resume_ms; s_info_duration_ms=0; s_info_duration_estimated=true;
    snprintf(s_track_path,sizeof(s_track_path),"%s",path);
    s_playing=true; s_paused=false; snprintf(s_status,sizeof(s_status),"PLAYING"); publish_view();
    while(s_playing && ok) {

        int32_t seek_delta=consume_seek_request();
        if(seek_delta && s_info_bitrate && info.sample_rate) {
            int64_t target_ms=(int64_t)s_info_elapsed_ms+seek_delta;
            int64_t payload=file_size>(uint64_t)data_offset?(int64_t)(file_size-(uint64_t)data_offset):0;
            int64_t duration_ms=s_info_bitrate?payload*1000LL/(int64_t)s_info_bitrate:0;
            if(target_ms<0) target_ms=0;
            if(duration_ms && target_ms>duration_ms) target_ms=duration_ms;
            long off=(long)data_offset+(long)((uint64_t)target_ms*s_info_bitrate/8000ULL);
            if(off<(long)file_size && fseek(f,off,SEEK_SET)==0) {
                decoded_frames=(uint64_t)target_ms*info.sample_rate/1000ULL;
                s_info_elapsed_ms=(uint32_t)target_ms;
                input_len=0;
                eof=false;
                seek_grace=32;
                snprintf(s_status,sizeof(s_status),"%s xS",seek_delta>0?"FWD":"REW");
                publish_track_info();
                publish_view();
                ESP_LOGI(TAG,"MP3 seek %+d ms -> %u ms (offset %ld)",
                         (int)seek_delta,(unsigned)target_ms,off);
            } else {
                ESP_LOGW(TAG,"MP3 seek to %u ms (offset %ld) rejected",
                         (unsigned)target_ms,off);
            }
        }
        if(!mp3_service_keys()) {
            if(!s_playing) break;
            continue;
        }
        if(input_len<MP3_INPUT_BYTES && !eof) {
            size_t n=fread(input+input_len,1,MP3_INPUT_BYTES-input_len,f);
            input_len+=(uint32_t)n;
            if(n==0) eof=true;
        }
        if(!input_len) break;
        esp_audio_dec_in_raw_t raw={.buffer=input,.len=input_len};
        esp_audio_dec_out_frame_t frame={.buffer=decoded,.len=sizeof(decoded)};
        dec_err=esp_mp3_dec_decode(decoder,&raw,&frame,&info);
        if(raw.consumed>input_len) { ok=false; break; }
        if(raw.consumed) {
            input_len-=raw.consumed;
            memmove(input,input+raw.consumed,input_len);
        }
        if(dec_err==ESP_AUDIO_ERR_OK && frame.decoded_size) {
            if(frame.decoded_size>MP3_OUTPUT_BYTES || (info.channel!=1 && info.channel!=2) ||
               frame.decoded_size%(2U*info.channel) ||
               info.bits_per_sample!=16 || info.sample_rate<8000 || info.sample_rate>96000) {
                snprintf(s_status,sizeof(s_status),"MP3 FORMAT UNSUPPORTED"); ok=false; break;
            }
            if(!have_info || s_info_rate!=info.sample_rate) {
                s_info_rate=info.sample_rate; s_info_bits=info.bits_per_sample;
                s_info_channels=info.channel; have_info=true;
                if(decoded_frames==0 && resume_ms)
                    decoded_frames=(uint64_t)resume_ms*info.sample_rate/1000ULL;
                if(audio_init(info.sample_rate,I2S_MCLK_MULTIPLE_256,false)!=ESP_OK) { snprintf(s_status,sizeof(s_status),"AUDIO INIT ERROR"); ok=false; break; }
                s_bass_lowpass[0]=s_bass_lowpass[1]=0.0f;
                s_bass_dc[0]=s_bass_dc[1]=0.0f; s_bass_dbfs=-120.0f; s_bass_rate=info.sample_rate;
            }
            if(info.bitrate) s_info_bitrate=info.bitrate;
            uint64_t payload=file_size>(uint64_t)data_offset?file_size-(uint64_t)data_offset:0;
            if(s_info_bitrate) s_info_duration_ms=(uint32_t)(payload*8000ULL/s_info_bitrate);
            publish_track_info();
            uint32_t source_frames=frame.decoded_size/(2U*info.channel);
            const int16_t *pcm=(const int16_t *)decoded;
            int32_t out[AUDIO_FRAMES*2];
            uint32_t offset=0;
            while(offset<source_frames && s_playing) {
                if(!mp3_service_keys()) { if(!s_playing) break; else continue; }
                uint32_t count=source_frames-offset;
                if(count>AUDIO_FRAMES) count=AUDIO_FRAMES;
                float gain=(s_volume/100.0f)*s_gain;
                float gain_l=gain*powf(10.0f,(float)s_channel_gain_db[4]*0.05f);
                float gain_r=gain*powf(10.0f,(float)s_channel_gain_db[5]*0.05f);
                for(uint32_t i=0;i<count;i++) {
                    int16_t left=pcm[(offset+i)*info.channel];
                    int16_t right=info.channel==2?pcm[(offset+i)*2+1]:left;
                    out[2*i]=(int32_t)((float)left*gain_l)*65536;
                    out[2*i+1]=(int32_t)((float)right*gain_r)*65536;
                }
                if(!output_pcm_block(out,count,info.sample_rate)) { ok=false; break; }
                offset+=count; decoded_frames+=count;
                s_info_elapsed_ms=(uint32_t)(decoded_frames*1000ULL/info.sample_rate);
                publish_track_info();
            }
        } else if(dec_err==ESP_AUDIO_ERR_DATA_LACK) {
            if(eof) break;
            if(!raw.consumed && input_len==MP3_INPUT_BYTES) {
                input_len--; memmove(input,input+1,input_len);
            }
        } else {

            if(seek_grace>0) { seek_grace--; continue; }
            ESP_LOGE(TAG,"MP3 decode error after %u ms of output",s_info_elapsed_ms);
            snprintf(s_status,sizeof(s_status),"MP3 DECODE ERROR"); ok=false;
        }
    }
    (void)esp_mp3_dec_close(decoder);
    free(decoded);
    free(input);
    fclose(f);
    volume_save_poll();
    if(s_playing && ok) {
        snprintf(s_status,sizeof(s_status),"TRACK FINISHED");
        s_player_focus=false; s_page=DISPLAY_STANDALONE_LISTEN;
    } else if(s_playing) {
        if(!strcmp(s_status,"PLAYING")) snprintf(s_status,sizeof(s_status),"PLAYBACK ERROR");
        s_player_focus=false; s_page=DISPLAY_STANDALONE_LISTEN;
    } else if(!s_request_play) {
        snprintf(s_status,sizeof(s_status),"STOPPED"); s_player_focus=false;
    }
    s_playing=false; s_paused=false;
    return ok;
}

// Plays currently highlighted file
static void play_selected(void)
{
    if(s_monitor_on) monitor_set(false);
    if(!s_entry_count) return;
    if(s_entries[s_selected].directory) {
        if(strlen(s_path)+strlen(s_entries[s_selected].name)+2>=sizeof(s_path)) return;
        strcat(s_path,"/"); strcat(s_path,s_entries[s_selected].name);
        s_selected=0; scan_directory(); return;
    }
    if(!supported_audio(s_entries[s_selected].name)) return;
    char selected_path[520];
    make_selected_path(selected_path,sizeof(selected_path));
    s_player_focus=true;
    uint32_t resume_ms=s_resume_request?s_resume_elapsed_ms:0;
    bool resume_mp3=s_resume_request && s_resume_mp3;
    if(s_resume_request) s_resume_available=false;
    s_resume_request=false;
    s_resume_mp3=resume_mp3;
    bool ok=ends_with_wav(s_entries[s_selected].name)
        ?play_wav_file(selected_path,resume_ms):play_mp3_file(selected_path,resume_ms);
    if(!ok) s_player_focus=false;
    publish_view();
}

// u/d/l/r handling
static void standalone_key(display_key_t key, bool long_press)
{
    if(!s_commands) return;
    key_command_t command={.key=(command_key_t)key,.long_press=long_press};
    (void)xQueueSend(s_commands,&command,0);
}

// watches active mode/monitor/recording states and restarts UI when switching 
static void player_task(void *arg)
{
    (void)arg;
    scan_directory();
    for(;;) {
        if(s_recording_start_pending) {
            s_recording_start_pending=false;
            esp_err_t rec_err=recording_start();
            if(rec_err!=ESP_OK) {
                ESP_LOGE(TAG,"Could not start recording: %s",esp_err_to_name(rec_err));
                snprintf(s_status,sizeof(s_status),"REC START FAILED");
            } else if(s_recording_monitor_pending) {
                while(recording_is_active() && !recording_is_ready())
                    vTaskDelay(pdMS_TO_TICKS(10));
                if(recording_is_ready()) {
                    esp_err_t monitor_err=standalone_recording_monitor_enable();
                    if(monitor_err!=ESP_OK) {
                        ESP_LOGE(TAG,"Could not initialize live monitor for recording: %s",
                                 esp_err_to_name(monitor_err));
                        snprintf(s_status,sizeof(s_status),"MON DAC ERROR");
                    }
                }
            }
            publish_view();
            continue;
        }
        if(s_request_play) {
            s_request_play=false;
            play_selected();
            if(s_stop_requested) break;
            continue;
        }
        key_command_t command;

        if(xQueueReceive(s_commands,&command,pdMS_TO_TICKS(400))==pdTRUE) {
            process_command(command);
            publish_view();
        } else volume_save_poll();
        volume_save_poll();
        if(s_stop_requested) break;
    }
    if(s_tx) {
        (void)i2s_channel_disable(s_tx);
        (void)i2s_del_channel(s_tx);
        s_tx=NULL;
    }
    s_player_task_running=false;
    xSemaphoreGive(s_player_stopped);
    vTaskDelete(NULL);
}

// Boot
esp_err_t standalone_start(void)
{
    if(!s_record_monitor_mutex) {
        s_record_monitor_mutex=xSemaphoreCreateMutex();
        if(!s_record_monitor_mutex) return ESP_ERR_NO_MEM;
    }
    nvs_handle_t gain_nvs;
    if(nvs_open("audio",NVS_READONLY,&gain_nvs)==ESP_OK) {
        for(uint8_t i=0;i<6;i++) {
            char key[8]; snprintf(key,sizeof(key),"ch%u",(unsigned)i);
            int8_t value=0;
            if(nvs_get_i8(gain_nvs,key,&value)==ESP_OK) {
                if(value < -12)value=-12;
                if(value > 12)value=12;
                s_channel_gain_db[i]=value;
            }
        }
        int8_t saved_yaw=0;
        if(nvs_get_i8(gain_nvs,"mon_rot",&saved_yaw)==ESP_OK && saved_yaw>=0 && saved_yaw<=3) {
            s_monitor_yaw=saved_yaw;
        }
        int8_t saved_val=1;
        if(nvs_get_i8(gain_nvs,"mon_dec",&saved_val)==ESP_OK) s_mon_decode=saved_val!=0;
        saved_val=0;
        if(nvs_get_i8(gain_nvs,"mon_eq",&saved_val)==ESP_OK) s_mon_eq_on=saved_val!=0;
        saved_val=0;
        if(nvs_get_i8(gain_nvs,"rec_mon",&saved_val)==ESP_OK)
            s_record_monitor_enabled=saved_val!=0;
        saved_val=0;
        if(nvs_get_i8(gain_nvs,"rec_agc",&saved_val)==ESP_OK)
            recording_set_auto_gain(saved_val!=0);
        saved_val=1;
        if(nvs_get_i8(gain_nvs,"wav_ambi",&saved_val)==ESP_OK)
            s_wav_ambi_decode=saved_val!=0;
        uint8_t saved_volume=0;
        if(nvs_get_u8(gain_nvs,"playvol",&saved_volume)==ESP_OK &&
           saved_volume>=1 && saved_volume<=100) {
            s_volume=saved_volume;
        }
        size_t eq_path_size=sizeof(s_eq_path);
        if(nvs_get_str(gain_nvs,"eq_path",s_eq_path,&eq_path_size)==ESP_OK &&
           eq_path_size>0 && eq_path_size<=sizeof(s_eq_path)) {
            s_eq_path[sizeof(s_eq_path)-1]='\0';
        } else {
            snprintf(s_eq_path,sizeof(s_eq_path),"%s",EQ_FILE_PATH);
        }

        nvs_close(gain_nvs);
    }
    if(s_mon_eq_on) eq_set_enabled(true);
    esp_err_t led_err=standalone_led_init();
    if(led_err==ESP_OK)     {
        recording_set_sd_mounted(false);
        const esp_timer_create_args_t led_timer_args={
            .callback=standalone_led_timer_callback,
            .name="standalone_led",
            .dispatch_method=ESP_TIMER_TASK,
        };
        led_err=esp_timer_create(&led_timer_args,&s_led_timer);
        if(led_err==ESP_OK) led_err=esp_timer_start_periodic(s_led_timer,5000);
        if(led_err!=ESP_OK) ESP_LOGW(TAG,"Could not start 100 Hz standalone LED blink: %s",esp_err_to_name(led_err));
    } else ESP_LOGW(TAG,"Could not initialize standalone LED: %s",esp_err_to_name(led_err));
    s_commands=xQueueCreate(8,sizeof(key_command_t));
    if(!s_commands) return ESP_ERR_NO_MEM;
    s_player_stopped=xSemaphoreCreateBinary();
    if(!s_player_stopped) return ESP_ERR_NO_MEM;
    s_stop_requested=false;
    display_set_standalone_key_callback(standalone_key);
    esp_ldo_channel_config_t ldo={.chan_id=4,.voltage_mv=3300,.voltage_stable_delay_us=200};
    esp_err_t err=esp_ldo_acquire_channel(&ldo,&s_sd_ldo);
    if(err!=ESP_OK) {
        snprintf(s_status,sizeof(s_status),"SD POWER ERROR");
        BaseType_t created=xTaskCreate(player_task,"standalone_player",24576,NULL,5,NULL);
        s_player_task_running=created==pdPASS;
        publish_view();
        return created==pdPASS?err:ESP_ERR_NO_MEM;
    }
    sdmmc_host_t host=SDMMC_HOST_DEFAULT(); host.max_freq_khz=SDMMC_FREQ_DEFAULT;
    sdmmc_slot_config_t slot=SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk=GPIO_NUM_43; slot.cmd=GPIO_NUM_44;
    slot.d0=GPIO_NUM_39; slot.d1=GPIO_NUM_40; slot.d2=GPIO_NUM_41; slot.d3=GPIO_NUM_42;
    slot.width=4; slot.flags|=SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    esp_vfs_fat_sdmmc_mount_config_t mount={.format_if_mount_failed=false,.max_files=8,.allocation_unit_size=16*1024};
    err=esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT,&host,&slot,&mount,&s_card);
    if(err!=ESP_OK) {
        ESP_LOGE(TAG,"SD FAT mount failed: %s",esp_err_to_name(err));
        snprintf(s_status,sizeof(s_status),"SD CARD / FAT ERROR");
        BaseType_t created=xTaskCreate(player_task,"standalone_player",24576,NULL,5,NULL);
        s_player_task_running=created==pdPASS;
        publish_view();
        return created==pdPASS?err:ESP_ERR_NO_MEM;
    }
    s_mounted=true;
    recording_set_sd_mounted(true);
    (void)settings_load_eq();
    eq_set_enabled(s_mon_eq_on && s_eq_loaded);
    BaseType_t created=xTaskCreate(player_task,"standalone_player",24576,NULL,5,NULL);
    s_player_task_running=created==pdPASS;
    return created==pdPASS?ESP_OK:ESP_ERR_NO_MEM;
}

// Stops before handing the app over to UAC2 Mode
esp_err_t standalone_stop_for_usb(void)
{
    volume_save_now();
    if(s_monitor_on) monitor_set(false);
    if(s_player_task_running) {
        if(!s_player_stopped || !s_commands) return ESP_ERR_INVALID_STATE;
        key_command_t stop={.key=CMD_STOP,.long_press=false};
        if(xQueueSend(s_commands,&stop,pdMS_TO_TICKS(100))!=pdTRUE) return ESP_ERR_TIMEOUT;
        if(xSemaphoreTake(s_player_stopped,portMAX_DELAY)!=pdTRUE) return ESP_FAIL;
    }
    if(s_led_timer) {
        (void)esp_timer_stop(s_led_timer);
        (void)esp_timer_delete(s_led_timer);
        s_led_timer=NULL;
    }
    if(s_led) {
        if(s_led_in_flight) {
            (void)led_strip_refresh_async_done(s_led);
            s_led_in_flight=false;
        }
        (void)led_strip_clear(s_led);
        (void)led_strip_del(s_led);
        s_led=NULL;
    }
    return ESP_OK;
}
