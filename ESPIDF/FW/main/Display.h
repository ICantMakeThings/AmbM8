// API for display.c

#pragma once
#include <stdbool.h>
#include <stdint.h>

// Display mode & SD IO state enums

typedef enum { DISPLAY_MODE_UAC2, DISPLAY_MODE_STORAGE, DISPLAY_MODE_STANDALONE } display_mode_t;
typedef enum { DISPLAY_IO_NONE, DISPLAY_IO_READING, DISPLAY_IO_WRITING, DISPLAY_IO_UNMOUNTED } display_io_t;
// AUDIO STATS page stuf 
typedef struct {
    uint32_t bytes_per_sec;
    uint32_t fifo_bytes_per_sec;
    uint32_t packets_per_sec;
    uint32_t queue_high_water;
    uint32_t drops;
    uint32_t queue_frames;
    int32_t asrc_ppm;
    uint32_t underruns;
    bool clipping;
    uint32_t rx_calls_per_sec;
    uint32_t rx_frames_per_sec;
    uint32_t rx_len_min;
    uint32_t rx_len_max;
    uint32_t rx_gap_max_us;
    uint32_t i2s_write_max_us;
    uint32_t i2s_write_errors;
} display_audio_stats_t;

typedef struct {
    uint32_t usb_packets_per_sec, usb_bytes_per_sec, fifo_bytes_per_sec;
    uint32_t task_packets_per_sec, queue_high_water, queue_depth, drops;
} display_usb_stats_t;

// mode and state setters / flag repaintes
void display_start(display_mode_t mode);
void display_set_mode(display_mode_t mode);
void display_set_io(display_io_t state);
void display_set_storage(uint64_t total_bytes, uint64_t free_bytes, bool mounted);
void display_set_audio_stats(const display_audio_stats_t *stats);
void display_set_usb_stats(const display_usb_stats_t *stats);
void display_set_clipping(bool clipping);
void display_set_audio_levels(const uint8_t levels[6]);
void display_set_output_meter(float o1_dbfs, float o2_dbfs, bool clipping);
void display_set_channel_gain(uint8_t channel, int8_t gain_db);
// spectrum
void display_set_main_volume(uint8_t percent);
void display_set_spectrum(const float dbfs[16]);
bool display_spectrum_active(void);
void display_select_spectrum_source(uint8_t source);
uint8_t display_spectrum_source(void);
bool display_advanced_page(void);

// Standalone key ids, page enums and control callback... sm sm..

typedef enum { DISPLAY_KEY_UP, DISPLAY_KEY_DOWN, DISPLAY_KEY_LEFT, DISPLAY_KEY_RIGHT } display_key_t;
typedef enum { DISPLAY_STANDALONE_HOME, DISPLAY_STANDALONE_EQ, DISPLAY_STANDALONE_INFO, DISPLAY_STANDALONE_LISTEN, DISPLAY_STANDALONE_MONITOR, DISPLAY_STANDALONE_SETTINGS, DISPLAY_STANDALONE_RECORDING } display_standalone_page_t;
typedef void (*display_standalone_key_cb_t)(display_key_t key, bool long_press);
void display_set_standalone_key_callback(display_standalone_key_cb_t callback);
typedef enum {
    DISPLAY_CONTROL_PLAY_PAUSE, DISPLAY_CONTROL_NEXT, DISPLAY_CONTROL_PREVIOUS,
    DISPLAY_CONTROL_REC, DISPLAY_CONTROL_ENCODER_STEP,
    DISPLAY_CONTROL_ENCODER_CLICK, DISPLAY_CONTROL_ENCODER_DOUBLE,
    DISPLAY_CONTROL_ENCODER_HOLD, DISPLAY_CONTROL_MONITOR,

    DISPLAY_CONTROL_SEEK_FWD, DISPLAY_CONTROL_SEEK_BACK
} display_control_event_t;
typedef void (*display_control_cb_t)(display_control_event_t event, int value);
void display_set_control_callback(display_control_cb_t callback);
// Standalone TRACK INFO data
void display_set_standalone_view(const char *path, const char *const *entries, uint8_t count,
                                 uint8_t selected, bool playing, uint8_t volume,
                                 display_standalone_page_t page, bool paused,
                                 const char *track, const char *status);
void display_set_standalone_track_info(const char *format, uint32_t sample_rate,
                                       uint32_t bitrate, uint8_t bits, uint8_t channels,
                                       uint64_t file_size, uint32_t elapsed_ms,
                                       uint32_t duration_ms, bool duration_estimated);
void display_set_recording_track(const char *filename);
void display_set_recording_active(bool active);
bool display_set_reduced_motion(bool enabled);
bool display_reduced_motion(void);
void display_set_standalone_status(const char *status);
void display_set_monitor_view(bool on, uint8_t yaw, const float wxyz_db[4],
                              const float out_db[2], bool clip);
// menu ids
typedef enum {
    SETTINGS_MENU_ROOT = 0, SETTINGS_MENU_MONITOR = 1, SETTINGS_MENU_INPUTS = 2,
    SETTINGS_MENU_RECPROC = 3, SETTINGS_MENU_MICMAP = 4, SETTINGS_MENU_RESET = 5,
    SETTINGS_MENU_RECORDING = 6, SETTINGS_MENU_EQ_FILES = 7
} display_settings_menu_t;

// settings setter
void display_set_settings_view(display_settings_menu_t menu, uint8_t selected,
                               uint8_t count, const char *const *items,
                               const char *title, const char *hint);

void display_set_tap_view(bool on, const uint8_t levels[4],
                          const char *const labels[4], uint8_t count);

void display_set_recording_labels(const char *l1, const char *l2,
                                  const char *l3, const char *l4, uint8_t channels);
