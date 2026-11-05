// API for UAC2.c

#pragma once
#include <stdbool.h>

void uac2_start(void);

void uac2_encoder_step(int direction);
void uac2_toggle_encoder_mode(void);
void uac2_encoder_click(void);
void uac2_toggle_mute(void);
void uac2_encoder_hold(void);
void uac2_probe_start(void);
bool uac2_high_speed_host_connected(void);

#define UAC2_MEDIA_PLAY_PAUSE 0x00CDu
#define UAC2_MEDIA_NEXT_TRACK 0x00B5u
#define UAC2_MEDIA_PREV_TRACK 0x00B6u

void uac2_host_media(uint16_t usage);
