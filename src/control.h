// ----------------------------------------------------------------------------
// defines functions for multipass to send events to the controller (grid 
// presses etc)
//
// defines functions for engine to send updates (note on etc)
//
// defines data structures for multipass preset management
// ----------------------------------------------------------------------------

#pragma once
#include "types.h"


// ----------------------------------------------------------------------------
// firmware dependent stuff starts here

#define ES_EVENTS_PER_PATTERN 128
#define VOICE_COUNT 8
#define DEVICE_COUNT 5
#define MOD_COUNT 8
#define PATTERN_COUNT 16


// ----------------------------------------------------------------------------
// shared types

typedef struct {
	u8 active;
    s8 x;
    s8 y;
    u32 start;
    u8 from_pattern;
} es_note_t;

typedef struct {
	u8 on;
    u8 index;
    u16 interval;
} es_event_t;

typedef struct {
	es_event_t e[ES_EVENTS_PER_PATTERN];
    u16 interval_ind;
	u16 length;
	u8 loop;
    u8 root_x;
    u8 root_y;
    u8 edge;
    u16 edge_time;
    u8 voices;
    u8 dir;
    u8 linearize;
    u8 start;
    u8 end;
} es_pattern_t;

typedef struct {
	u8 arp;
    u8 p_select;
    u8 p_leader;
    u8 leader_reset;
    u8 is_leader_clock;
    u16 keymap[128];
    es_pattern_t p[PATTERN_COUNT];
    u8 midi_cc[MOD_COUNT];
    u8 jf_mode;

    // current values
    u8 octave[VOICE_COUNT][DEVICE_COUNT];
    u8 transpose[VOICE_COUNT][DEVICE_COUNT];
    u8 er301_volume[VOICE_COUNT];
    u8 jf_volume[VOICE_COUNT];
    u8 txo_volume[VOICE_COUNT];
    u8 txo_wave[VOICE_COUNT];
    u8 txo_wavef[VOICE_COUNT];
    u8 txo_attack[VOICE_COUNT];
    u8 txo_decay[VOICE_COUNT];
    
    // selected range
    u8 octave_mm[VOICE_COUNT][DEVICE_COUNT];
    u8 er301_volume_mm[VOICE_COUNT];
    u8 jf_volume_mm[VOICE_COUNT];
    u8 txo_volume_mm[VOICE_COUNT];
    u8 txo_wave_mm[VOICE_COUNT];
    u8 txo_wavef_mm[VOICE_COUNT];
    u8 txo_attack_mm[VOICE_COUNT];
    u8 txo_decay_mm[VOICE_COUNT];

    // voices mapped to things
    // bitmaps, make u16 if more than 8 voices
    u8 voice_map[VOICE_COUNT][DEVICE_COUNT]; // bitmap is outputs
    u8 pstart_map[PATTERN_COUNT][DEVICE_COUNT];
    u8 cv_x_map[VOICE_COUNT];
    u8 cv_y_map[VOICE_COUNT];
    u8 cv_key_map[VOICE_COUNT]; // bitmap is outputs
    u8 er301_x_map[VOICE_COUNT];
    u8 er301_y_map[VOICE_COUNT];
    u8 er301_key_map[VOICE_COUNT];
    u8 txo_x_map[VOICE_COUNT];
    u8 txo_y_map[VOICE_COUNT];
    u8 txo_key_map[VOICE_COUNT];
    u8 mod_x[MOD_COUNT];
    u8 mod_y[MOD_COUNT];
    u8 mod_key[MOD_COUNT];
    u8 mod_gate[MOD_COUNT];
    u16 mod_pstart[MOD_COUNT];
    
    // mod dest mapped to buses
    u8 mod_octave[MOD_COUNT]; // bitmap is outputs
    u8 mod_er301_octave[MOD_COUNT];
    u8 mod_jf_octave[MOD_COUNT];
    u8 mod_txo_octave[MOD_COUNT];
    u8 mod_er301_volume[MOD_COUNT];
    u8 mod_jf_volume[MOD_COUNT];
    u8 mod_txo_volume[MOD_COUNT];
    u8 mod_txo_attack[MOD_COUNT];
    u8 mod_txo_decay[MOD_COUNT];
    u8 mod_txo_wave[MOD_COUNT];
    u8 mod_txo_wavef[MOD_COUNT];
    
    u8 mod_value[MOD_COUNT];
} preset_data_t;

typedef struct {
	u8 glyph[8];
} preset_meta_t;

typedef struct {
    u8 i2c_mode;
    u8 device_on[DEVICE_COUNT];
    u8 voice_on[VOICE_COUNT];
} shared_data_t;


// ----------------------------------------------------------------------------
// firmware settings/variables main.c needs to know


// ----------------------------------------------------------------------------
// functions control.c needs to implement (will be called from main.c)

void init_presets(void);
void init_control(void);
void process_event(u8 event, u8 *data, u8 length);
void render_grid(void);
void render_arc(void);


// ----------------------------------------------------------------------------
// functions engine needs to call
