// -----------------------------------------------------------------------------
// controller - the glue between the engine and the hardware
//
// reacts to events (grid press, clock etc) and translates them into appropriate
// engine actions. reacts to engine updates and translates them into user 
// interface and hardware updates (grid LEDs, CV outputs etc)
//
// should talk to hardware via what's defined in interface.h only
// should talk to the engine via what's defined in engine.h only
// ----------------------------------------------------------------------------

#include "compiler.h"
#include "string.h"

#include "control.h"
#include "interface.h"
#include "engine.h"


// ----------------------------------------------------------------------------
// firmware dependent stuff starts here

#define I2C_ADDRESS 0x50
#define TXOWAVF 15

#define MAX_HELD_KEYS 32
#define GRID_KEY_HOLD_TIME 15

#define ES_CHORD_THRESHOLD 30
#define ES_EDGE_PATTERN 0
#define ES_EDGE_FIXED 1
#define ES_EDGE_DRONE 2

#define ES_BLINKER_TIMER    (VOICE_COUNT + 1)
#define ES_PLAY_POS_TIMER   (VOICE_COUNT + 2)
#define ES_PLAY_TIMER       (VOICE_COUNT + 3) // 16 of these for each pattern
#define ES_PSTART_TIMER     (VOICE_COUNT + 3 + 16) // 16 of these for each pattern
#define ES_KNOB_TIMER       (VOICE_COUNT + 3 + 32)

typedef enum {
	es_stopped,
	es_armed,
	es_recording,
	es_playing
} es_mode_t;

typedef enum {
	es_main,
	es_patterns_held,
	es_patterns,
    es_voices,
    es_voices_held,
    es_devices,
    es_devices_held
} es_view_t;

u8 key_count = 0;
u8 held_keys[MAX_HELD_KEYS];
u8 key_times[128];

u8 playing[16];
u8 preset_mode;
u8 preset, selected_preset, selected_voice;
u8 sel_device, sel_param, sel_mapping, sel_pattern_start;
u8 prev_param_x, prev_param_y;
u8 sel_mod = 255;
preset_meta_t meta;
preset_data_t e;
shared_data_t shared;
es_mode_t es_mode;
es_view_t es_view;
u8 es_blinker;
u8 es_runes, es_edge, es_voices_changed, es_devices_changed;
u32 es_tick;
u16 es_pos[16];
es_note_t es_notes[VOICE_COUNT];
u32 es_p_start, es_p_total;
u8 es_ignore_arm_release;
u8 help_on, help_sel, pattern_leader, pattern_held = 255;
static u8 midi_notes[VOICE_COUNT];
static u8 midi_cc_ind;
static u8 wtf_update_devices = 0;

static void read_knobs(void);
static void save_to_preset(u8 p);
static void update_devices(void);
static void remap_voices(void);
static void pattern_start(u8 pattern, u8 on);
static void update_mod(u8 voice, u8 x, u8 y, u8 index);
static void update_mod_off(u8 voice);
static void update_mod_arc(u8 enc, s8 delta);
static void update_i2c(void);
static void event_main_clock(u8 external, u8 phase);
static void event_front_button(u8 pressed);
static void load_preset(void);
static void load_preset(void);
static void external_clock_received(void);
static void external_clock_switched(u8 external);
static void handler_ESGridKey(u8 x, u8 y, u8 z);
static void es_blinker_callback(void);
static void es_play_callback(u8 pattern);
static void es_play_pos_callback(void);
static void es_note_off_callback(u8 index);
static void es_reset_all_playing(void);
static void process_timed_event(u8 index);
static void ii_es(uint8_t *data, uint8_t l);
static void grid_key_held(u8 x, u8 y);
static void copy_pattern(u8 src, u8 dest);
static void es_start_playback(u8 pattern, u8 pos);
static void es_stop_playback(u8 pattern);
static void es_prev_pattern(void);
static void es_next_pattern(void);
static u8 es_note_on(s8 x, s8 y, u8 from_pattern, u16 timer, u8 voices);
static u8 es_note_off(s8 x, s8 y);
static void update_dest(u8 mod);
static void clock_next_note(u8 is_leader);
static void es_start_recording(void);
static void es_record_pattern_note(u8 x, u8 y, u8 z);
static void es_kill_pattern_notes(u8 pattern);


// ----------------------------------------------------------------------------
// functions for main.c

void init_control(void) {
    load_shared_data_from_flash(&shared);
    update_i2c();
    
    preset = selected_preset = get_preset_index();
    load_preset();
    
    es_mode = es_stopped;
    es_view = es_main;
    preset_mode = 0;
    selected_voice = 0;
    for (u8 i = 0; i < 16; i++) playing[i] = 0;
    
    add_timed_event(ES_BLINKER_TIMER, 288, 1);
    add_timed_event(ES_KNOB_TIMER, 100, 1);
    
    clear_all_grid_leds();
    refresh_grid();
    update_devices();
}

void process_event(u8 event, u8 *data, u8 length) {
    u8 found;
    u8 voice;
    u8 index;
    
    switch (event) {
        case MAIN_CLOCK_RECEIVED:
            event_main_clock(data[0], data[1]);
            break;
        
        case MAIN_CLOCK_SWITCHED:
            external_clock_switched(data[0]);
            break;
    
        case GATE_RECEIVED:
            if (data[0] == 7) event_main_clock(1, data[1]);
            else if (data[1]) es_start_playback(e.p_select, 0);
            break;
        
        case GRID_CONNECTED:
            es_view = es_main;
            break;
        
        case GRID_KEY_PRESSED:
            handler_ESGridKey(data[0], data[1], data[2]);
            break;
    
        case GRID_KEY_HELD:
            grid_key_held(data[0], data[1]);
            break;
            
        case ARC_ENCODER_COARSE:
            update_mod_arc(data[0], data[1]);
            break;
    
        case FRONT_BUTTON_PRESSED:
            event_front_button(data[0]);
            break;
    
        case FRONT_BUTTON_HELD:
            save_to_preset(preset);
            break;
    
        case BUTTON_PRESSED:
            if (data[1]) {
                if (data[0] == 0)
                    es_prev_pattern();
                else if (data[0] == 1)
                    es_next_pattern();
            }
            break;
    
        case I2C_RECEIVED:
            ii_es(data, length);
            break;
            
        case TIMED_EVENT:
            process_timed_event(data[0]);
            break;
        
        case MIDI_CONNECTED:
            break;
        
        case MIDI_NOTE:
            found = 0;
            data[1] -= 24;
            for (u8 x = 1; x < 16; x++) {
                for (u8 y = 0; y < 7; y++) {
                    u16 note_index = x + (7 - y) * 5 - 1;
                    if (note_index == data[1]) {
                        found = 1;
                        if (data[0] == 0) {
                            if (es_mode == es_armed) es_start_recording(); // will change es_mode to es_recording
                            if (es_mode == es_recording) es_record_pattern_note(x, y, data[3]);
                        }
                        if (data[3]) {
                            index = data[0] ? data[0] : e.p_select;
                            voice = es_note_on(x, y, 255, 0, e.p[index].voices);
                            if (voice != 255) midi_notes[voice] = data[1];
                        } else {
                            voice = es_note_off(x, y);
                            if (voice != 255) midi_notes[voice] = 255;
                        }
                        break;
                    }
                }
                if (found) break;
            }
            
            break;
        
        case MIDI_CC:
            found = MOD_COUNT;
            for (u8 i = 0; i < MOD_COUNT; i++) {
                if (e.midi_cc[i] == data[1]) {
                    found = i;
                    break;
                }
            }
            if (es_mode == es_armed && found == MOD_COUNT && midi_cc_ind < MOD_COUNT) {
                e.midi_cc[midi_cc_ind] = data[1];
                found = midi_cc_ind;
                midi_cc_ind++;
            }
            if (found < MOD_COUNT) {
                index = get_knob_count() + found;
                if (index < MOD_COUNT) {
                    e.mod_value[index] = data[2] << 1;
                    update_dest(index);
                }
            }
            break;
            
        case MIDI_AFTERTOUCH:
        
            /*
            voice = 2;
            for (u8 i = 0; i < VOICE_COUNT; i++) {
                if (midi_notes[i] == data[1]) voice = i;
            }
            set_cv(voice, (MAX_LEVEL >> 7) * data[2]);
            set_cv(0, MAX_LEVEL);
            */
            break;
            
        case SHNTH_BAR:
            index = get_knob_count() + data[0];
            if (index < MOD_COUNT) {
                e.mod_value[index] = data[1];
                update_dest(index);
                // set_cv(index, (MAX_LEVEL >> 8) * data[1]);
            }
            break;
            
        case SHNTH_ANTENNA:
            index = get_knob_count() + data[0] + 4;
            if (index < MOD_COUNT) {
                e.mod_value[index] = data[1];
                update_dest(index);
            }
            break;
            
        case SHNTH_BUTTON:
            index = data[0] >> 1;
            if (data[0] & 1) { // play/stop
                if (data[1]) {
                    if (playing[index]) {
                        print_int("STOPPING", index);
                        es_stop_playback(index);
                    } else {
                        print_int("STARTING", index);
                        es_start_playback(index, 0);
                    }
                }
            } else { // pause
                if (playing[index]) {
                    print_int("PAUSING", index);
                    es_stop_playback(index);
                } else {
                    print_int("UNPAUSING", index);
                    es_start_playback(index, es_pos[index]);
                }
            }
            break;
            
        default:
            break;
    }
}

void event_main_clock(u8 external, u8 phase) {
    if (external & phase) external_clock_received();
}

void event_front_button(u8 pressed) {
    if (!pressed) return;
    
    if (is_midi_connected()) {
        if (es_mode == es_playing)
            es_stop_playback(e.p_select);
        else if (es_mode == es_stopped) {
            es_mode = es_armed;
            midi_cc_ind = 0;
        } else {
            e.p[e.p_select].loop = 1;
            es_start_playback(e.p_select, 0);
        }
        return;
    }
    
    preset_mode = !preset_mode;
    refresh_grid();
}

void process_timed_event(u8 index) {
    if (index == ES_BLINKER_TIMER)
        es_blinker_callback();
    else if (index == ES_KNOB_TIMER)
        read_knobs();
    else if (index == ES_PLAY_POS_TIMER)
        es_play_pos_callback();
    else if (index < VOICE_COUNT)
        es_note_off_callback(index);
    else if (index >= ES_PLAY_TIMER && index < ES_PSTART_TIMER)
        es_play_callback(index - ES_PLAY_TIMER);
    else
        pattern_start(index - ES_PSTART_TIMER, 0);
}

// ----------------------------------------------------------------------------
// functions for engine.c


// ----------------------------------------------------------------------------
// main logic

static void es_note_off_i(u8 i) {
    //if (!es_notes[i].active) return;
    es_notes[i].active = 0;
    es_notes[i].from_pattern = 255;
    note_off(i);
    update_mod_off(i);
}

static u8 es_note_off(s8 x, s8 y) {
    u8 voice = 255;
    for (u8 i = 0; i < VOICE_COUNT; i++)
        if (es_notes[i].x == x && es_notes[i].y == y) {
            es_note_off_i(i);
            voice = i;
            //break;
        }
        
    return voice;
}

void es_blinker_callback() {
    es_blinker = !es_blinker;
    refresh_grid();
}

static void es_note_off_callback(u8 note) {
    es_note_off_i(note);
    refresh_grid();
}

static void es_kill_all_notes(void) {
    for (u8 i = 0; i < VOICE_COUNT; i++) es_note_off_i(i);
    refresh_grid();
}

static void update_key_x_y(u8 voice, u8 x, u8 y, u8 key) {
    u8 output;
    for (u8 i = 0; i < VOICE_COUNT; i++) {
        output = 1 << i;
        
        if (e.cv_key_map[voice] & output) set_cv(i, MAX_LEVEL / 49 * key);
        if (e.er301_key_map[voice] & output) set_er301_cv(i, MAX_LEVEL / 49 * key);
        if (e.txo_key_map[voice] & output) set_txo_cv(i, MAX_LEVEL / 49 * key);

        if (e.cv_x_map[voice] & output) set_cv(i, MAX_LEVEL / 15 * (x - 1));
        if (e.er301_x_map[voice] & output) set_er301_cv(i, MAX_LEVEL / 15 * (x - 1));
        if (e.txo_x_map[voice] & output) set_txo_cv(i, MAX_LEVEL / 15 * (x - 1));
        
        if (e.cv_y_map[voice] & output) set_cv(i, MAX_LEVEL / 8 * (7 - y));
        if (e.er301_key_map[voice] & output) set_er301_cv(i, MAX_LEVEL / 8 * (7 - y));
        if (e.txo_y_map[voice] & output) set_txo_cv(i, MAX_LEVEL / 8 * (7 - y));
    }
}

static u16 scale(u16 a, u16 b, u16 x, u16 y, u16 i) {
    if ((b - a) == 0) return 0;
    
    u32 result = (((i - a) * (y - x)) << 1) / (b - a);
    return (result >> 1) + (result & 1) + x;
}

static int16_t push_random(int16_t a, int16_t b) {
    int16_t min, max;
    if (a < b) {
        min = a;
        max = b;
    }
    else {
        min = b;
        max = a;
    }
    int64_t range = max - min + 1;
    if (range == 0 || min == max) return min;

    int64_t rrand = ((int64_t)rand() << 15) + rand();
    rrand = rrand % range + min;
    return rrand;
}

static void update_dest(u8 mod) {
    u8 v, mi, ma;
    
    for (u8 i = 0; i < VOICE_COUNT; i++) {
        v = 1 << i;
        if (e.mod_octave[mod] & v) {
            mi = min(e.octave_mm[i][VOICE_CV_GATE] & 15, e.octave_mm[i][VOICE_CV_GATE] >> 4);
            ma = max(e.octave_mm[i][VOICE_CV_GATE] & 15, e.octave_mm[i][VOICE_CV_GATE] >> 4);
            e.octave[i][VOICE_CV_GATE] = scale(0, 255, mi, ma, e.mod_value[mod]);
            set_output_transpose(VOICE_CV_GATE, i, e.octave[i][VOICE_CV_GATE] * 12 + e.transpose[i][VOICE_CV_GATE]);
        }
        if (e.mod_er301_octave[mod] & v) {
            mi = min(e.octave_mm[i][VOICE_ER301] & 15, e.octave_mm[i][VOICE_ER301] >> 4);
            ma = max(e.octave_mm[i][VOICE_ER301] & 15, e.octave_mm[i][VOICE_ER301] >> 4);
            e.octave[i][VOICE_ER301] = scale(0, 255, mi, ma, e.mod_value[mod]);
            set_output_transpose(VOICE_ER301, i, e.octave[i][VOICE_ER301] * 12 + e.transpose[i][VOICE_ER301]);
        }
        if (e.mod_jf_octave[mod] & v) {
            mi = min(e.octave_mm[i][VOICE_JF] & 15, e.octave_mm[i][VOICE_JF] >> 4);
            ma = max(e.octave_mm[i][VOICE_JF] & 15, e.octave_mm[i][VOICE_JF] >> 4);
            e.octave[i][VOICE_JF] = scale(0, 255, mi, ma, e.mod_value[mod]);
            set_output_transpose(VOICE_JF, i, e.octave[i][VOICE_JF] * 12 + e.transpose[i][VOICE_JF]);
        }
        if (e.mod_txo_octave[mod] & v) {
            mi = min(e.octave_mm[i][VOICE_TXO_NOTE] & 15, e.octave_mm[i][VOICE_TXO_NOTE] >> 4);
            ma = max(e.octave_mm[i][VOICE_TXO_NOTE] & 15, e.octave_mm[i][VOICE_TXO_NOTE] >> 4);
            e.octave[i][VOICE_TXO_NOTE] = scale(0, 255, mi, ma, e.mod_value[mod]);
            mi = min(e.octave_mm[i][VOICE_TXO_CV_GATE] & 15, e.octave_mm[i][VOICE_TXO_CV_GATE] >> 4);
            ma = max(e.octave_mm[i][VOICE_TXO_CV_GATE] & 15, e.octave_mm[i][VOICE_TXO_CV_GATE] >> 4);
            set_output_transpose(VOICE_TXO_NOTE, i, e.octave[i][VOICE_TXO_NOTE] * 12 + e.transpose[i][VOICE_TXO_NOTE]);
            set_output_transpose(VOICE_TXO_CV_GATE, i, e.octave[i][VOICE_TXO_CV_GATE] * 12 + e.transpose[i][VOICE_TXO_CV_GATE]);
        }
        if (e.mod_er301_volume[mod] & v) {
            mi = min(e.er301_volume_mm[i] & 15, e.er301_volume_mm[i] >> 4);
            ma = max(e.er301_volume_mm[i] & 15, e.er301_volume_mm[i] >> 4);
            e.er301_volume[i] = scale(0, 255, mi, ma, e.mod_value[mod]);
            set_output_max_volume(VOICE_ER301, i, (u16)e.er301_volume[i] * (MAX_LEVEL / 11));
        }
        if (e.mod_jf_volume[mod] & v) {
            mi = min(e.jf_volume_mm[i] & 15, e.jf_volume_mm[i] >> 4);
            ma = max(e.jf_volume_mm[i] & 15, e.jf_volume_mm[i] >> 4);
            e.jf_volume[i] = scale(0, 255, mi, ma, e.mod_value[mod]);
            set_output_max_volume(VOICE_JF, i, (u16)e.jf_volume[i] * (MAX_LEVEL / 11));
        }
        if (e.mod_txo_volume[mod] & v) {
            mi = min(e.txo_volume_mm[i] & 15, e.txo_volume_mm[i] >> 4);
            ma = max(e.txo_volume_mm[i] & 15, e.txo_volume_mm[i] >> 4);
            e.txo_volume[i] = scale(0, 255, mi, ma, e.mod_value[mod]);
            set_output_max_volume(VOICE_TXO_NOTE, i, (u16)e.txo_volume[i] * (MAX_LEVEL / 11));
        }
        if (e.mod_txo_attack[mod] & v) {
            mi = min(e.txo_attack_mm[i] & 15, e.txo_attack_mm[i] >> 4);
            ma = max(e.txo_attack_mm[i] & 15, e.txo_attack_mm[i] >> 4);
            e.txo_attack[i] = scale(0, 255, mi, ma, e.mod_value[mod]);
            set_txo_attack(i, e.txo_attack[i] * 50);
        }
        if (e.mod_txo_decay[mod] & v) {
            mi = min(e.txo_decay_mm[i] & 15, e.txo_decay_mm[i] >> 4);
            ma = max(e.txo_decay_mm[i] & 15, e.txo_decay_mm[i] >> 4);
            e.txo_decay[i] = scale(0, 255, mi, ma, e.mod_value[mod]);
            set_txo_decay(i, e.txo_decay[i] * 50);
        }
        if (e.mod_txo_wave[mod] & v) {
            mi = min(e.txo_wave_mm[i] & 15, e.txo_wave_mm[i] >> 4);
            ma = max(e.txo_wave_mm[i] & 15, e.txo_wave_mm[i] >> 4);
            e.txo_wave[i] = scale(0, 255, mi, ma, e.mod_value[mod]);
            set_txo_waveform(i, e.txo_wave[i] * 409 + e.txo_wavef[i] * TXOWAVF); // 0..4500*/
        }
        if (e.mod_txo_wavef[mod] & v) {
            mi = min(e.txo_wavef_mm[i] & 15, e.txo_wavef_mm[i] >> 4);
            ma = max(e.txo_wavef_mm[i] & 15, e.txo_wavef_mm[i] >> 4);
            e.txo_wavef[i] = scale(0, 255, mi, ma, e.mod_value[mod]);
            set_txo_waveform(i, e.txo_wave[i] * 409 + e.txo_wavef[i] * TXOWAVF); // 0..4500*/
        }
    }
    refresh_grid();
}

void update_mod(u8 voice, u8 x, u8 y, u8 index) {
    u8 v = 1 << voice;
    for (u8 i = 0; i < MOD_COUNT; i++) {
        if (e.mod_x[i] & v) {
            e.mod_value[i] = scale(1, 15, 0, 255, x);
            update_dest(i);
        }
        if (e.mod_y[i] & v) {
            e.mod_value[i] = scale(0, 7, 0, 255, 7 - y);
            update_dest(i);
        }
        if (e.mod_key[i] & v) {
            e.mod_value[i] = scale(0, 49, 0, 255, index);
            update_dest(i);
        }
        if (e.mod_gate[i] & v) {
            e.mod_value[i] = push_random(0, 255);
            update_dest(i);
        }
    }
    // set_txo_cv(3, scale(0, 255, 0, MAX_LEVEL, e.mod_value[0]));
}

void update_mod_off(u8 voice) {
    // u8 mod_value[MOD_COUNT];
}

void update_mod_arc(u8 index, s8 delta) {
    u8 enc = index + get_knob_count();
    if (enc >= MOD_COUNT) return;

    s16 new_val = e.mod_value[enc];
    if (delta)
        new_val += 4;
    else
        new_val -= 4;
    
    if (new_val < 0) new_val = 0;
    else if (new_val > 255) new_val = 255;
    e.mod_value[enc] = new_val;
    update_dest(enc);
    refresh_arc();
}

static void read_knobs(void) {
    // if (is_arc_connected() || is_midi_connected()) return;
    u8 value;
    u8 changed = 0;
    
    for (u8 i = 0; i < min(MOD_COUNT, get_knob_count()); i++) {
        value = get_knob_value(i) >> 8;
        if (value != e.mod_value[i]) {
            e.mod_value[i] = value;
            update_dest(i);
            changed = 1;
        }
    }
    
    if (changed) refresh_grid();
}

void pattern_start(u8 pattern, u8 on) {
    if (on) add_timed_event(ES_PSTART_TIMER + pattern, 15, 0);
    
    if (pattern == e.p_select || !on) set_led(0, on > 0);
    
    u8 map;
    for (u8 i = 0; i < VOICE_COUNT; i++) {
        map = 1 << i;
        if (e.pstart_map[pattern][VOICE_CV_GATE] & map) set_gate(i, on);
        if (e.pstart_map[pattern][VOICE_ER301] & map) set_er301_gate(i, on);
        if (e.pstart_map[pattern][VOICE_JF] & map) set_jf_gate(i, on);
        if (e.pstart_map[pattern][VOICE_TXO_CV_GATE] & map) set_txo_gate(i, on);
    }
    
    if (!on) return;
    
    map = 1 << pattern;
    for (u8 i = 0; i < MOD_COUNT; i++) {
        if (e.mod_pstart[i] & map) {
            e.mod_value[i] = push_random(0, 255);
            update_dest(i);
        }
    }
}

static u8 es_note_on(s8 x, s8 y, u8 from_pattern, u16 timer, u8 voices) {
    u8 voice = 255;
    for (u8 i = 0; i < VOICE_COUNT; i++)
        if ((voices & (1 << i)) && (!es_notes[i].active || (es_notes[i].x == x && es_notes[i].y == y))) {
            voice = i;
            break;
        }

    if (voice == 255) {
        u32 earliest = 0xffffffff;
        for (u8 i = 0; i < VOICE_COUNT; i++)
            if ((voices & (1 << i)) && es_notes[i].start < earliest) {
                earliest = es_notes[i].start;
                voice = i;
            }
    }
    
    if (voice == 255) return 255;
    if (!shared.voice_on[voice]) return 255;
    
    es_note_off_i(voice);

    es_notes[voice].active = 1;
    es_notes[voice].x = x;
    es_notes[voice].y = y;
    es_notes[voice].start = get_global_time();
    es_notes[voice].from_pattern = from_pattern;

    s16 note_index = x + (7 - y) * 5 - 1;
    if (note_index < 0)
        note_index = 0;
    else if (note_index > 119)
        note_index = 119;
    
    update_key_x_y(voice, x, y, note_index);
    update_mod(voice, x, y, note_index);
    
    note_on(voice, note_index, 16383);
    if (timer) add_timed_event(voice, timer, 0);
    
    return voice;
}

/*
static void es_update_pitches(void) {
    u8 first_note_x = e.p[e.p_select].e[0].index & 15;
    u8 first_note_y = e.p[e.p_select].e[0].index >> 4;
    
    s16 x, y, note_index;
    for (u8 i = 0; i < VOICE_COUNT; i++) {
        es_notes[i].x = x = es_notes[i].x + e.p[e.p_select].root_x - first_note_x;
        es_notes[i].y = y = es_notes[i].y + e.p[e.p_select].root_y - first_note_y;
        note_index = x + (7 - y) * 5 - 1;
        if (note_index < 0)
            note_index = 0;
        else if (note_index > 119)
            note_index = 119;
        update_note_output(i, note_index);
    }
}
*/

static void es_complete_recording(void) {
    if (!e.p[e.p_select].length) return;
    
    e.p[e.p_select].e[e.p[e.p_select].length - 1].interval = get_global_time() - es_tick;
    
    if (e.p[e.p_select].dir) {
        for (s16 i = e.p[e.p_select].length - 1; i >= 0; i--) {
            if (e.p[e.p_select].e[i].interval > ES_CHORD_THRESHOLD) {
                e.p[e.p_select].interval_ind = i;
                break;
            }
        }
    } else {
        for (u16 i = 0; i < e.p[e.p_select].length; i++) {
            if (e.p[e.p_select].e[i].interval > ES_CHORD_THRESHOLD) {
                e.p[e.p_select].interval_ind = i;
                break;
            }
        }
    }
    
    playing[e.p_select] = 0;
    es_mode = es_stopped;
    es_kill_pattern_notes(e.p_select);
    refresh_grid();
}

static void es_record_pattern_note(u8 x, u8 y, u8 on) {
    u16 l = e.p[e.p_select].length;
    if (l >= ES_EVENTS_PER_PATTERN) {
        es_complete_recording(); // will update interval for the last event
        return;
    }
    
    if (!l) {
        e.p[e.p_select].root_x = x;
        e.p[e.p_select].root_y = y;
    } 
    
    if (l) e.p[e.p_select].e[l - 1].interval = get_global_time() - es_tick;
    es_tick = get_global_time();
    
    e.p[e.p_select].e[l].index = x + (y << 4);
    if (x == 15 && y == 0) // rest
        e.p[e.p_select].e[l].on = on ? 3 : 2;
    else
        e.p[e.p_select].e[l].on = on ? 1 : 0;
    e.p[e.p_select].length++;
}

static void es_play_pattern_note(u8 pattern, u8 ext_clock) {
    u16 i = e.p[pattern].dir ? e.p[pattern].length - 1 : 0;
    u8 first_note_x = e.p[pattern].e[i].index & 15;
    u8 first_note_y = e.p[pattern].e[i].index >> 4;
    s16 x = (e.p[pattern].e[es_pos[pattern]].index & 15) + e.p[pattern].root_x - first_note_x;
    s16 y = (e.p[pattern].e[es_pos[pattern]].index >> 4) + e.p[pattern].root_y - first_note_y;
    
    if (e.p[pattern].e[es_pos[pattern]].on == 1)
        es_note_on(x, y, pattern, 
            e.p[pattern].edge == ES_EDGE_FIXED || ext_clock ? e.p[pattern].edge_time : 0,
            e.p[pattern].voices);
    else if (e.p[pattern].e[es_pos[pattern]].on == 0 && e.p[pattern].edge == ES_EDGE_PATTERN)
        es_note_off(x, y);
    
    if (!ext_clock && e.is_leader_clock && pattern == e.p_leader) {
        clock_next_note(1);
    }
    
    refresh_grid();
}

void es_kill_pattern_notes(u8 pattern) {
    for (u8 i = 0; i < VOICE_COUNT; i++)
        if (es_notes[i].from_pattern == pattern) es_note_off_i(i);
    refresh_grid();
}

static void es_kill_all_pattern_notes(void) {
    for (u8 i = 0; i < 16; i++) es_kill_pattern_notes(i);
}

static void es_update_total_time(void) {
    u16 interval;
    es_p_total = 0;
    for (u16 i = 0; i < e.p[e.p_select].length; i++) {
        interval = e.p[e.p_select].e[i].interval;
        if (e.p[e.p_select].linearize) {
            if (interval < ES_CHORD_THRESHOLD)
                interval = 1;
            else
                interval = e.p[e.p_select].e[e.p[e.p_select].interval_ind].interval;
        }
        es_p_total += interval;
    }
}

static void es_play_pos_callback() {
    if (es_mode == es_playing) refresh_grid();
}

static u8 es_next_note(u8 pattern) {
    if (++es_pos[pattern] >= e.p[pattern].length) {
        es_pos[pattern] = 0;
        if (pattern == e.p_select) {
            es_p_start = get_global_time();
        }
        
        if (!e.p[pattern].loop) {
            es_stop_playback(pattern);
            return 1;
        }
        
        pattern_start(pattern, 1);
        if (pattern == e.p_leader && e.leader_reset) es_reset_all_playing();
    }
    return 0;
}

static void es_play_callback(u8 pattern) {
    if (is_external_clock_connected() || (e.is_leader_clock && (pattern != e.p_leader))) return;

    if (es_next_note(pattern)) return;

    u16 interval = e.p[pattern].e[es_pos[pattern]].interval;
    if (e.p[pattern].linearize) {
        if (interval < ES_CHORD_THRESHOLD)
            interval = 1;
        else
            interval = e.p[pattern].e[e.p[pattern].interval_ind].interval;
    }
    if (!interval) interval = 1;
    add_timed_event(ES_PLAY_TIMER + pattern, interval, 0);
    es_play_pattern_note(pattern, 0);
}

void es_stop_playback(u8 pattern) {
    if (pattern == e.p_select) {
        stop_timed_event(ES_PLAY_POS_TIMER);
        es_mode = es_stopped;
    }
    playing[pattern] = 0;
    
    stop_timed_event(ES_PLAY_TIMER + pattern);
    es_kill_pattern_notes(pattern);
}

void es_start_playback(u8 pattern, u8 pos) {
    if (!wtf_update_devices) {
        wtf_update_devices = 1;
        update_devices();
    }
    
    if (pattern == e.p_select) {
        if (es_mode == es_playing) {
            es_stop_playback(pattern);
        } else if (es_mode == es_recording) {
            es_complete_recording();
        }
    } else if (playing[pattern]) {
        es_stop_playback(pattern);
    }
    
    if (!e.p[pattern].length) {
        es_stop_playback(pattern);
        refresh_grid();
        return;
    }
    
    if (pattern == e.p_select) es_mode = es_playing;
    playing[pattern] = 1;
    
    u32 interval;

    if (pos) {
        u32 start = (es_p_total * pos) >> 4;
        u32 tick = 0;
        for (es_pos[pattern] = 0; es_pos[pattern] < e.p[pattern].length; es_pos[pattern]++) {
            interval = e.p[pattern].e[es_pos[pattern]].interval;
            if (e.p[pattern].linearize) {
                if (interval < ES_CHORD_THRESHOLD)
                    interval = 1;
                else
                    interval = e.p[pattern].e[e.p[pattern].interval_ind].interval;
            }
            if (tick + interval > start) break;
            tick += interval;
        }
        if (es_pos[pattern] >= e.p[pattern].length) {
            es_pos[pattern] = e.p[pattern].length - 1;
            interval = 1;
        } else {
            interval = tick + interval - start;
            if (!interval) interval = 1;
        }

        if (is_external_clock_connected() || (e.is_leader_clock && (pattern != e.p_leader))) return;

        if (pattern == e.p_select) {
            es_p_start = get_global_time() - start;
            add_timed_event(ES_PLAY_POS_TIMER, 25, 1);
        }
        add_timed_event(ES_PLAY_TIMER + pattern, interval, 0);
        return;
    }
    
    es_pos[pattern] = 0;
    if (pattern == e.p_leader && e.leader_reset) es_reset_all_playing();
    
    if (pattern == e.p_select) {
        es_p_start = get_global_time();
        es_update_total_time();
    }
    
    if (is_external_clock_connected() || (e.is_leader_clock && (pattern != e.p_leader))) return;
    
    interval = e.p[pattern].e[0].interval;
    if (e.p[pattern].linearize) {
        if (interval < ES_CHORD_THRESHOLD)
            interval = 1;
        else
            interval = e.p[pattern].e[e.p[pattern].interval_ind].interval;
    }
    if (!interval) interval = 1;
    if (pattern == e.p_select)add_timed_event(ES_PLAY_POS_TIMER, 25, 1);
    add_timed_event(ES_PLAY_TIMER + pattern, interval, 0);
    
    pattern_start(pattern, 1);
    es_play_pattern_note(pattern, 0);
}

void es_reset_all_playing() {
    for (u8 i = 0; i < PATTERN_COUNT; i++)
        if (playing[i] && i != e.p_leader) es_start_playback(i, 0);
}

static void es_start_recording(void) {
    e.p[e.p_select].length = 0;
    e.p[e.p_select].start = 0;
    e.p[e.p_select].end = 15;
    e.p[e.p_select].dir = 0;
    playing[e.p_select] = 0;
    es_mode = es_recording;
    refresh_grid();
}

static u8 is_arm_pressed(void) {
    u8 found = 0;
    for (u8 i = 0; i < key_count; i++) {
        if (held_keys[i] == 32) found = 1;
        break;
    }
    return found;
}

/*
static s8 top_row_pressed(void) {
    s8 found = -1;
    for (u8 i = 0; i < key_count; i++) {
        if (held_keys[i] < 16) {
            found = held_keys[i];
            break;
        }
    }
    return found;
}
*/

static u8 rest_pressed(void) {
    for (u8 i = 0; i < key_count; i++) {
        if (held_keys[i] == 15) return 1;
    }
    return 0;
}

static void es_prev_pattern(void) {
    if (!e.p_select) return;
    e.p_select--;
    es_start_playback(e.p_select, 0);
}

static void es_next_pattern(void) {
    if (e.p_select >= 15) return;
    e.p_select++;
    es_start_playback(e.p_select, 0);
}

static void es_double_speed(void) {
    for (u16 i = 0; i < e.p[e.p_select].length; i++) {
        if (e.p[e.p_select].e[i].interval > (ES_CHORD_THRESHOLD << 1))
            e.p[e.p_select].e[i].interval >>= 1;
        else if (e.p[e.p_select].e[i].interval > ES_CHORD_THRESHOLD)
            e.p[e.p_select].e[i].interval = ES_CHORD_THRESHOLD + 1;
    }
    es_update_total_time();
}

static void es_faster_speed(void) {
    u16 interval;
    for (u16 i = 0; i < e.p[e.p_select].length; i++) {
        if (e.p[e.p_select].e[i].interval < ES_CHORD_THRESHOLD) continue;
        interval = e.p[e.p_select].e[i].interval - (e.p[e.p_select].e[i].interval / 10);
        if (interval > ES_CHORD_THRESHOLD)
            e.p[e.p_select].e[i].interval = interval;
        else
            e.p[e.p_select].e[i].interval = ES_CHORD_THRESHOLD + 1;
    }
    es_update_total_time();
}

static void es_half_speed(void) {
    u16 interval;
    for (u16 i = 0; i < e.p[e.p_select].length; i++)
        if (e.p[e.p_select].e[i].interval > ES_CHORD_THRESHOLD) {
            interval = e.p[e.p_select].e[i].interval << 1;
            if (interval > e.p[e.p_select].e[i].interval) e.p[e.p_select].e[i].interval = interval;
        }
    es_update_total_time();
}

static void es_slower_speed(void) {
    u16 interval;
    for (u16 i = 0; i < e.p[e.p_select].length; i++)
        if (e.p[e.p_select].e[i].interval > ES_CHORD_THRESHOLD) {
            interval = e.p[e.p_select].e[i].interval + (e.p[e.p_select].e[i].interval / 10);
            if (interval > e.p[e.p_select].e[i].interval) e.p[e.p_select].e[i].interval = interval;
        }
    es_update_total_time();
}

static void es_reverse(void) {
    u16 l = e.p[e.p_select].length;
    if (!l) return;

    es_event_t te[ES_EVENTS_PER_PATTERN];
    
    for (u16 i = 0; i < l; i++) {
        te[i] = e.p[e.p_select].e[i];
        if (te[i].on == 3)
            te[i].on = 2;
        else if (te[i].on == 2)
            te[i].on = 3;
        else if (te[i].on == 1)
            te[i].on = 0;
        else if (te[i].on == 0)
            te[i].on = 1;
    }
    
    for (u16 i = 0; i < l; i++) 
        e.p[e.p_select].e[i] = te[l - i - 1];
    for (u16 i = 0; i < l - 1; i++) 
        e.p[e.p_select].e[i].interval = te[l - i - 2].interval;
    
    e.p[e.p_select].e[l - 1].interval = te[l - 1].interval;
    
    if (e.p[e.p_select].dir) {
        for (s16 i = e.p[e.p_select].length - 1; i >= 0; i--)
            if ((e.p[e.p_select].e[i].on == 3 || e.p[e.p_select].e[i].on == 1)
                && e.p[e.p_select].e[i].interval > ES_CHORD_THRESHOLD) {
                e.p[e.p_select].interval_ind = i;
                break;
            }
    } else {
        for (u16 i = 0; i < e.p[e.p_select].length; i++)
            if ((e.p[e.p_select].e[i].on == 3 || e.p[e.p_select].e[i].on == 1)
                && e.p[e.p_select].e[i].interval > ES_CHORD_THRESHOLD) {
                e.p[e.p_select].interval_ind = i;
                break;
            }
    }
}

/////////////////////////

void clock_next_note(u8 is_leader) {
    for (u8 pattern = 0; pattern < 16; pattern++) {
        if (is_leader && pattern == e.p_leader) continue;
        if (!playing[pattern] || !e.p[pattern].length) continue;
        
        u16 pos = es_pos[pattern];
        u8 played = 0;
        while (true) {
            //if (es_pos[pattern] == 0) pattern_start(pattern, 1);
            if (e.p[pattern].e[es_pos[pattern]].on) {
                es_play_pattern_note(pattern, 1);
                played = 1;
            }
            if (es_next_note(pattern)) return;
            if (played && e.p[pattern].e[es_pos[pattern]].interval > ES_CHORD_THRESHOLD) break;
            if (es_pos[pattern] == pos) break;
        }
    }
}

void external_clock_received() {
    clock_next_note(0);
}

void external_clock_switched(u8 external) {
    es_kill_all_pattern_notes();
    if (external) {
        for (u8 i = 0; i < 16; i++) stop_timed_event(ES_PLAY_TIMER + i);
        stop_timed_event(ES_PLAY_POS_TIMER);
    } else {
        add_timed_event(ES_PLAY_POS_TIMER, 25, 1);
        for (u8 i = 0; i < 16; i++) if (playing[i]) es_play_callback(i);
    }
}

void grid_key_held(u8 x, u8 y) {
    if (preset_mode) save_to_preset(y);
}

void save_to_preset(u8 p) {
    preset = p;
    store_preset_index(preset);
    store_preset_to_flash(preset, &meta, &e);
    store_shared_data_to_flash(&shared);
    preset_mode = 0;
    refresh_grid();
}

void handler_ESGridKey(u8 x, u8 y, u8 z) {
    u8 index = (y << 4) + x;

    // track held keys and long presses
    if (z) {
        held_keys[key_count] = index;
        if (key_count < MAX_HELD_KEYS) key_count++;
        key_times[index] = 10;
    } else {
        u8 found = 0;
        for (u8 i = 0; i < key_count; i++) {
            if (held_keys[i] == index) found++;
            if (found) held_keys[i] = held_keys[i + 1];
        }
        if (found) key_count--;
    }

    // preset screen
    if (preset_mode) {
        if (!z && x == 0) {
            if (y != selected_preset) {
                selected_preset = y;
                load_preset_meta_from_flash(selected_preset, &meta);
            } else {
                // flash read
                preset = selected_preset;
                load_preset();
                preset_mode = 0;
                es_view = es_main;
            }
        } else if (z && x > 7) {
            meta.glyph[y] ^= 1 << (x - 8);
        }

        refresh_grid();
        return;
    }

    if (x == 0) {
        if (es_edge && y == 7 && z) {
            e.p[e.p_select].edge = ES_EDGE_FIXED;
            e.p[e.p_select].edge_time = 16;
            es_kill_all_notes();
        } else if (z && y == 0) { // start/stop
            if (es_view == es_patterns_held) {
                es_view = es_patterns;
            } else if (es_mode == es_stopped || es_mode == es_armed) {
                es_start_playback(e.p_select, 0);
                if (is_arm_pressed()) es_ignore_arm_release = 1;
            } else if (es_mode == es_recording) {
                e.p[e.p_select].loop = 1;
                es_start_playback(e.p_select, 0);
            } else if (es_mode == es_playing) {
                if (is_arm_pressed()) {
                    es_start_playback(e.p_select, 0);
                    es_ignore_arm_release = 1;
                } else
                    es_stop_playback(e.p_select);
            }
        } else if (y == 1) { // p_select
            if (z && es_mode == es_recording) {
                es_complete_recording();
                es_mode = es_stopped;
            }
            if (z && es_view == es_patterns)
                es_view = es_main;
            else if (!z && es_view == es_patterns_held)
                es_view = es_main;
            else if (z)
                es_view = es_patterns_held;
        } else if (y == 2) { // arm
            es_view = es_main;
            if (z) {
                if (es_mode == es_armed) {
                    es_mode = es_stopped;
                    es_ignore_arm_release = 1;
                } else if (es_mode == es_recording) {
                    es_complete_recording();
                    es_mode = es_stopped;
                    es_ignore_arm_release = 1;
                }
            } else {
                if (es_ignore_arm_release) {
                    es_ignore_arm_release = 0;
                    return;
                }
                if (es_mode == es_stopped) {
                    es_mode = es_armed;
                } else if (es_mode == es_playing) {
                    es_stop_playback(e.p_select);
                    es_mode = es_armed;
                }
            }
        } else if (z && y == 3) { // loop
            e.p[e.p_select].loop = !e.p[e.p_select].loop;
        } else if (z && y == 4) { // arp
            e.arp = !e.arp;
        } else if (y == 5) { // edge mode
            es_edge = z;
        } else if (y == 6) { // runes
            if (es_view == es_voices_held) {
                if (!z) return;
                es_view = es_devices_held;
                es_devices_changed = 0;
            } else if (es_view == es_devices_held) {
                if (z) return;
                if (es_devices_changed)
                    es_view = es_main;
                else
                    es_view = es_devices;
            } else if (es_view == es_voices && z) {
                es_view = es_devices_held;
                es_devices_changed = 0;
            } else es_runes = z;
        } else if (y == 7) { // voices/devices
            if (es_view == es_devices) {
                if (z) es_view = es_main;
            } else if (z && (es_view == es_main || es_view == es_patterns)) {
                es_view = es_voices_held;
                es_voices_changed = 0;
            } else if (!z && es_view == es_voices) {
                es_view = es_main;
            } else if (!z && es_view == es_voices_held) {
                if (es_voices_changed)
                    es_view = es_main;
                else
                    es_view = es_voices;
            }
        }
        
        refresh_grid();
        return;
    }

    if (es_runes) {
        if (!z) return;
        
        if (x > 1 && x < 5 && y > 2 && y < 6) {
            e.p[e.p_select].linearize = !e.p[e.p_select].linearize;
            es_update_total_time();
        } else if (x > 5 && x < 8 && y > 2 && y < 6) {
            if (e.p[e.p_select].dir != 1) es_reverse();
            e.p[e.p_select].dir = 1;
            if (es_mode == es_playing) es_kill_pattern_notes(e.p_select);
        } else if (x > 8 && x < 11 && y > 2 && y < 6) {
            if (e.p[e.p_select].dir != 0) es_reverse();
            e.p[e.p_select].dir = 0;
            if (es_mode == es_playing) es_kill_pattern_notes(e.p_select);
        } else if ((y == 1 && x == 13) || (x == 12 && y == 2) || (x == 14 && y == 2))
            es_double_speed();
        else if ((y == 2 && x == 13) || (x == 12 && y == 3) || (x == 14 && y == 3))
            es_faster_speed();
        else if ((y == 6 && x == 13) || (x == 12 && y == 5) || (x == 14 && y == 5))
            es_slower_speed();
        else if ((y == 7 && x == 13) || (x == 12 && y == 6) || (x == 14 && y == 6))
            es_half_speed();
        
        refresh_grid();
        return;
    } 
    
    if (es_edge) {
        if (!z) return;
        
        if (y == 7) {
            e.p[e.p_select].edge = ES_EDGE_FIXED;
            e.p[e.p_select].edge_time = (x + 1) << 4;
            es_kill_all_notes();
        } else {
            if (x) {
                if (x < 6) {
                    e.p[e.p_select].edge = ES_EDGE_PATTERN;
                    es_kill_all_notes();
                } else if (x < 11) {
                    e.p[e.p_select].edge = ES_EDGE_FIXED;
                    es_kill_all_notes();
                } else {
                    e.p[e.p_select].edge = ES_EDGE_DRONE;
                }
            }
        }
        
        refresh_grid();
        return;
    }
    
    if (es_view == es_voices || es_view == es_voices_held) {
        es_voices_changed = 1;
        u8 voice = 1 << y;
        u8 sel_voice = 1 << selected_voice;
        
        if (y == 7 && x == 1) {
            help_on = z;
            help_sel = 0;
            refresh_grid();
            return;
        }
        if (y == 7 && x == 15) {
            help_on = z << 1;
            if (!help_sel) help_sel = 8;
            refresh_grid();
            return;
        }
        
        if (x == 2 && z) {
            e.p[e.p_select].voices ^= voice;
            es_kill_all_notes();
            refresh_grid();
            return;
        } 
        
        if (x == 3 && z) {
            shared.voice_on[y] = !shared.voice_on[y];
            if (!shared.voice_on[y]) es_note_off_i(y);
            refresh_grid();
            return;
        } 

        if (help_on == 2) {
            if (x > 7 && z) {
                help_sel = x;
                refresh_grid();
            }
            return;
        }
        
        if (z && x == 5) { // select mapping
            if (y < 4)
                sel_mapping = y;
            else if (y > 5)
                sel_mapping = y - 2;
            refresh_grid();
            return;
        } 
        
        if (help_on == 1 && x > 7) return;

        if (x == 15 && !help_on) {
            if (!z) return;
            if (y < DEVICE_COUNT) {
                shared.device_on[y] = !shared.device_on[y];
                update_i2c();
            } else if (y == 5) {
                shared.i2c_mode = !shared.i2c_mode;
                update_i2c();
            }
            refresh_grid();
            return;
        }
        
        if (!z || y >= VOICE_COUNT) return;
        
        if (x == 6) { // select voice or pattern
            if (sel_mapping < 4)
                selected_voice = min(y, VOICE_COUNT - 1);
            else if (sel_mapping == 4)
                sel_pattern_start = y;
            else if (sel_mapping == 5)
                sel_pattern_start = y + 8;
            refresh_grid();
            return;
        } 
        
        if (x == 10 && y == 7) {
            e.jf_mode = !e.jf_mode;
            es_kill_all_notes();
            update_i2c();
            // set_debug(e.jf_mode);
            refresh_grid();
            return;
        }

        if (y >= VOICE_COUNT) {
            refresh_grid();
            return;
        }
        
        u8 txo_gate;
        if (sel_mapping == 0) { // note 
            if (x == 8) {
                e.voice_map[selected_voice][VOICE_CV_GATE] ^= voice;
                remap_voices();
                if (!(e.voice_map[selected_voice][VOICE_CV_GATE] & voice)) set_gate(y, 0);
            } else if (x == 9) {
                e.voice_map[selected_voice][VOICE_ER301] ^= voice;
                remap_voices();
                if (!(e.voice_map[selected_voice][VOICE_ER301] & voice)) set_er301_gate(y, 0);
            } else if (x == 10 && y < 6) {
                e.voice_map[selected_voice][VOICE_JF] ^= voice;
                remap_voices();
                if (!(e.voice_map[selected_voice][VOICE_JF] & voice)) set_jf_gate(y, 0);
            } else if (x == 11) {
                if ((e.voice_map[selected_voice][VOICE_TXO_NOTE] & voice)) {
                    e.voice_map[selected_voice][VOICE_TXO_NOTE] &= ~voice;
                    remap_voices();
                    set_txo_gate(y, 0);
                } else {
                    e.voice_map[selected_voice][VOICE_TXO_NOTE] |= voice;
                    txo_gate = e.voice_map[selected_voice][VOICE_TXO_CV_GATE] & voice;
                    e.voice_map[selected_voice][VOICE_TXO_CV_GATE] &= ~voice;
                    remap_voices();
                    if (txo_gate) set_txo_gate(y, 0);
                }
            } else if (x == 12) {
                if ((e.voice_map[selected_voice][VOICE_TXO_CV_GATE] & voice)) {
                    e.voice_map[selected_voice][VOICE_TXO_CV_GATE] &= ~voice;
                    remap_voices();
                    set_txo_gate(y, 0);
                } else {
                    e.voice_map[selected_voice][VOICE_TXO_CV_GATE] |= voice;
                    txo_gate = e.voice_map[selected_voice][VOICE_TXO_NOTE] & voice;
                    e.voice_map[selected_voice][VOICE_TXO_NOTE] &= ~voice;
                    remap_voices();
                    if (txo_gate) set_txo_gate(y, 0);
                }
            } else if (x == 13) {
                if (e.mod_gate[y] & sel_voice) {
                    e.mod_gate[y] &= ~sel_voice;
                } else {
                    e.mod_gate[y] |= sel_voice;
                }
            }
        } else if (sel_mapping == 1) { // key
            if (x == 8) {
                if (e.cv_key_map[selected_voice] & voice)
                    e.cv_key_map[selected_voice] &= ~voice;
                else
                    e.cv_key_map[selected_voice] |= voice;
            } else if (x == 9) {
                if (e.txo_key_map[selected_voice] & voice)
                    e.txo_key_map[selected_voice] &= ~voice;
                else
                    e.txo_key_map[selected_voice] |= voice;
            } else if (x == 12) {
                if (e.txo_key_map[selected_voice] & voice)
                    e.txo_key_map[selected_voice] &= ~voice;
                else
                    e.txo_key_map[selected_voice] |= voice;
            } else if (x == 13) {
                if (e.mod_key[y] & sel_voice) {
                    e.mod_key[y] &= ~sel_voice;
                } else {
                    e.mod_key[y] |= sel_voice;
                }
            }
        } else if (sel_mapping == 2) { // x
            if (x == 8) {
                if (e.cv_x_map[selected_voice] & voice)
                    e.cv_x_map[selected_voice] &= ~voice;
                else
                    e.cv_x_map[selected_voice] |= voice;
            } else if (x == 9) {
                if (e.er301_x_map[selected_voice] & voice)
                    e.er301_x_map[selected_voice] &= ~voice;
                else
                    e.er301_x_map[selected_voice] |= voice;
            } else if (x == 12) {
                if (e.txo_x_map[selected_voice] & voice)
                    e.txo_x_map[selected_voice] &= ~voice;
                else
                    e.txo_x_map[selected_voice] |= voice;
            } else if (x == 13) {
                if (e.mod_x[y] & sel_voice) {
                    e.mod_x[y] &= ~sel_voice;
                } else {
                    e.mod_x[y] |= sel_voice;
                }
            }
        } else if (sel_mapping == 3) { // y
            if (x == 8) {
                if (e.cv_y_map[selected_voice] & voice)
                    e.cv_y_map[selected_voice] &= ~voice;
                else
                    e.cv_y_map[selected_voice] |= voice;
            } else if (x == 9) {
                if (e.er301_y_map[selected_voice] & voice)
                    e.er301_y_map[selected_voice] &= ~voice;
                else
                    e.er301_y_map[selected_voice] |= voice;
            } else if (x == 12) {
                if (e.txo_y_map[selected_voice] & voice)
                    e.txo_y_map[selected_voice] &= ~voice;
                else
                    e.txo_y_map[selected_voice] |= voice;
            } else if (x == 13) {
                if (e.mod_y[y] & sel_voice) {
                    e.mod_y[y] &= ~sel_voice;
                } else {
                    e.mod_y[y] |= sel_voice;
                }
            }
        } else if (sel_mapping == 4 || sel_mapping == 5) { // pstart
            if (x == 8) {
                if (e.pstart_map[sel_pattern_start][VOICE_CV_GATE] & voice)
                    e.pstart_map[sel_pattern_start][VOICE_CV_GATE] &= ~voice;
                else
                    e.pstart_map[sel_pattern_start][VOICE_CV_GATE] |= voice;
            } else if (x == 9) {
                if (e.pstart_map[sel_pattern_start][VOICE_ER301] & voice)
                    e.pstart_map[sel_pattern_start][VOICE_ER301] &= ~voice;
                else
                    e.pstart_map[sel_pattern_start][VOICE_ER301] |= voice;
            } else if (x == 10) {
                if (e.pstart_map[sel_pattern_start][VOICE_JF] & voice)
                    e.pstart_map[sel_pattern_start][VOICE_JF] &= ~voice;
                else
                    e.pstart_map[sel_pattern_start][VOICE_JF] |= voice;
            } else if (x == 12) {
                if (e.pstart_map[sel_pattern_start][VOICE_TXO_CV_GATE] & voice)
                    e.pstart_map[sel_pattern_start][VOICE_TXO_CV_GATE] &= ~voice;
                else
                    e.pstart_map[sel_pattern_start][VOICE_TXO_CV_GATE] |= voice;
            } else if (x == 13) {
                u16 ps = 1 << sel_pattern_start;
                if (e.mod_pstart[y] & ps) {
                    e.mod_pstart[y] &= ~ps;
                } else {
                    e.mod_pstart[y] |= ps;
                }
            }
        }

        refresh_grid();
        return;
    }
    
    if (es_view == es_devices || es_view == es_devices_held) {
        if (y == 7 && x == 1) {
            help_on = z;
            refresh_grid();
            return;
        }
        
        if (!z) {
            if (x == 3) sel_mod = 255;
            
            if (prev_param_x == x && prev_param_y == y) {
                prev_param_x = 0;
                prev_param_y = 0;
            }
            return;
        }
        
        es_devices_changed = 1;

        if (x == 1 && y < 4) {
            sel_device = y;
            if (sel_device == 0 && sel_param > 1) sel_param = 0;
            else if (sel_device && sel_device < 3 && sel_param > 2) sel_param = 0;
        } else if (x == 2) {
            if (sel_device == 0 && y < 2) sel_param = y;
            else if (sel_device && sel_device < 3 && y < 3) sel_param = y;
            else if (sel_device == 3 && y < 7) sel_param = y;
        } else if (x == 3 && !help_on) {
            sel_mod = y;
        } else if (x > 3 && !help_on) {
            if (prev_param_x > 3 && 
                ((prev_param_x != x && prev_param_y == y) || (prev_param_x == x && prev_param_y != y))) {
                // held
                u8 range;
                if (prev_param_y != y) {
                    range = ((x - 4) << 4) + x - 4;
                    y = prev_param_y;
                } else {
                    range = ((max(prev_param_x, x) - 4) << 4) + min(prev_param_x, x) - 4;
                }
                if (sel_device == 0 && sel_param == 0 && x < 9) {
                    e.octave_mm[y][VOICE_CV_GATE] = range;
                } else if (sel_device == 1 && sel_param == 0 && x < 9) {
                    e.octave_mm[y][VOICE_ER301] = range;
                } else if (sel_device == 2 && sel_param == 0 && x < 9 && y < 6) {
                    e.octave_mm[y][VOICE_JF] = range;
                } else if (sel_device == 3 && sel_param == 0 && x < 9) {
                    e.octave_mm[y][VOICE_TXO_NOTE] = e.octave[y][VOICE_TXO_CV_GATE] = range;
                } else if (sel_device == 1 && sel_param == 2) {
                    e.er301_volume_mm[y] = range;
                } else if (sel_device == 2 && sel_param == 2 && y < 6) {
                    e.jf_volume_mm[y] = range;
                } else if (sel_device == 3 && sel_param == 2) {
                    e.txo_volume_mm[y] = range;
                } else if (sel_device == 3 && sel_param == 3) {
                    e.txo_wave_mm[y] = range;
                } else if (sel_device == 3 && sel_param == 4) {
                    e.txo_wavef_mm[y] = range;
                } else if (sel_device == 3 && sel_param == 5) {
                    e.txo_attack_mm[y] = range;
                } else if (sel_device == 3 && sel_param == 6) {
                    e.txo_decay_mm[y] = range;
                }
            } else {
                prev_param_x = x;
                prev_param_y = y;
                
                if (sel_mod != 255) {
                    u8 out = 1 << y;
                    if (sel_device == 0 && sel_param == 0 && x < 9) {
                        e.mod_octave[sel_mod] ^= out;
                    } else if (sel_device == 1 && sel_param == 0 && x < 9) {
                        e.mod_er301_octave[sel_mod] ^= out;
                    } else if (sel_device == 2 && sel_param == 0 && x < 9 && y < 6) {
                        e.mod_jf_octave[sel_mod] ^= out;
                    } else if (sel_device == 3 && sel_param == 0 && x < 9) {
                        e.mod_txo_octave[sel_mod] ^= out;
                    } else if (sel_device == 1 && sel_param == 2) {
                        e.mod_er301_volume[sel_mod] ^= out;
                    } else if (sel_device == 2 && sel_param == 2 && y < 6) {
                        e.mod_jf_volume[sel_mod] ^= out;
                    } else if (sel_device == 3 && sel_param == 2) {
                        e.mod_txo_volume[sel_mod] ^= out;
                    } else if (sel_device == 3 && sel_param == 3) {
                        e.mod_txo_wave[sel_mod] ^= out;
                    } else if (sel_device == 3 && sel_param == 4) {
                        e.mod_txo_wavef[sel_mod] ^= out;
                    } else if (sel_device == 3 && sel_param == 5) {
                        e.mod_txo_attack[sel_mod] ^= out;
                    } else if (sel_device == 3 && sel_param == 6) {
                        e.mod_txo_decay[sel_mod] ^= out;
                    }
                } else {
                    if (sel_device == 0 && sel_param == 0 && x < 9) {
                        e.octave[y][VOICE_CV_GATE] = x - 4;
                    } else if (sel_device == 1 && sel_param == 0 && x < 9) {
                        e.octave[y][VOICE_ER301] = x - 4;
                    } else if (sel_device == 2 && sel_param == 0 && x < 9 && y < 6) {
                        e.octave[y][VOICE_JF] = x - 4;
                    } else if (sel_device == 3 && sel_param == 0 && x < 9) {
                        e.octave[y][VOICE_TXO_NOTE] = e.octave[y][VOICE_TXO_CV_GATE] = x - 4;
                    } else if (sel_device == 0 && sel_param == 1) {
                        e.transpose[y][VOICE_CV_GATE] = x - 4;
                    } else if (sel_device == 1 && sel_param == 1) {
                        e.transpose[y][VOICE_ER301] = x - 4;
                    } else if (sel_device == 2 && sel_param == 1 && y < 6) {
                        e.transpose[y][VOICE_JF] = x - 4;
                    } else if (sel_device == 3 && sel_param == 1) {
                        e.transpose[y][VOICE_TXO_NOTE] = e.transpose[y][VOICE_TXO_CV_GATE] = x - 4;
                    } else if (sel_device == 1 && sel_param == 2) {
                        e.er301_volume[y] = x - 4;
                    } else if (sel_device == 2 && sel_param == 2 && y < 6) {
                        e.jf_volume[y] = x - 4;
                    } else if (sel_device == 3 && sel_param == 2) {
                        e.txo_volume[y] = x - 4;
                    } else if (sel_device == 3 && sel_param == 3) {
                        e.txo_wave[y] = x - 4;
                    } else if (sel_device == 3 && sel_param == 4) {
                        e.txo_wavef[y] = x - 4;
                    } else if (sel_device == 3 && sel_param == 5) {
                        e.txo_attack[y] = x - 4;
                    } else if (sel_device == 3 && sel_param == 6) {
                        e.txo_decay[y] = x - 4;
                    }
                }
            }
            update_devices();
        }
        
        refresh_grid();
        return;
    }
    
    if (es_view == es_patterns_held || es_view == es_patterns) {
        if (y < 2 || y > 5) return;

        if (x >= 2 && x <= 5) {
            if (z) e.p_select = x - 2 + ((y - 2) << 2);
            pattern_held = z ? e.p_select : 255;
            // if (es_view == es_patterns) es_start_playback(e.p_select, 0);
        } else if (z && x >= 10 && x <= 13) {
            u8 pattern = x - 10 + ((y - 2) << 2);
            if (pattern_held != 255) {
                copy_pattern(pattern_held, pattern);
                refresh_grid();
                return;
            }
            
            if (pattern_leader) {
                if (e.p_leader == pattern) {
                    e.p_leader = PATTERN_COUNT;
                    if (e.is_leader_clock) 
                        for (u8 i = 0; i < PATTERN_COUNT; i++)
                            if (playing[i]) es_start_playback(i, es_pos[i]);
                } else {
                    e.p_leader = pattern;
                    if (e.is_leader_clock)
                        if (!playing[e.p_leader]) es_kill_all_notes();
                }
            } else {
                if (playing[pattern]) {
                    es_stop_playback(pattern);
                } else {
                    es_start_playback(pattern, 0);
                }
            }
        } else if (x == 15 && y == 2) {
            pattern_leader = z;
        } else if (x == 15 && y == 3 && z) {
            e.leader_reset = !e.leader_reset;
        } else if (x == 15 && y == 4 && z) {
            e.is_leader_clock = !e.is_leader_clock;
            if (e.is_leader_clock) {
                if (!playing[e.p_leader]) es_kill_all_notes();
            } else {
                for (u8 i = 0; i < PATTERN_COUNT; i++)
                    if (playing[i]) es_start_playback(i, es_pos[i]);
            }
            
        }
        
        refresh_grid();
        return;
    }
    
    if (y == 0 && es_mode == es_playing) {
        if (!z) return;
        es_start_playback(e.p_select, e.p[e.p_select].dir ? 15 - x : x);
        /*
        s8 start = top_row_pressed();
        if (start == -1 || start == x) {
            es_start_playback(x);
        } else {
            e.p[e.p_select].start = min(x, start);
            e.p[e.p_select].end = max(x, start);
        }
        */
        refresh_grid();
        return;
    }
    
    if (x == 0) return;

    if (es_mode == es_armed) es_start_recording(); // will change es_mode to es_recording
    if (es_mode == es_recording) es_record_pattern_note(x, y, z);
    
    if (e.arp && es_mode != es_recording) {
        if (!z) return;
        e.p[e.p_select].root_x = x;
        e.p[e.p_select].root_y = y;
        es_start_playback(e.p_select, 0);
        // es_update_pitches();
    } else if (es_mode == es_stopped && rest_pressed() && z) {
        // keymap
        e.keymap[(y << 4) + x] = (e.keymap[(y << 4) + x] + 1) % 3;
    } else {        
        if (e.p[e.p_select].edge == ES_EDGE_DRONE) {
            if (z) {
                u8 found = 0;
                for (u8 i = 0; i < VOICE_COUNT; i++)
                    if (x == es_notes[i].x && y == es_notes[i].y && es_notes[i].active && es_notes[i].from_pattern == e.p_select) {
                        es_note_off(x, y);
                        found = 1;
                    }
                if (!found) es_note_on(x, y, 255, 0, e.p[e.p_select].voices);
            }
        } else {
            if (z) {
                if (x != 15 || y != 0)
                    es_note_on(x, y, 255, 0, e.p[e.p_select].voices);
            } else es_note_off(x, y);
        }
    }

    refresh_grid();
}

void render_grid() {
    
    clear_screen();
    draw_str("POLYEARTHSEA ON TELETYPE", 0, 15, 0);
    
    if (es_mode == es_recording) {
        draw_str("recording..", 2, 15, 0);
    } else if (es_mode == es_playing) {
        draw_str("playing ", 2, 15, 0);
    }
    
    refresh_screen();

    if (!is_grid_connected()) return;
    
    clear_all_grid_leds();

    if (preset_mode) {
        set_grid_led(0, preset, 4);
        set_grid_led(0, selected_preset, 11);

        for(u8 i1=0;i1<8;i1++)
            for(u8 i2=0;i2<8;i2++)
                if(meta.glyph[i1] & (1<<i2))
                    set_grid_led(i2 + 8, i1, 9);
        
        return;
    }
    
    for (u8 i = 0; i < 8; i++)
        set_grid_led(0, i, 2);
    
    if (es_mode == es_playing)
        set_grid_led(0, 0, 15);
    else if (e.p[e.p_select].length)
        set_grid_led(0, 0, 8);
    
    if (es_view == es_patterns) set_grid_led(0, 1, 15);
    
    if (es_mode == es_recording)
        set_grid_led(0, 2, es_blinker ? 0 : 4);
    else if (es_mode == es_armed)
        set_grid_led(0, 2, 7);
    
    if (e.p[e.p_select].loop) set_grid_led(0, 3, 11);
    if (e.arp) set_grid_led(0, 4, 11);
    
    if (es_mode == es_playing && 
        (es_view == es_main || es_view == es_patterns || es_view == es_patterns_held)) {
        //for (u8 i = e.p[e.p_select].start; i <= e.p[e.p_select].end; i++) 
        //    monomeLedBuffer[i] = 4;
        u8 pos;
        if (is_external_clock_connected() || (e.is_leader_clock && (e.p_select != e.p_leader)))
            pos = e.p[e.p_select].length ? (es_pos[e.p_select] << 4) / (e.p[e.p_select].length - 1) : 0;
        else
            pos = ((get_global_time() - es_p_start) << 4) / es_p_total;
        if (e.p[e.p_select].dir) pos = 15 - pos;
        for (u8 i = 1; i < 16; i++)
            if (i <= pos) set_grid_led(i, 0, 8);
    }

    u8 l;
    if (es_runes) {
        l = e.p[e.p_select].linearize ? 15 : 7;
        
        // linearize
        set_grid_led(2, 3, l);
        set_grid_led(4, 3, l);
        set_grid_led(2, 5, l);
        set_grid_led(4, 5, l);
        
        l = e.p[e.p_select].dir ? 15 : 7;
        // reverse
        set_grid_led(7, 3, l);
        set_grid_led(6, 4, l);
        set_grid_led(7, 5, l);
        
        l = e.p[e.p_select].dir ? 7 : 15;
        // forward
        set_grid_led(9, 3, l);
        set_grid_led(10, 4, l);
        set_grid_led(9, 5, l);
        
        l = 8;
        // increase speed
        set_grid_led(13, 1, 13);
        set_grid_led(12, 2, 13);
        set_grid_led(14, 2, 13);
        
        set_grid_led(13, 2, l);
        set_grid_led(12, 3, l);
        set_grid_led(14, 3, l);

        // decrease speed
        set_grid_led(12, 5, l);
        set_grid_led(14, 5, l);
        set_grid_led(13, 6, l);

        set_grid_led(12, 6, 13);
        set_grid_led(14, 6, 13);
        set_grid_led(13, 7, 13);

        return;
    }

    if (es_edge) {
        l = e.p[e.p_select].edge == ES_EDGE_PATTERN ? 15 : 7;
        set_grid_led_i(34, l);
        set_grid_led_i(35, l);
        set_grid_led_i(36, l);
        set_grid_led_i(50, l);
        set_grid_led_i(52, l);
        set_grid_led_i(66, l);
        set_grid_led_i(68, l);
        set_grid_led_i(82, l);
        set_grid_led_i(84, l);
        set_grid_led_i(85, l);

        l = e.p[e.p_select].edge == ES_EDGE_FIXED ? 15 : 7;
        set_grid_led_i(39, l);
        set_grid_led_i(40, l);
        set_grid_led_i(41, l);
        set_grid_led_i(42, l);
        set_grid_led_i(55, l);
        set_grid_led_i(58, l);
        set_grid_led_i(71, l);
        set_grid_led_i(74, l);
        set_grid_led_i(87, l);
        set_grid_led_i(90, l);
        
        l = e.p[e.p_select].edge == ES_EDGE_DRONE ? 15 : 7;
        set_grid_led_i(44, l);
        set_grid_led_i(45, l);
        set_grid_led_i(46, l);
        set_grid_led_i(47, l);
        
        if (e.p[e.p_select].edge == ES_EDGE_FIXED) {
            for (u8 i = 0; i < 16; i++)
                set_grid_led_i(112 + i, 4);
            u8 edge_index = 111 + (e.p[e.p_select].edge_time >> 4);
            if (edge_index <= 127) set_grid_led_i(edge_index, 11);
        }
        
        return;
    }

    u8 blink = help_on ? es_blinker : 0;
    if (es_view == es_voices || es_view == es_voices_held) {
        set_grid_led(1, 7, 4 + es_blinker);
        set_grid_led(15, 7, 4 + es_blinker);
        
        u8 un = 4;

        if (!help_on || help_sel < 8) {
            set_grid_led(5, 0, blink + (sel_mapping == 0 ? 8 : un));
            set_grid_led(5, 1, blink + (sel_mapping == 1 ? 8 : un));
            set_grid_led(5, 2, blink + (sel_mapping == 2 ? 8 : un));
            set_grid_led(5, 3, blink + (sel_mapping == 3 ? 8 : un));
            set_grid_led(5, 6, blink + (sel_mapping == 4 ? 8 : un)); // pattern start 1-8
            set_grid_led(5, 7, blink + (sel_mapping == 5 ? 8 : un)); // pattern start 9-16
        }
        
        u8 voice, sel_v;
        
        for (u8 i = 0; i < VOICE_COUNT; i++) {
            set_grid_led_i(2 + (i << 4), e.p[e.p_select].voices & (1 << i) ? 15 : un);
            set_grid_led(3, i, shared.voice_on[i] ? 8 : un);
        }
        
        if (help_on) {
            u8 l = 8;
            
            if (help_sel > 7) {
                for (u8 i = 8; i < 14; i++)
                    set_grid_led(i, 0, blink + (help_sel == i ? 8 : un));
                
                if (help_sel == 8) {
                    set_grid_led(5, 2, l); // m
                    set_grid_led(6, 2, l-4);
                    set_grid_led(7, 2, l);
                    set_grid_led(5, 3, l);
                    set_grid_led(6, 3, l-4);
                    set_grid_led(7, 3, l);
                    set_grid_led(5, 4, l);
                    set_grid_led(7, 4, l);
                    set_grid_led(9, 2, l); // o
                    set_grid_led(10, 2, l);
                    set_grid_led(11, 2, l);
                    set_grid_led(9, 3, l);
                    set_grid_led(11, 3, l);
                    set_grid_led(9, 4, l);
                    set_grid_led(10, 4, l);
                    set_grid_led(11, 4, l);
                    set_grid_led(13, 2, l); // d
                    set_grid_led(14, 2, l);
                    set_grid_led(13, 3, l);
                    set_grid_led(15, 3, l);
                    set_grid_led(13, 4, l);
                    set_grid_led(14, 4, l);
                } else if (help_sel == 9) {
                    set_grid_led(5, 2, l); // 3
                    set_grid_led(6, 2, l);
                    set_grid_led(7, 2, l);
                    set_grid_led(6, 3, l-4);
                    set_grid_led(7, 3, l-4);
                    set_grid_led(5, 4, l);
                    set_grid_led(6, 4, l);
                    set_grid_led(7, 4, l);
                    set_grid_led(9, 2, l); // o
                    set_grid_led(10, 2, l);
                    set_grid_led(11, 2, l);
                    set_grid_led(9, 3, l);
                    set_grid_led(11, 3, l);
                    set_grid_led(9, 4, l);
                    set_grid_led(10, 4, l);
                    set_grid_led(11, 4, l);
                    set_grid_led(13, 2, l); // 1
                    set_grid_led(13, 3, l);
                    set_grid_led(13, 4, l);
                } else if (help_sel == 10) {
                    set_grid_led(7, 2, l); // j
                    set_grid_led(7, 3, l);
                    set_grid_led(6, 4, l);
                    set_grid_led(7, 4, l);
                    set_grid_led(9, 2, l); // f
                    set_grid_led(10, 2, l);
                    set_grid_led(11, 2, l);
                    set_grid_led(9, 3, l-4);
                    set_grid_led(10, 3, l-4);
                    set_grid_led(9, 4, l);
                } else if (help_sel == 11) {
                    set_grid_led(5, 2, l); // t
                    set_grid_led(6, 2, l);
                    set_grid_led(7, 2, l);
                    set_grid_led(6, 3, l);
                    set_grid_led(6, 4, l);
                    set_grid_led(9, 2, l); // x
                    set_grid_led(11, 2, l);
                    set_grid_led(10, 3, l);
                    set_grid_led(9, 4, l);
                    set_grid_led(11, 4, l);
                    set_grid_led(13, 2, l); // n
                    set_grid_led(14, 2, l);
                    set_grid_led(15, 2, l);
                    set_grid_led(13, 3, l);
                    set_grid_led(15, 3, l);
                    set_grid_led(13, 4, l);
                    set_grid_led(15, 4, l);
                } else if (help_sel == 12) {
                    set_grid_led(5, 2, l); // t
                    set_grid_led(6, 2, l);
                    set_grid_led(7, 2, l);
                    set_grid_led(6, 3, l);
                    set_grid_led(6, 4, l);
                    set_grid_led(9, 2, l); // x
                    set_grid_led(11, 2, l);
                    set_grid_led(10, 3, l);
                    set_grid_led(9, 4, l);
                    set_grid_led(11, 4, l);
                    set_grid_led(13, 2, l); // c
                    set_grid_led(14, 2, l);
                    set_grid_led(15, 2, l);
                    set_grid_led(13, 3, l);
                    set_grid_led(13, 4, l);
                    set_grid_led(14, 4, l);
                    set_grid_led(15, 4, l);
                } else if (help_sel == 13) {
                    // b
                    set_grid_led(5, 2, l);
                    set_grid_led(6, 2, l);
                    set_grid_led(5, 3, l);
                    set_grid_led(7, 3, l);
                    set_grid_led(5, 4, l);
                    set_grid_led(6, 4, l);
                    set_grid_led(5, 5, l);
                    set_grid_led(7, 5, l);
                    set_grid_led(5, 6, l);
                    set_grid_led(6, 6, l);
                    // u
                    set_grid_led(9, 2, l);
                    set_grid_led(11, 2, l);
                    set_grid_led(9, 3, l);
                    set_grid_led(11, 3, l);
                    set_grid_led(9, 4, l);
                    set_grid_led(11, 4, l);
                    set_grid_led(9, 5, l);
                    set_grid_led(11, 5, l);
                    set_grid_led(9, 6, l);
                    set_grid_led(10, 6, l);
                    set_grid_led(11, 6, l);
                    // s
                    set_grid_led(13, 2, l);
                    set_grid_led(14, 2, l);
                    set_grid_led(15, 2, l);
                    set_grid_led(13, 3, l);
                    set_grid_led(13, 4, l);
                    set_grid_led(14, 4, l);
                    set_grid_led(15, 4, l);
                    set_grid_led(15, 5, l);
                    set_grid_led(13, 6, l);
                    set_grid_led(14, 6, l);
                    set_grid_led(15, 6, l);
                }
                return;
            }
            
            if (sel_mapping == 0) {
                // n
                set_grid_led(7, 1, l);
                set_grid_led(8, 1, l);
                set_grid_led(9, 1, l);
                set_grid_led(7, 2, l);
                set_grid_led(9, 2, l);
                set_grid_led(7, 3, l);
                set_grid_led(9, 3, l);
                // o
                set_grid_led(11, 1, l);
                set_grid_led(12, 1, l);
                set_grid_led(13, 1, l);
                set_grid_led(11, 2, l);
                set_grid_led(13, 2, l);
                set_grid_led(11, 3, l);
                set_grid_led(12, 3, l);
                set_grid_led(13, 3, l);
                // t
                set_grid_led(7, 5, l);
                set_grid_led(8, 5, l);
                set_grid_led(9, 5, l);
                set_grid_led(8, 6, l);
                set_grid_led(8, 7, l);
                // e
                set_grid_led(11, 5, l);
                set_grid_led(12, 5, l);
                set_grid_led(13, 5, l);
                set_grid_led(11, 6, l-4);
                set_grid_led(12, 6, l-4);
                set_grid_led(11, 7, l);
                set_grid_led(12, 7, l);
                set_grid_led(13, 7, l);
            } else if (sel_mapping == 1) {
                // k
                set_grid_led(7, 1, l);
                set_grid_led(9, 1, l);
                set_grid_led(7, 2, l);
                set_grid_led(8, 2, l);
                set_grid_led(7, 3, l);
                set_grid_led(9, 3, l);
                // e
                set_grid_led(11, 1, l);
                set_grid_led(12, 1, l);
                set_grid_led(13, 1, l);
                set_grid_led(11, 2, l-4);
                set_grid_led(12, 2, l-4);
                set_grid_led(11, 3, l);
                set_grid_led(12, 3, l);
                set_grid_led(13, 3, l);
                // y
                set_grid_led(9, 5, l);
                set_grid_led(11, 5, l);
                set_grid_led(10, 6, l);
                set_grid_led(10, 7, l);
            } else if (sel_mapping == 2) {
                // x
                set_grid_led(9, 3, l);
                set_grid_led(11, 3, l);
                set_grid_led(10, 4, l);
                set_grid_led(9, 5, l);
                set_grid_led(11, 5, l);
            } else if (sel_mapping == 3) {
                // y
                set_grid_led(9, 3, l);
                set_grid_led(11, 3, l);
                set_grid_led(10, 4, l);
                set_grid_led(10, 5, l);
            } else if (sel_mapping == 4) {
                // p
                set_grid_led(7, 0, l);
                set_grid_led(8, 0, l);
                set_grid_led(9, 0, l);
                set_grid_led(7, 1, l);
                set_grid_led(9, 1, l);
                set_grid_led(7, 2, l);
                set_grid_led(8, 2, l);
                set_grid_led(9, 2, l);
                set_grid_led(7, 3, l);
                set_grid_led(7, 4, l);
                // s
                set_grid_led(11, 0, l);
                set_grid_led(12, 0, l);
                set_grid_led(13, 0, l);
                set_grid_led(11, 1, l);
                set_grid_led(11, 2, l);
                set_grid_led(12, 2, l);
                set_grid_led(13, 2, l);
                set_grid_led(13, 3, l);
                set_grid_led(11, 4, l);
                set_grid_led(12, 4, l);
                set_grid_led(13, 4, l);
                // i
                set_grid_led(10, 6, l);
                set_grid_led(10, 7, l);
            } else if (sel_mapping == 5) {
                // p
                set_grid_led(7, 0, l);
                set_grid_led(8, 0, l);
                set_grid_led(9, 0, l);
                set_grid_led(7, 1, l);
                set_grid_led(9, 1, l);
                set_grid_led(7, 2, l);
                set_grid_led(8, 2, l);
                set_grid_led(9, 2, l);
                set_grid_led(7, 3, l);
                set_grid_led(7, 4, l);
                // s
                set_grid_led(11, 0, l);
                set_grid_led(12, 0, l);
                set_grid_led(13, 0, l);
                set_grid_led(11, 1, l);
                set_grid_led(11, 2, l);
                set_grid_led(12, 2, l);
                set_grid_led(13, 2, l);
                set_grid_led(13, 3, l);
                set_grid_led(11, 4, l);
                set_grid_led(12, 4, l);
                set_grid_led(13, 4, l);
                // ii
                set_grid_led(9, 6, l);
                set_grid_led(9, 7, l);
                set_grid_led(11, 6, l);
                set_grid_led(11, 7, l);
            }

            return;
        }
        
        set_grid_led(15, 5, shared.i2c_mode ? 8 : un);
        for (u8 i = 0; i < DEVICE_COUNT; i++)
            set_grid_led(15, i, shared.device_on[i] ? 8 : un);
        
        for (u8 i = 0; i < VOICE_COUNT; i++) {
            voice = 1 << i;
            sel_v = 1 << selected_voice;
            
            if (sel_mapping < 4)
                set_grid_led(6, i, i == selected_voice ? 8 : un);
            else if (sel_mapping == 4)
                set_grid_led(6, i, i == sel_pattern_start ? 8 : un);
            else if (sel_mapping == 5)
                set_grid_led(6, i, i == (sel_pattern_start - 8) ? 8 : un);

            if (sel_mapping == 0) { // note
                set_grid_led(8, i, e.voice_map[selected_voice][VOICE_CV_GATE] & voice ? 8 : un);
                set_grid_led(9, i, e.voice_map[selected_voice][VOICE_ER301] & voice? 8 : un);
                if (i < 6) set_grid_led(10, i, e.voice_map[selected_voice][VOICE_JF] & voice ? 8 : un);
                set_grid_led(11, i, e.voice_map[selected_voice][VOICE_TXO_NOTE] & voice ? 8 : un);
                set_grid_led(12, i, e.voice_map[selected_voice][VOICE_TXO_CV_GATE] & voice ? 8 : un);
                set_grid_led(13, i, e.mod_gate[i] & sel_v ? 8 : un);
            } else if (sel_mapping == 1) { // key
                set_grid_led(8, i, e.cv_key_map[selected_voice] & voice ? 8 : un);
                set_grid_led(9, i, e.er301_key_map[selected_voice] & voice ? 8 : un);
                set_grid_led(12, i, e.txo_key_map[selected_voice] & voice ? 8 : un);
                set_grid_led(13, i, e.mod_key[i] & sel_v ? 8 : un);
            } else if (sel_mapping == 2) { // x
                set_grid_led(8, i, e.cv_x_map[selected_voice] & voice ? 8 : un);
                set_grid_led(9, i, e.er301_x_map[selected_voice] & voice ? 8 : un);
                set_grid_led(12, i, e.txo_x_map[selected_voice] & voice ? 8 : un);
                set_grid_led(13, i, e.mod_x[i] & sel_v ? 8 : un);
            } else if (sel_mapping == 3) { // y
                set_grid_led(8, i, e.cv_y_map[selected_voice] & voice ? 8 : un);
                set_grid_led(9, i, e.er301_y_map[selected_voice] & voice ? 8 : un);
                set_grid_led(12, i, e.txo_y_map[selected_voice] & voice ? 8 : un);
                set_grid_led(13, i, e.mod_y[i] & sel_v ? 8 : un);
            } else if (sel_mapping == 4) { // pstart 1
                set_grid_led(8, i, e.pstart_map[sel_pattern_start][0] & voice ? 8 : un);
                set_grid_led(9, i, e.pstart_map[sel_pattern_start][1] & voice? 8 : un);
                if (i < 6) set_grid_led(10, i, e.pstart_map[sel_pattern_start][2] & voice ? 8 : un);
                set_grid_led(12, i, e.pstart_map[sel_pattern_start][4] & voice ? 8 : un);
                set_grid_led(13, i, e.mod_pstart[i] & (1 << sel_pattern_start) ? 8 : un);
            } else if (sel_mapping == 5) { // pstart 2
                set_grid_led(8, i, e.pstart_map[sel_pattern_start][0] & voice ? 8 : un);
                set_grid_led(9, i, e.pstart_map[sel_pattern_start][1] & voice? 8 : un);
                if (i < 6) set_grid_led(10, i, e.pstart_map[sel_pattern_start][2] & voice ? 8 : un);
                set_grid_led(12, i, e.pstart_map[sel_pattern_start][4] & voice ? 8 : un);
                set_grid_led(13, i, e.mod_pstart[i] & (1 << sel_pattern_start) ? 8 : un);
            }
        }
        
        set_grid_led(10, 7, e.jf_mode ? 8 : un);
        
        return;
    }

    if (es_view == es_devices || es_view == es_devices_held) {
        for (u8 i = 0; i < 4; i++)
            set_grid_led(1, i, blink + (sel_device == i ? 12 : 4));

        set_grid_led(1, 7, 4 + es_blinker);
        
        set_grid_led(2, 0, blink + (sel_param == 0 ? 12 : 4)); // octave
        set_grid_led(2, 1, blink + (sel_param == 1 ? 12 : 4)); // transpose
        
        if (sel_device) set_grid_led(2, 2, blink + (sel_param == 2 ? 12 : 4)); // volume
        
        if (sel_device == 3) {
            set_grid_led(2, 3, blink + (sel_param == 3 ? 12 : 4)); // txo waveform
            set_grid_led(2, 4, blink + (sel_param == 4 ? 12 : 4)); // txo waveform fine
            set_grid_led(2, 5, blink + (sel_param == 5 ? 12 : 4)); // txo attack
            set_grid_led(2, 6, blink + (sel_param == 6 ? 12 : 4)); // txo decay
        }
        
        if (!help_on) 
            for (u8 i = 0; i < MOD_COUNT; i++)
                set_grid_led(3, i, (es_blinker & 1) + (i == sel_mod ? 12 : 4));
        
        u8 l, o;
        if (help_on) {
            l = 8;
            if (sel_device == VOICE_CV_GATE) {
                set_grid_led(5, 1, l); // m
                set_grid_led(6, 1, l-4);
                set_grid_led(7, 1, l);
                set_grid_led(5, 2, l);
                set_grid_led(6, 2, l-4);
                set_grid_led(7, 2, l);
                set_grid_led(5, 3, l);
                set_grid_led(7, 3, l);
                set_grid_led(9, 1, l); // o
                set_grid_led(10, 1, l);
                set_grid_led(11, 1, l);
                set_grid_led(9, 2, l);
                set_grid_led(11, 2, l);
                set_grid_led(9, 3, l);
                set_grid_led(10, 3, l);
                set_grid_led(11, 3, l);
                set_grid_led(13, 1, l); // d
                set_grid_led(14, 1, l);
                set_grid_led(13, 2, l);
                set_grid_led(15, 2, l);
                set_grid_led(13, 3, l);
                set_grid_led(14, 3, l);
            } else if (sel_device == VOICE_ER301) {
                set_grid_led(5, 1, l); // 3
                set_grid_led(6, 1, l);
                set_grid_led(7, 1, l);
                set_grid_led(6, 2, l-4);
                set_grid_led(7, 2, l-4);
                set_grid_led(5, 3, l);
                set_grid_led(6, 3, l);
                set_grid_led(7, 3, l);
                set_grid_led(9, 1, l); // o
                set_grid_led(10, 1, l);
                set_grid_led(11, 1, l);
                set_grid_led(9, 2, l);
                set_grid_led(11, 2, l);
                set_grid_led(9, 3, l);
                set_grid_led(10, 3, l);
                set_grid_led(11, 3, l);
                set_grid_led(13, 1, l); // 1
                set_grid_led(13, 2, l);
                set_grid_led(13, 3, l);
            } else if (sel_device == VOICE_JF) {
                set_grid_led(7, 1, l); // j
                set_grid_led(7, 2, l);
                set_grid_led(6, 3, l);
                set_grid_led(7, 3, l);
                set_grid_led(9, 1, l); // f
                set_grid_led(10, 1, l);
                set_grid_led(11, 1, l);
                set_grid_led(9, 2, l-4);
                set_grid_led(10, 2, l-4);
                set_grid_led(9, 3, l);
            } else if (sel_device == VOICE_TXO_NOTE) {
                set_grid_led(5, 1, l); // t
                set_grid_led(6, 1, l);
                set_grid_led(7, 1, l);
                set_grid_led(6, 2, l);
                set_grid_led(6, 3, l);
                set_grid_led(9, 1, l); // x
                set_grid_led(11, 1, l);
                set_grid_led(10, 2, l);
                set_grid_led(9, 3, l);
                set_grid_led(11, 3, l);
                set_grid_led(13, 1, l); // o
                set_grid_led(14, 1, l);
                set_grid_led(15, 1, l);
                set_grid_led(13, 2, l);
                set_grid_led(15, 2, l);
                set_grid_led(13, 3, l);
                set_grid_led(14, 3, l);
                set_grid_led(15, 3, l);
            }
            
            if (sel_param == 0) {
                set_grid_led(5, 5, l); // o
                set_grid_led(6, 5, l);
                set_grid_led(7, 5, l);
                set_grid_led(5, 6, l);
                set_grid_led(7, 6, l);
                set_grid_led(5, 7, l);
                set_grid_led(6, 7, l);
                set_grid_led(7, 7, l);
                set_grid_led(9, 5, l); // c
                set_grid_led(10, 5, l);
                set_grid_led(11, 5, l);
                set_grid_led(9, 6, l);
                set_grid_led(9, 7, l);
                set_grid_led(10, 7, l);
                set_grid_led(11, 7, l);
                set_grid_led(13, 5, l); // t
                set_grid_led(14, 5, l);
                set_grid_led(15, 5, l);
                set_grid_led(14, 6, l);
                set_grid_led(14, 7, l);
            } else if (sel_param == 1) {
                set_grid_led(5, 5, l); // t
                set_grid_led(6, 5, l);
                set_grid_led(7, 5, l);
                set_grid_led(6, 6, l);
                set_grid_led(6, 7, l);
                set_grid_led(9, 5, l); // r
                set_grid_led(10, 5, l);
                set_grid_led(11, 5, l);
                set_grid_led(9, 6, l);
                set_grid_led(9, 7, l);
                set_grid_led(13, 5, l); // n
                set_grid_led(14, 5, l);
                set_grid_led(15, 5, l);
                set_grid_led(13, 6, l);
                set_grid_led(15, 6, l);
                set_grid_led(13, 7, l);
                set_grid_led(15, 7, l);
            } else if (sel_param == 2) {
                set_grid_led(5, 5, l); // v
                set_grid_led(7, 5, l);
                set_grid_led(5, 6, l);
                set_grid_led(7, 6, l);
                set_grid_led(6, 7, l);
                set_grid_led(9, 5, l); // o
                set_grid_led(10, 5, l);
                set_grid_led(11, 5, l);
                set_grid_led(9, 6, l);
                set_grid_led(11, 6, l);
                set_grid_led(9, 7, l);
                set_grid_led(10, 7, l);
                set_grid_led(11, 7, l);
                set_grid_led(13, 5, l); // l
                set_grid_led(13, 6, l);
                set_grid_led(13, 7, l);
                set_grid_led(14, 7, l);
                set_grid_led(15, 7, l);
            } else if (sel_param == 3) {
                set_grid_led(5, 5, l); // w
                set_grid_led(7, 5, l);
                set_grid_led(5, 6, l);
                set_grid_led(6, 6, l-4);
                set_grid_led(7, 6, l);
                set_grid_led(5, 7, l);
                set_grid_led(6, 7, l-4);
                set_grid_led(7, 7, l);
                set_grid_led(9, 5, l); // a
                set_grid_led(10, 5, l);
                set_grid_led(11, 5, l);
                set_grid_led(9, 6, l);
                set_grid_led(10, 6, l-4);
                set_grid_led(11, 6, l);
                set_grid_led(9, 7, l);
                set_grid_led(11, 7, l);
                set_grid_led(13, 5, l); // v
                set_grid_led(15, 5, l);
                set_grid_led(13, 6, l);
                set_grid_led(15, 6, l);
                set_grid_led(14, 7, l);
            } else if (sel_param == 4) {
                set_grid_led(5, 5, l); // w
                set_grid_led(7, 5, l);
                set_grid_led(5, 6, l);
                set_grid_led(6, 6, l-4);
                set_grid_led(7, 6, l);
                set_grid_led(5, 7, l);
                set_grid_led(6, 7, l-4);
                set_grid_led(7, 7, l);
                set_grid_led(9, 5, l); // v
                set_grid_led(11, 5, l);
                set_grid_led(9, 6, l);
                set_grid_led(11, 6, l);
                set_grid_led(10, 7, l);
                set_grid_led(13, 5, l); // f
                set_grid_led(14, 5, l);
                set_grid_led(15, 5, l);
                set_grid_led(13, 6, l-4);
                set_grid_led(14, 6, l-4);
                set_grid_led(13, 7, l);
            } else if (sel_param == 5) {
                set_grid_led(5, 5, l); // a
                set_grid_led(6, 5, l);
                set_grid_led(7, 5, l);
                set_grid_led(5, 6, l);
                set_grid_led(6, 6, l-4);
                set_grid_led(7, 6, l);
                set_grid_led(5, 7, l);
                set_grid_led(7, 7, l);
                set_grid_led(9, 5, l); // t
                set_grid_led(10, 5, l);
                set_grid_led(11, 5, l);
                set_grid_led(10, 6, l);
                set_grid_led(10, 7, l);
                set_grid_led(13, 5, l); // t
                set_grid_led(14, 5, l);
                set_grid_led(15, 5, l);
                set_grid_led(14, 6, l);
                set_grid_led(14, 7, l);
            } else if (sel_param == 6) {
                set_grid_led(5, 5, l); // d
                set_grid_led(6, 5, l);
                set_grid_led(5, 6, l);
                set_grid_led(7, 6, l);
                set_grid_led(5, 7, l);
                set_grid_led(6, 7, l);
                set_grid_led(9, 5, l); // e
                set_grid_led(10, 5, l);
                set_grid_led(11, 5, l);
                set_grid_led(9, 6, l-4);
                set_grid_led(10, 6, l-4);
                set_grid_led(9, 7, l);
                set_grid_led(10, 7, l);
                set_grid_led(11, 7, l);
                set_grid_led(13, 5, l); // c
                set_grid_led(14, 5, l);
                set_grid_led(15, 5, l);
                set_grid_led(13, 6, l);
                set_grid_led(13, 7, l);
                set_grid_led(14, 7, l);
                set_grid_led(15, 7, l);
            }
            
            return;
        }
        
        u8 mod_on = sel_mod != 255;
        for (u8 i = 0; i < VOICE_COUNT; i++) {
            o = 1 << i;
            if (sel_device == VOICE_CV_GATE) {
                if (sel_param == 0) {
                    l = mod_on && (e.mod_octave[sel_mod] & o) ? 8 : 4;
                    for (u8 x = e.octave_mm[i][VOICE_CV_GATE] & 15;
                        x <= e.octave_mm[i][VOICE_CV_GATE] >> 4; x++) {
                            set_grid_led(4 + x, i, l);
                        }
                    set_grid_led(4 + e.octave[i][VOICE_CV_GATE], i, 12);
                } else if (sel_param == 1) {
                    set_grid_led(4 + e.transpose[i][VOICE_CV_GATE], i, 12);
                }
            } else if (sel_device == VOICE_ER301) {
                if (sel_param == 0) {
                    for (u8 x = e.octave_mm[i][VOICE_ER301] & 15;
                        x <= e.octave_mm[i][VOICE_ER301] >> 4; x++) {
                            l = mod_on && (e.mod_er301_octave[sel_mod] & o) ? 8 : 4;
                            set_grid_led(4 + x, i, l);
                        }
                    set_grid_led(4 + e.octave[i][VOICE_ER301], i, 12);
                } else if (sel_param == 1) {
                    set_grid_led(4 + e.transpose[i][VOICE_ER301], i, 12);
                } else if (sel_param == 2) {
                    for (u8 x = e.er301_volume_mm[i] & 15;
                        x <= e.er301_volume_mm[i] >> 4; x++) {
                            l = mod_on && (e.mod_er301_volume[sel_mod] & o) ? 8 : 4;
                            set_grid_led(4 + x, i, l);
                        }
                    set_grid_led(4 + e.er301_volume[i], i, 12);
                }
            } else if (sel_device == VOICE_JF) {
                if (sel_param == 0) {
                    if (i < 6) {
                        for (u8 x = e.octave_mm[i][VOICE_JF] & 15;
                            x <= e.octave_mm[i][VOICE_JF] >> 4; x++) {
                                l = mod_on && (e.mod_jf_octave[sel_mod] & o) ? 8 : 4;
                                set_grid_led(4 + x, i, l);
                            }
                        set_grid_led(4 + e.octave[i][VOICE_JF], i, 12);
                    }
                } else if (sel_param == 1) {
                    if (i < 6)
                        set_grid_led(4 + e.transpose[i][VOICE_JF], i, 12);
                } else if (sel_param == 2) {
                    if (i < 6) {
                        for (u8 x = e.jf_volume_mm[i] & 15;
                            x <= e.jf_volume_mm[i] >> 4; x++) {
                                l = mod_on && (e.mod_jf_volume[sel_mod] & o) ? 8 : 4;
                                set_grid_led(4 + x, i, l);
                            }
                        set_grid_led(4 + e.jf_volume[i], i, 12);
                    }
                }
            } else if (sel_device == VOICE_TXO_NOTE) {
                if (sel_param == 0) {
                    for (u8 x = e.octave_mm[i][VOICE_TXO_NOTE] & 15;
                        x <= e.octave_mm[i][VOICE_TXO_NOTE] >> 4; x++) {
                            l = mod_on && (e.mod_txo_octave[sel_mod] & o) ? 8 : 4;
                            set_grid_led(4 + x, i, l);
                        }
                    set_grid_led(4 + e.octave[i][VOICE_TXO_NOTE], i, 12);
                } else if (sel_param == 1) {
                    set_grid_led(4 + e.transpose[i][VOICE_TXO_NOTE], i, 12);
                } else if (sel_param == 2) {
                    for (u8 x = e.txo_volume_mm[i] & 15;
                        x <= e.txo_volume_mm[i] >> 4; x++) {
                            l = mod_on && (e.mod_txo_volume[sel_mod] & o) ? 8 : 4;
                            set_grid_led(4 + x, i, l);
                        }
                    set_grid_led(4 + e.txo_volume[i], i, 12);
                } else if (sel_param == 3) {
                    for (u8 x = e.txo_wave_mm[i] & 15;
                        x <= e.txo_wave_mm[i] >> 4; x++) {
                            l = mod_on && (e.mod_txo_wave[sel_mod] & o) ? 8 : 4;
                            set_grid_led(4 + x, i, l);
                        }
                    set_grid_led(4 + e.txo_wave[i], i, 12);
                } else if (sel_param == 4) {
                    for (u8 x = e.txo_wavef_mm[i] & 15;
                        x <= e.txo_wavef_mm[i] >> 4; x++) {
                            l = mod_on && (e.mod_txo_wavef[sel_mod] & o) ? 8 : 4;
                            set_grid_led(4 + x, i, l);
                        }
                    set_grid_led(4 + e.txo_wavef[i], i, 12);
                } else if (sel_param == 5) {
                    for (u8 x = e.txo_attack_mm[i] & 15;
                        x <= e.txo_attack_mm[i] >> 4; x++) {
                            l = mod_on && (e.mod_txo_attack[sel_mod] & o) ? 8 : 4;
                            set_grid_led(4 + x, i, l);
                        }
                    set_grid_led(4 + e.txo_attack[i], i, 12);
                } else if (sel_param == 6) {
                    for (u8 x = e.txo_decay_mm[i] & 15;
                        x <= e.txo_decay_mm[i] >> 4; x++) {
                            l = mod_on && (e.mod_txo_decay[sel_mod] & o) ? 8 : 4;
                            set_grid_led(4 + x, i, l);
                        }
                    set_grid_led(4 + e.txo_decay[i], i, 12);
                }
            }
        }
        
        return;
    }
    
    s16 index, x, y;
    if (es_view == es_main) {
        for (u8 i = 0; i < 128; i++)
            if (e.keymap[i]) set_grid_led_i(i, e.keymap[i] << 1);
        
        if (e.arp)
            set_grid_led_i(e.p[e.p_select].root_x + (e.p[e.p_select].root_y << 4), 7);
        for (u8 i = 0; i < VOICE_COUNT; i++)
            if (es_notes[i].active) {
                x = es_notes[i].x;
                y = es_notes[i].y;
                while (x < 0) {
                    y++;
                    x += 5;
                }
                while (x > 15) {
                    y--;
                    x -= 5;
                }
                index = (y << 4) + x;
                if (index >= 0 && (index & 15) != 0)
                    set_grid_led_i(index, 15);
            }
    } else { // pattern view
    
        u8 l = es_blinker ? 2 : 0;
        for (u8 i = 0; i < 16; i++) {
            set_grid_led_i((i & 3) + 34 + ((i >> 2) << 4), e.p[i].length ? 7 : 4);
            set_grid_led(10 + (i & 3), 2 + (i >> 2), i == e.p_leader ? (playing[i] ? 13 + l : 13) : (playing[i] ? 8 + l : 4));
        }
        set_grid_led_i((e.p_select & 3) + 34 + ((e.p_select >> 2) << 4), 15);
        set_grid_led(15, 2, 8);
        set_grid_led(15, 3, e.leader_reset ? 8 : 4);
        set_grid_led(15, 4, e.is_leader_clock ? 8 : 4);
    }
}

void render_arc() {
    if (!is_arc_connected()) return;
    
    clear_all_arc_leds();
    
    u8 index;
    for (u8 i = 0; i < get_arc_encoder_count(); i++) {
        set_arc_led(i, 0, 6);
        set_arc_led(i, 16, 6);
        set_arc_led(i, 32, 6);
        set_arc_led(i, 48, 6);
        set_arc_led(i, 8, 2);
        set_arc_led(i, 24, 2);
        set_arc_led(i, 40, 2);
        set_arc_led(i, 56, 2);
        index = get_knob_count() + i;
        set_arc_led(i, e.mod_value[index] >> 2, 8 + (e.mod_value[index] & 3));
    }
}

void remap_voices() {
    u8 out;
    for (u8 i = 0; i < VOICE_COUNT; i++)
        for (u8 output = 0; output < VOICE_COUNT; output++) {
            out = 1 << output;
            map_voice(i, VOICE_CV_GATE, output, e.voice_map[i][VOICE_CV_GATE] & out);
            map_voice(i, VOICE_ER301, output, e.voice_map[i][VOICE_ER301] & out);
            map_voice(i, VOICE_JF, output, e.voice_map[i][VOICE_JF] & out);
            map_voice(i, VOICE_TXO_NOTE, output, e.voice_map[i][VOICE_TXO_NOTE] & out);
            map_voice(i, VOICE_TXO_CV_GATE, output, e.voice_map[i][VOICE_TXO_CV_GATE] & out);
        }
}

void update_devices() {
    for (u8 i = 0; i < VOICE_COUNT; i++) {
        for (u8 j = 0; j < DEVICE_COUNT; j++)
            set_output_transpose(j, i, e.octave[i][j] * 12 + e.transpose[i][j]);
        set_output_max_volume(VOICE_ER301, i, (u16)e.er301_volume[i] * (MAX_LEVEL / 11));
        set_output_max_volume(VOICE_JF, i, (u16)e.jf_volume[i] * (MAX_LEVEL / 11));
        set_output_max_volume(VOICE_TXO_NOTE, i, (u16)e.txo_volume[i] * (MAX_LEVEL / 11));
        
        set_txo_attack(i, e.txo_attack[i] * 50);
        set_txo_decay(i, e.txo_decay[i] * 50);
        set_txo_waveform(i, e.txo_wave[i] * 409 + e.txo_wavef[i] * TXOWAVF); // 0..4500
        
        set_txo_mode(i, 1);
    }
}

void update_i2c() {
    es_kill_all_notes();
    for (u8 i = 0; i < DEVICE_COUNT; i++) mute_device(i, !shared.device_on[i]);

    if (!shared.i2c_mode) {
        set_as_i2c_follower(I2C_ADDRESS);
        return;
    }
    
    set_as_i2c_leader();
    set_jf_mode(e.jf_mode);
    
    update_devices();
}

void ii_es(uint8_t *data, uint8_t l) {
    if (!l) return;

	s16 d = (data[1] << 8) | data[2];
    u8 value;
    
    switch(data[0]) {
        case 0x51: // ES_PRESET:
            if (d >= 0 && d < get_preset_count()) {
                preset = selected_preset = d;
                load_preset();
                refresh_grid();
            }
            break;
                
        case 0x55: // ES_PATTERN:
            if (d >= 0 && d < 16) {
                e.p_select = d;
                refresh_grid();
            }
            break;
            
        case 0x53: // ES_CLOCK:
            clock_next_note(0);
            break;

        case 0x54: // ES_RESET:
            value = d;
            if (value > 15) value = 15;
            es_start_playback(e.p_select, value);
            break;
            
        case 0x57: // ES_STOP:
            es_stop_playback(e.p_select);
            break;
            
        case 0x56: // ES_TRANS:
            if (d > 0) {
                for (u8 i = 0; i < d; i++) {
                    if (e.p[e.p_select].root_y == 1 && e.p[e.p_select].root_x == 15) break;
                    e.p[e.p_select].root_x++;
                    if (e.p[e.p_select].root_x == 16)
                    {
                        e.p[e.p_select].root_x = 11;
                        e.p[e.p_select].root_y--;
                    }
                }
            } else {
                for (u8 i = 0; i < abs(d); i++) {
                    if (e.p[e.p_select].root_y == 7 && e.p[e.p_select].root_x == 1) break;
                    e.p[e.p_select].root_x--;
                    if (e.p[e.p_select].root_x == 0)
                    {
                        e.p[e.p_select].root_x = 5;
                        e.p[e.p_select].root_y++;
                    }
                }
            }
            es_start_playback(e.p_select, 0);
            refresh_grid();
            break;
            
        case 0x59: // ES_MAGIC:
            switch (d) {
                case 1:
                    es_half_speed();
                    break;
                
                case 2:
                    es_double_speed();
                    break;

                case 3:
                    e.p[e.p_select].linearize = 1;
                    es_update_total_time();
                    break;

                case 4:
                    e.p[e.p_select].linearize = 0;
                    es_update_total_time();
                    break;
                    
                case 5:
                    if (e.p[e.p_select].dir) es_reverse();
                    e.p[e.p_select].dir = 0;
                    if (es_mode == es_playing) es_kill_pattern_notes(e.p_select);
                    break;
                
                case 6:
                    if (!e.p[e.p_select].dir) es_reverse();
                    e.p[e.p_select].dir = 1;
                    if (es_mode == es_playing) es_kill_pattern_notes(e.p_select);
                    break;
            }
            refresh_grid();
            break;
            
        case 0x52: // ES_MODE:
            if (d < 0 || d >= 16) {
                e.p[e.p_select].edge = ES_EDGE_PATTERN;
                es_kill_all_notes();
            } else if (d == 0) {
                e.p[e.p_select].edge = ES_EDGE_DRONE;
            } else {
                u8 fixed = d;
                if (fixed > 15) fixed = 15;
                e.p[e.p_select].edge = ES_EDGE_FIXED;
                e.p[e.p_select].edge_time = (fixed + 1) << 4;
                es_kill_all_notes();
            }
            refresh_grid();
            break;
    }
}

void init_presets() {
    preset = selected_preset = 0;
    uint8_t i;
    
    shared.i2c_mode = 0;
    for (u8 i = 0; i < DEVICE_COUNT; i++) shared.device_on[i] = 1;
    for (u8 i = 0; i < VOICE_COUNT; i++) shared.voice_on[i] = 1;
    
    e.jf_mode = 1;
    for (u8 i = 0; i < VOICE_COUNT; i++) {
        for (u8 j = 0; j < DEVICE_COUNT; j++) {
            e.voice_map[i][j] = 0;
            e.pstart_map[i][j] = 0;
            e.octave[i][j] = 1;
            e.octave_mm[i][j] = 17;
            e.transpose[i][j] = 0;
        }
        
        if (i < get_gate_output_count() || i < get_cv_output_count()) 
            e.voice_map[i][VOICE_CV_GATE] |= 1 << i;
        
        e.er301_volume[i] = 8;
        e.jf_volume[i] = 8;
        e.txo_volume[i] = 8;
        e.txo_wave[i] = 0;
        e.txo_wavef[i] = 0;
        e.txo_attack[i] = 0;
        e.txo_decay[i] = 1;

        e.er301_volume_mm[i] = 72;
        e.jf_volume_mm[i] = 72;
        e.txo_volume_mm[i] = 72;
        e.txo_wave_mm[i] = 0;
        e.txo_wavef_mm[i] = 0;
        e.txo_attack_mm[i] = 0;
        e.txo_decay_mm[i] = 17;

        e.cv_x_map[i] = 0;
        e.cv_y_map[i] = 0;
        e.cv_key_map[i] = 0;
        e.er301_x_map[i] = 0;
        e.er301_y_map[i] = 0;
        e.er301_key_map[i] = 0;
        e.txo_x_map[i] = 0;
        e.txo_y_map[i] = 0;
        e.txo_key_map[i] = 0;
    }
    
    for (u8 i = 0; i < MOD_COUNT; i++) {
        e.mod_x[i] = 0;
        e.mod_y[i] = 0;
        e.mod_key[i] = 0;
        e.mod_gate[i] = 0;
        e.mod_pstart[i] = 0;
        
        e.mod_octave[i] = 0;
        e.mod_er301_octave[i] = 0;
        e.mod_jf_octave[i] = 0;
        e.mod_txo_octave[i] = 0;
        e.mod_er301_volume[i] = 0;
        e.mod_jf_volume[i] = 0;
        e.mod_txo_volume[i] = 0;
        e.mod_txo_attack[i] = 0;
        e.mod_txo_decay[i] = 0;
        e.mod_txo_wave[i] = 0;
        e.mod_txo_wavef[i] = 0;
        
        e.mod_value[i] = 0;
        
        e.midi_cc[i] = 0;
    }
    
    e.arp = 0;
    e.p_select = 0;
    e.p_leader = PATTERN_COUNT;
    e.is_leader_clock = 0;
    
    for (u8 i = 0; i < 128; i++) e.keymap[i] = 0;
    for (u8 i = 0; i < 16; i++) {
        e.p[i].interval_ind = 0;
        e.p[i].length = 0;
        e.p[i].loop = 0;
        e.p[i].edge = ES_EDGE_PATTERN;
        e.p[i].edge_time = 16;
        e.p[i].voices = 0;
        for (u8 j = 0; j < VOICE_COUNT; j++) e.p[i].voices = (e.p[i].voices  << 1) | 1;
        e.p[i].dir = 0;
        e.p[i].linearize = 0;
        e.p[i].start = 0;
        e.p[i].end = 15;
    }
    for (u8 i = 0; i < 8; i++) meta.glyph[i] = 0;

    for (i = 0; i < get_preset_count(); i++) {
        store_preset_to_flash(i, &meta, &e);
    }

    store_shared_data_to_flash(&shared);
    store_preset_index(0);
}

void copy_pattern(u8 src, u8 dest) {
    memcpy(&(e.p[dest]), &(e.p[src]), sizeof(es_pattern_t));
}

void load_preset() {
    load_preset_from_flash(preset, &e);
    load_preset_meta_from_flash(preset, &meta);
    es_kill_all_notes();
    remap_voices();
    update_devices();
}
