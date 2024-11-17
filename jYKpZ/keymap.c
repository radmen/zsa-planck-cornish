#include QMK_KEYBOARD_H
#ifdef AUDIO_ENABLE
#include "muse.h"
#endif
#include "eeprom.h"

enum planck_keycodes {
  RGB_SLD = EZ_SAFE_RANGE,
  ST_MACRO_0,
  ST_MACRO_1,
  ST_MACRO_2,
  ST_MACRO_3,
  ST_MACRO_4,
  ST_MACRO_5,
  ST_MACRO_6,
  ST_MACRO_7,
  ST_MACRO_8,
  ST_MACRO_9,
  ST_MACRO_10,
  ST_MACRO_11,
  ST_MACRO_12,
  ST_MACRO_13,
  ST_MACRO_14,
  ST_MACRO_15,
  ST_MACRO_16,
  ST_MACRO_17,
  ST_MACRO_18,
  ST_MACRO_19,
  ST_MACRO_20,
  ST_MACRO_21,
  ST_MACRO_22,
  ST_MACRO_23,
  ST_MACRO_24,
  ST_MACRO_25,
  ST_MACRO_26,
  ST_MACRO_27,
};


enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
  DANCE_4,
  DANCE_5,
  DANCE_6,
  DANCE_7,
  DANCE_8,
  DANCE_9,
  DANCE_10,
  DANCE_11,
  DANCE_12,
};

enum planck_layers {
  _BASE,
  _LOWER,
  _RAISE,
  _ADJUST,
  _LAYER4,
  _LAYER5,
  _LAYER6,
  _LAYER7,
  _LAYER8,
};

#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [_BASE] = LAYOUT_planck_grid(
    KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,           KC_NO,          KC_NO,          KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           
    MT(MOD_LCTL, KC_A),MT(MOD_LALT, KC_S),MT(MOD_RGUI, KC_D),MT(MOD_LSFT, KC_F),KC_G,           KC_NO,          KC_NO,          KC_H,           MT(MOD_RSFT, KC_J),MT(MOD_RGUI, KC_K),MT(MOD_LALT, KC_L),MT(MOD_RCTL, KC_SCLN),
    MT(MOD_RALT, KC_Z),KC_X,           LT(5,KC_C),     LT(7,KC_V),     KC_B,           KC_NO,          KC_NO,          KC_N,           LT(7,KC_M),     LT(5,KC_COMMA), KC_DOT,         MT(MOD_RALT, KC_SLASH),
    KC_NO,          KC_NO,          MT(MOD_LCTL, KC_ESCAPE),KC_SPACE,       OSL(4),         KC_NO,          KC_NO,          MT(MOD_LGUI, KC_BSPC),OSL(8),         OSL(6),         KC_NO,          KC_NO
  ),

  [_LOWER] = LAYOUT_planck_grid(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_RAISE] = LAYOUT_planck_grid(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_ADJUST] = LAYOUT_planck_grid(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_LAYER4] = LAYOUT_planck_grid(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, CW_TOGG,        KC_DELETE,      
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       KC_ENTER,       
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_HOME,        KC_PGDN,        KC_PAGE_UP,     KC_END,         KC_TAB,         
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_LAYER5] = LAYOUT_planck_grid(
    KC_GRAVE,       KC_LBRC,        KC_RBRC,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_LCBR,        KC_RCBR,        KC_TRANSPARENT, 
    MT(MOD_LCTL, KC_1),MT(MOD_LALT, KC_2),MT(MOD_LGUI, KC_3),MT(MOD_LSFT, KC_4),KC_5,           KC_TRANSPARENT, KC_TRANSPARENT, KC_6,           MT(MOD_RSFT, KC_7),MT(MOD_RGUI, KC_8),MT(MOD_LALT, KC_9),MT(MOD_RCTL, KC_0),
    KC_BSLS,        KC_LPRN,        KC_RPRN,        KC_QUOTE,       KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_EQUAL,       KC_MINUS,       KC_LABK,        KC_RABK,        KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_LAYER6] = LAYOUT_planck_grid(
    KC_TRANSPARENT, LALT(LGUI(LCTL(LSFT(KC_W)))),KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,LALT(LGUI(LCTL(LSFT(KC_T)))),KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, LCTL(KC_PAGE_UP),LCTL(KC_PGDN),  KC_TRANSPARENT, QK_BOOT,        
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TD(DANCE_0),    TD(DANCE_1),    TD(DANCE_2),    TD(DANCE_3),    KC_TRANSPARENT, 
    KC_MEDIA_PREV_TRACK,KC_MEDIA_NEXT_TRACK,KC_MEDIA_PLAY_PAUSE,TD(DANCE_4),    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_LAYER7] = LAYOUT_planck_grid(
    KC_TILD,        KC_LBRC,        KC_RBRC,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_LCBR,        KC_RCBR,        KC_TRANSPARENT, 
    TD(DANCE_5),    TD(DANCE_6),    TD(DANCE_7),    TD(DANCE_8),    KC_PERC,        KC_TRANSPARENT, KC_TRANSPARENT, KC_CIRC,        TD(DANCE_9),    TD(DANCE_10),   TD(DANCE_11),   TD(DANCE_12),   
    KC_PIPE,        KC_LPRN,        KC_RPRN,        KC_DQUO,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_PLUS,        KC_UNDS,        KC_LABK,        KC_RABK,        KC_QUES,        
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_LAYER8] = LAYOUT_planck_grid(
    ST_MACRO_0,     ST_MACRO_1,     ST_MACRO_2,     ST_MACRO_3,     ST_MACRO_4,     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, ST_MACRO_5,     ST_MACRO_6,     ST_MACRO_7,     KC_TRANSPARENT, 
    ST_MACRO_8,     ST_MACRO_9,     ST_MACRO_10,    ST_MACRO_11,    ST_MACRO_12,    KC_TRANSPARENT, KC_TRANSPARENT, ST_MACRO_13,    ST_MACRO_14,    ST_MACRO_15,    ST_MACRO_16,    ST_MACRO_17,    
    ST_MACRO_18,    ST_MACRO_19,    ST_MACRO_20,    ST_MACRO_21,    ST_MACRO_22,    KC_TRANSPARENT, KC_TRANSPARENT, ST_MACRO_23,    ST_MACRO_24,    ST_MACRO_25,    ST_MACRO_26,    ST_MACRO_27,    
    KC_TRANSPARENT, KC_TRANSPARENT, OSM(MOD_LSFT),  KC_TRANSPARENT, CW_TOGG,        KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

};



extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {0,0,0}, {0,0,0}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {0,0,0}, {0,0,0}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {0,0,0}, {0,0,0}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {139,227,202}, {0,0,0}, {0,0,0}, {139,227,202}, {139,227,202}, {139,227,202}, {0,0,0}, {139,227,202}, {139,227,202}, {139,227,202}, {0,0,0}, {0,0,0} },

    [4] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {28,203,236}, {28,203,236}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {28,203,236}, {28,203,236}, {28,203,236}, {28,203,236}, {28,203,236}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {28,203,236}, {28,203,236}, {28,203,236}, {28,203,236}, {28,203,236}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {28,203,236}, {0,0,0}, {0,0,0}, {0,0,0} },

    [5] = { {203,129,219}, {203,129,219}, {203,129,219}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {203,129,219}, {203,129,219}, {0,0,0}, {203,129,219}, {203,129,219}, {203,129,219}, {203,129,219}, {203,129,219}, {0,0,0}, {0,0,0}, {203,129,219}, {203,129,219}, {203,129,219}, {203,129,219}, {203,129,219}, {203,129,219}, {203,129,219}, {203,129,219}, {203,129,219}, {0,0,0}, {0,0,0}, {0,0,0}, {203,129,219}, {203,129,219}, {203,129,219}, {203,129,219}, {203,129,219}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [6] = { {0,0,0}, {105,192,219}, {105,192,219}, {105,192,219}, {105,192,219}, {0,0,0}, {0,0,0}, {0,0,0}, {105,192,219}, {105,192,219}, {0,0,0}, {105,192,219}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {105,192,219}, {105,192,219}, {105,192,219}, {105,192,219}, {0,0,0}, {105,192,219}, {105,192,219}, {105,192,219}, {105,192,219}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [7] = { {203,201,230}, {203,201,230}, {203,201,230}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {203,201,230}, {203,201,230}, {0,0,0}, {203,201,230}, {203,201,230}, {203,201,230}, {203,201,230}, {203,201,230}, {0,0,0}, {0,0,0}, {203,201,230}, {203,201,230}, {203,201,230}, {203,201,230}, {203,201,230}, {203,201,230}, {203,201,230}, {203,201,230}, {203,201,230}, {0,0,0}, {0,0,0}, {0,0,0}, {203,201,230}, {203,201,230}, {203,201,230}, {203,201,230}, {203,201,230}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 4:
      set_layer_color(4);
      break;
    case 5:
      set_layer_color(5);
      break;
    case 6:
      set_layer_color(6);
      break;
    case 7:
      set_layer_color(7);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_N) SS_DELAY(100) SS_TAP(X_E) SS_DELAY(100) SS_TAP(X_W));
    }
    break;
    case ST_MACRO_1:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_T) SS_DELAY(100) SS_TAP(X_H) SS_DELAY(100) SS_TAP(X_I) SS_DELAY(100) SS_TAP(X_S));
    }
    break;
    case ST_MACRO_2:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_F) SS_DELAY(100) SS_TAP(X_O) SS_DELAY(100) SS_TAP(X_R));
    }
    break;
    case ST_MACRO_3:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_E) SS_DELAY(100) SS_TAP(X_A) SS_DELAY(100) SS_TAP(X_C) SS_DELAY(100) SS_TAP(X_H));
    }
    break;
    case ST_MACRO_4:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_W) SS_DELAY(100) SS_TAP(X_H) SS_DELAY(100) SS_TAP(X_I) SS_DELAY(100) SS_TAP(X_L) SS_DELAY(100) SS_TAP(X_E));
    }
    break;
    case ST_MACRO_5:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_F) SS_DELAY(100) SS_TAP(X_I) SS_DELAY(100) SS_TAP(X_X) SS_DELAY(100) SS_LSFT(SS_TAP(X_SCLN)));
    }
    break;
    case ST_MACRO_6:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_SPACE) SS_DELAY(100) SS_TAP(X_G) SS_DELAY(100) SS_TAP(X_S));
    }
    break;
    case ST_MACRO_7:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_W) SS_DELAY(100) SS_TAP(X_I) SS_DELAY(100) SS_TAP(X_P) SS_DELAY(100) SS_LSFT(SS_TAP(X_SCLN)));
    }
    break;
    case ST_MACRO_8:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_G) SS_DELAY(100) SS_TAP(X_E) SS_DELAY(100) SS_TAP(X_T));
    }
    break;
    case ST_MACRO_9:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_F) SS_DELAY(100) SS_TAP(X_I) SS_DELAY(100) SS_TAP(X_N) SS_DELAY(100) SS_TAP(X_D));
    }
    break;
    case ST_MACRO_10:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_T) SS_DELAY(100) SS_TAP(X_R) SS_DELAY(100) SS_TAP(X_U) SS_DELAY(100) SS_TAP(X_E));
    }
    break;
    case ST_MACRO_11:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_F) SS_DELAY(100) SS_TAP(X_A) SS_DELAY(100) SS_TAP(X_L) SS_DELAY(100) SS_TAP(X_S) SS_DELAY(100) SS_TAP(X_E));
    }
    break;
    case ST_MACRO_12:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_N) SS_DELAY(100) SS_TAP(X_U) SS_DELAY(100) SS_TAP(X_L) SS_DELAY(100) SS_TAP(X_L));
    }
    break;
    case ST_MACRO_13:
    if (record->event.pressed) {
      SEND_STRING(SS_LSFT(SS_TAP(X_DOT)) SS_DELAY(100) SS_TAP(X_I) SS_DELAY(100) SS_LSFT(SS_TAP(X_RBRC)));
    }
    break;
    case ST_MACRO_14:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_LBRC) SS_DELAY(100) SS_TAP(X_C));
    }
    break;
    case ST_MACRO_15:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_RBRC) SS_DELAY(100) SS_TAP(X_C));
    }
    break;
    case ST_MACRO_16:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_Z) SS_DELAY(100) SS_TAP(X_F) SS_DELAY(100) SS_TAP(X_I) SS_DELAY(100) SS_LSFT(SS_TAP(X_LBRC)));
    }
    break;
    case ST_MACRO_17:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_ESCAPE) SS_DELAY(100) SS_TAP(X_COMMA) SS_DELAY(100) SS_TAP(X_SPACE));
    }
    break;
    case ST_MACRO_18:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_I) SS_DELAY(100) SS_TAP(X_S));
    }
    break;
    case ST_MACRO_19:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_H) SS_DELAY(100) SS_TAP(X_A) SS_DELAY(100) SS_TAP(X_S));
    }
    break;
    case ST_MACRO_20:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_I) SS_DELAY(100) SS_TAP(X_F));
    }
    break;
    case ST_MACRO_21:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_E) SS_DELAY(100) SS_TAP(X_L) SS_DELAY(100) SS_TAP(X_S) SS_DELAY(100) SS_TAP(X_E));
    }
    break;
    case ST_MACRO_22:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_W) SS_DELAY(100) SS_TAP(X_H) SS_DELAY(100) SS_TAP(X_E) SS_DELAY(100) SS_TAP(X_N));
    }
    break;
    case ST_MACRO_23:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_V) SS_DELAY(100) SS_TAP(X_I) SS_DELAY(100) SS_LSFT(SS_TAP(X_RBRC)));
    }
    break;
    case ST_MACRO_24:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_D) SS_DELAY(100) SS_TAP(X_O));
    }
    break;
    case ST_MACRO_25:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_D) SS_DELAY(100) SS_TAP(X_P));
    }
    break;
    case ST_MACRO_26:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_Z) SS_DELAY(100) SS_TAP(X_F) SS_DELAY(100) SS_TAP(X_I) SS_DELAY(100) SS_TAP(X_LBRC));
    }
    break;
    case ST_MACRO_27:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_ESCAPE) SS_DELAY(100) SS_LSFT(SS_TAP(X_MINUS)));
    }
    break;

    case RGB_SLD:
        if (rawhid_state.rgb_control) {
            return false;
        }
        if (record->event.pressed) {
            rgblight_mode(1);
        }
        return false;
  }
  return true;
}

#ifdef AUDIO_ENABLE
bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
    if (muse_mode) {
        if (IS_LAYER_ON(_RAISE)) {
            if (clockwise) {
                muse_offset++;
            } else {
                muse_offset--;
            }
        } else {
            if (clockwise) {
                muse_tempo+=1;
            } else {
                muse_tempo-=1;
            }
        }
    } else {
        if (clockwise) {
        #ifdef MOUSEKEY_ENABLE
            register_code(KC_MS_WH_DOWN);
            unregister_code(KC_MS_WH_DOWN);
        #else
            register_code(KC_PGDN);
            unregister_code(KC_PGDN);
        #endif
        } else {
        #ifdef MOUSEKEY_ENABLE
            register_code(KC_MS_WH_UP);
            unregister_code(KC_MS_WH_UP);
        #else
            register_code(KC_PGUP);
            unregister_code(KC_PGUP);
        #endif
        }
    }
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    }
#endif
}

bool music_mask_user(uint16_t keycode) {
    switch (keycode) {
    case RAISE:
    case LOWER:
        return false;
    default:
        return true;
    }
}
#endif

uint16_t layer_state_set_user(uint16_t state) {
    return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[13];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LGUI(KC_LEFT));
        tap_code16(LGUI(KC_LEFT));
        tap_code16(LGUI(KC_LEFT));
    }
    if(state->count > 3) {
        tap_code16(LGUI(KC_LEFT));
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(LGUI(KC_LEFT)); break;
        case SINGLE_HOLD: register_code16(LGUI(KC_HOME)); break;
        case DOUBLE_TAP: register_code16(LGUI(KC_LEFT)); register_code16(LGUI(KC_LEFT)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LGUI(KC_LEFT)); register_code16(LGUI(KC_LEFT));
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(LGUI(KC_LEFT)); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_HOME)); break;
        case DOUBLE_TAP: unregister_code16(LGUI(KC_LEFT)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LGUI(KC_LEFT)); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LGUI(KC_PGDN));
        tap_code16(LGUI(KC_PGDN));
        tap_code16(LGUI(KC_PGDN));
    }
    if(state->count > 3) {
        tap_code16(LGUI(KC_PGDN));
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(LGUI(KC_PGDN)); break;
        case SINGLE_HOLD: register_code16(LGUI(KC_DOWN)); break;
        case DOUBLE_TAP: register_code16(LGUI(KC_PGDN)); register_code16(LGUI(KC_PGDN)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LGUI(KC_PGDN)); register_code16(LGUI(KC_PGDN));
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(LGUI(KC_PGDN)); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_DOWN)); break;
        case DOUBLE_TAP: unregister_code16(LGUI(KC_PGDN)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LGUI(KC_PGDN)); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(tap_dance_state_t *state, void *user_data);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LGUI(KC_PAGE_UP));
        tap_code16(LGUI(KC_PAGE_UP));
        tap_code16(LGUI(KC_PAGE_UP));
    }
    if(state->count > 3) {
        tap_code16(LGUI(KC_PAGE_UP));
    }
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(LGUI(KC_PAGE_UP)); break;
        case SINGLE_HOLD: register_code16(LGUI(KC_UP)); break;
        case DOUBLE_TAP: register_code16(LGUI(KC_PAGE_UP)); register_code16(LGUI(KC_PAGE_UP)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LGUI(KC_PAGE_UP)); register_code16(LGUI(KC_PAGE_UP));
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(LGUI(KC_PAGE_UP)); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_UP)); break;
        case DOUBLE_TAP: unregister_code16(LGUI(KC_PAGE_UP)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LGUI(KC_PAGE_UP)); break;
    }
    dance_state[2].step = 0;
}
void on_dance_3(tap_dance_state_t *state, void *user_data);
void dance_3_finished(tap_dance_state_t *state, void *user_data);
void dance_3_reset(tap_dance_state_t *state, void *user_data);

void on_dance_3(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LGUI(KC_RIGHT));
        tap_code16(LGUI(KC_RIGHT));
        tap_code16(LGUI(KC_RIGHT));
    }
    if(state->count > 3) {
        tap_code16(LGUI(KC_RIGHT));
    }
}

void dance_3_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case SINGLE_TAP: register_code16(LGUI(KC_RIGHT)); break;
        case SINGLE_HOLD: register_code16(LGUI(KC_END)); break;
        case DOUBLE_TAP: register_code16(LGUI(KC_RIGHT)); register_code16(LGUI(KC_RIGHT)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LGUI(KC_RIGHT)); register_code16(LGUI(KC_RIGHT));
    }
}

void dance_3_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
        case SINGLE_TAP: unregister_code16(LGUI(KC_RIGHT)); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_END)); break;
        case DOUBLE_TAP: unregister_code16(LGUI(KC_RIGHT)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LGUI(KC_RIGHT)); break;
    }
    dance_state[3].step = 0;
}
void on_dance_4(tap_dance_state_t *state, void *user_data);
void dance_4_finished(tap_dance_state_t *state, void *user_data);
void dance_4_reset(tap_dance_state_t *state, void *user_data);

void on_dance_4(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(KC_TAB));
        tap_code16(LALT(KC_TAB));
        tap_code16(LALT(KC_TAB));
    }
    if(state->count > 3) {
        tap_code16(LALT(KC_TAB));
    }
}

void dance_4_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[4].step = dance_step(state);
    switch (dance_state[4].step) {
        case SINGLE_TAP: register_code16(LALT(KC_TAB)); break;
        case SINGLE_HOLD: register_code16(LALT(LSFT(KC_TAB))); break;
        case DOUBLE_TAP: register_code16(LALT(KC_TAB)); register_code16(LALT(KC_TAB)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(KC_TAB)); register_code16(LALT(KC_TAB));
    }
}

void dance_4_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[4].step) {
        case SINGLE_TAP: unregister_code16(LALT(KC_TAB)); break;
        case SINGLE_HOLD: unregister_code16(LALT(LSFT(KC_TAB))); break;
        case DOUBLE_TAP: unregister_code16(LALT(KC_TAB)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(KC_TAB)); break;
    }
    dance_state[4].step = 0;
}
void on_dance_5(tap_dance_state_t *state, void *user_data);
void dance_5_finished(tap_dance_state_t *state, void *user_data);
void dance_5_reset(tap_dance_state_t *state, void *user_data);

void on_dance_5(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_EXLM);
        tap_code16(KC_EXLM);
        tap_code16(KC_EXLM);
    }
    if(state->count > 3) {
        tap_code16(KC_EXLM);
    }
}

void dance_5_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[5].step = dance_step(state);
    switch (dance_state[5].step) {
        case SINGLE_TAP: register_code16(KC_EXLM); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_CTRL); break;
        case DOUBLE_TAP: register_code16(KC_EXLM); register_code16(KC_EXLM); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_EXLM); register_code16(KC_EXLM);
    }
}

void dance_5_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[5].step) {
        case SINGLE_TAP: unregister_code16(KC_EXLM); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_CTRL); break;
        case DOUBLE_TAP: unregister_code16(KC_EXLM); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_EXLM); break;
    }
    dance_state[5].step = 0;
}
void on_dance_6(tap_dance_state_t *state, void *user_data);
void dance_6_finished(tap_dance_state_t *state, void *user_data);
void dance_6_reset(tap_dance_state_t *state, void *user_data);

void on_dance_6(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_AT);
        tap_code16(KC_AT);
        tap_code16(KC_AT);
    }
    if(state->count > 3) {
        tap_code16(KC_AT);
    }
}

void dance_6_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[6].step = dance_step(state);
    switch (dance_state[6].step) {
        case SINGLE_TAP: register_code16(KC_AT); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_ALT); break;
        case DOUBLE_TAP: register_code16(KC_AT); register_code16(KC_AT); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_AT); register_code16(KC_AT);
    }
}

void dance_6_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[6].step) {
        case SINGLE_TAP: unregister_code16(KC_AT); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_ALT); break;
        case DOUBLE_TAP: unregister_code16(KC_AT); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_AT); break;
    }
    dance_state[6].step = 0;
}
void on_dance_7(tap_dance_state_t *state, void *user_data);
void dance_7_finished(tap_dance_state_t *state, void *user_data);
void dance_7_reset(tap_dance_state_t *state, void *user_data);

void on_dance_7(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_HASH);
        tap_code16(KC_HASH);
        tap_code16(KC_HASH);
    }
    if(state->count > 3) {
        tap_code16(KC_HASH);
    }
}

void dance_7_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[7].step = dance_step(state);
    switch (dance_state[7].step) {
        case SINGLE_TAP: register_code16(KC_HASH); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_GUI); break;
        case DOUBLE_TAP: register_code16(KC_HASH); register_code16(KC_HASH); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_HASH); register_code16(KC_HASH);
    }
}

void dance_7_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[7].step) {
        case SINGLE_TAP: unregister_code16(KC_HASH); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_GUI); break;
        case DOUBLE_TAP: unregister_code16(KC_HASH); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_HASH); break;
    }
    dance_state[7].step = 0;
}
void on_dance_8(tap_dance_state_t *state, void *user_data);
void dance_8_finished(tap_dance_state_t *state, void *user_data);
void dance_8_reset(tap_dance_state_t *state, void *user_data);

void on_dance_8(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_DLR);
        tap_code16(KC_DLR);
        tap_code16(KC_DLR);
    }
    if(state->count > 3) {
        tap_code16(KC_DLR);
    }
}

void dance_8_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[8].step = dance_step(state);
    switch (dance_state[8].step) {
        case SINGLE_TAP: register_code16(KC_DLR); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_SHIFT); break;
        case DOUBLE_TAP: register_code16(KC_DLR); register_code16(KC_DLR); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_DLR); register_code16(KC_DLR);
    }
}

void dance_8_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[8].step) {
        case SINGLE_TAP: unregister_code16(KC_DLR); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_SHIFT); break;
        case DOUBLE_TAP: unregister_code16(KC_DLR); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_DLR); break;
    }
    dance_state[8].step = 0;
}
void on_dance_9(tap_dance_state_t *state, void *user_data);
void dance_9_finished(tap_dance_state_t *state, void *user_data);
void dance_9_reset(tap_dance_state_t *state, void *user_data);

void on_dance_9(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_AMPR);
        tap_code16(KC_AMPR);
        tap_code16(KC_AMPR);
    }
    if(state->count > 3) {
        tap_code16(KC_AMPR);
    }
}

void dance_9_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[9].step = dance_step(state);
    switch (dance_state[9].step) {
        case SINGLE_TAP: register_code16(KC_AMPR); break;
        case SINGLE_HOLD: register_code16(KC_RIGHT_SHIFT); break;
        case DOUBLE_TAP: register_code16(KC_AMPR); register_code16(KC_AMPR); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_AMPR); register_code16(KC_AMPR);
    }
}

void dance_9_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[9].step) {
        case SINGLE_TAP: unregister_code16(KC_AMPR); break;
        case SINGLE_HOLD: unregister_code16(KC_RIGHT_SHIFT); break;
        case DOUBLE_TAP: unregister_code16(KC_AMPR); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_AMPR); break;
    }
    dance_state[9].step = 0;
}
void on_dance_10(tap_dance_state_t *state, void *user_data);
void dance_10_finished(tap_dance_state_t *state, void *user_data);
void dance_10_reset(tap_dance_state_t *state, void *user_data);

void on_dance_10(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_ASTR);
        tap_code16(KC_ASTR);
        tap_code16(KC_ASTR);
    }
    if(state->count > 3) {
        tap_code16(KC_ASTR);
    }
}

void dance_10_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[10].step = dance_step(state);
    switch (dance_state[10].step) {
        case SINGLE_TAP: register_code16(KC_ASTR); break;
        case SINGLE_HOLD: register_code16(KC_RIGHT_GUI); break;
        case DOUBLE_TAP: register_code16(KC_ASTR); register_code16(KC_ASTR); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_ASTR); register_code16(KC_ASTR);
    }
}

void dance_10_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[10].step) {
        case SINGLE_TAP: unregister_code16(KC_ASTR); break;
        case SINGLE_HOLD: unregister_code16(KC_RIGHT_GUI); break;
        case DOUBLE_TAP: unregister_code16(KC_ASTR); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_ASTR); break;
    }
    dance_state[10].step = 0;
}
void on_dance_11(tap_dance_state_t *state, void *user_data);
void dance_11_finished(tap_dance_state_t *state, void *user_data);
void dance_11_reset(tap_dance_state_t *state, void *user_data);

void on_dance_11(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_LPRN);
        tap_code16(KC_LPRN);
        tap_code16(KC_LPRN);
    }
    if(state->count > 3) {
        tap_code16(KC_LPRN);
    }
}

void dance_11_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[11].step = dance_step(state);
    switch (dance_state[11].step) {
        case SINGLE_TAP: register_code16(KC_LPRN); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_ALT); break;
        case DOUBLE_TAP: register_code16(KC_LPRN); register_code16(KC_LPRN); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_LPRN); register_code16(KC_LPRN);
    }
}

void dance_11_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[11].step) {
        case SINGLE_TAP: unregister_code16(KC_LPRN); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_ALT); break;
        case DOUBLE_TAP: unregister_code16(KC_LPRN); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_LPRN); break;
    }
    dance_state[11].step = 0;
}
void on_dance_12(tap_dance_state_t *state, void *user_data);
void dance_12_finished(tap_dance_state_t *state, void *user_data);
void dance_12_reset(tap_dance_state_t *state, void *user_data);

void on_dance_12(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_RPRN);
        tap_code16(KC_RPRN);
        tap_code16(KC_RPRN);
    }
    if(state->count > 3) {
        tap_code16(KC_RPRN);
    }
}

void dance_12_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[12].step = dance_step(state);
    switch (dance_state[12].step) {
        case SINGLE_TAP: register_code16(KC_RPRN); break;
        case SINGLE_HOLD: register_code16(KC_RIGHT_CTRL); break;
        case DOUBLE_TAP: register_code16(KC_RPRN); register_code16(KC_RPRN); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_RPRN); register_code16(KC_RPRN);
    }
}

void dance_12_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[12].step) {
        case SINGLE_TAP: unregister_code16(KC_RPRN); break;
        case SINGLE_HOLD: unregister_code16(KC_RIGHT_CTRL); break;
        case DOUBLE_TAP: unregister_code16(KC_RPRN); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_RPRN); break;
    }
    dance_state[12].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
        [DANCE_4] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_4, dance_4_finished, dance_4_reset),
        [DANCE_5] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_5, dance_5_finished, dance_5_reset),
        [DANCE_6] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_6, dance_6_finished, dance_6_reset),
        [DANCE_7] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_7, dance_7_finished, dance_7_reset),
        [DANCE_8] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_8, dance_8_finished, dance_8_reset),
        [DANCE_9] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_9, dance_9_finished, dance_9_reset),
        [DANCE_10] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_10, dance_10_finished, dance_10_reset),
        [DANCE_11] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_11, dance_11_finished, dance_11_reset),
        [DANCE_12] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_12, dance_12_finished, dance_12_reset),
};
