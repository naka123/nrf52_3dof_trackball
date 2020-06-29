//
// Created by user on 15-Jun-20.
//

#pragma once

#ifndef NRF52_TEST_HID_3DX_H
#define NRF52_TEST_HID_3DX_H

#include <RingBuffer.h>

#include <Arduino.h>
#include <Adafruit_USBD_HID.h>

#include "bipbuf.hpp"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

static const int JOYSTICK_REPORT_SIZE = 2 * 8 + 4 + 4 * 2;

extern bipbuf_t BipBuf;

enum {
    REPORT_ID_6DOF = 1,
    REPORT_ID_BUTTONS = 3,

    REPORT_ID_MOUSE = 0x84,
    REPORT_ID_JOYSTICK = 0x85,
    REPORT_ID_RAW_GYRO = 0x87,

    REPORT_ID_MODE = 0x80,
    REPORT_ID_R_SCALE = 0x81,
    REPORT_ID_T_SCALE = 0x82,
    REPORT_ID_MIRROR_FEED = 0x88,
};

enum {
    MODE_ROT_3DOF = 0,
    MODE_TRANS2_ROT1 = 1,
    MODE_MOUSE_1RDOF = 2,
};

enum {
    V3DK_MENU       = 0x0001,
    V3DK_FIT        = 0x0002,
    V3DK_TOP        = 0x0004,
    V3DK_RIGHT      = 0x0010,
    V3DK_V3DK_FRONT = 0x0020,
    V3DK_ROLL_CW    = 0x0100,
    V3DK_1          = 0x1000,
    V3DK_2          = 0x2000,
    V3DK_3          = 0x4000,
    V3DK_4          = 0x8000,
    V3DK_ESC        = 0x00400000,
    V3DK_ALT        = 0x00800000,
    V3DK_SHIFT      = 0x01000000,
    V3DK_CTRL       = 0x02000000,
    V3DK_ROTATE     = 0x04000000,

};

#define TUD_HID_REPORT_DESC_MULTIAXIS_CONTROLLER() \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP      )                   ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_MOUSE     )                   ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION  )                   ,\
    /* Report ID if any */\
    HID_REPORT_ID       ( REPORT_ID_MOUSE ), \
    HID_USAGE      ( HID_USAGE_DESKTOP_POINTER )                   ,\
    HID_COLLECTION ( HID_COLLECTION_PHYSICAL   )                   ,\
      HID_USAGE_PAGE  ( HID_USAGE_PAGE_BUTTON  )                   ,\
        HID_USAGE_MIN   ( 1                                      ) ,\
        HID_USAGE_MAX   ( 5                                      ) ,\
        HID_LOGICAL_MIN ( 0                                      ) ,\
        HID_LOGICAL_MAX ( 1                                      ) ,\
        /* Left, Right, Middle, Backward, Forward buttons */ \
        HID_REPORT_COUNT( 5                                      ) ,\
        HID_REPORT_SIZE ( 1                                      ) ,\
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
        /* 3 bit padding */ \
        HID_REPORT_COUNT( 1                                      ) ,\
        HID_REPORT_SIZE ( 3                                      ) ,\
        HID_INPUT       ( HID_CONSTANT                           ) ,\
      HID_USAGE_PAGE  ( HID_USAGE_PAGE_DESKTOP )                   ,\
        /* X, Y position [-127, 127] */ \
        HID_USAGE       ( HID_USAGE_DESKTOP_X                    ) ,\
        HID_USAGE       ( HID_USAGE_DESKTOP_Y                    ) ,\
        /* Verital wheel scroll [-127, 127] */ \
        HID_USAGE       ( HID_USAGE_DESKTOP_WHEEL                )  ,\
        HID_LOGICAL_MIN ( 0x81                                   ) ,\
        HID_LOGICAL_MAX ( 0x7f                                   ) ,\
        HID_REPORT_COUNT( 3                                      ) ,\
        HID_REPORT_SIZE ( 8                                      ) ,\
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE ) ,\
      HID_USAGE_PAGE  ( HID_USAGE_PAGE_CONSUMER ), \
       /* Horizontal wheel scroll [-127, 127] */ \
        HID_USAGE_N     ( HID_USAGE_CONSUMER_AC_PAN, 2           ), \
        HID_LOGICAL_MIN ( 0x81                                   ), \
        HID_LOGICAL_MAX ( 0x7f                                   ), \
        HID_REPORT_COUNT( 1                                      ), \
        HID_REPORT_SIZE ( 8                                      ), \
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE ), \
    HID_COLLECTION_END                                            , \
  HID_COLLECTION_END, \
  \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )        ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_JOYSTICK  )        ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )        ,\
    /* Report ID if any */\
    HID_REPORT_ID       ( REPORT_ID_JOYSTICK ), \
    /* X, Y, Z, Rz (min -127, max 127 ) */ \
    HID_USAGE_PAGE   ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_X                    ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_Y                    ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_Z                    ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_RX                   ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_RY                   ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_RZ                   ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_SLIDER               ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_DIAL                 ) ,\
    HID_REPORT_SIZE  ( 16                                     ) ,\
    HID_REPORT_COUNT ( 8                                      ) ,\
    HID_PHYSICAL_MIN ( 0), \
    HID_PHYSICAL_MAX_N ( 0xFFFF, 2 ), \
    HID_LOGICAL_MIN ( 0), \
    HID_LOGICAL_MAX_N ( 0xFFFF, 2 ), \
    HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    /* 32 bit Button Map */ \
    HID_USAGE_PAGE   ( HID_USAGE_PAGE_BUTTON                  ) ,\
    HID_USAGE_MIN    ( 1                                      ) ,\
    HID_USAGE_MAX    ( 32                                     ) ,\
    HID_LOGICAL_MIN  ( 0                                      ) ,\
    HID_LOGICAL_MAX  ( 1                                      ) ,\
    HID_REPORT_SIZE  ( 1                                      ) ,\
    HID_REPORT_COUNT ( 32                                     ) ,\
    HID_INPUT        ( HID_DATA | HID_ARRAY | HID_ABSOLUTE    ) ,\
    /* 4 hats */ \
    HID_COLLECTION ( HID_COLLECTION_PHYSICAL ), \
        HID_LOGICAL_MIN  ( 0                                      ) ,\
        HID_PHYSICAL_MIN ( 0 ), \
        HID_UNIT ( 0x14 ), /* Eng Rot: Degree) */ \
        /* 0x55, 0x0f,  # UNIT_EXPONENT (-1) */ \
        HID_UNIT_EXPONENT ( 0x0f ), \
        HID_PHYSICAL_MAX_N ( 3599, 2 ), \
        HID_LOGICAL_MAX_N ( 3599, 2 ), \
        HID_USAGE_PAGE   ( HID_USAGE_PAGE_DESKTOP                 ) ,\
        HID_USAGE        ( HID_USAGE_DESKTOP_HAT_SWITCH           ) ,\
        HID_REPORT_SIZE  ( 16                                     ) ,\
        HID_REPORT_COUNT ( 1                                     ) ,\
        HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE | HID_NULL_STATE    ) ,\
        HID_USAGE        ( HID_USAGE_DESKTOP_HAT_SWITCH           ) ,\
        HID_REPORT_COUNT ( 1                                     ) ,\
        HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE | HID_NULL_STATE    ) ,\
        HID_USAGE        ( HID_USAGE_DESKTOP_HAT_SWITCH           ) ,\
        HID_REPORT_COUNT ( 1                                     ) ,\
        HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE | HID_NULL_STATE    ) ,\
        HID_USAGE        ( HID_USAGE_DESKTOP_HAT_SWITCH           ) ,\
        HID_REPORT_COUNT ( 1                                     ) ,\
        HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE | HID_NULL_STATE    ) ,\
    HID_COLLECTION_END, \
  HID_COLLECTION_END, \
  \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )        ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_MULTI_AXIS_CONTROLLER  )        ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )        ,\
      HID_COLLECTION ( HID_COLLECTION_PHYSICAL )        ,\
          HID_REPORT_ID       ( REPORT_ID_6DOF ) , \
          HID_LOGICAL_MIN_N   ( -500,   2 ),\
          HID_LOGICAL_MAX_N   (  500,    2 ),\
          HID_PHYSICAL_MIN_N ( -32768,  2 ),\
          HID_PHYSICAL_MAX_N (  32768,   2 ),\
          HID_USAGE       (  HID_USAGE_DESKTOP_X  ),\
          HID_USAGE       (  HID_USAGE_DESKTOP_Y ),\
          HID_USAGE       (  HID_USAGE_DESKTOP_Z ),\
          HID_REPORT_SIZE ( 16 ),\
          HID_REPORT_COUNT( 3 ),\
          HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE ),\
          HID_USAGE       (  HID_USAGE_DESKTOP_RX ),\
          HID_USAGE       (  HID_USAGE_DESKTOP_RY ),\
          HID_USAGE       (  HID_USAGE_DESKTOP_RZ ),\
          HID_USAGE       (  HID_USAGE_DESKTOP_VBRX ),\
          HID_USAGE       (  HID_USAGE_DESKTOP_VBRY ),\
          HID_USAGE       (  HID_USAGE_DESKTOP_VBRZ ),\
          HID_REPORT_SIZE ( 16 ),\
          HID_REPORT_COUNT( 6 ),\
          HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE ),\
      HID_COLLECTION_END, \
      /* Buttons */ \
      HID_COLLECTION ( HID_COLLECTION_LOGICAL )        ,\
          HID_REPORT_ID       ( REPORT_ID_BUTTONS ), \
          HID_USAGE_PAGE   ( HID_USAGE_PAGE_DESKTOP     )        ,\
          HID_USAGE_PAGE   ( HID_USAGE_PAGE_BUTTON  )        ,\
          HID_USAGE_MIN ( 1 ),\
          HID_USAGE_MAX ( 13 ),\
          HID_LOGICAL_MIN ( 0 ),\
          HID_LOGICAL_MAX ( 1 ),\
          HID_PHYSICAL_MIN ( 0 ),\
          HID_PHYSICAL_MAX ( 1 ),\
          HID_REPORT_SIZE ( 1 ),\
          HID_REPORT_COUNT( 32 ),\
          HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ),\
      HID_COLLECTION_END, \
      \
      /* Vendor */ \
      HID_COLLECTION ( 0x80 )  /* vendor */       ,\
          HID_USAGE_PAGE_N ( HID_USAGE_PAGE_VENDOR, 2     )        ,\
          \
          HID_REPORT_ID       ( REPORT_ID_RAW_GYRO ), \
          HID_USAGE      ( 0x1  )        ,\
          HID_LOGICAL_MIN ( 0x00 ),\
          HID_LOGICAL_MAX ( 0xff ),\
          HID_REPORT_SIZE ( 16 ),\
          HID_REPORT_COUNT( 6 ),\
          HID_INPUT      ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE  ),\
          \
          HID_REPORT_ID       ( REPORT_ID_MODE ), \
          HID_USAGE      ( 0x1  )        ,\
          HID_LOGICAL_MIN ( 0x00 ),\
          HID_LOGICAL_MAX ( 0xff ),\
          HID_REPORT_SIZE ( 8 ),\
          HID_REPORT_COUNT( 1 ),\
          HID_FEATURE      ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE  ),\
          \
          HID_REPORT_ID       ( REPORT_ID_R_SCALE ), \
          HID_USAGE      ( 0x2  )        ,\
          HID_LOGICAL_MIN ( 0 ),\
          HID_LOGICAL_MAX ( 20 ),\
          HID_PHYSICAL_MIN ( 0 ),\
          HID_PHYSICAL_MAX_N ( 2000, 2 ),\
          HID_UNIT ( 0), /* none */ \
          /* 0x55, 0x0e,  # UNIT_EXPONENT (-2) */ \
          HID_UNIT_EXPONENT ( 0x0e ), \
          HID_REPORT_SIZE ( 16 ),\
          HID_REPORT_COUNT( 1 ),\
          HID_FEATURE      ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE  ),\
           \
          HID_REPORT_ID       ( REPORT_ID_T_SCALE ), \
          HID_USAGE      ( 0x3  )        ,\
          HID_REPORT_COUNT( 1 ),\
          HID_FEATURE      ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE  ),\
          \
          HID_REPORT_ID       ( REPORT_ID_MIRROR_FEED ), \
          HID_USAGE      ( 0x4  )        ,\
          HID_REPORT_SIZE ( 8 ),\
          HID_REPORT_COUNT( 63 ),\
          HID_FEATURE      ( HID_DATA | HID_ARRAY | HID_ABSOLUTE  ),\
      HID_COLLECTION_END,\
      \
  HID_COLLECTION_END

/*
    0x15, 0x00,  # Logical Minimum (0)
    0x35, 0x00,  # Physical Minimum (0)
    0x65, 0x14,  # Unit (Eng Rot: Degree)

    # 0x26, 0x67, 0x01, # Logical Maximum (359)
    # 0x46, 0x67, 0x01, # Physical Maximum (359)

    0x55, 0x0f,  # UNIT_EXPONENT (-1)
    0x46, 0x0f, 0x0e,  # PHYSICAL_MAXIMUM (3599)
    0x26, 0x0f, 0x0e,  # LOGICAL_MAXIMUM (3599)

 */

typedef struct TU_ATTR_PACKED
{
    int16_t  x;
    int16_t  y;
    int16_t  z;
    int16_t  rx;
    int16_t  ry;
    int16_t  rz;
    int16_t  vrx;
    int16_t  vry;
    int16_t  vrz;
} hid_3dx_report_6dof_t;

typedef struct TU_ATTR_PACKED
{
    uint8_t  report_id;
    uint8_t  mode;

} hid_3dx_raw_feature_mode_t;

typedef struct TU_ATTR_PACKED
{
    uint8_t  report_id;
    uint16_t  scale;

} hid_3dx_raw_feature_scale_t;


typedef struct TU_ATTR_PACKED
{
    int16_t  ax;
    int16_t  ay;
    int16_t  az;
    int16_t  gx;
    int16_t  gy;
    int16_t  gz;

} hid_3dx_report_raw_gyro_t;


void send_motion(int16_t dx, int16_t dy, int16_t dz, int16_t d_enc);
void map_as_3Tdof(int16_t dx, int16_t dy, int16_t dz, hid_3dx_report_6dof_t *report);
void map_as_2T1Rdof(int16_t dx, int16_t dy, int16_t dz, hid_3dx_report_6dof_t *report);
void map_as_3Rdof_and_zoom(int16_t dx, int16_t dy, int16_t dz, int16_t d_enc, hid_3dx_report_6dof_t *report);
void map_as_2Rdof(int16_t dx, int16_t dy, int16_t dz, int16_t d_enc, hid_3dx_report_6dof_t *report);


bool send_3dx_report_6dof(const hid_3dx_report_6dof_t *report);
bool send_3dx_report_buttons(uint32_t buttons);
bool send_3dx_report_raw_gyro(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);


uint16_t get_report_callback (uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen);
void set_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize);



inline float clamp(const float v, const float v_min, const float v_max) {
    if (v < v_min) {
        return v_min;
    } else if (v > v_max) {
        return v_max;
    }
    return v;
}

inline float cubic_curve(const float x, const float a, const float range=1.) {
    if (a == 0) {
        return x;
    }
    const float xf = x / range;
    return clamp(a * (xf*xf*xf) + (1 - a) * xf, -1., 1.) * range;
}

extern uint8_t translation_mode;

extern float SENSOR_R_SCALE;
extern float SENSOR_T_SCALE;
extern float CUBIC_COEF;

#ifdef __cplusplus
}
#endif

#endif //NRF52_TEST_HID_3DX_H
