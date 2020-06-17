//
// Created by user on 15-Jun-20.
//

#pragma once

#ifndef NRF52_TEST_HID_3DX_H
#define NRF52_TEST_HID_3DX_H

#include <Arduino.h>
#include <Adafruit_USBD_HID.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

enum {
     REPORT_ID_6DOF = 1,
     REPORT_ID_BUTTONS = 3,
     REPORT_ID_MODE = 0x80,
     REPORT_ID_R_SCALE = 0x81,
     REPORT_ID_T_SCALE = 0x82,
};

enum {
    MODE_ROT_3DOF = 0,
    MODE_TRANS2_ROT1 = 1,
};

#define TUD_HID_REPORT_DESC_MULTIAXIS_CONTROLLER() \
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
          HID_REPORT_SIZE ( 16 ),\
          HID_REPORT_COUNT( 3 ),\
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
      /* Features */ \
      HID_PUSH, \
      HID_COLLECTION ( 0x80 )  /* vendor */       ,\
          HID_USAGE_PAGE_N ( HID_USAGE_PAGE_VENDOR, 2     )        ,\
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
      HID_COLLECTION_END,\
      HID_POP, \
      \
  HID_COLLECTION_END \


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


void map_as_2T1Rdof(int16_t cx, int16_t cy, int16_t cz, hid_3dx_report_6dof_t *report);
void map_as_3Rdof_and_zoom(int16_t cx, int16_t cy, int16_t cz, int16_t zoom, hid_3dx_report_6dof_t *report);


bool send_3dx_report_6dof(const hid_3dx_report_6dof_t *report);
bool send_3dx_report_buttons(uint32_t buttons);


uint16_t get_report_callback (uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen);
void set_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize);


extern uint8_t translation_mode;

extern float SENSOR_R_SCALE;
extern float SENSOR_T_SCALE;

#ifdef __cplusplus
}
#endif

#endif //NRF52_TEST_HID_3DX_H
