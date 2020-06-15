//
// Created by user on 15-Jun-20.
//

#ifndef NRF52_TEST_HID_3DX_H
#define NRF52_TEST_HID_3DX_H

#include <Arduino.h>
#include <Adafruit_USBD_HID.h>

#define TUD_HID_REPORT_DESC_MULTIAXIS_CONTROLLER() \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )        ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_MULTI_AXIS_CONTROLLER  )        ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )        ,\
      HID_COLLECTION ( HID_COLLECTION_PHYSICAL )        ,\
          HID_REPORT_ID       (1 ), \
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
          HID_REPORT_SIZE ( 16                                      ),\
          HID_REPORT_COUNT( 3                            ),\
          HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE ),\
      HID_COLLECTION_END, \
      /* Output */ \
      HID_REPORT_ID       (0x80 ), \
      HID_USAGE_PAGE_N ( HID_USAGE_PAGE_VENDOR, 2     )        ,\
      HID_USAGE      ( 0x20  )        ,\
      HID_LOGICAL_MIN ( 0x00                                    ),\
      HID_LOGICAL_MAX ( 0xff                                    ),\
      HID_REPORT_SIZE ( 8                                       ),\
      HID_REPORT_COUNT( 1                             ),\
      HID_FEATURE      ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE  ),\
      \
  HID_COLLECTION_END \


typedef struct TU_ATTR_PACKED
{
    int16_t  x;
    int16_t  y;
    int16_t  z;
    int16_t  rx;
    int16_t  ry;
    int16_t  rz;

} hid_3dx_report_t;

bool send_3dx_report1(const int16_t cx, const int16_t cy, const int16_t cz);
bool send_3dx_report2(const int16_t cx, const int16_t cy, const int16_t cz);

const float SENSOR_R_SCALE = 1.5f;
const float SENSOR_T_SCALE = 1.5f;


#endif //NRF52_TEST_HID_3DX_H
