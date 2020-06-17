# Install python3 HID package https://pypi.org/project/hid/
import struct

import hid

VID = 0x046D
PID = 0xC62b

REPORT_ID_MODE = 0x80
REPORT_ID_R_SCALE = 0x81
REPORT_ID_T_SCALE = 0x82

MODE_ROT_3DOF = 0
MODE_TRANS2_ROT1 = 1


print("Openning HID device with VID = 0x%X and PID = 0x%X" % (VID, PID))

for dict in hid.enumerate(VID, PID):
    print(dict)
    dev = hid.Device(dict['vendor_id'], dict['product_id'])
    if dev:
        # r = struct.pack("<BH", REPORT_ID_R_SCALE, int(0.5*100))
        # dev.send_feature_report(r)

        r = struct.pack("<BB", REPORT_ID_MODE, 1)
        dev.send_feature_report(r)


        d = dev.get_feature_report(REPORT_ID_R_SCALE, 3)
        res = struct.unpack("<BH", d)

        print(res[1]/100.)

        d = dev.get_feature_report(REPORT_ID_T_SCALE, 3)
        res = struct.unpack("<BH", d)

        print(res[1]/100.)
