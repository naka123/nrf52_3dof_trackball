Import("env")

board_config = env.BoardConfig()

board_config.update("build.hwids", [
    ["0x046D",  "0xC62b" ]
])

board_config.update("vendor", "Naka")
board_config.update("build.usb_product", "SpaceMouse PRO")
