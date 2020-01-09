set(COMPONENT_SRCS "main.c"
                   "ble.c"
                   "bsec_integration.c"
                   "display.c"
                   "u8g2_esp32_hal.c"
                   "i2c.c")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()