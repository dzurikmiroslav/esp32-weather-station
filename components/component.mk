COMPONENT_SRCDIRS := bsec/src/bme680 u8g2/csrc
COMPONENT_ADD_INCLUDEDIRS := bsec/src/bme680 bsec/src/inc u8g2/csrc
COMPONENT_ADD_LDFLAGS += $(COMPONENT_PATH)/bsec/src/esp32/libalgobsec.a