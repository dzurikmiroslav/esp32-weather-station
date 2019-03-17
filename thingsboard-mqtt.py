#! /usr/bin/python3
# pip3 install bluepy paho-mqtt

import struct
import json
import paho.mqtt.client as mqtt
from bluepy.btle import Peripheral, UUID, DefaultDelegate

dev_adr = 'XX:XX:XX:XX:XX:XX'
thingsboard_host = 'demo.thingsboard.io'
access_token = 'xCxHxeXexKixBxreXexkI'

hum_uuid = UUID(0x2A6F)
temp_uuid = UUID(0x2A6E)
press_uuid = UUID(0x2A6D)
co2_uuid = UUID(0xFF01)
tvoc_uuid = UUID(0xFF02)


class NotifyDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data):
        if cHandle in char_handle:
            char_handle[cHandle](data)


def send_telemetry(name, value):
    client.publish('v1/devices/me/telemetry', json.dumps({name: value}), 1)


peripheral = Peripheral(dev_adr, "public")

hum_handle = peripheral.getCharacteristics(uuid=hum_uuid)[0].getHandle()
temp_handle = peripheral.getCharacteristics(uuid=temp_uuid)[0].getHandle()
co2_handle = peripheral.getCharacteristics(uuid=co2_uuid)[0].getHandle()
tvoc_handle = peripheral.getCharacteristics(uuid=tvoc_uuid)[0].getHandle()
out_hum_handle = peripheral.getCharacteristics(uuid=hum_uuid)[1].getHandle()
out_temp_handle = peripheral.getCharacteristics(uuid=temp_uuid)[1].getHandle()
out_press_handle = peripheral.getCharacteristics(uuid=press_uuid)[0].getHandle()

char_handle = {
    hum_handle: lambda data: send_telemetry("humidity", struct.unpack('H', data)[0] / 100.0),
    temp_handle: lambda data: send_telemetry("temperature", struct.unpack('h', data)[0] / 100.0),
    co2_handle: lambda data: send_telemetry("co2", struct.unpack('H', data)[0]),
    tvoc_handle: lambda data: send_telemetry("tvoc", struct.unpack('H', data)[0]),
    out_hum_handle: lambda data: send_telemetry("out_humidity", struct.unpack('H', data)[0] / 100.0),
    out_temp_handle: lambda data: send_telemetry("out_temperature", struct.unpack('h', data)[0] / 100.0),
    out_press_handle: lambda data: send_telemetry("out_pressure", struct.unpack('I', data)[0])
}

peripheral.setDelegate(NotifyDelegate())

try:
    client = mqtt.Client()
    client.username_pw_set(access_token)
    client.connect(thingsboard_host, 1883, 60)
    client.loop_start()
    print("Start sending to", thingsboard_host, "with token", access_token)
    while True:
        peripheral.waitForNotifications(1.0)

except KeyboardInterrupt:
    pass

finally:
    client.loop_stop()
    peripheral.disconnect()
