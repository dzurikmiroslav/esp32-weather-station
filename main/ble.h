#ifndef BLE_H_
#define BLE_H_

#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void ble_init(QueueHandle_t connection_queue);

void ble_set_telemetry(float humidity, float temperature, float pressure, float iaq, float co2, float voc);

void ble_set_out_telemetry(float humidity, float temperature, uint8_t battery);

void ble_remove_paired_device();

void ble_enable_pairing(uint32_t passkey);

void ble_disable_pairing();

#endif /* BLE_H_ */
