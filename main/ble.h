#ifndef BLE_H_
#define BLE_H_

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stddef.h>

void ble_init(QueueHandle_t connection_queue);

void ble_set_int_humidity_temperature(float humidity, float temperature);

void ble_set_int_co2_tvoc(uint16_t co2, uint16_t tvoc);

void ble_set_ext(float humidity, float temperature, float pressure, uint8_t battery);

void ble_remove_paired_device();

void ble_enable_pairing(uint32_t passkey);

void ble_disable_pairing();

#endif /* BLE_H_ */
