#ifndef BLE_H_
#define BLE_H_

#include <stddef.h>

void ble_init();

void ble_start_advertising();

void ble_stop_advetising();

void ble_set_int_humidity_temperature(float humidity, float temperature);

void ble_set_int_co2_tvoc(uint16_t co2, uint16_t tvoc);

void ble_set_ext_humidity_temperature_pressure(float humidity, float temperature, float pressure);

bool ble_has_paired_device();

void ble_remove_paired_device();

#endif /* BLE_H_ */