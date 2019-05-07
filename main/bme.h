#ifndef BME_H_
#define BME_H_

typedef struct
{
    int16_t temperature; // 0.01 °C
    uint16_t humidity;   // 0.01 %
} bme_data_t;

void bme_init();

bme_data_t bme_read();

#endif /* BME_H_ */
