#ifndef CCS_H_
#define CCS_H_

typedef struct
{
    uint16_t co2;
    uint16_t tvoc;
} ccs_data_t;

void ccs_init();

void ccs_set_env_data(float humidity, float temperature);

ccs_data_t ccs_read();

#endif /* CCS_H_ */
