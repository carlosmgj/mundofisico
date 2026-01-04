// EJERCICIO EF11
// MSEEI-UMA

#ifndef MAIN_ACME31416DRIVER_H_
#define MAIN_ACME31416DRIVER_H_

#include "driver/i2c.h"
#include "esp_err.h"

//*****************************************************************************
//      DEFINICIONES
//*****************************************************************************

// CONSTANTES
#define ACME31416_BASE_ADDRESS         0x44

#define ACME31416_P1DIR_REG 0
#define ACME31416_P1OUT_REG 1
#define ACME31416_P1REN_REG 2
#define ACME31416_P1IN_REG 3
#define ACME31416_P2DIR_REG 4
#define ACME31416_P2OUT_REG 5
#define ACME31416_P2REN_REG 6
#define ACME31416_P2IN_REG 7

extern esp_err_t acme_init(i2c_port_t i2c_master_port);
extern esp_err_t acme_write_register(uint8_t registerdir, uint8_t registercontent);
extern esp_err_t acme_read_register(uint8_t registerdir, uint8_t *registercontent);

#endif /* MAIN_ACME31416DRIVER_H_ */
