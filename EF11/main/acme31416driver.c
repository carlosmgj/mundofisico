// EJERCICIO EF11
// MSEEI-UMA

#include "acme31416driver.h"
//****************************************************************************
//      VARIABLES
//****************************************************************************


static i2c_port_t acme_i2c_port;

//****************************************************************************
//      DEFINICIÓN DE FUNCIONES
//****************************************************************************

esp_err_t acme_init(i2c_port_t i2c_master_port)
{

    acme_i2c_port = i2c_master_port;
    return 0;

}


esp_err_t acme_write_register(uint8_t registerdir, uint8_t registercontent)
{
    unsigned char data_buffer[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ACME31416_BASE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    data_buffer[0] = registerdir;
    data_buffer[1] = registercontent;
    i2c_master_write(cmd, data_buffer, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(acme_i2c_port, cmd, configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    return ret;
};


esp_err_t acme_read_register(uint8_t registerdir, uint8_t *registercontent)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ACME31416_BASE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, registerdir, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ACME31416_BASE_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, registercontent, 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(acme_i2c_port, cmd, configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(0.2*configTICK_RATE_HZ);
    return ret;
};
