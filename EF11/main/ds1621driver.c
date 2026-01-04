// EJERCICIO EF11
// MSEEI-UMA

#include "ds1621driver.h"

//****************************************************************************
//      VARIABLES
//****************************************************************************

static unsigned char ds1621_slave_address = 0;
static i2c_port_t ds1621_i2c_port;

//****************************************************************************
//     FUNCIONES
//****************************************************************************

esp_err_t ds1621_i2c_master_init(uint8_t address, i2c_port_t i2c_master_port)
{


    ds1621_slave_address = DS1621_BASE_ADDRESS | (address & 0x7);
    ds1621_i2c_port = i2c_master_port;

    return 0;

}

esp_err_t ds1621_config(uint8_t config)
{
    unsigned char data_buffer[2];
    if(ds1621_slave_address == 0)
        return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    data_buffer[0] = DS1621_CMD_ACCESS_CONFIG;
    data_buffer[1] = config;
    i2c_master_write(cmd, data_buffer, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    return ret;
};

esp_err_t ds1621_read_temperature_low_resolution(float* temperature)
{
	uint8_t buffer[2];
    if(ds1621_slave_address == 0) return ESP_FAIL;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_READ_TEMP, true); // Comando 0xAA
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);

    // Conversión: MSB es la parte entera, el bit 7 del LSB es 0.5 grados
    *temperature = ((((int16_t)((int8_t)buffer[0])) << 1) | (buffer[1] >> 7)) / 2.0;

    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    return ret;
};

esp_err_t ds1621_write_TH(float temperature)
{
    uint8_t data_buffer[2];
    int16_t value;
   // Conversion de la temperatura a valores a escribir en el comando TH
    value= (int16_t)(temperature*2.0);
    data_buffer[0]=(value>>1)&0xFF; // 8 bits m�s significativos
    data_buffer[1]= (value & 0x01)<<7; // noveno bit

    if(ds1621_slave_address == 0)
        return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_ACCESS_TH, true);
    i2c_master_write(cmd, data_buffer, sizeof(data_buffer), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS/1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    return ret;
};

esp_err_t ds1621_write_TL(float temperature)
{
    uint8_t data_buffer[2];
    int16_t value;
   // Conversion de la temperatura a valores a escribir en el comando TH
    value= (int16_t)(temperature*2.0);
    data_buffer[0]=(value>>1)&0xFF; // 8 bits m�s significativos
    data_buffer[1]= (value & 0x01)<<7; // noveno bit

    if(ds1621_slave_address == 0)
        return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_ACCESS_TL, true);
    i2c_master_write(cmd, data_buffer, sizeof(data_buffer), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    return ret;
};


esp_err_t ds1621_read_TH(float* temperature)
{
    uint8_t buffer[2];
    if(ds1621_slave_address == 0)
        return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_ACCESS_TH, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    *temperature = ((((int16_t)((int8_t)buffer[0])) << 1) | (buffer[1] >> 7)) / 2.0;

    vTaskDelay((DS1621_BUS_FREE_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    return ret;
};

esp_err_t ds1621_read_counter(uint8_t* counter)
{
	if(ds1621_slave_address == 0) return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_READ_COUNTER, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, counter, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    return ret;
};

esp_err_t ds1621_read_slope(uint8_t* slope)
{
	if(ds1621_slave_address == 0) return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_READ_SLOPE, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, slope, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    return ret;
};

esp_err_t ds1621_start_conversion(void)
{
	if(ds1621_slave_address == 0) return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_START_CONVERT, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    return ret;
};

esp_err_t ds1621_stop_conversion(void)
{
	if(ds1621_slave_address == 0) return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS1621_CMD_STOP_CONVERT, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, (DS1621_WAIT_TIME_MS / 1000.0) * configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    return ret;
};

esp_err_t ds1621_read_temperature_high_resolution(float* temperature)
{
	float t_read;
    uint8_t count_remain, count_per_c;
    esp_err_t ret;

    // 1. Leer temperatura base
    ret = ds1621_read_temperature_low_resolution(&t_read);
    if (ret != ESP_OK) return ret;

    // 2. Leer Counter (Remain)
    ret = ds1621_read_counter(&count_remain);
    if (ret != ESP_OK) return ret;

    // 3. Leer Slope (Count Per C)
    ret = ds1621_read_slope(&count_per_c);
    if (ret != ESP_OK) return ret;

    // Fórmula del Datasheet: T = T_read - 0.25 + (Count_Per_C - Count_Remain) / Count_Per_C
    // Nota: Se trunca la parte fraccionaria de T_read (0.5) antes del cálculo
    float t_integer = (float)((int)t_read);
    *temperature = t_integer - 0.25 + (((float)count_per_c - (float)count_remain) / (float)count_per_c);

    return ESP_OK;
};
