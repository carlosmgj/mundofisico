/*
 * miscomandos.c
 *
 *  Created on: 9 dic. 2020
 *      Author: jcgar
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "esp_log.h"
#include "esp_console.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_spi_flash.h"
#include "esp_flash.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "miscomandos.h"
#include "sdkconfig.h"
#include "driver/adc.h"
#include "gpio_leds.h"

#include "acme31416driver.h"
#include "ds1621driver.h"

static int Cmd_led(int argc, char **argv)
{
	if (argc == 1)
	{
		//Si los parametros no son suficientes, muestro la ayuda
		printf(" LED [on|off]\r\n");
	}
	else
	{
		/* chequeo el parametro */
		if (0==strncmp( argv[1], "on",2))
		{
			printf("Enciendo el LED\r\n");
			gpio_set_level(BLINK_GPIO_1, 1);
		}
		else if (0==strncmp( argv[1], "off",3))
		{
			printf("Apago el LED\r\n");
			gpio_set_level(BLINK_GPIO_1, 0);
		}
		else
		{
			//Si el parametro no es correcto, muestro la ayuda
			printf(" LED [on|off]\r\n");
		}

	}
    return 0;

}

static void register_Cmd_led(void)
{
    const esp_console_cmd_t cmd = {
        .command = "led",
        .help = "Enciende y apaga el led rojo",
        .hint = NULL,
        .func = &Cmd_led,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static int Cmd_blink(int argc, char **argv)
{
	uint16_t veces;
	float frecuencia;

	if (argc != 4)
	{
		//Si los parámetros no son suficientes, se muestra la ayuda
		printf(" blink  [red|green|blue] [veces] [frecuencia]\r\n");
	}
	else
	{
		veces=strtoul(argv[2],NULL,10);
		frecuencia=strtoul(argv[3],NULL,10);

		//se comprueban los valores de los parámetros aquí falta el led
		if ((veces<=0)||(frecuencia<=0))
		{
			//Si los parámetros no son correctos, se muestra la ayuda
			printf("  blink  [red|green|blue] [veces] [frecuencia]\r\n");
		}
		else
		{
			//Hacer algo con los parámetros, ahora mismo lo que hago es imprimir....
			printf(" %d %f \r\n",(int)veces,frecuencia);
		}

	}

	return 0;
}

static void register_Cmd_blink(void)
{
    const esp_console_cmd_t cmd = {
        .command = "blink",
        .help = "Hace parpadear el LED x",
        .hint = NULL,
        .func = &Cmd_blink,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static int Cmd_adcRead(int argc, char **argv)
{
	int muestra;
	float voltaje;
	muestra=adc1_get_raw(ADC_CHANNEL_6); //Toma la muestra
	voltaje=(float)muestra*3.6/4096.0;	//Pasa a voltios (muestra de 12 bits con rango de 3,6V)
	printf (" ADC RAW: %d Volt: %f \r\n",muestra,voltaje);

	return 0;
}

static void register_Cmd_adcRead(void)
{
    const esp_console_cmd_t cmd = {
        .command = "adcRead",
        .help = "Lee valores del ADC",
        .hint = NULL,
        .func = &Cmd_adcRead,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

// EJERCICIO EF11
// MSEEI-UMA
//Comando que lee del sensor EMULADO ACME31416
static int Cmd_acme(int argc, char **argv)
{
	if (argc < 3)
	{
		//Si los par�metros no son suficientes, muestro la ayuda
		printf(" acme [read|write] <reg> <val> \r\n");
	}
	else
	{
		/* chequeo el parametro */
		if (0==strncmp( argv[1], "read",4))
		{
			uint8_t registro=strtoul(argv[2],NULL,16);
			uint8_t valor=0;
			esp_err_t ret =acme_read_register(registro,&valor);
			if (ret!=0) printf ("Error: %x ",ret);
			printf(" %x \r\n",(unsigned int)valor);


		}
		else if (0==strncmp( argv[1], "write",5))
		{
			if (argc<4)
			{
				printf(" acme [write] <reg> <val> \r\n");
				return 0;
			}
			uint8_t registro=strtoul(argv[2],NULL,16);
			uint8_t valor=strtoul(argv[3],NULL,16);
			esp_err_t ret = acme_write_register(registro,valor);
			if (ret!=0) printf ("Error: %x \r\n",ret);
		}
		else
		{
			//Si el par�metro no es correcto, muestro la ayuda
			printf(" acme [read|write] <reg> <val> \r\n");
		}

	}
    return 0;

}

static void register_Cmd_acme(void)
{
    const esp_console_cmd_t cmd = {
        .command = "acme",
        .help = "Lecturas o escrituras del dispositivo ACME31416",
        .hint = NULL,
        .func = &Cmd_acme,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}




//Comando que lee del sensor de temperatura DS1621
static int Cmd_temp_DS1621(int argc, char **argv)
{
	float grados, grados2;

	esp_err_t ret = ds1621_read_temperature_high_resolution(&grados);
	ds1621_write_TH(25.5);
	ds1621_write_TL(23.5);
	ds1621_read_TH (&grados2);

	printf("temperatura DS1621: %f  %f\r\n",grados, grados2);


    return 0;

}

static void register_Cmd_temp_DS1621(void)
{
    const esp_console_cmd_t cmd = {
        .command = "temp_DS1621",
        .help = "Lee la temperatura DS1621",
        .hint = NULL,
        .func = &Cmd_temp_DS1621,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

void init_MisComandos(void)
{
	register_Cmd_led();
	//Descomentar
	//register_Cmd_blink();
	//register_Cmd_adcRead();
	register_Cmd_acme();
	register_Cmd_temp_DS1621();
}
