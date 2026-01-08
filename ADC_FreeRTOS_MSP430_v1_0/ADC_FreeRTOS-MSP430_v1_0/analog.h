/*
 * analog.h
 *
 *  Created on: 30/01/2018
 *      Author: EGP
 */

#ifndef ANALOG_H_
#define ANALOG_H_

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"


/* msp430 includes */
#include "msp430.h"
#include "MSP430F5xx_6xx/driverlib.h"

typedef struct {
    uint16_t Chan0Ref;
    uint16_t Chan0Vcc;
    uint16_t BattRef;
    uint16_t BattVcc;
    uint16_t TempRef;
    uint16_t TempVcc;

} conversionData;

// CMGJ: Typedef para los voltajes, Se hace así porque desde la aplicación de QT el orden está establecido así y ya manda estos datos.
typedef enum {
    VREF_15 = 0,
    VREF_20,
    VREF_25
} VREFS_t;


void AnalogInit(void);
void AnalogStart(uint16_t period, uint8_t mode);
void AnalogConfigADC(uint8_t mode );
void AnalogStop(void);
inline BaseType_t AnalogRead(conversionData *ptrtodato, TickType_t ticks);

//CMGJ: Añadimos el parámetro de Vref para poder discernir qué referencia se está usando
int16_t AnalogTempCompensate(uint16_t lectura, VREFS_t v_ref);
uint16_t AnalogValueCompensate(uint16_t lectura, VREFS_t v_ref);

#endif /* ANALOG_H_ */
