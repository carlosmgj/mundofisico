// analog.c
// Created on: 30/01/2015
// Modified: 30/11/2020
// Author: JMCG, EGP
// DTE UMA


#ifndef ANALOG_C_
#define ANALOG_C_

#include <MSP430F5xx_6xx/driverlib.h>
#include<stdbool.h>
#include<stdint.h>


// UMA
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"


/* msp430 includes */
#include "msp430.h"
#include "analog.h"

static QueueHandle_t adcqueue; // UMA
static struct s_TLV_ADC_Cal_Data *adccal;
static struct s_TLV_REF_Cal_Data *refcal;



/**
 * Initializa y configura el ADC
 * para convertir muestrar  varios canales con distintas referencias
 * crea una cola de mensaje
 **/

void AnalogInit(void)
{
        uint8_t longitud;

        //lee los datos de calibracion de temperatura, ganancia, offset y referencia.
        TLV_getInfo(TLV_TAG_ADCCAL,
                    0,
                    &longitud,
                    (uint16_t **)&adccal
                    );

        TLV_getInfo(TLV_TAG_REFCAL,
                    0,
                    &longitud,
                    (uint16_t **)&refcal
                    );

        ADC12_A_configureMemoryParam param0 = {0};

        //Habilita el channel A0 del ADC
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
                                                   GPIO_PIN0
                                                   );

        // Inicializa  ADC12
        ADC12_A_init (ADC12_A_BASE,ADC12_A_SAMPLEHOLDSOURCE_SC, ADC12_A_CLOCKSOURCE_ADC12OSC, ADC12_A_CLOCKDIVIDER_1);

        //Enciende  ADC12
        ADC12_A_enable(ADC12_A_BASE);

        //Programa la conversión por disparo sotfware y MCS=1
        ADC12_A_setupSamplingTimer (ADC12_A_BASE , ADC12_A_CYCLEHOLD_1024_CYCLES, ADC12_A_CYCLEHOLD_1024_CYCLES, ADC12_A_MULTIPLESAMPLESENABLE);



        while(REF_BUSY == Ref_isRefGenBusy(REF_BASE));

        //Enciende el módulo de referencia de tensión
        //Activa la referencia interna de 2.0V (valor por defecto)
        //
        Ref_setReferenceVoltage(REF_BASE,REF_VREF2_0V);
        Ref_enableReferenceVoltage(REF_BASE);

        // Configuración de los registros de memoria asociados a la conversión
        //se define distintos canales y distintas referencias para cada canal (interna y Vcc)
        //Definición de la secuencia de muestro y conversión

        //Canal 0 con referencia interna
        param0.memoryBufferControlIndex = ADC12_A_MEMORY_0;
        param0.inputSourceSelect = ADC12_A_INPUT_A0;
        param0.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_INT;
        param0.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
        param0.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
        ADC12_A_configureMemory(ADC12_A_BASE,&param0);

        //Canal 0 con referencia Vcc
        param0.memoryBufferControlIndex = ADC12_A_MEMORY_1;
        param0.inputSourceSelect = ADC12_A_INPUT_A0;
        param0.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
        param0.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
        param0.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
        ADC12_A_configureMemory(ADC12_A_BASE,&param0);

        //Vcc/2 con referencia interna
        param0.memoryBufferControlIndex = ADC12_A_MEMORY_2;
        param0.inputSourceSelect = ADC12_A_INPUT_BATTERYMONITOR;
        param0.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_INT;
        param0.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
        param0.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
        ADC12_A_configureMemory(ADC12_A_BASE,&param0);

        //Vcc/2 con referencia vcc (inutil)
        param0.memoryBufferControlIndex = ADC12_A_MEMORY_3;
        param0.inputSourceSelect = ADC12_A_INPUT_BATTERYMONITOR;
        param0.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
        param0.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
        param0.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
        ADC12_A_configureMemory(ADC12_A_BASE,&param0);

        //Temperatura con referencia interna
        param0.memoryBufferControlIndex = ADC12_A_MEMORY_4;
        param0.inputSourceSelect = ADC12_A_INPUT_TEMPSENSOR;;
        param0.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_INT;
        param0.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
        param0.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
        ADC12_A_configureMemory(ADC12_A_BASE,&param0);

        //Temperatura con referencia vcc
        param0.memoryBufferControlIndex = ADC12_A_MEMORY_5;
        param0.inputSourceSelect = ADC12_A_INPUT_TEMPSENSOR;
        param0.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
        param0.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
        param0.endOfSequence = ADC12_A_ENDOFSEQUENCE;
        ADC12_A_configureMemory(ADC12_A_BASE,&param0);



        //Crea la cola en la que se van almacenando los datos convertidos
        adcqueue=xQueueCreate(8,sizeof(conversionData)); // UMA
        if (!adcqueue)
        {
            while(1);
        }



}


/**
* Configura los modos de disparo de conversión del ADC
* Gestiona el cambio de la fuente de disparo de la conversión
* Dos posibilidades: disparo SW de la conversión
* y el disparo mediante el TIMER B
**/

void AnalogConfigADC(uint8_t mode )
{

		switch (mode){

		//Disparo SW
		case 0:
		    ADC12_A_init (ADC12_A_BASE,ADC12_A_SAMPLEHOLDSOURCE_SC, ADC12_A_CLOCKSOURCE_ADC12OSC, ADC12_A_CLOCKDIVIDER_1);
		    ADC12_A_enable(ADC12_A_BASE);
		    ADC12_A_setupSamplingTimer (ADC12_A_BASE , ADC12_A_CYCLEHOLD_1024_CYCLES, ADC12_A_CYCLEHOLD_1024_CYCLES, ADC12_A_MULTIPLESAMPLESENABLE);
		    break;

		//Disparo Timer
		case 1:
		    ADC12_A_init (ADC12_A_BASE,ADC12_A_SAMPLEHOLDSOURCE_3, ADC12_A_CLOCKSOURCE_ADC12OSC, ADC12_A_CLOCKDIVIDER_1);
		    ADC12_A_enable(ADC12_A_BASE);

		    //CMGJ: Configuracion de ADC para disparo por timer cambiando los bits correspondientes:
		    ADC12_A_setupSamplingTimer(ADC12_A_BASE , ADC12_A_CYCLEHOLD_1024_CYCLES, ADC12_A_CYCLEHOLD_1024_CYCLES, ADC12_A_MULTIPLESAMPLESDISABLE);

		    break;
		}
}


/**
 * Arranca la conversión dependiendo de la fuente de disparo seleccionada.
 * Habilita la interrupción del último elemento de la secuencia
 **/

void AnalogStart(uint16_t period, uint8_t mode  )
{

    switch (mode){
    //Disparo SW
    case 0:{

        //Arranca la conversion
         ADC12_A_startConversion(ADC12_A_BASE,ADC12_A_MEMORY_0,ADC12_A_SEQOFCHANNELS);
        break;
    }
    //Disparo Timer
    case 1:{

        // Vamos a configurar el Timer en Modo UP, con el CCR correspondiente en modo comparación.

        Timer_B_initUpModeParam UpMode_Params = {0}; //CMGJ Inicializamos todos los campos numéricos a 0 y el bool a False.
        UpMode_Params.clockSource = TIMER_B_CLOCKSOURCE_ACLK; // CMGJ: Fuente de Timer B es ACLK (Por defecto 32,768Hz)
        // CMGJ: Por defecto el divisor del reloj está en 1. La siguiente línea no es necesaria.
        //ConfigTimerB.clockDivider = TIMER_B_CLOCKSOURCE_DIVIDER_1;
        UpMode_Params.timerPeriod = period; //Limited to 16 bits[uint16_t]

        Timer_B_initUpMode(TIMER_B0_BASE, &UpMode_Params);

        Timer_B_initCompareModeParam CompareMode_Params = {0};
        CompareMode_Params.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_1; // Vamos a usar el CCR1 como se indica en la guía
        CompareMode_Params.compareValue = period/2; //Señal PWM Cuadrada
        CompareMode_Params.compareOutputMode = TIMER_B_OUTPUTMODE_RESET_SET;
        // Modo Ascendente + Reset Set : Output is reset when the timer counts to the TAxCCRn value. It is set when the timer counts to the TAxCCR0
        // Con modo ascendente el Duty Cycle es =CCR0/(CCR0+1)

        Timer_B_initCompareMode(TIMER_B0_BASE, &CompareMode_Params);
        Timer_B_startCounter(TIMER_B0_BASE,TIMER_B_UP_MODE);

        //Arranca la conversion
        ADC12_A_startConversion(ADC12_A_BASE,ADC12_A_MEMORY_0,ADC12_A_REPEATED_SEQOFCHANNELS);
        break;
    }

    default:
        return;
    }

    //Habilita la interrupción del último elemento de la secuencia
     ADC12_A_clearInterrupt(ADC12_A_BASE,
                               ADC12IFG5);
     ADC12_A_enableInterrupt(ADC12_A_BASE,
                                ADC12IE5);




}


/**
 * Para la conversion (detiene el Timer B y deshabilita la interrupcion)
 */
void AnalogStop(void)
{
	Timer_B_stop(TIMER_B0_BASE);
	ADC12_A_disableConversions(ADC12_A_BASE,true);
	ADC12_A_disableInterrupt(ADC12_A_BASE,ADC12IE5);
}

/**
 * Espera de forma bloqueante la conversion y lee el dato
 * @param ptrtodato: dirección de memoria donde queremos guardar el dato (de tipo uint16_t). Se puede pasar una variable de dicho tipo por referencia
 * @param ticks: máximo numero de ticks del sistema a esperar (portMAX_DELAY para indefinido).
 **/
inline BaseType_t AnalogRead(conversionData *ptrtodato, TickType_t ticks)
{
	return xQueueReceive(adcqueue,ptrtodato,ticks);
}


/**
 * Calcula la temperatura en función de los coeficientes de calibración
 * del sensor de temperatura almacenados en la flash
 * @param lectura: El dato "en crudo" que hemos leido del conversor
 * @return temperatura en grados centígrados tras la caliabración
 * Calculos realizados para la referencia por defecto 2.0V
 */

int16_t AnalogTempCompensate(uint16_t lectura, VREFS_t v_ref )
{

	//Cálculos aritmética punto flotante. Carga elevada para la CPU del uC
//	float temporal;
//
//	temporal=(((float)(lectura&0xFFF)-(float)adccal->adc_ref20_30_temp)*(85.0-30.0));
//	temporal=temporal/((float)adccal->adc_ref20_85_temp-(float)adccal->adc_ref20_30_temp);
//	temporal+=30.0;
//	return ((int16_t)temporal); // Se devuelve un entero se podría cambiar la función para devolver float

    int32_t temp30, temp85 = 0;
    switch (v_ref)
    {
    case VREF_15: // CMGJ: Para referencia de 1,5V
        temp30= (int32_t)adccal->adc_ref15_30_temp;
        temp85= (int32_t)adccal->adc_ref15_85_temp;
        break;
    case VREF_20: // CMGJ: Para referencia de 2V
        temp30= (int32_t)adccal->adc_ref20_30_temp;
        temp85= (int32_t)adccal->adc_ref20_85_temp;
        break;
    case VREF_25: // CMGJ: Para referencia de 2,5V
        temp30= (int32_t)adccal->adc_ref25_30_temp;
        temp85= (int32_t)adccal->adc_ref25_85_temp;
        break;
    default:
        return 0;
    }

    //Cálculos en aritmética entera. Primero se realizan todas las operaciones de multiplicación y luego la de división
    int32_t temporal;
    temporal=(((int32_t)lectura&0xFFF)-temp30)*(85-30);
    temporal=temporal/(temp85-temp30);
    temporal+=30;
    return (0xFFFF&((uint16_t)temporal));

}


/**
 * Corrige la medida del ADC utilizando los coeficientes de calibración
 * grabados en flash para la referencia el ADC.
 * @param lectura: El dato "en crudo" que hemos leido del conversor
 * @return valor corregido (multiplicado por 16 --> los 12 bits mas significativos son la medida y los 4 últimos, la parte decimal).
 */
uint16_t AnalogValueCompensate(uint16_t lectura, VREFS_t v_ref )
{
	int32_t temporal;
	uint16_t factor_vref = 0;
	switch (v_ref)
	    {
	    case VREF_15: // CMGJ: Para referencia de 1,5V
	        factor_vref = refcal->ref_ref15;
	        break;
	    case VREF_20: // CMGJ: Para referencia de 2V
	        factor_vref = refcal->ref_ref20;
	        break;
	    case VREF_25: // CMGJ: Para referencia de 2,5V
	        factor_vref = refcal->ref_ref25;
	        break;
	    default:
	        return 0;
	    }


	// CMGJ: Aplicacion de la formila como se indica en la guia usando máscaras y desplazamientos
	temporal = ((int32_t)(lectura & 0x0FFF)) << 1;
	temporal = temporal * factor_vref;
	temporal >>= 16;
	temporal <<= 1;
	temporal = temporal * adccal->adc_gain_factor;
	temporal >>= 16;
	temporal += adccal->adc_offset;


    return (0xFFFF&((uint16_t)temporal));
}



/** Rutina de interrupción del ADC
 * Su ejecución se programa cuando se produce la
 * conversión del último elemento de la secuencia
 * */

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC12_VECTOR)))
#endif
void ADC12ISR(void)
{
	BaseType_t xHigherPriorityTaskWoken=pdFALSE; //debe ponerse a false

//    uint16_t RawTemperature;
//    uint16_t RawValueA0; //UMA:
	conversionData values;

    switch(__even_in_range(ADC12IV,34))
    {
    case  0: break;       //Vector  0:  No interrupt
    case  2: break;       //Vector  2:  ADC overflow
    case  4: break;       //Vector  4:  ADC timing overflow
    case  6: break;       //Vector  6:  ADC12IFG0
    case  8: break;		  //Vector  8:  ADC12IFG1
    case 10: break;       //Vector 10:  ADC12IFG2
    case 12: break;       //Vector 12:  ADC12IFG3
    case 14: break;       //Vector 14:  ADC12IFG4
    case 16:        //Vector 16:  ADC12IFG5
        values.Chan0Ref=ADC12_A_getResults(ADC12_A_BASE,ADC12_A_MEMORY_0);
        values.Chan0Vcc=ADC12_A_getResults(ADC12_A_BASE,ADC12_A_MEMORY_1);
        values.BattRef=ADC12_A_getResults(ADC12_A_BASE,ADC12_A_MEMORY_2);
        values.BattVcc=ADC12_A_getResults(ADC12_A_BASE,ADC12_A_MEMORY_3);
        values.TempRef=ADC12_A_getResults(ADC12_A_BASE,ADC12_A_MEMORY_4);
        values.TempVcc=ADC12_A_getResults(ADC12_A_BASE,ADC12_A_MEMORY_5);
        __no_operation(); //para poner punto de ruptura.
        xQueueSendFromISR(adcqueue,&values,&xHigherPriorityTaskWoken); //UMA
        break;
    case 18: break;       //Vector 18:  ADC12IFG6
    case 20: break;       //Vector 20:  ADC12IFG7
    case 22: break;       //Vector 22:  ADC12IFG8
    case 24: break;       //Vector 24:  ADC12IFG9
    case 26: break;       //Vector 26:  ADC12IFG10
    case 28: break;       //Vector 28:  ADC12IFG11
    case 30: break;       //Vector 30:  ADC12IFG12
    case 32: break;       //Vector 32:  ADC12IFG13
    case 34: break;       //Vector 34:  ADC12IFG14
    default: break;
    }

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

#endif /* ANALOG_C_ */
