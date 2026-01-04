
// MSP430F5529
// -----------------
// | |
// | (40)P3.3|-------/CS---------> #CS MCP4261
// | (37)P3.0|-------SIMO---------> SDI MCP4261
// | (38)P3.1|<------SOMI--------- SDO MCP4261
// | (39)P3.2|-------UCLK---------> SDK MCP4261
// | |
//
//Set /CS Inactive State (“1”) to start a command /CS signal
//must transition from inactive state to active state (“0”)

#include "driverlib.h"

uint8_t returnValue = 0x00;
void main(void){
    WDT_A_hold(WDT_A_BASE);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN3);
    //P3.0,1,2 option select
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
    //Initialize Master
    USCI_B_SPI_initMasterParam param = {0};
    param.selectClockSource = USCI_B_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = UCS_getSMCLK();
    param.desiredSpiClock = 400000;
    param.msbFirst = USCI_B_SPI_MSB_FIRST;

    //Para el modo 0,0:
    param.clockPhase = USCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    param.clockPolarity = USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    //Para el modo 1,1:
    //param.clockPhase = USCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    //param.clockPolarity = USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
    returnValue = USCI_B_SPI_initMaster(USCI_B0_BASE, &param);
    if (STATUS_FAIL == returnValue){
        return;}

   /*
    * CUESTION 2:
    *
    * Los dispositivos como el MCP4261 solo implementan los modos 0,0 y 1,1 porque en ambos el dato se captura en el primer flanco del reloj.
    *
    * Esto simplifica el diseño del hardware interno del esclavo, ya que no necesita circuitería compleja para gestionar diferentes retardos de fase; simplemente reacciona al primer cambio que detecta en la línea de reloj tras la activación de CS.
    */
}
