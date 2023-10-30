//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// Copyright (c) 2012-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.2.0.295 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/adc.h"

#define XTAL 16000000       // Se coloca el reloj a 16MHz

//Variables
int bandera = 0;
uint32_t i;
int pushbtn;

//Prototipos
void setup(void);
void delay1ms(void);
void delay(uint32_t msec);

//Main
int main(void){
    setup();

//Loop
    while(1){
        pushbtn = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);     // Lee push
        if (pushbtn == 0){                                      // Si el push se encuentra presionado y el ciclo se ha completado
            if (bandera == 1){
                bandera = 0;                                    // Vuelve a emepzar
                }
        }
        if (pushbtn == 0 && bandera == 0){                              // Si se presiona el push y es posible comenzar
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);      // Se enciende el led verde
            delay(3000); //delay de 3s
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);               // Se apaga el led verde
            delay(200); //delay de 200ms                                // Parpadea el led verde
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
            delay(200); //delay de 200ms
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
            delay(200); //delay de 200ms
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
            delay(200); //delay de 200ms
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
            delay(200); //delay de 200ms
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
            delay(200); //delay de 200ms
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
            delay(1000); //delay de 1s

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_1, GPIO_PIN_3 | GPIO_PIN_1);    //Encender led amariila
            delay(3000); //delay de 3s
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_1, 0);                          //Apagar led amarilla
            delay(1000); // delay de 1s

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);                              //Encender led rojo
            delay(3000);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);                                       //Apagar led rojo
            bandera = 1;                                                                        //Se termin  un ciclo
        }
    }
}

void setup(void){
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);     // Se coloca el eeloj a 40MHz
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                                                // Se habilita el puerto F
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));                                         // Se espera a que se inicialice
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);                                          // Se configura push1 como entrada
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);               // Se configuran los LEDS como salidas
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);    // Se le coloca pull-up al push1
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);                     // Se inicia la secuencia con todos los leds apgados hasta presioar el botón asignado
}

void delay(uint32_t ms){
    for (i = 0; i < ms; i++){       // Se ejecuta "n" número de veces el argumento descrito
        delay1ms();
    }
}

void delay1ms(void){
    SysTickDisable();               // Se deshabilita el SysTick
    SysTickPeriodSet(40000-1);
    SysTickEnable();                // Se inicia el set del periodo

    while((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0);
}
