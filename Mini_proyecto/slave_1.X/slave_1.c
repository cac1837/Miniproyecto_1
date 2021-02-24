/*
 * File:   slave_1.c
 * Author: Katherine Caceros
 *
 * Created on 21 de febrero de 2021, 09:00 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#define _XTAL_FREQ   4000000


// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "spi.h"
//------------------------------------------------------------------------------
//                       Declaracion de variables  
//------------------------------------------------------------------------------

uint8_t data_ADC = 0;
uint8_t prueba = 0;

//------------------------------------------------------------------------------
//                         Declaracion de las funciones
//------------------------------------------------------------------------------
void setup(void);
void config_adc(void);
void read_ADC(void);

//------------------------------------------------------------------------------
//                          Interrupcion
//------------------------------------------------------------------------------
void __interrupt() isr(void) { 

    if (PIR1bits.ADIF) { //guarda datos ADC
        PIR1bits.ADIF = 0;
        data_ADC = ADRESH;
    }

    if (SSPIF == 1) { //recibe datos SPI
        prueba = SSPBUF;
        SSPIF = 0;
    }

}
//------------------------------------------------------------------------------
//                              FUNCIONES
//------------------------------------------------------------------------------
void setup(void) { //habilitamos puertos analogicos
    ANSEL = 0x01;
    ANSELH = 0x00;

    TRISA = 0x21; // entradas para S y el potenciometro
    TRISB = 0x00;
    PORTA = 0x00;
    PORTB = 0x00;

    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC5 = 0;

    config_adc();

    spiInit(SPI_SLAVE_SS_EN, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE); // configuramos el SPI en slave

    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    PIE1bits.ADIE = 1;
    PIR1bits.ADIF = 0;

    PIE1bits.SSPIE = 1;
    PIR1bits.SSPIF = 0;
    return;
}

//------------------------------------------------------------------------------
//                                     MAIN
//------------------------------------------------------------------------------
void main(void) {
    setup();

    while (1) {
        read_ADC(); // lee el ADC
        SSPBUF = data_ADC;
        PORTB = data_ADC; // muestra en el puerto B
    }
}

void config_adc(void) {
    ADCON0 = 0x41;
    ADCON1bits.ADFM = 0;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    return;
}

void read_ADC(void) { //lectura del ADC
    if (ADCON0bits.GO_DONE == 0) {
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        ADCON0bits.GO_DONE = 1; // termina y enciende bandera
    }
}

