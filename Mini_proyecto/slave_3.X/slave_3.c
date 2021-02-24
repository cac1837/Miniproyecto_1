/*
 * File:   slave_3.c
 * Author: Katherine Caceros
 *
 * Created on 21 de febrero de 2021, 09:50 PM
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


uint8_t prueba = 0;
uint8_t temp = 0;


void setup(void);
void config_ADC(void);
void read_ADC(void);
void semaforo(void);

void __interrupt() isr(void) {

    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0;
        temp = ADRESL;
        temp = temp >> 1;
    }
    
    if (SSPIF == 1) {
        prueba = SSPBUF;
        SSPIF = 0;
    }
}

void setup(void) {
    ANSEL = 0x01;
    ANSELH = 0x00;

    TRISA = 0x21;
    TRISB = 0x00;
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC5 = 0;
    TRISE = 0x00;

    PORTA = 0x00;
    PORTB = 0x00;
    PORTE = 0x00;

    config_ADC();
    spiInit(SPI_SLAVE_SS_EN, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);
    
    INTCONbits.GIE = 1; //Interrupciones globales
    INTCONbits.PEIE = 1;
    PIE1bits.ADIE = 1;
    PIR1bits.ADIF = 0; //Interrupciones ADC
    
    return;
}

void main(void) {
    setup();
    
    while (1) {
        read_ADC();
        SSPBUF = temp;
        semaforo();
        PORTB = temp;
        
    }
}

void config_ADC(void) {
    ADCON0 = 0b01000001; //inicialmente se configuro el canal 0
    ADCON1 = 0b10001000;
    //Datos justificados a la izquierda
    ADCON1bits.ADFM = 1;
    //Valores VCC y GND de referencia
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0; //Recordatorio esperar 10us antes de iniciar conversions
    return;
}

void read_ADC(void) {
    if (ADCON0bits.GO_DONE == 0) {
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        ADCON0bits.GO_DONE = 1; // iniciar cconversion
    }
}

void semaforo(void){
    if (temp < 25)
    {
        PORTEbits.RE2 = 1;
        PORTEbits.RE1 = 0;
        PORTEbits.RE0 = 0;
    }
    else if ((temp > 25) && (temp < 36))
    {
        PORTEbits.RE2 = 0;
        PORTEbits.RE1 = 1;
        PORTEbits.RE0 = 0;
    }
    else if (temp > 36)
    {
        PORTEbits.RE2 = 0;
        PORTEbits.RE1 = 0;
        PORTEbits.RE0 = 1;
    }
    else
    {
        return;
    }
}