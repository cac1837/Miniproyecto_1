/*
 * File:   Master_1.c
 * Author: Katherine Caceros
 *
 * Created on 19 de febrero de 2021, 08:25 AM
 */


#include <xc.h> // Llamamos a las librerias
#include <stdint.h>
#include <stdio.h>
#include <pic16f887.h>
#include "lcd.h"
#include "spi.h"

//------------------------------------------------------------------------------
//                       Declaracion de variables  
//------------------------------------------------------------------------------
//variables para las interrupciones
uint8_t prueba_1 = 0;
uint8_t prueba_2 = 0;
uint8_t prueba_3 = 0;
uint8_t slave_0 = 0;
uint8_t slave1_SPI = 0; //recibiran los datos de SPI
uint8_t slave2_SPI = 0;
uint8_t slave3_SPI = 0;
char slave_1[6];
char slave_2[6];
char slave_3[6];
char str[20]; //guarda caracteres de pantalla

//------------------------------------------------------------------------------
//                         Declaracion de las funciones
//------------------------------------------------------------------------------

void setup(void);
void init_UART(void);

//------------------------------------------------------------------------------
//                              FUNCIONES
//------------------------------------------------------------------------------

void Text_Uart(char *text) // Es para textos donde se envia el caracter al uart
{
  int i;
  for(i=0;text[i]!='\0';i++){
    TXREG = (text[i]);
  __delay_ms(5);}
}
void init_UART(void) {
    SPBRG = 25;
    BRGH = 1;
    SYNC = 0;
    SPEN = 1;
    TRISC7 = 1;
    TRISC6 = 0;
    TXSTAbits.TX9 = 0;
    CREN = 1;
    TXEN = 1;
}

void setup(void) { // Se declaran puerto como digitales
    ANSEL = 0x00;
    ANSELH = 0x00;

    TRISA = 0x00;
    TRISB = 0x00;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC5 = 0; //bits del SPI en master
    TRISD = 0x00;
    TRISE = 0x00;           // Entradas y salidas

    PORTA = 0x00;
    PORTB = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;

    lcd_init();
    lcd_cmd(0x0c);

    init_UART();

    spiInit(SPI_MASTER_OSC_DIV4, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE); //se configura SPI en el master
    PIE1bits.SSPIE = 1;
    PIR1bits.SSPIF = 0;
    return;
}
//------------------------------------------------------------------------------
//                          Interrupcion
//------------------------------------------------------------------------------
void __interrupt() isr(void) { // activamos la bandera y acumulamos el valor SPI

    if (PIR1bits.SSPIF == 1) {
        prueba_1 = SSPBUF;
        SSPIF = 0;
    }
}
//------------------------------------------------------------------------------
//                                     MAIN
//------------------------------------------------------------------------------
void main(void) {
    setup();

    while (1) {
        lcd_clear(); //limpiamos la pantalla

        if (slave_0 == 2) { // habilitamos los slaves 
            PORTAbits.RA0 = 0;
            PORTAbits.RA1 = 1;
            PORTAbits.RA2 = 1;
            SSPBUF = 0xFF; // enviamos los datos para iniciar el clock
            __delay_ms(10);
            slave1_SPI = SSPBUF; // datos del buffer
            PORTAbits.RA0 = 1;
            PORTAbits.RA1 = 1;
            PORTAbits.RA2 = 1;
            slave_0 = 0;
        }

        if (slave_0 == 1) {
            PORTAbits.RA0 = 1;
            PORTAbits.RA1 = 0;
            PORTAbits.RA2 = 1;
            SSPBUF = 0x0F;
            __delay_ms(10);
            slave2_SPI = SSPBUF;
            PORTAbits.RA0 = 1;
            PORTAbits.RA1 = 1;
            PORTAbits.RA2 = 1;
            slave_0++; //sumamos al contador 
        }

        if (slave_0 == 0) {
            PORTAbits.RA0 = 1;
            PORTAbits.RA1 = 1;
            PORTAbits.RA2 = 0;
            SSPBUF = 0xFC;
            __delay_ms(10);
            slave3_SPI = SSPBUF;
            PORTAbits.RA0 = 1;
            PORTAbits.RA1 = 1;
            PORTAbits.RA2 = 1;
            slave_0++; // sumamos al contador
        }

        lcd_write_string("POT  CONT  TEMP"); //enviamos el texto a mostrar en la lcd
        lcd_move_cursor(1, 0);
        sprintf(slave_1, "%d", slave1_SPI);
        lcd_write_string(slave_1);
        lcd_move_cursor(1, 6);
        sprintf(slave_2, "%d", slave2_SPI);
        lcd_write_string(slave_2);
        lcd_move_cursor(1, 11);
        sprintf(slave_3, "%d", slave3_SPI);
        lcd_write_string(slave_3);
  
        Text_Uart("POT  CONT  TEMP"); //escribimos la descripcion de las variables 
        TXREG = (0x0d);

        TXREG = (slave_1[0]);
        __delay_ms(5);
        TXREG = (slave_1[1]);
        __delay_ms(5);
        TXREG = (slave_1[2]);
        __delay_ms(5);
        Text_Uart("   ");

        TXREG = (slave_2[0]);
        __delay_ms(5);
        TXREG = (slave_2[1]);
        __delay_ms(5);
        TXREG = (slave_2[2]);
        __delay_ms(5);
        Text_Uart("    ");

        TXREG = (slave_3[0]);
        __delay_ms(5);
        TXREG = (slave_3[1]);
        __delay_ms(5);
        TXREG = (slave_3[2]);
        __delay_ms(5);
        TXREG = (' ');
        TXREG = (0x0d);
    }
}

