 /*
 * File:   main.c
 * PROGRAMMER: Mark Watson; DATE: May 11, 2014
 *
 * Program to bit-bang data from ADNS-2610 Optical 
 * Mouse Sensor and send to PC over RS232
 * 
 * Micocontroller: PIC 16F887
 * uses MPLAB x and XC8 compiler
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <xc.h> 

/*******************************************************************/
/**********************PREPROCESSOR COMMANDS************************/
/*******************************************************************/
#pragma config LVP = OFF                //disable low voltage programming
#pragma config FCMEN = OFF              //disable fail safe clock monitor
#pragma config IESO = OFF               //disable int/ext oscillator switchover
#pragma config BOREN = OFF              //disable brown out reset
#pragma config PWRTE = ON               //enable power up timer
#pragma config WDTE = OFF               //disable watchdog timer
//#pragma config FOSC = HS              //if using a HS external oscillator
#pragma config FOSC = INTRC_NOCLKOUT    //if using the internal oscillator

/*******************************************************************/
/******************************MACROS*******************************/
/*******************************************************************/
#define _XTAL_FREQ      8000000        // set internal oscillator to 8MHz
#define CLOCK           PORTAbits.RA1  // CLOCK is on pin 3 [RA1]
#define SDIO            PORTBbits.RB5  // SDIO is on pin 38 [RB5]
#define CLOCK_LINE      TRISA          // TRISA and TRISB set data direction
#define DATA_LINE       TRISB

/*******************************************************************/
/*************************GLOBAL VARIABLES**************************/
/*******************************************************************/
// none

/*******************************************************************/
/************************Function Prototypes************************/
/*******************************************************************/
void init(void);
void EUSART_init(void);
void WriteAndRead(int8_t address, int8_t *temp);
void EUSART_write(unsigned char c);
void HexToDec(int8_t temp, int *flag, int16_t *DY);
void DecToASCII(int16_t DY);


/*******************************************************************/
/*******************************MAIN********************************/
/*******************************************************************/
void main(void){

    // address to be sent. [address of dy = 0x02. address of dX = 0x03]
    int8_t addressDY = 0x03;

    // zero DY and temp [DY is the total displacement in Y direction]
    int16_t DY = 0x0000;        // [temp stores the data read from the sensor
    int8_t temp = 0x00;         // to be added to DY in the HexToDec() function]
                                // DY is a signed number. temp is binary

    // declare counter and flag [counter sends DY to PC every once in a while, 
    int counter = 0;            // flag indicates if DY is negative]
    int flag = 0;

    // call initialization functions
    init();
    EUSART_init();

    while(1){    
        // send address, read data, and increment DY and counter
        WriteAndRead(addressDY, &temp);
        counter++;

        // send current DY value over serial com
        if(counter == 0x20){
            HexToDec(temp, &flag, &DY);
            DecToASCII(DY);
            counter = 0;
        }
    }
    
return;
}


/*******************************************************************/
/****************************FUNCTIONS*******************************/
/********************************************************************/
// initialize port/pins
void init(void){

    // configure internal oscillator
    OSCCON = 0x75;                      // sets internal oscillator to 8MHz 
                                        // by setting [ OSCON = 0b01110101]
                                        // IRCF2 = 1, IRCF1 = 1, and IRCF0 = 1.
    // configure pins
    CLOCK_LINE = 0x00;                  // configue ports A and B as outputs
    DATA_LINE = 0x00;
    ANSEL = 0x00;                       // diable Analog input pins
    ANSELH = 0x00;                      // make RB5 a digital I/O

    __delay_ms(100);

    // toggle CLOCK to reset??
    CLOCK = 1;                          // set CLOCK high
    __delay_us(5);
    CLOCK ^= 1;                         // CLOCK low
    __delay_us(1);
    CLOCK ^= 1;                         // CLOCK high
    SDIO = 1;                           // set data line high
    __delay_us(1500);                   // wait for tSPTT (o.9 sec min) for the
                                        // OptiMouse serial transaction timer to 
                                        // time out
return;
}

// initialize EUSART module in PIC to send data to MAX232 for debugging
void EUSART_init(void) {

    // asynchronous EUSART Set-up:
    TXSTAbits.TXEN = 1;                 // enable transmitter
    TXSTAbits.BRGH = 1;                 // high baud rate mode
    TXSTAbits.SYNC = 0;                 // enable asynchronous mode
    INTCONbits.GIE = 0;                 // enable interrupts
    RCSTAbits.SPEN = 1;                 // enable USART

    // configure I/O pins
    TRISCbits.TRISC6 = 1;               // TX pin is output

    SPBRG = 12;                         // set baud rate to 38400 baud 
                                        // baud rate = (8MHz/(16*baudrate))-1
return;
}

// writes a byte to the I2C bus beginning with the MSB [most significant bit]
void WriteAndRead(int8_t address, int8_t *temp){

    int i;                              // loop counter
    *temp = 0x00;                       // temp holds current read data

    // write address
    DATA_LINE &= ~(1 << 5);             // clear RB5 to configure as output

    for (i = 7; i >= 0; i--){           // loop through bits [starts with MSB]
        CLOCK = 0;
        SDIO = (1  & (address >> i));   // output bit
        CLOCK ^= 1;                     // toggle CLOCK high
        __delay_us(22);                 // delay to make clock pulse symmetrical
    }

    DATA_LINE |= (1 << 5);              // set RB5 as input [goes high-z]
    __delay_us(120);                    // tHOLD = 100us min

    // read data
    for (i = 7; i >= 0; i--){           // loop through each bit
        CLOCK = 0;
        __delay_us(22);                 // delay to make clock pulse symmetrical
        CLOCK ^= 1;                     // toggle CLOCK
        *temp |= (SDIO << i);           // shift left for next bit
    }

    // toggle CLOCK so sensor can release data line
    CLOCK ^= 1;                         // CLOCK low
    __delay_us(22);                     // delay to make clock pulse symmetrical
    DATA_LINE &= ~(1 << 5);             // configure data line as an output
    SDIO = 1;                           // set data line high
    CLOCK ^= 1;                         // CLOCK high
   
    __delay_us(120);
    
return;
}

// convert input [in Hex] to Decimal
void HexToDec(int8_t temp, int *flag, int16_t *DY){

    int8_t dec = 0x0000;                // dec is decimal value of temp
                                        // temp is the current data being read
    int8_t bits[7] = {64, 32, 16, 8, 4, 2, 1};
    int i;

    if((temp >> 7) & 1){                // if MSB is one, the
        *flag = 1;                      // number is negative.
        for(i = 6; i >= 0; i--)
            if((temp >> i) & 1)         // if the i'th bit is one, then
                dec += bits[6 - i];     // generate the 2's compliment
        dec = 128 - dec;                // decimal value.
    }
    else{
        for(i = 6; i >= 0; i--)
            if((temp >> i) & 1)
                dec += bits[6 - i];
        *flag = 0;
    }
    
    if(*flag)
        *DY -= dec;                 // decrease DY if flag is set... 
    else                               
        *DY += dec;                 // increase DY if no flag


return;
}

// output Decimal as ASCII character
void DecToASCII(int16_t DY){

    int K = 0;
    char w[] = "dY = ";

    TXSTAbits.TXEN = 1;                 // enable transmitter

    while(w[K] != '\0'){
        while(!TXSTAbits.TRMT);         // TRMT bit is 0 then transmit
        TXREG = w[K];                   // shift register is busy
        K++;
    }

    unsigned char ASCII[16];
    int j =0;
    
    // sprintf converts signed decimal number into an array of ASCII codes
    sprintf(ASCII,"%d",DY);

    while(ASCII[j] != '\0'){
        EUSART_write(ASCII[j]);
        j++;
    }

    while(!TXSTAbits.TRMT);                 // wait until flag is set 
    EUSART_write(0x0A);                     // 0x0A = '\n' skip line
    while(!TXSTAbits.TRMT);                 // wait until TSR is done

return;
}

// write to MAX232/PC
void EUSART_write(unsigned char c) {

    TXSTAbits.TXEN = 1;                     // enable transmitter
    while(!TXSTAbits.TRMT);                 // wait until flag is set 
    TXREG = c;                              // output data on PORTA to MAX232
    while(!TXSTAbits.TRMT);                 // wait until TSR is done
    TXSTAbits.TXEN = 0;                     // disable transmitter

return;
}