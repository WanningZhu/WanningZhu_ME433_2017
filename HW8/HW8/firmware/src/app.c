/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include"ILI9163C.h"
#include<stdio.h>
#include<math.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

static volatile unsigned char address=0b1101011;  // LSM6DS33 address
int fram = 40;
static volatile char s[130];

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/
void i2c_master_setup(void) {
  I2C2BRG = 233;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
                                    // look up PGD for your PIC32
  I2C2CONbits.ON = 1;               // turn on the I2C2 module
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;            // send the start bit
    while(I2C2CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {     
    I2C2CONbits.RSEN = 1;           // send a restart 
    while(I2C2CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C2TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }  // wait for the transmission to finish
//  if(I2C2STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
//    // ("I2C2 Master: failed to receive ACK\r\n");
//  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // start receiving data
    while(!I2C2STATbits.RBF) { ; }    // wait to receive the data
    return I2C2RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }        // wait for STOP to complete
}
void writeIMU(unsigned char add,unsigned char addRegister, unsigned char value){
    i2c_master_start(); 
    i2c_master_send( add << 1 | 0 );
    i2c_master_send( addRegister );
    i2c_master_send( value );
    i2c_master_stop();
}
void initIMU(void){ //initiate expander
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup(); 
    writeIMU(address, 0x10 , 0b10000010); //set CTRL1_XL
    writeIMU(address, 0x11 , 0b10001000); //set CTRL2_G
    writeIMU(address ,0x12 , 0b00000100); //set CTRL3_C
}

void I2C_read_multiple(unsigned char add, unsigned char addRegister, unsigned char * data, int length){
    int i;
    i2c_master_start();
    i2c_master_send(add<<1|0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing
    i2c_master_send(addRegister); // the register to read from
    i2c_master_restart(); // make the restart bit
    i2c_master_send(add<<1|1); // write the address, shifted left by 1, or'ed with a 1 to indicate reading
    for(i=0 ; i<length ; i++){
        if(i<(length-1)){
        data[i] = i2c_master_recv();// save the value returned
        i2c_master_ack(0); //make the nack so the salve knows sending continuously
        }
        if(i == length-1){
        data[i] = i2c_master_recv();
        i2c_master_ack(1); // make the ack so the slave knows we got it
    }
    }
    i2c_master_stop(); // make the stop bit
}

void displayCharacter (char c, unsigned short x, unsigned short y, unsigned short color_c, unsigned short color_b){
    //color_c character color, color_b background color
    char row = c-0x20;  //row of the char
    int i , j;
    for (i=0 ; i<5 ; i++){
        if (x+i < 129){
            for (j=0 ; j< 8 ; j++){
                if (ASCII[row][i] >> j & 1){
                    if (y+j < 129){
                        LCD_drawPixel(x+i , y+j , color_c);
                    }
                }
                else {
                    if (y+j < 129){
                        LCD_drawPixel(x+i , y+j , color_b);
                    }
                }
            }
        }
    }
}

void displayString(char *s , unsigned short x, unsigned short y, unsigned short color_c, unsigned short color_b){
    int i = 0;
    for (i=0 ; i<130; i++){
        if (s[i]){
            displayCharacter(s[i] , x+5*i , y , color_c , color_b);
        }
        else {
            break;
        }
    }
}

void getAccel(short * sdata, unsigned char * sdatatemp){
    int j;
    I2C_read_multiple(address, 0x20, sdatatemp, 14);          
        for(j=0 ; j<7 ; j++){
            sdata[j] = ((sdatatemp[2*j+1]<<8)| sdatatemp[2*j]); //convert to short   
        }
}

void drawmove(short * sdata, unsigned short x, unsigned short y, unsigned short color_c, unsigned short color_b, int dep,int f){
    //f is the area of frame
    //dep is the depth of bar
    //stx sty start position of the bar
    int per = 16384/f;
    short ax = sdata[4];
    short ay = sdata[5];
    int wx = ax/per; //width of the bar
    int wy = ay/per;
    unsigned short x0 = x-dep/2, y0 = y-dep/2;
    displayBar(x0,y0,color_c,color_b,dep,dep);
    displayBar_x(x,y,color_c,color_b,wx,dep,f);
    displayBar_y(x,y,color_c,color_b,wy,dep,f);
}
//
void displayBar_x(unsigned short x0, unsigned short y0, unsigned short color_c, unsigned short color_b, int w, int dep,int f){
    //w is the width of the fragment 
    //f is the area of frame
    int i , j , sign;
    unsigned short x, y;
    if (w<=0){
        sign = 1;
        x = x0 + dep/2;
        y = y0 - dep/2;
    }
    else {
        sign = -1;
        x = x0 - dep/2;
        y = y0 - dep/2;
    }
        
    for (i=0 ; i<abs(w) ; i++){               //draw the fragment
        for (j=0 ; j<dep ; j++){
             LCD_drawPixel(x+sign*i , y+j , color_c);
        } 
    }
    for(i=(dep+1) ; i<=f ; i++){               //make sure other part except bar is background color
        for (j=0 ; j<dep ; j++){
             LCD_drawPixel(x-sign*i, y+j , color_b);
        } 
    }
    for(i=1 ; i<=(f-abs(w)) ; i++){           //make sure other part except bar is background color
        for (j=0 ; j<dep ; j++){
             LCD_drawPixel(x+sign*abs(w)+sign*i , y+j , color_b);
        } 
    }
}

void displayBar_y(unsigned short x0, unsigned short y0, unsigned short color_c, unsigned short color_b, int w, int dep,int f){
    //w is the width of the fragment 
    //f is the area of frame
    int i , j , sign;
    unsigned short x,y;
    if (w<=0){
        sign = 1;
        x = x0 - dep/2;
        y = y0 + dep/2;
    }
    else {
        sign = -1;
        x = x0 - dep/2;
        y = y0 - dep/2;
    }
        
    for (i=0 ; i<abs(w) ; i++){               //draw the fragment
        for (j=0 ; j<dep ; j++){
             LCD_drawPixel(x+j , y+sign*i , color_c);
        } 
    }
    if(sign == 1){
    for(i=(dep+1) ; i<=f ; i++){               //make sure other part except bar is background color
        for (j=0 ; j<dep ; j++){
             LCD_drawPixel(x+j, y-sign*i , color_b);
        } 
    }
    }
    else {
       for(i=dep ; i<=f ; i++){               //make sure other part except bar is background color
        for (j=0 ; j<dep ; j++){
             LCD_drawPixel(x+j, y-sign*i , color_b);
        } 
    } 
    }
    
    for(i=1 ; i<=(f-abs(w)) ; i++){           //make sure other part except bar is background color
        for (j=0 ; j<dep ; j++){
             LCD_drawPixel(x+j , y+sign*abs(w)+sign*i , color_b);
        } 
    }
}

void displayBar(unsigned short x, unsigned short y, unsigned short color_c, unsigned short color_b, int w, int dep){
    //w is the width of the fragment 
    int i , j , n;
    for (i=0 ; i<w ; i++){               //draw the fragment
        for (j=0 ; j<dep ; j++){
             LCD_drawPixel(x+i , y+j , color_c);
        } 
    }
    for(n=1 ; n<x ; n++){               //make sure other part except bar is background color
        for (j=0 ; j<dep ; j++){
             LCD_drawPixel(n , y+j , color_b);
        } 
    }
    for(n=x+w ; n<128 ; n++){           //make sure other part except bar is background color
        for (j=0 ; j<dep ; j++){
             LCD_drawPixel(n , y+j , color_b);
        } 
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB4 = 1;
    LATAbits.LATA4 = 1;
    
    initIMU();
    SPI1_init();
    LCD_init();
    LCD_clearScreen (WHITE);

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
     unsigned char datatemp[14];
    short data[7]; //[temperature,gyroX,gyroY,gyroZ,accelX,accelY,accelZ]

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < 4800000) {;} //5HZ
        getAccel(data,datatemp);
        drawmove(data,64,64,BLACK,WHITE,6,40);
            
            
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
