#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include"ILI9163C.h"
#include<stdio.h>
#include<math.h>

// DEVCFG0
#pragma config DEBUG = 0b10 // no debugging
#pragma config JTAGEN = 0  // no jtag
#pragma config ICESEL = 0b11 // use PGED1 and PGEC1
#pragma config PWP = 0x1FF // no write protect
#pragma config BWP = 0 // no boot write protect
#pragma config CP = 1 // no code protect

// DEVCFG1
#pragma config FNOSC = 0b11 // use primary oscillator with pll
#pragma config FSOSCEN = 0 // turn off secondary oscillator
#pragma config IESO = 0 // no switching clocks
#pragma config POSCMOD = 0b10 // high speed crystal mode
#pragma config OSCIOFNC = 1 // disable secondary osc
#pragma config FPBDIV = 0b00 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = 0b11 // do not enable clock switch
#pragma config WDTPS = 0 // use slowest wdt
#pragma config WINDIS = 0b10100 // wdt no window mode
#pragma config FWDTEN = 0 // wdt disabled
#pragma config FWDTWINSZ = 0b11// wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = 0b001 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = 0b111 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = 0b001 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = 0b001 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = 0 // USB clock on

// DEVCFG3
#pragma config USERID = 0xFFFE // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = 0 // allow multiple reconfigurations
#pragma config IOL1WAY = 0 // allow multiple reconfigurations
#pragma config FUSBIDIO = 1 // USB pins controlled by USB module
#pragma config FVBUSONIO = 1 // USB BUSON controlled by USB module

// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be callled in the correct order as per the I2C protocol
// Change I2C1 to the I2C channel you are using
// I2C pins need pull-up resistors, 2k-10k

static volatile unsigned char address=0b1101011;  // LSM6DS33 address
int fram = 40;
static volatile char s[130];

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

int main () {
    
      __builtin_disable_interrupts();
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;
    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;
    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB4 = 1;
    LATAbits.LATA4 = 1;
    // do your TRIS and LAT commands here

    initIMU();
    SPI1_init();
    LCD_init();
    __builtin_enable_interrupts();
     LCD_clearScreen (WHITE);
    unsigned char datatemp[14];
    short data[7]; //[temperature,gyroX,gyroY,gyroZ,accelX,accelY,accelZ]

    while (1){
        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < 4800000) {;} //5HZ
        getAccel(data,datatemp);
        drawmove(data,64,64,BLACK,WHITE,6,40);
            
    }
    return 0;   
}
