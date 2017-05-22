#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
//#include<math.h>
//#include <proc/p32mx250f128b.h>

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

void i2c_master_setup(void) {
  I2C2BRG = 233;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
                                    // look up PGD for your PIC32
  I2C2CONbits.ON = 1;               // turn on the I2C1 module
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

void initExpander(void){ //initiate expander
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
    
    i2c_master_start();      //make start bits
    i2c_master_send( 0x20 << 1 | 0 ); // write the address, indicate writing
    i2c_master_send( 0x00 ); //the register to write to 
    i2c_master_send( 0xF0 ); //the value to write to, set GP0-3 output, GP4-7 input
    i2c_master_stop();       //make stop bits  
}

void setExpander(char pin, char level){
    i2c_master_start();
    i2c_master_send( 0x20 << 1 | 0 );
    i2c_master_send( 0x0A );
    if (level == 0){    //decrease the level of pin to 0
        i2c_master_send( 0b0 << pin );
    }
    else if (level == 1){ //pull up the level of pin to 1
        i2c_master_send( 0b1 << pin );
    }
    i2c_master_stop();
}

char getExpander(char pin){
    i2c_master_start();
    i2c_master_send( 0x20 << 1 | 0 );
    i2c_master_send( 0x09 );
    i2c_master_restart();
    i2c_master_send( 0x20 << 1 | 1 ); // write the address and set to read
    char r = i2c_master_recv() >> pin;     //get the value of the desired pin
    i2c_master_ack( 1 );              //tell the master already got the byte
    i2c_master_stop();
    return r;
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

    initExpander();
    char pinInput = 7; //get the value of GP7
    char pinOutput = 0; //set the value of GP0
    __builtin_enable_interrupts();
    
    while (1){
        if (getExpander( pinInput ) == 0){  //if GP7 is low
            setExpander (pinOutput , 1);       //set GP0 to high
        }
        else if (getExpander( pinInput) == 1){ //if GP7 is high
            setExpander (pinOutput , 0);         //set GP0 to low
        }   
    }
    return 0;   
}
