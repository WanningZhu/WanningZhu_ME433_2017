#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
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

#define CS LATBbits.LATB7       // chip select pin
static volatile unsigned int sinWave[100];
static volatile  unsigned int triangleWave[100];


// send a byte via spi and return the response
unsigned char spi_io(unsigned char write) {
  SPI1BUF = write;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void initSPI1() {
  SDI1Rbits.SDI1R = 0b0000; //SDI1 set pin 3 (A1)
  RPB8Rbits.RPB8R = 0b0011; //SDO1 set pin 17 (B8))
  TRISBbits.TRISB7 = 0;     //SS1 set pin 16 (B7)
  CS = 1;
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x1;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 4
}
void setVoltage(char channel, unsigned char voltage) {
    unsigned char b1 = voltage >> 4;
    unsigned char b2 = voltage << 4;
    b1 = b1 | 0b11110000;
    if (channel == 0 ){
        b1^=1<<7;
    }
    CS = 0;
    spi_io (b1);
    spi_io (b2);
    CS = 1;
}

void makeWave(){
    int i = 0;
    for (i = 0; i<100;i++){
        sinWave[i] = 127.5 * (1.0 + sin(2.0 * M_PI * i / 100.0));
        triangleWave[i] = 255.0 / 100.0 * i;
    }
}

int main(void) {
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
    makeWave();
    initSPI1();
    __builtin_enable_interrupts();
    int j = 0;
    int i = 0;
    int ii = 0;
    while(1){
        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < 24000) {;} //wait for 0.001s
        setVoltage(0 , sinWave[i]);
        if (j == 0){
           setVoltage(1 , triangleWave[ii]);  
           ii++;
        }
        j = !j;
        i++;
        if(i == 100){
            i = 0;
        }
        if(ii == 100){
            ii = 0;
        }
    }
  return 0;
}