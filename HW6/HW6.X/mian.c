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



int main(){
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
    SPI1_init();
    LCD_init();   
    __builtin_enable_interrupts(); 
    
    LCD_clearScreen (WHITE);
    char s[130];
    int count = 0; //timing variable 
    float spf;     // SPF
    int len = 60 , dep = 10;  //length and depth of the bar
    int sp = len/50;  // increase length of every step when drawing the bar
    int startx = 64 , starty = 55;  //bar start position
    
    while (1){
        _CP0_SET_COUNT(0);
        //display "Hello world!"
        sprintf(s , "Hello world %d! ", count);  
        displayString(s , 28 , 32 , BLUE , WHITE);  //display string
        
        //draw bar
        if(count >= 0){
            displayBar(startx , starty , BLUE, WHITE ,sp*count , dep);  //display right half bar
        }
        if (count < 0){
            displayBar(startx+count*sp, starty, BLUE, WHITE, abs(count)*sp, dep); //display left half bar
        } 
        count++;
        if (count == 51){   //set count from 50 to -50
            count = -50;
        }
        
        //calculate SPF
        spf =24000000.0/ _CP0_GET_COUNT();  // get SPF
        sprintf(s , "FPS = %.2f", spf);
        displayString(s , 35 , 80 , BLUE , WHITE); 
        while (_CP0_GET_COUNT() < 48000){ ; } // 5HZ from -50 to 50
    }
    return 0 ;
}