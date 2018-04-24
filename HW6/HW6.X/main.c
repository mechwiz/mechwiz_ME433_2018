#include <xc.h>
#include<sys/attribs.h>  // __ISR macro
#include"ST7735.h"           // processor SFR definitions
#include<stdio.h>

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

void drawChar(unsigned short x, unsigned short y, unsigned char msg, unsigned short c1, unsigned short c2);
void drawString(unsigned short x, unsigned short y, unsigned char *msg, unsigned short c1, unsigned short c2);

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
    
    LCD_init();
    __builtin_enable_interrupts();
    
    LCD_clearScreen(CYAN);
    unsigned char message[30];
    int p = 0;
    while(1) {
        
        sprintf(message,"Hello World %d    \0",p);
        drawString(28,32,message,MAGENTA,CYAN);
        drawProgressBar(14,60,10,p,BLUE,100,YELLOW);
        _CP0_SET_COUNT(0);
        
        // .1s / (2/48000000) == 2400000
        while (_CP0_GET_COUNT() < 2400000) {
            
        }
        p++;
        if (p==101){
            p=0;
        }
    }
    return 0;
}

void drawChar(unsigned short x, unsigned short y, unsigned char msg, unsigned short c1, unsigned short c2){
    unsigned char row = msg-0x20;
    int col;
    for (col=0;col<5;col++){
        unsigned char pixels = ASCII[row][col];
        int i;
        for (i=0;i<8;i++){
            if ((x+col)<128 && (y+i)<160){
                if (((pixels>>i) & 1) == 1){
                    LCD_drawPixel(x+col,y+i,c1);
                } else {
                    LCD_drawPixel(x+col,y+i,c2);
                }
            }
        }
    }
}

void drawString(unsigned short x, unsigned short y, unsigned char *msg, unsigned short c1, unsigned short c2){
   int i = 0;
   
   while(msg[i]){
       drawChar(x+5*i,y,msg[i],c1,c2);
       i++;
   }
    
}

void drawProgressBar(unsigned short x, unsigned short y, unsigned short h, unsigned short l1, unsigned short c1, unsigned short l2, unsigned short c2){
    int i;
    int j;
    int k;
    for (i=0;i<h;i++){
        for (j=0;j<l1;j++){
            LCD_drawPixel(x+j,y+i,c1);
        }
        for (k=l1;k<l2;k++){
            LCD_drawPixel(x+k,y+i,c2);
        }
    }
}