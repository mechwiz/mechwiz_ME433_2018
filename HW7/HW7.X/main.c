#include<xc.h>
#include<sys/attribs.h>  // __ISR macro
#include "ST7735.h"           // processor SFR definitions
#include "i2c_master_noint.h"
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

// Set address
#define SLAVE_ADDR 0b1101011

void initIMU();
void setIMU(unsigned char pin,unsigned char level);
void getIMU(unsigned char register, unsigned char * data, int length);

void drawChar(unsigned short x, unsigned short y, unsigned char msg, unsigned short c1, unsigned short c2);
void drawString(unsigned short x, unsigned short y, unsigned char *msg, unsigned short c1, unsigned short c2);
void drawHorizontalBar(unsigned short x, unsigned short y, unsigned short h, signed short l1, unsigned short c1, unsigned short l2, unsigned short c2);
void drawVerticalBar(unsigned short x, unsigned short y, unsigned short h, signed short l1, unsigned short c1, unsigned short l2, unsigned short c2);
void drawMiddle(unsigned short x, unsigned short y, unsigned short h, unsigned short c);
short map(float input, float input_min, float input_max, float output_min, float output_max);

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
    
    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;  // make RA4 an output
    LATAbits.LATA4 = 1; // make RA4 high to turn LED on initially
    
    LCD_init();
    initIMU();
    __builtin_enable_interrupts();
    
    LCD_clearScreen(CYAN);
    unsigned char message[30];
    unsigned char data[14];
    short values[7];
    int i;
    unsigned char whoami;
    short x_pix, y_pix;
    
    while(1) {
        
        _CP0_SET_COUNT(0);
        getIMU(0x20,data,14);
        for (i=0;i<14;i+=2){
            values[i/2] = data[i] | (data[i+1]<<8);
        }
        getIMU(0x0F,data,1);
        whoami = data[0];
        sprintf(message,"WHOAMI %d  ",whoami);
        drawString(10,10,message,MAGENTA,CYAN);
        sprintf(message,"AX %d  ",values[4]);
        drawString(10,20,message,MAGENTA,CYAN);
        sprintf(message,"AY %d  ",values[5]);
        drawString(10,30,message,MAGENTA,CYAN);
        //map values to be +/- 1g. Could do +/- 2g too.
        x_pix = map(values[4],-16384.0,16383.0,-60.0,60.0);
        y_pix = map(values[5],-16384.0,16383.0,-60.0,60.0); 
        drawHorizontalBar(4,94,4,-x_pix,YELLOW,120,BLUE);
        drawVerticalBar(62,36,4,-y_pix,YELLOW,120,BLUE);
        drawMiddle(62,94,4,BLUE);
        // .05s / (2/48000000) == 1200000
        while (_CP0_GET_COUNT() < 1200000) {
            
        }
        //invert RA4
        LATAINV = 0x10;
        
    }
    return 0;
}

void initIMU(){
    //turn off analog input on I2C2 pins
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
    
    //set 1.66 kHz, 2g, and 100 Hz
    setIMU(0x10,0b10000010);
    //set 1.66 kHz and 1000 dps
    setIMU(0x11,0b10001000);

}

void setIMU(unsigned char pin, unsigned char level){
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(pin);
    i2c_master_send(level);
    i2c_master_stop();
    
}

void getIMU(unsigned char reg, unsigned char * data, int length){
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send(SLAVE_ADDR << 1 | 1);
    int i;
    for(i=0;i<length;i++){
        data[i]=i2c_master_recv();
        if(i==length-1){
            i2c_master_ack(1);
        }else {
            i2c_master_ack(0);
        }
    }
    i2c_master_stop(); // make the stop bit
  
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

void drawHorizontalBar(unsigned short x, unsigned short y, unsigned short h, signed short l1, unsigned short c1, unsigned short l2, unsigned short c2){
    int i;
    int j;
    int k;
    int m;
    for (i=0;i<h;i++){
        if(l1<0){
            l1 = max(-60,l1);
            for (j=0;j<(60+l1);j++){
                LCD_drawPixel(x+j,y+i,c1);
            }
            for (k=(60+l1);k<60;k++){
                LCD_drawPixel(x+k,y+i,c2);
            }
            for (m=60;m<l2;m++){
                LCD_drawPixel(x+m,y+i,c1);
            }
        } else {
            l1 = min(60,l1);
            for (j=0;j<60;j++){
                LCD_drawPixel(x+j,y+i,c1);
            }
            for (k=60;k<(60+l1);k++){
                LCD_drawPixel(x+k,y+i,c2);
            }
            for (m=(60+l1);m<l2;m++){
                LCD_drawPixel(x+m,y+i,c1);
            }
        }
    }
}

void drawVerticalBar(unsigned short x, unsigned short y, unsigned short h, signed short l1, unsigned short c1, unsigned short l2, unsigned short c2){
    int i;
    int j;
    int k;
    int m;
    for (i=0;i<h;i++){
        if(l1<0){
            l1 = max(-60,l1);
            for (j=0;j<(60+l1);j++){
                LCD_drawPixel(x+i,y+j,c1);
            }
            for (k=(60+l1);k<60;k++){
                LCD_drawPixel(x+i,y+k,c2);
            }
            for (m=60;m<l2;m++){
                LCD_drawPixel(x+i,y+m,c1);
            }
        } else {
            l1 = min(60,l1);
            for (j=0;j<60;j++){
                LCD_drawPixel(x+i,y+j,c1);
            }
            for (k=60;k<(60+l1);k++){
                LCD_drawPixel(x+i,y+k,c2);
            }
            for (m=(60+l1);m<l2;m++){
                LCD_drawPixel(x+i,y+m,c1);
            }
        }
    }
}

void drawMiddle(unsigned short x, unsigned short y, unsigned short h, unsigned short c){
    int i, j;
    for (i=0;i<h;i++){
        for (j=0;j<h;j++){
            LCD_drawPixel(x+i,y+j,c);
        }
    }
}

short map(float input, float input_min, float input_max, float output_min, float output_max){
    short output = (short)(output_min + (input - input_min)*((output_max - output_min) / (input_max - input_min)));
    return output;
}

