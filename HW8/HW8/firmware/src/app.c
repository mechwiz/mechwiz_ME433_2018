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
#include "ST7735.h"           // processor SFR definitions
#include "i2c_master_noint.h"
#include<stdio.h>

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
// Set address
#define SLAVE_ADDR 0b1101011
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

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
void initIMU();
void setIMU(unsigned char pin,unsigned char level);
void getIMU(unsigned char register, unsigned char * data, int length);

void drawChar(unsigned short x, unsigned short y, unsigned char msg, unsigned short c1, unsigned short c2);
void drawString(unsigned short x, unsigned short y, unsigned char *msg, unsigned short c1, unsigned short c2);
void drawHorizontalBar(unsigned short x, unsigned short y, unsigned short h, signed short l1, unsigned short c1, unsigned short l2, unsigned short c2);
void drawVerticalBar(unsigned short x, unsigned short y, unsigned short h, signed short l1, unsigned short c1, unsigned short l2, unsigned short c2);
void drawMiddle(unsigned short x, unsigned short y, unsigned short h, unsigned short c);
short map(float input, float input_min, float input_max, float output_min, float output_max);

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
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
    
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

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
            static unsigned char message[30];
            static unsigned char data[14];
            static short values[7];
            static int i;
            static unsigned char whoami;
            static short x_pix, y_pix;
            
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
 

/*******************************************************************************
 End of File
 */
