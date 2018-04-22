#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>

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

#define CS LATAbits.LATA0       // chip select pin

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

// initialize spi1
void initSPI1() {
  // set up the chip select pin as an output
  // the chip select pin is used to indicate
  // when a command is beginning (clear CS to low) and when it
  // is ending (set CS high)
  TRISAbits.TRISA0 = 0;
  CS = 1;
 
  // setup spi1
  RPA1Rbits.RPA1R = 0b0011; // set RA1 to be SDO1
  RPA0Rbits.RPA0R = 0b0011; // set RA0 to be chip select
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x1;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1
  SPI1CONbits.MODE16 = 0;
  SPI1CONbits.MODE32 = 0;

                            // send a ram set status command.
  CS = 0;                   // enable the ram
  spi_io(0x01);             // ram write status
  spi_io(0x41);             // sequential mode (mode = 0b01), hold disabled (hold = 0)
  CS = 1;                   // finish the command
}

void setVoltage(char c,int v) {
  unsigned short t = 0;
  t = c<<15; // bit shift so that channel is at bit 15
  t = t | 0b0111000000000000; // get the config bits in there
  t = t | v << 2;   //bit-shift voltage two places to the left
                    //stick voltage into message bit
  
  
  CS = 0;                   // enable the ram
  spi_io(t>>8);             // write the first 8 bits
  spi_io(t&0xff);             // write the last 8 bits
  CS = 1;                   // finish the command
  
  
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
    
    initSPI1();
    __builtin_enable_interrupts();
    
    int i = 0;
    int j = 0;
    int mid = 0;
    while(1) {
        _CP0_SET_COUNT(0);
        //start in middle since sin-wave goes negative
        float f = 512.0 + 512.0*sin(i*2.0*3.14/1000.0*10.0);
        setVoltage(0,f);
        i++;
        //start at 0V and ramp up to 3.3V in 10 cycles, then ramp down for 10 cycles.
        float tri = 10.0*j/1000.0*1023.0;
        setVoltage(1,tri);
        // If still ramping up
        if (mid == 0 && j< 100) {
            // If about to reach midpoint
            if (j==99){
                mid = 1; // set flag that midpoint is reached
            }
            j++;    //increase voltage
        }else{  //If after midpoint
            // If about to reach 0 V
            if (j==1){
                mid = 0;   //clear midpoint flag
            }
            j--;    // decrease voltage
        }
        
        // 1ms / (2/48000000) == 24000
        while (_CP0_GET_COUNT() < 24000) {
            
        }
    }
    return 0;
}