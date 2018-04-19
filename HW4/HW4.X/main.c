#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

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
// Demonstrates spi by accessing external ram
// PIC is the master, ram is the slave
// Uses microchip 23K256 ram chip (see the data sheet for protocol details)
// SDO4 -> SI (pin F5 -> pin 5)
// SDI4 -> SO (pin F4 -> pin 2)
// SCK4 -> SCK (pin B14 -> pin 6)
// SS4 -> CS (pin B8 -> pin 1)
// Additional SRAM connections
// Vss (Pin 4) -> ground
// Vcc (Pin 8) -> 3.3 V
// Hold (pin 7) -> 3.3 V (we don't use the hold function)
// 
// Only uses the SRAM's sequential mode
//
#define CS LATAbits.LATA0       // chip select pin

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

// initialize spi4 and the ram module
void ram_init() {
  // set up the chip select pin as an output
  // the chip select pin is used by the sram to indicate
  // when a command is beginning (clear CS to low) and when it
  // is ending (set CS high)
  TRISBbits.TRISB8 = 0;
  CS = 1;

  // Master - SPI4, pins are: SDI4(F4), SDO4(F5), SCK4(F13).  
  // we manually control SS4 as a digital output (F12)
  // since the pic is just starting, we know that spi is off. We rely on defaults here
 
  // setup spi4
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 4

                            // send a ram set status command.
  CS = 0;                   // enable the ram
  spi_io(0x01);             // ram write status
  spi_io(0x41);             // sequential mode (mode = 0b01), hold disabled (hold = 0)
  CS = 1;                   // finish the command
}

// write len bytes to the ram, starting at the address addr
void ram_write(unsigned short addr, const char data[], int len) {
  int i = 0;
  CS = 0;                        // enable the ram by lowering the chip select line
  spi_io(0x2);                   // sequential write operation
  spi_io((addr & 0xFF00) >> 8 ); // most significant byte of address
  spi_io(addr & 0x00FF);         // the least significant address byte
  for(i = 0; i < len; ++i) {
    spi_io(data[i]);
  }
  CS = 1;                        // raise the chip select line, ending communication
}

// read len bytes from ram, starting at the address addr
void ram_read(unsigned short addr, char data[], int len) {
  int i = 0;
  CS = 0;
  spi_io(0x3);                   // ram read operation
  spi_io((addr & 0xFF00) >> 8);  // most significant address byte
  spi_io(addr & 0x00FF);         // least significant address byte
  for(i = 0; i < len; ++i) {
    data[i] = spi_io(0);         // read in the data
  }
  CS = 1;
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

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;  // make RA4 an output
    LATAbits.LATA4 = 1; // make RA4 high to turn LED on initially
    TRISBbits.TRISB4 = 1; // make RB4 an input
    __builtin_enable_interrupts();
  
    unsigned short addr1 = 0x1234;                  // the address for writing the ram
    char data[] = "Help, I'm stuck in the RAM!";    // the test message
    char read[] = "***************************";    // buffer for reading from ram
    char buf[100];                                  // buffer for comm. with the user
    unsigned char status;                           // used to verify we set the status 
    
    ram_init(); 

    // check the ram status
    CS = 0;
    spi_io(0x5);                                      // ram read status command
    status = spi_io(0);                               // the actual status
    CS = 1;

    sprintf(buf, "Status 0x%x\r\n",status);
   

    sprintf(buf,"Writing \"%s\" to ram at address 0x%x\r\n", data, addr1);
    
                                                      // write the data to the ram
    ram_write(addr1, data, strlen(data) + 1);         // +1, to send the '\0' character
    ram_read(addr1, read, strlen(data) + 1);          // read the data back
    sprintf(buf,"Read \"%s\" from ram at address 0x%x\r\n", read, addr1);
    

    while(1) {
      ;
    }
    return 0;
}