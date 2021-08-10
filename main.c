/*
 * File:   main.c
 * Author: Kayden Rice, Paul Walsh, Evan Fitzgerald, Greg Kean
 *
 * Created on April 10th, 2021, 8:21 PM
 */

// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = OFF            // Alternate Interrupt Vector Table bit (Disabled AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Enter Hexadecimal value)

// FSIGN

// FOSCSEL
#pragma config FNOSC = FRCDIVN        // Oscillator Source Selection (Internal Fast RC (FRC) Oscillator with postscaler)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)
#pragma config PLLKEN = ON              // PLL Lock Status Control (PLL lock signal will be used to disable PLL clock output if lock is lost)
#pragma config XTCFG = G3               // XT Config (24-32 MHz crystals)
#pragma config XTBST = ENABLE           // XT Boost (Boost the kick-start)

// FWDT
// RWDTPS = No Setting
#pragma config RCLKSEL = LPRC           // Watchdog Timer Clock Select bits (Always use LPRC)
#pragma config WINDIS = ON              // Watchdog Timer Window Enable bit (Watchdog Timer operates in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
// SWDTPS = No Setting
#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (WDT enabled in hardware)

// FPOR
#pragma config BISTDIS = DISABLED       // Memory BIST Feature Disable (mBIST on reset feature disabled)

// FICD
#pragma config ICS = 2               // ICD Communication Channel Select bits (Communicate on PGC1 and PGD1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#pragma config NOBTSWP = DISABLED       // BOOTSWP instruction disable bit (BOOTSWP instruction is disabled)

// FDMTIVTL
#pragma config DMTIVTL = 0xFFFF         // Dead Man Timer Interval low word (Enter Hexadecimal value)

// FDMTIVTH
#pragma config DMTIVTH = 0xFFFF         // Dead Man Timer Interval high word (Enter Hexadecimal value)

// FDMTCNTL
#pragma config DMTCNTL = 0xFFFF         // Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF) (Enter Hexadecimal value)

// FDMTCNTH
#pragma config DMTCNTH = 0xFFFF         // Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF) (Enter Hexadecimal value)

// FDMT
#pragma config DMTDIS = OFF             // Dead Man Timer Disable bit (Dead Man Timer is Disabled and can be enabled by software)

// FDEVOPT
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 Pin bit (I2C2 mapped to SDA2/SCL2 pins)
#pragma config ALTI2C3 = OFF            // Alternate I2C3 Pin bit (I2C3 mapped to SDA3/SCL3 pins)
#pragma config SMBEN = SMBUS            // SM Bus Enable (SMBus input threshold is enabled)
#pragma config SPI2PIN = PPS            // SPI2 Pin Select bit (SPI2 uses I/O remap (PPS) pins)

// FALTREG
#pragma config CTXT1 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config CTXT2 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config CTXT3 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits (Not Assigned)
#pragma config CTXT4 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits (Not Assigned)

// FBTSEQ
#pragma config BSEQ = 0xFFF             // Relative value defining which partition will be active after device Reset; the partition containing a lower boot number will be active (Enter Hexadecimal value)
#pragma config IBSEQ = 0xFFF            // The one's complement of BSEQ; must be calculated by the user and written during device programming. (Enter Hexadecimal value)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include "xc.h"
#define FCY 8000000UL 
#include <libpic30.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <p33CK256MP506.h>
#include <math.h>

// AT Commands set
const char* atc0 = "AT+RST";   //AT Reset 
const char* atc1 = "AT+CWMODE=3";    //AT Station/AP mode
const char* atc2 = "AT+CWJAP=\"BELL660\",\"7F76743A5E66\""; // SSID = Service Set IDentifier   
const char* atc3 ="AT+CWSAP=\"ESP32\",\"1234567890\",5,3";  // ESP Hotspot info
const char* atc4 = "AT+CIPSTART=\"TCP\",\"192.168.4.2\",8080";   //connect to server IP 
const char* atc6 = "AT+CIPSEND=4";
const char* atc7 = "2000";


unsigned char receive[100];                        //Variable being watched 
char transmit;
int errorCheck;
char get_Internet[100];
unsigned ATcount = 0;
unsigned int i = 0;
unsigned int j = 0;
unsigned int buttonPressed = 0;
float rollingAverage;
int timerCounter = 0;
int server = 0;
#define FP 4000000                 //Fp = FCY/2 = 4Mhz
#define BAUDRATE 115200            //Baudrate = 115200 bit/sec
#define BRGVAL (FP/(BAUDRATE*4))-1 //BRGVAL ~= 8, we do not use this

// This variable will keep the conversion result. 
unsigned int dataAN15 = 0; //(CS_2)

int count = 0;
float powers[50] = {0};
float RMS[50] = {0};
char powerVal[200];
char bufval[200]; // contains ADCBUF15 Value
double mV_Value = 0; // contains the mV value of the ADC *new*
int DMA_COUNT = 0;

//ENABLE ADC
void enableADC(){
//(10) Set ADC WARMTIME 
    ADCON5Hbits.WARMTIME = 15;
//turn on ADC
    ADCON1Lbits.ADON = 1;
// Turn on analog power for shared core 
    ADCON5Lbits.SHRPWR = 1;
// Wait when the shared core is ready for operation 
    while(ADCON5Lbits.SHRRDY == 0);  
// Turn on digital power to enable triggers to the shared core
    ADCON3Hbits.SHREN = 1; 
}

//SETUP ADC
void setupADC(){
    //(2) Configure the common ADC clock.
    ADCON3Hbits.CLKSEL = 1; // clock source: FRC (peripheral clock)
    ADCON3Hbits.CLKDIV = 0;  // no clock divider (1:1) 
    
    //(3) Configure clock period
    ADCON2Lbits.SHRADCS = 1; // 2 source clock periods
    
    //(4) Configure the ADC reference sources. 
    ADCON3Lbits.REFSEL = 0; //AVdd as voltage reference 
    
    //(5)Configure the resolution
    ADCON1Hbits.SHRRES = 3; //12-bit res.
    
    //(6) Configure the integer of fractional output format.
    ADCON1Hbits.FORM = 0; // integer format
    
    //(7)Select single-ended input config and unsigned output format
    ADMOD0Hbits.SIGN15 = 0; // AN15/RC3
    ADMOD0Hbits.DIFF15 = 0; // AN15/RC3
    
    //(8) Configure the shared ADC core sample time 
    ADCON2Hbits.SHRSAMC = 40;
    
    //(9) Configure and enable ADC interrupts. 
    ADIELbits.IE15 = 1; // enable interrupt for AN15
    IFS6bits.ADCAN15IF = 0; // clear interrupt flag for AN15
    IEC6bits.ADCAN15IE = 1 ; // enable interrupt for AN15
}

//                  **LCD CODE**
// Written by: Rajendra Bhatt, Sep 6, 2010. SOURCE: https://www.electronics-lab.com/project/3-wire-serial-lcd-using-a-shift-register/
// Modified by: Kayden Rice, Mar 18, 2021 to work with the dsPIC33CK256MP506

unsigned short Low_Nibble, High_Nibble, p, q,  Mask, N,t, RS, Flag, temp;

void Write_LCD_Nibble(unsigned short N){
    LATCbits.LATC12 = 1; //Enable pin
 // ****** Write RS *********
    LATBbits.LATB14 = 0; //Clock pin
    LATBbits.LATB15 = RS; //Data pin
    LATBbits.LATB14 = 1; //Clock pin
    LATBbits.LATB14 = 0; //Clock pin
 // ****** End RS Write
 
 // Shift in 4 bits
    Mask = 8;
     for (t=0; t<4; t++){
        Flag = N & Mask;
        
        if(Flag==0){ 
            LATBbits.LATB15 = 0; //Data pin
        }else{
            LATBbits.LATB15 = 1; //Data pin
        }
        LATBbits.LATB14 = 1; //Clock pin
        LATBbits.LATB14 = 0; //Clock pin
        Mask = Mask >> 1;
    }
  // One more clock because SC and ST clks are tied
    LATBbits.LATB14 = 1; //Clock pin
    LATBbits.LATB14 = 0; //Clock pin
    LATBbits.LATB15 = 0; //Data pin
    LATCbits.LATC12 = 0; //Enable pin
    LATCbits.LATC12 = 1; //Enable pin
}
// ******* Write Nibble Ends

void Write_LCD_Data(unsigned short D){
    RS = 1; // It is Data, not command
    Low_Nibble = D & 15;
    High_Nibble = D/16;
    Write_LCD_Nibble(High_Nibble);
    Write_LCD_Nibble(Low_Nibble);
 }
 
void Write_LCD_Cmd(unsigned short C){
    RS = 0; // It is command, not data
    Low_Nibble = C & 15;
    High_Nibble = C/16;
    Write_LCD_Nibble(High_Nibble);
    Write_LCD_Nibble(Low_Nibble);
}
 
void Initialize_LCD(){
    __delay_ms(50);
    Write_LCD_Cmd(0x20); // Wake-Up Sequence
    __delay_ms(50);
    Write_LCD_Cmd(0x20);
    __delay_ms(50);
    Write_LCD_Cmd(0x20);
    __delay_ms(50);
    Write_LCD_Cmd(0x28); // 4-bits, 2 lines, 5x7 font
    __delay_ms(50);
    Write_LCD_Cmd(0x0C); // Display ON, No cursors
    __delay_ms(50);
    Write_LCD_Cmd(0x06); // Entry mode- Auto-increment, No Display shifting
    __delay_ms(50);
    Write_LCD_Cmd(0x01);
    __delay_ms(50);
}

//Sets the position of the LCD to start writing at
void Position_LCD(unsigned short x, unsigned short y){
    temp = 127 + y;
    if (x == 2){
        temp = temp + 64;
    }
    Write_LCD_Cmd(temp);
}

//Writes to the LCD
void Write_LCD_Text(char *StrData){
    q = strlen(StrData);
    for (p = 0; p<q; p++){
        temp = StrData[p];
        Write_LCD_Data(temp);
    }
}

// initialize UART set up 
void UART_Module(void) {
    __builtin_write_RPCON(0x0000);     // unlock registers
    _U1RXR = 71;                       // Remap RX pin to RP70
    _RP70R = 1;                        // Remap TX pin to RP1
    TRISDbits.TRISD0 = 0;              // set wifi_en to output
    LATDbits.LATD0 = 0;                // disable wifi 
    TRISBbits.TRISB10 = 0;             // set wifi_rst to output 
    __delay_ms(1000);                  // delay for 1 sec
    
    
    U1MODEbits.UARTEN =0;              //UART disable
    U1MODEHbits.BCLKSEL = 0;           //BAUD CLK source set to Fosc/2
    U1MODEHbits.BCLKMOD = 0;           //BAUD Clock Generation Mode Select Bit 
    U1MODEbits.BRGH = 1;
    U1BRG = 8;                         // BRGVal = 8, 4% error
    U1STAHbits.UTXISEL0 = 0;           // Interrupt after one TX character is transmitted
    U1STAHbits.UTXISEL1 = 0;  
    U1STAHbits.URXISEL = 0b00;         // Interrupt after 1 word in buffer
    U1MODEHbits.STSEL = 0;             // 1 Stop bit , No parity
    
    U1MODEbits.URXEN = 0;              // Disable Rx
    U1MODEbits.UTXEN = 0;              // Disable Tx
    
    IEC0bits.U1RXIE = 1;               // enable Receiver interrupt 
    IEC0bits.U1TXIE = 1;               // enable Transmitter interrupt
    IFS0bits.U1TXIF = 0;               // clear U1 TX interrupt flag
    IFS0bits.U1RXIF = 0;               // clear U1 RX interrupt flag
    IPC3bits.U1TXIP = 2;               // TX highest priority
    IPC2bits.U1RXIP = 6;               // RX Priority
    U1MODEbits.URXEN = 1;              // UART RX enable
    U1MODEbits.UTXEN = 1;              // UART TX enable 
    U1MODEbits.UARTEN =1;              // UART enable 
    __delay_ms(500);                   // Delay 0.5 sec
    LATBbits.LATB10 = 0;               // reset WIFI
    LATDbits.LATD0 = 1;                // enable WIFI
    __delay_ms(500);                   // Delay 0.5 sec
    
}

// function to send characters 1 by 1
void charSend(char c){
    transmit = c;                  // Load global transmit variable with function parameter
    U1TXREG = transmit;            // Load Tx Buffer with global transmit variable
   
    while(U1STAbits.TRMT == 0);    // Wait for Tx shifter
    while(!U1STAHbits.UTXBE);      // Wait for Tx Buffer to empty
}

void uart_send_s(char* s){
    while(*s){
        char d = *s;
        charSend(d);
        s++;
    }
    Nop();
    charSend(0xd);
    charSend(0xa);
    
}
void uart_send_s_tcp_ip(char s){
        charSend(s);
}


void WIFI_Module(void){
    uart_send_s(atc0);
    __delay_ms(4000);
    ATcount++;
    uart_send_s(atc1);
    __delay_ms(3000);
    ATcount++;
    uart_send_s(atc2);
    __delay_ms(10000);
    i = 0;
    ATcount++;
   uart_send_s(atc3);
   __delay_ms(3000);
   Position_LCD(1,1);
   Write_LCD_Text("WIFI Connected");
   __delay_ms(2000); 
    
   while(PORTDbits.RD11 == 1);
   Nop();
   __delay_ms(1000);
   uart_send_s(atc4);
   __delay_ms(5000);
   
   Position_LCD(1,1);
   Write_LCD_Text("Server Connected");
   

   
   uart_send_s(atc6);
   __delay_ms(2000);
 
     
}


unsigned short int Array2[50];
int k;
void DMA_INIT(void){
     for (k=50;k>40;k--)
    {
        Array2[k]=0; //fill with 0
    }
    DMACONbits.DMAEN=1;
    DMACONbits.PRSSEL=1;
    DMAH=0x5000; //set lower and upper address limit
    DMAL=0x850;
    DMASRC0=(unsigned short int)& RMS; // load the source address
    DMADST0=(unsigned short int)& Array2; // load destination address
    DMACNT0=100; // 100 Transaction per trigger
    DMACH0=0;
    DMACH0bits.SAMODE=1; //Source address increment mode
    DMACH0bits.DAMODE=1; //Destination address increment mode
    DMACH0bits.TRMODE=3; //Transfer mode Repeat continous
    DMACH0bits.RELOAD=1; //Reload Source and Destination Address
    DMACH0bits.CHEN=1; //Channel enable
    IFS0bits.DMA0IF=0;
}

void DMA_Trigger(void){
     DMACH0bits.CHREQ=1; //TIGGER
        while(!IFS0bits.DMA0IF);
        /* DO Something with Values
           Send to server??*/
        uart_send_s_tcp_ip((char)rollingAverage);
        for (k=50;k>40;k--) //Clear the Destination Memory
        {
        Array2[k]=0; //fill with 0
            }
        IFS0bits.DMA0IF=0;
        PORTAbits.RA0=0;
        DMAINT0bits.DONEIF=0;
        DMAINT0bits.HALFIF=0;
}

//converts the ADC reading to an analog voltage
double convertADC(unsigned int adcVal){
    double mV = 0; //mV value
    double sysVoltage = 5; //system voltage
    double adcRes = 4096; //ADC resultion (12-bit res -> 2^12 = 4096)
    
    mV = 1000 * (sysVoltage * (adcVal/adcRes));
    
    return mV;
}

int main(void){
    UART_Module();
    DMA_INIT();
    //(1) Configure the I/O pins to be used as analog inputs. (for ADC)
    ANSELCbits.ANSELC3 = 1; //(AN15/RC3) -> CS_2
    TRISCbits.TRISC3 = 1; // AN15/RC3 connected the shared core
    
    //Setup the ADC
    setupADC();
    //(11) Enable ADC the module. (calibrate not required for our device)
    enableADC();
    
    //(13) Set triggers (read conversion value)
    //TRIGGER - common software trigger
    ADTRIG3Hbits.TRGSRC15 = 00001; // common software trigger source selected for AN15
    ADCON3Lbits.CNVCHSEL = 15; //selected channel (AN15))
    
    //Digital Outputs(for LCD)
    TRISBbits.TRISB15 = 0; //output (Serial Data Input pin of shift registers) (INT_2)
    TRISCbits.TRISC12 = 0; //output (Enable pin on LCD) (PWM_2)
    TRISBbits.TRISB14 = 0; //output (Clock bit of shift register) (RX_2)
    //LED Pin
    TRISCbits.TRISC8 = 0; //Digital output (SDA_2)
    
    //Digital Inputs
    //Button Pins
    TRISCbits.TRISC9 = 1; //Digital Input (SDL_2) wifi button
    ANSELDbits.ANSELD11 = 0; // setting AN pin to a digital pin (RST_2)
    TRISDbits.TRISD11 = 1;   // setting AN pin to an output (RST_2) server button
    
    Initialize_LCD();
    
    while(buttonPressed == 0){
        if(PORTCbits.RC9 == 0){
            buttonPressed = 1;
            WIFI_Module();
        } 
    }
    
    while(1){
        
        //TIMER INITS
        T1CON = 0x0; // Stop timer and clear control register,
                    // set pre-scaler at 1:1, internal clock source

        TMR1 = 0x0; // Clear timer register
        PR1 = 0xFFFF; // Load period register - end time

        T1CONbits.TCKPS = 1; // set to 1:8 pre-scale value

        // Interrupt nesting enabled here 
        INTCON1bits.NSTDIS = 0;
        // Set Timer1 interrupt priority to 2  
        IPC0bits.T1IP = 2;
        // Reset Timer1 interrupt flag 
        IFS0bits.T1IF = 0;
        // Enable Timer1 interrupt 
        IEC0bits.T1IE = 1;
        
        INTCON2bits.GIE = 1;
        
        T1CONbits.TON = 1; // Start timer
        
        int flag = 0;
        
        while(flag == 0){
            if (TMR1 > 0xFFFD){ // 0x4E20 = 10 sec, 0x9C4 = 1.25 sec
                // took too long        
                T1CONbits.TON = 0; // end timer
                IFS0bits.T1IF = 1; // generate timer1 interrupt
                timerCounter++;
                flag = 1;
                break;
            }
        }
        
        if (buttonPressed == 1){
            LATCbits.LATC8 = 1; // HIGH (corresponding to CS pin of mikroBUS B)
        }
        else{
            LATCbits.LATC8 = 0; // LOW (corresponding to CS pin of mikroBUS B)
        }
    }   
}


//TIMER1 Interrupt
void __attribute__((__interrupt__,no_auto_psv)) _T1Interrupt(void) {
    if (timerCounter == 15){
        ADCON3Lbits.CNVRTCH = 1; //start sampling ADC

        ADCON3Lbits.SWCTRG = 1; //trigger ADC
        while((ADSTATLbits.AN15RDY) == 0){
                 Nop();
                 Nop();
        }
        
        //NEW STUFF
        char mV_Val[20]; //char array to display the voltage value to the LCD
        mV_Value = convertADC(dataAN15); //get the analog voltage value of the ADC
        sprintf(mV_Val, "Voltage: %.2fmV     ", mV_Value);
        Position_LCD(1,1);
        Write_LCD_Text(mV_Val);
        //END OF NEW STUFF
        
        //Calculating the average RMS
        int cnt;
        float total;
        RMS[49] = mV_Value * mV_Value;
        for(cnt = 0; cnt < 49; cnt++){
            RMS[cnt] = RMS[cnt+1];
            total += RMS[cnt];
            if (RMS[cnt] != 0){
                count++;
            }
        }
        if (count == 0 || count < 50){
            rollingAverage = sqrt(total/50);
        }
        else{
            rollingAverage = sqrt(total/count);
        }
        DMA_COUNT++;
        if (DMA_COUNT > 9){
            DMA_Trigger();
            DMA_COUNT = 0;
        }
        total = 0;
        count = 0;
        
        sprintf(powerVal,"Avg. RMS: %.2f   ", (float)rollingAverage); 
        Position_LCD(2,1);
        Write_LCD_Text(powerVal);
        
//        //Display ADCBUF15 Value to LCD
//        sprintf(bufval,"Buffer: %.2f   ", (float)dataAN15); 
//        Position_LCD(2,1);
//        Write_LCD_Text(bufval);
        
        for(cnt = 0; cnt < 199; cnt++){
            powerVal[cnt] = 0;
        }
        timerCounter = 0;
    }
    // clears Timer1 interrupt
    IFS0bits.T1IF = 0; 
    INTCON2bits.GIE = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0;    // clear TX interrupt flag  
   
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    /**/
    if(ATcount < 3){
        if(i == 100){
            i = 0;
        }
       receive[i] = 0b01111111 & U1RXREG;   // isolate first 7 bits 
        i++;
    }
    else {
        if(j == 50){
            
            j = 0;
        }
        
        get_Internet[j] = 0b01111111 & U1RXREG; 
        if(get_Internet[j] == ">"){
            server = 1;
        }
        j++;
    }
       
    IFS0bits.U1RXIF = 0;    // clear RX interrupt flag
}    

void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void){
    Nop();
    Nop();
    IFS0bits.DMA0IF=0;
}

// ADC AN15 ISR
void __attribute__((interrupt ,no_auto_psv)) _ADCAN15Interrupt(void) {
    dataAN15 = ADCBUF15;
    IFS6bits.ADCAN15IF = 0; //clear interrupt flag
    Nop();
    ADCON3Lbits.SWCTRG = 1;
    Nop();   
}
