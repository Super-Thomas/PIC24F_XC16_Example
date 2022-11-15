#include <xc.h>
#include <stdio.h>
#define FCY 32000000UL
#include <libpic30.h> // __delayXXX() functions macros defined here
#include <pps.h>

// CONFIG3
#pragma config WPFP = WPFP511           // Write Protection Flash Page Segment Boundary (Highest Page (same as page 170))
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable bit (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS         // Configuration Word Code Page Protection Select bit (Last page(at the top of program memory) and Flash configuration words are not protected)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select bit (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = XT             // Primary Oscillator Select (XT oscillator mode selected)
#pragma config I2C2SEL = PRI            // I2C2 Pin Select bit (Use Default SCL2/SDA2 pins for I2C2)
#pragma config IOL1WAY = ON             // IOLOCK One-Way Set Enable bit (Write RP Registers Once)
#pragma config OSCIOFNC = OFF           // Primary Oscillator Output Function (OSCO functions as CLKO (FOSC/2))
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-safe Clock Monitor are disabled)
#pragma config FNOSC = PRI              // Oscillator Select (Primary oscillator (XT, HS, EC))
#pragma config IESO = ON                // Internal External Switch Over Mode (IESO mode (Two-speed start-up) enabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Standard Watchdog Timer is enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config ICS = PGx2               // Comm Channel Select (Emulator functions are shared with PGEC2/PGED2)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)

void PPS_Init(void)
{
    PPSUnLock;
    
    PPSInput(PPS_SDI1, PPS_RPI44);
    PPSOutput(PPS_RP30, PPS_SS1OUT);
    PPSOutput(PPS_RP16, PPS_SCK1OUT);
    PPSOutput(PPS_RP10, PPS_SDO1);
    
    PPSOutput(PPS_RP17, PPS_U2TX);
    PPSInput(PPS_U2RX, PPS_RP10);
    
    PPSLock;
}

void SPI1_Init(void)
{
    SPI1STATbits.SPIEN 		= 0;	// disable SPI port
    SPI1STATbits.SPISIDL 	= 0; 	// Continue module operation in Idle mode
    SPI1BUF 				= 0;   	// clear SPI buffer data Master (SPI1BUF==SSPBUF as in the powerpoint)
    IFS0bits.SPI1IF 		= 0;	// clear interrupt flag
    IEC0bits.SPI1IE 		= 0;	// disable interrupt
    SPI1CON1bits.DISSCK		= 0;	// Internal SPIx clock is enabled
    SPI1CON1bits.DISSDO		= 0;	// SDOx pin is controlled by the module (enable SDO)
    SPI1CON1bits.MODE16 	= 0;	// set in 16-bit mode, clear in 8-bit mode
    SPI1CON1bits.SMP		= 0;	// Input data sampled at middle of data output time
    SPI1CON1bits.CKP 		= 1;	// CKP and CKE is subject to change ...
    SPI1CON1bits.CKE 		= 0;	// ... based on your communication mode.
	SPI1CON1bits.MSTEN 		= 1; 	// 1 =  Master mode; 0 =  Slave mode
	SPI1CON1bits.SPRE 		= 4; 	// Secondary Prescaler = 4:1
	SPI1CON1bits.PPRE 		= 2; 	// Primary Prescaler = 4:1
    SPI1CON2 				= 0;	// non-framed mode
    
    SPI1STATbits.SPIEN 		= 1; 	// enable SPI port, clear status
}

void UART2_Init(void)
{
    U2BRG = 34;
    U2MODE = 0x8008;
    U2STA = 0x0400;
}

uint8_t SPI1_Exchange(uint8_t *pu8TxByte)
{
    uint8_t u8ReadByte = 0x00;
    uint8_t u8Dummy;
    
    IFS0bits.SPI1IF = 0;
    u8Dummy = SPI1BUF;
    SPI1BUF = *((uint8_t*)pu8TxByte);
    while (IFS0bits.SPI1IF == 0);
    u8ReadByte = (SPI1BUF & 0xFF);
    return u8ReadByte;
}

uint8_t SPI1_Exchange8bit(uint8_t u8TxData)
{
    uint8_t u8RxData = 0x00;
    u8RxData = SPI1_Exchange(&u8TxData);
    return (u8RxData);
}

uint16_t SPI1_StatusGet()
{
    return (SPI1STAT);
}

// Temp Sensor: Maxim MAX31855
uint16_t Read_TempSensor(uint16_t *pu16HData, uint16_t *pu16LData)
{
    uint16_t u16Status = 0;
    uint8_t u8ReadData[4] = { 0, };
    uint8_t u8WriteData = 0;
    uint16_t u16Data[2] = { 0, };
    
    u8ReadData[0] = SPI1_Exchange8bit(u8WriteData);
    u8ReadData[1] = SPI1_Exchange8bit(u8WriteData);
    u8ReadData[2] = SPI1_Exchange8bit(u8WriteData);
    u8ReadData[3] = SPI1_Exchange8bit(u8WriteData);
    
    //printf("%02x ", u8ReadData[0]);
    //printf("%02x ", u8ReadData[1]);
    //printf("%02x ", u8ReadData[2]);
    //printf("%02x ", u8ReadData[3]);
    
    u16Status = SPI1_StatusGet();
    //printf("%02x ", u16Status);
    
    //printf("\n");
    
    u16Data[0] |= (uint16_t)((u8ReadData[0] << 8) & 0xFF00);
    u16Data[0] |= (uint16_t)((u8ReadData[1]) & 0x00FF);
    u16Data[1] |= (uint16_t)((u8ReadData[2] << 8) & 0xFF00);
    u16Data[1] |= (uint16_t)((u8ReadData[3]) & 0x00FF);
    
    //printf("%04x\n", u16Data[0]);
    //printf("%04x\n", u16Data[1]);
    
    *pu16HData = u16Data[0];
    *pu16LData = u16Data[1];

    return u16Status;
}

extern int __C30_UART = 2;

int main(void)
{
    uint16_t u16RawHData = 0x00;
    uint16_t u16RawLData = 0x00;
    uint8_t u8ExtTempSign = 0;
    uint8_t u8IntTempSign = 0;
    int16_t s16ExtTempRaw = 0;
    int16_t s16IntTempRaw = 0;
    
    PPS_Init();
    SPI1_Init();
    UART2_Init();
    
    TRISFbits.TRISF2 = 0;
    PORTFbits.RF2 = 1;
    
    while (1)
    {
        PORTFbits.RF2 = 0;
        if (Read_TempSensor(&u16RawHData, &u16RawLData) == 0x8000)
        {
            //printf("Raw Data: %04x %04x\n", u16RawHData, u16RawLData);
            
            if (u16RawLData & (1 << 2))
            {
                // SCV Fault
                printf("SCV Fault\n");
            }
            if (u16RawLData & (1 << 1))
            {
                // SCG Fault
                printf("SCG Fault\n");
            }
            if (u16RawLData & (1 << 0))
            {
                // OC Fault
                printf("OC Fault\n");
            }
            if (u16RawHData & (1 << 0)) // (1 << 16)
            {
                // Fault
                printf("Fault\n");
            }
            
            if (u16RawHData & (1 << 15)) // (1 << 31)
            {
                // Ext Temp Sign Bit
                u8ExtTempSign = 1;
            }
            if (u16RawLData & (1 << 15))
            {
                // Int Temp Sign Bit
                u8IntTempSign = 1;
            }
            
            s16ExtTempRaw = (u16RawHData >> 2) & 0x1FFF;
            s16IntTempRaw = (u16RawLData >> 4) & 0x7FF;
            
            if (u8ExtTempSign)
            {
                s16ExtTempRaw = ~(s16ExtTempRaw & 0x1FFF);
            }
            if (u8IntTempSign)
            {
                s16IntTempRaw = ~(s16IntTempRaw & 0x7FF);
            }
            
            printf("Ext Temp: %.2f\n", (double)(s16ExtTempRaw * 0.25f));
            printf("Int Temp: %.2f\n", (double)(s16IntTempRaw * 0.0625f));
        }
        PORTFbits.RF2 = 1;

        __delay_ms(200);
    }
    
    return 0;
}