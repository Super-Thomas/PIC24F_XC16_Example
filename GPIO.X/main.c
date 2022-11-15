#include <xc.h>
#include <stdio.h>
#define FCY 16000000UL
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
    
    PPSOutput(PPS_RP17, PPS_U2TX);
    PPSInput(PPS_U2RX, PPS_RP10);
    
    PPSLock;
}

void GPIO_Init(void)
{
    TRISAbits.TRISA15 = 1;
    PORTAbits.RA15 = 1;
}

void UART2_Init(void)
{
    U2BRG = 34;
    U2MODE = 0x8008;
    U2STA = 0x0400;
}

uint8_t g_u8iButton = 1;
uint8_t g_u8Array[8];

 void DI(void)
 {
 	INTCON1bits.NSTDIS = 1; // Interrupt nesting is disabled
 }
 
 void EI(void)
 {
 	INTCON1bits.NSTDIS = 0; // Interrupt nesting is enabled
 }
 
 void OWWriteByte(uint8_t u8Byte)
{
    uint8_t i;
	DI();
    
	for (i=0; i<8; i++)
	{
		g_u8Array[i] = (u8Byte >> i) & 1;
	}
    
	for (i=0; i<8; i++)
	{
        TRISAbits.TRISA15 = 0;
		PORTAbits.RA15 = 0;
		__delay_us(15);

		if (g_u8Array[i])
            PORTAbits.RA15 = 1;

		__delay_us(15);
		PORTAbits.RA15 = 1;
        
		__delay_us(15);
        TRISAbits.TRISA15 = 1;
	}
    
	EI();
}

void GetUID(void)
{
    uint8_t i, j;
    
	for (i=0; i<8; i++)
        g_u8Array[i] = 0;
	
    DI();
    TRISAbits.TRISA15 = 0;
	PORTAbits.RA15 = 0;
	__delay_us(600);
	PORTAbits.RA15 = 1;
    TRISAbits.TRISA15 = 1;

	__delay_us(100);
	if(PORTAbits.RA15 == 1)
	{
		EI();
		return ;
	}
    
	__delay_us(80);
	EI();

	OWWriteByte(0x33);

	for (i=0; i<8; i++)
	{
		g_u8Array[i] = 0;
		DI();
        
		for (j=0; j<8; j++)
		{
            TRISAbits.TRISA15 = 0;
			PORTAbits.RA15 = 0;
            __delay_us(10);

			PORTAbits.RA15 = 1;
            TRISAbits.TRISA15 = 1;
            __delay_us(10);

			uint8_t u8Bit = PORTAbits.RA15;
			g_u8Array[i] |= (u8Bit ? 1 : 0) << j;

			__delay_us(10);
		}
        
		EI();
	}
}

uint8_t CalciButtonCrc(void)
{
	uint8_t i, j, u8Crc = 0;
    
	for (i=0; i<8; i++)
	{
		u8Crc ^= g_u8Array[i];
		for(j=0; j<8; j++)
		{
			if(u8Crc & 0x01)
				u8Crc = (u8Crc >> 1) ^ 0x8c;
			else
				u8Crc >>= 1;
		}
	}
    
	return u8Crc;
}

static uint8_t g_u8CrcValue = 0;

uint8_t ReadKey(void)
{
	__delay_ms(2);
	if  (PORTAbits.RA15 == 1) // High
	{
		// Write Command.
		GetUID();
		g_u8CrcValue = CalciButtonCrc();
        printf("CRC: %02x\n", g_u8CrcValue);
        printf("%02x %02x %02x %02x %02x %02x %02x %02x \n", g_u8Array[0], g_u8Array[1], g_u8Array[2], g_u8Array[3], g_u8Array[4], g_u8Array[5], g_u8Array[6], g_u8Array[7]);
        
		if((g_u8CrcValue == 0) && (g_u8Array[0] == 0x01))
			return 0; // TRUE
	}
    
	return 1; // FALSE
}

void CheckiButton(void)
{
	printf("%d \n", g_u8iButton);
    
	switch (g_u8iButton)
	{
		case 1: // Maybe it is worked or you touched.
		case 2: // Try to read for second.
		case 3:
		case 4:
		case 5:
		{
			if (ReadKey() == 0) // TRUE
			{
				
			}
			else
				g_u8iButton++;
		}
		break;
		case 6:
			DI();
			g_u8iButton = 0;
			EI();
			break;
	}
}

extern int __C30_UART = 2;

int main(void)
{
    uint8_t u8ToggleFlag = 0;
    
    PPS_Init();
    GPIO_Init();
    UART2_Init();
    
    while (1)
    {
        printf("Hello World!\n");
        CheckiButton();
        //u8ToggleFlag = !u8ToggleFlag;
        //PORTAbits.RA15 = u8ToggleFlag;

        __delay_ms(1000);
    }
    
    return 0;
}