#include <xc.h>
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

volatile uint8_t g_u8TimerFlag = 0;
uint8_t g_u8TimerCount = 0;
uint8_t g_u8ToggleFlag = 0;

void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0; // Reset the interrupt flag
    g_u8TimerFlag = 1;
}

void PPS_Init(void)
{
    PPSUnLock;
    PPSOutput(PPS_RP15, PPS_OC1);
    PPSLock;
}

void Timer1_Init(void)
{
    PR1 = 12499; // 200 ms
    IPC0bits.T1IP = 1; // Set interrupt priority
    T1CONbits.TCKPS = 0x03; // Timer prescaler bits
    T1CONbits.TCS = 0; // Using FOSC/2 
    IFS0bits.T1IF = 0; // Reset interrupt flag
    IEC0bits.T1IE = 1; // Turn on the timer1 interrupt
}

void Timer2_PWM_Init(void)
{
    T2CON = 0x0000;
    PR2 = 2124;
    OC1CON2 = 0x001F;
    OC1CON1 = 0;
    OC1RS = 1062; // 66.438 us = period
    // On time
    //OC1R = 22; // 1.436 us
    //OC1R = 58; // 3.684 us
    //OC1R = 90; // 5.686 us
    //OC1R = 122; // 7.684 us
    //OC1R = 154; // 9.686 us
    //OC1R = 186; // 11.686 us
    //OC1R = 218; // 13.684 us
    //OC1R = 250; // 15.684 us
    //OC1R = 296; // 18.56 us
    OC1R = 319; // 19.998 us
}

int main(void)
{
    PPS_Init();
    Timer1_Init();
    Timer2_PWM_Init();
    
    T1CONbits.TON = 1; // Start Timer1
    T2CONbits.TON = 1; // Start Timer2
    
    while (1)
    {
        if (g_u8TimerFlag == 1) // 200 ms
        {
            g_u8TimerFlag = 0;
            g_u8TimerCount++;
            if (g_u8TimerCount > 4) // 1 sec
            {
                g_u8TimerCount = 0;
                g_u8ToggleFlag = !g_u8ToggleFlag;
                
#if 0
                // GPIO Toggle Test
                TRISBbits.TRISB11 = 0; // Set Output
                PORTBbits.RB11 = g_u8ToggleFlag;
#else
                // PWM On/Off
                if (g_u8ToggleFlag)
                {
                    OC1CON1 = 0x0006; // ON
                }
                else
                {
                    OC1CON1 = 0; // OFF
                }
#endif
            }
        }
        
    }
    
    return 0;
}