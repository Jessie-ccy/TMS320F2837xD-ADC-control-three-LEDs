/*
 1101_AC & DC DRIVERS
 proj.5 ADC
 input_ - ADCINA0 is on  (PIN 30)
 output_ - ePWM2A is on  (PIN 38)
 output_ - ePWM1A is on  (PIN 40)
 output_ - ePWM3A is on  (PIN 36)
 */
#include "F28x_Project.h"
#include "math.h"

void ConfigureADC(void);
void ConfigureSOC(void);

void InitEPwm1(void);
void InitEPwm2(void);
void InitEPwm3(void);
void SetupADCEpwm(void);

interrupt void adca1_isr(void);

//
// Defines
//
#define RESULTS_BUFFER_SIZE 20 //data pool size, 20 data
#define PI 3.14159
//
// Globals
//
Uint16 AdcaResults[RESULTS_BUFFER_SIZE]; //ADC data pool
Uint16 resultsIndex = 0;    //data input index
Uint16 DutyTable[101];  //output table for PWM
Uint16 set_period;  //output data, work with PWM period TimeBase
const Uint16 adcres = 4095;     //2^12-1
const Uint16 TimeBase = 5000;  //period=0.1ms
Uint16 tableIndex = 0;
Uint16 sumindex = 0;

/**
 * main.c
 */
void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

// Step 2. Initialize GPIO pins for ePWM2
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();

    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;  //enable epwm1 clock
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;  //enable epwm2 clock
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;  //enable epwm3 clock

//
// Configure the ADC and power it up
//
    ConfigureADC();
//
// Configure the ePWM SOC
//
    ConfigureSOC();
//
// Setup the ADC for ePWM triggered conversions
//
    SetupADCEpwm();
//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();

//
// Map ISR functions
//
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    EDIS;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Closed synchronize between base clock and all enabled ePWM modules
    EDIS;

//
// InitEPwm2 - Initialize EPWM2 configuration
//
    InitEPwm1();
    InitEPwm2();
    InitEPwm3();

//
// sync ePWM
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

//
// Enable global Interrupts and higher priority real-time debug events:
//
    IER |= M_INT1; //Enable group 1 interrupts

//
// enable PIE interrupt
//
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    EINT;
    // Enable Global interrupt INTM
    ERTM;
    // Enable Global realtime interrupt DBGM

    Uint16 i; //for index, only used in Main
    for (i = 0; i <= 100; i++)
    {
        DutyTable[i] = TimeBase * sin(PI / 2 * i / 100); // build sin table 0 to 90 degree
    }

    while (1)
    {
        //empty
    }
}

void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000); //This delay can also be used for kind of low inefficient debounce

    EDIS;
}

//
// SetupADCEpwm - Setup ADC EPWM acquisition window
//
void SetupADCEpwm(void)
{

    //
    //Select the channels to convert and end of conversion flag
    //
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
/*
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 2;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
*/
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //0-> EOC0 is trigger for ADCINT1, will set INT1 flag

    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
}

//
// ConfigureEPWM - Configure EPWM SOC and compare values
//
void ConfigureSOC(void)                                  ///only used for SOC!!!
{
    EALLOW;
    // Assumes ePWM clock is already enabled
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;    // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 2;   // Select SOC on period
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on 1st event
    EPwm1Regs.TBPRD = TimeBase * 0.05;    // Set period to TimeBase/20
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;      // up count mode
    EDIS;
}

void InitEPwm1()                                                 ///used for LED
{
    EPwm1Regs.TBPRD = TimeBase;                   // Set timer period
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count UPDOWN
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = 1;

    EPwm1Regs.CMPA.bit.CMPA = TimeBase;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
    EPwm1Regs.AQCTLA.bit.PRD = AQ_CLEAR;

    EPwm1Regs.ETSEL.bit.INTEN = 0;                // Disable interrupt

}

void InitEPwm2()                                                 ///used for LED
{
    EPwm2Regs.TBPRD = TimeBase;                   // Set timer period
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count UPDOWN
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = 1;

    EPwm2Regs.CMPA.bit.CMPA = TimeBase;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
    EPwm2Regs.AQCTLA.bit.PRD = AQ_CLEAR;

    EPwm2Regs.ETSEL.bit.INTEN = 0;                // Disable interrupt

}

void InitEPwm3()                                                 ///used for LED
{
    EPwm3Regs.TBPRD = TimeBase;                   // Set timer period
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                     // Clear counter

    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count UPDOWN
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1;       // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = 1;

    EPwm3Regs.CMPA.bit.CMPA = TimeBase;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
    EPwm3Regs.AQCTLA.bit.PRD = AQ_CLEAR;

    EPwm3Regs.ETSEL.bit.INTEN = 0;                // Disable interrupt

}

void Reverse(int *p, int size)
{
    int i, tmp;
    for(i = 0;i < size/2;i++)
    {
        tmp = p[i] ;
        p[i] = p[size-1-i];
        p[size-1-i] = tmp;
    }
}
//
// adca1_isr - Read ADC Buffer in ISR
//
interrupt void adca1_isr(void)
{
    //
    AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT0; //catch the ADC results
    //AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT1; //catch the ADC results
    //
    if (RESULTS_BUFFER_SIZE <= resultsIndex)    //when data full enough
    {
        Uint32 sum = 0; //set 0, also check format

        for (sumindex = 0; sumindex < RESULTS_BUFFER_SIZE; sumindex++)
        {                                  //
            sum += AdcaResults[sumindex]; //sum the result, can work as filtering
        }                                     //
        tableIndex = sum * 100 / adcres / RESULTS_BUFFER_SIZE;                //

        //set_period = DutyTable[tableIndex];
        //EPwm2Regs.CMPA.bit.CMPA = TimeBase - set_period;
        EPwm1Regs.CMPA.bit.CMPA = TimeBase - DutyTable[tableIndex];
        EPwm2Regs.CMPA.bit.CMPA = TimeBase - (TimeBase - DutyTable[tableIndex]);
        EPwm3Regs.CMPA.bit.CMPA = abs(TimeBase - 2*DutyTable[tableIndex]);
        resultsIndex = 0;
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// End of file
//
