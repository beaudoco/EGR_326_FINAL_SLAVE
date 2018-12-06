/*********************************************************************
 * Name            : Collin Beaudoin & Larson
 * Date            : October 26, 2018
 * Class           : EGR 326 Section 902
 * Instructor      : Dr. Nabeeh Kandalaft
 * Description     : This Program uses hall effect sensor to detect
 * if a  magnetic field if present or not.
 ********************************************************************/

#include "driverlib.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "COMMONCLOCKS.h"
//#include "ST7735.h"
#include <msp.h>
#include <stdlib.h>

#define SLAVE_ADDRESS 0x48

//uint16_t textColor = ST7735_WHITE;
//uint16_t bgColor   = ST7735_BLACK;
uint8_t textSize   = 2;

volatile uint8_t flag;      // Used to check SysTick Flag

float User_Speed;           // Value inserted by the user for the controlled motor speed
int User_Speed_Count = 0;
int Seconds          = 0;   // Used to determine how many seconds have passed
int Magnet_Counter   = 0;
int Calculated_Speed = 0;
int previous_Speed = 0;

int sysTikToggleSpeed    = 15000;
int debounceFlag         = 0;
int debounceFlag2        = 0;
int colorState           = 0;
int msDelay              = 25;
int count                = 0;
uint8_t key;

char RXData;
int i = 0;
int pos = 0;

uint32_t interruptButton1Status = 0;
uint32_t interruptButton2Status = 0;

void initClocks();
void Configure(void);

void port_Init(void);
void driveMotor(int);
void timer32setup();

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // Halt Watch-dog


    port_Init();
    COMMONCLOCKS_sysTick_Init();

    timer32setup();
    initClocks();
    Configure();
//    ST7735_InitR(INITR_REDTAB);
//    ST7735_FillScreen(0);
//    ST7735_FillScreen(bgColor);
//    ST7735_SetRotation(3);

    int i;

    for(i=0;i<(400*4);i++) {
        driveMotor(0);
    }

    while(1)
    {
        if(flag)
        {
            flag = 0;                   //Setting SysTick Flag to zero

            if(Seconds >= 8) //If four seconds in real time has passed
            {
                if(Magnet_Counter == 0)
                    Calculated_Speed = 0; //If a magnet has not passed in a second, make the calculated speed zero
                else {

                    Calculated_Speed = (((2.04*3.14) * ((Magnet_Counter)*15)) * 60)/5280; //Calculating the display speed

//                    Calculated_Speed = 35;

                    if(Calculated_Speed > previous_Speed) {
                        int j = Calculated_Speed-previous_Speed;
                        previous_Speed = Calculated_Speed;
                        for(i=0;i<(j*4);i++) {
                            driveMotor(1);
                        }
                        if(pos > 0)
                            pos = pos - (j);
                    }
                    if(Calculated_Speed < previous_Speed) {
                        interruptButton1Status = 0;
                        int j = previous_Speed-Calculated_Speed;
                        previous_Speed = Calculated_Speed;
                        for(i=0;i<(j*4);i++) {
                            driveMotor(0);
                        }
                        if(pos < 170)
                            pos = pos + (j);
                    }

                    char buf[12];
                    //ST7735_FillRect(2,8,10,2,bgColor);
                    sprintf(buf, "MPH : %d", Calculated_Speed);
                    //ST7735_DrawString(2, 8,buf, textColor);
                    printf("%s \n",buf);
                    Seconds        = 0;
                    Magnet_Counter = 0;
                }
            }
        }
    }
}

void Configure(void)
{
    /* Configuring SysTick to trigger at 3000000 (MCLK is 3MHz so this will make it toggle every 0.5s) */
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(3000000);
//    MAP_SysTick_enableInterrupt();

    /* Enabling MASTER interrupts */
    MAP_Interrupt_enableMaster();

    /*Configuring MOTOR */
    P5->DIR  |= BIT6 ;                                                              // Configured as output
    P5->SEL0 |= BIT6 ;                                                              // Configure P5 for GPIO
    P5->SEL1 &= ~(BIT6);                                                            // Configure P5 for GPIO

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P6,GPIO_PIN6);                   // Sets External PB1
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P6,GPIO_PIN7);                   // Sets External PB2
    GPIO_clearInterruptFlag(GPIO_PORT_P6,GPIO_PIN6);                                // Clear interrupt
    GPIO_clearInterruptFlag(GPIO_PORT_P6,GPIO_PIN7);                                // Clear interrupt
    GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN6);                                  // Enable interrupt
    GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN7);                                  // Enable interrupt

    GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION); // Set interrupt to trigger on a falling edge.
    GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN7, GPIO_HIGH_TO_LOW_TRANSITION); // Set interrupt to trigger on a falling edge.

    Interrupt_enableInterrupt(INT_PORT6);                                           // Enable interrupt

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN5);                  // Configuring P3.0 as peripheral input for capture
    GPIO_clearInterruptFlag(GPIO_PORT_P2,GPIO_PIN5);                                // Clear interrupt
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN5);                                  // Enable interrupt
    Interrupt_enableInterrupt(INT_PORT2);                                           // Enable interrupt

    Interrupt_enableMaster();
}

/*************************************************
 * Initializes MCLK to run from external crystal.
 * Code taken from lecture notes
 ************************************************/
void initClocks()
{
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN3 | GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION); // Configuring pins for XTL usage
    MAP_CS_setExternalClockSourceFrequency(32000,48000000);                                                             // Setting external clock frequency
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);                                                                            // Starting HFXT
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    MAP_CS_startHFXT(false);
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);                                             // Initializing MCLK to HFXT (48MHz)
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);                                            // Setting SMCLK to 12MHz (HFXT/4)
}

void port_Init(void) {

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);                       // Configuring P4.1-.4 as an output
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);                       // Configuring P4.1-.4 as an output
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN3);                       // Configuring P4.1-.4 as an output
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN4);                       // Configuring P4.1-.4 as an output

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);                   //
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);                   //
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);                    //
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);                    //

}

void driveMotor(int right) {
    switch(count) {

    case 1:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);                   //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);                   //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);                    //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);                    //
        COMMONCLOCKS_sysTick_delay_3MHZ(msDelay);
        break;

    case 2:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);                   //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN3);                   //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);                    //
        COMMONCLOCKS_sysTick_delay_3MHZ(msDelay);
        break;

    case 3:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);                    //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN3);                   //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);                   //
        COMMONCLOCKS_sysTick_delay_3MHZ(msDelay);
        break;

    case 4:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);                   //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);                    //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);                    //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);                   //
        COMMONCLOCKS_sysTick_delay_3MHZ(msDelay);
        break;

    default:
        //printf("Collin doesn't know how to code");
        break;
    }

    if(right) {
        if(count == 4)
            count = 0;
        else
            count++;
    }
    else {
        if(count == 0)
            count = 4;
        else
            count--;
    }

}

//Interrupt checking for button presses within Port 1
void PORT6_IRQHandler(void)
{
    uint32_t status;                                            // Status of PB
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P6);  // Enabling Interrupt
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P6, status);          // Clear Flag on Port 1

    if(status & GPIO_PIN6)                                      // Setting Flag for PB 1
    {
        User_Speed_Count++;                                     // Increments when user wants to change the speed
    }
    else if(status & GPIO_PIN7)                                 // Setting Flag for PB 2
    {
        User_Speed_Count = 0;                                   // Setting the count to zero, displaying a stop
    }
}

//Interrupt checking for button presses within Port 3
void PORT2_IRQHandler(void)
{
    uint32_t status; //Status of PB
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2); //Enabling Interrupt
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, status);  //Clear Flag on Port 3

    if(status & GPIO_PIN5) //If a magnet is seen by Hall Effect
    {
        Magnet_Counter++;  //Increments every time a magnet is sensed
    }
}

void timer32setup(void) {
    TIMER32_1->CONTROL = 0b11100011;
    NVIC_EnableIRQ(T32_INT1_IRQn);
    TIMER32_1->LOAD = 24000000; //loads the counter to interupt every half second
}

void T32_INT1_IRQHandler() {
    TIMER32_1->INTCLR = 1;
    flag = 1;
    Seconds++;

    TIMER32_1->LOAD = 24000000; //Re-load the same interrupt time
}
