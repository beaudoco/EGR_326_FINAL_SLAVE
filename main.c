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

#define SLAVE_ADDRESS       0b1101000

#define NUM_OF_REC_BYTES    10
#define SIZE_ARRAY          25
#define NUMBER_ARRAY        5
#define CALIBRATION_START   0x000200000         // CALIBRATION START

//uint16_t textColor = ST7735_WHITE;
//uint16_t bgColor   = ST7735_BLACK;
uint8_t textSize   = 2;

volatile uint8_t flag =0;      // Used to check SysTick Flag

float User_Speed;           // Value inserted by the user for the controlled motor speed
int User_Speed_Count = 0;
int Seconds          = 0;   // Used to determine how many seconds have passed
int Magnet_Counter   = 0;
int Calculated_Speed = 0;
int Calculated_Rev = 0;
int previous_Rev = 0;
int previous_Speed = 0;

int sysTikToggleSpeed    = 15000;
int debounceFlag         = 0;
int debounceFlag2        = 0;
int colorState           = 0;
int msDelay              = 25;
int count                = 0;
int count2               = 0;
uint8_t key;

int firstRead         = 1;

char RXData;
int i = 0;
int pos = 0;

uint8_t bcdSecond, bcdMinute, bcdHour, bcdDay, bcdDate, bcdMonth, bcdYear;

uint8_t inline convertToBCD(uint8_t dec) {return (dec%10) | (((dec/10)%10) << 4);}
uint8_t inline convertFromBCD(uint8_t bcd) {return (bcd & 0x0F) + (((bcd & 0xF0)>>4) * 10);}

uint8_t row, col, value;
uint8_t key;

int firstClock = 0;
int firstFlag = 0;

char timeArr1[25];
char timeArr2[25];
char timeArr3[25];
char timeArr4[25];
char timeArr5[25];
char RTC_registers[20];

void initClocks();
void Configure(void);
void port_Init(void);
void driveMotor(int);
void driveMotor2(int);

uint16_t promptUser(char* str);
void timer32setup(void);
void iicInit(void);
void readFromSlave(void);
void writeFromMaster(void);
void setTime(void);
void getTime(uint8_t optionKey);
void printTime(void);
void printDay(uint8_t day);
int getKey(void);

const eUSCI_I2C_MasterConfig i2cConfig2 =
{
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        3000000,                                // SMCLK = 3MHz
        EUSCI_B_I2C_SET_DATA_RATE_400KBPS,      // Desired I2C Clock of 400khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
};

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // Halt Watch-dog


    port_Init();
    COMMONCLOCKS_sysTick_Init();

    timer32setup();
    iicInit();                  // Init iic to RTC
    initClocks();
    Configure();
//    ST7735_InitR(INITR_REDTAB);
//    ST7735_FillScreen(0);
//    ST7735_FillScreen(bgColor);
//    ST7735_SetRotation(3);

    //Flashes Light
    P1DIR = 0x01;
    P1OUT = 0x00;
    P1OUT ^= 0x01;

    int i;

    for(i=0;i<(400*4);i++) {
        driveMotor(0);
    }

    //TAKING OUT TACHOMETER FOR NOW

//    for(i=0;i<(400*4);i++) {
//        driveMotor2(0);
//    }

    uint8_t* addr_pointer;                                      // pointer to address in flash for reading back values

    while(1)
    {
        //MSPgets(EUSCI_A1_BASE, Buffer, BUFFER_SIZE);
        //TOGGLE THE LIGHT AND CHECK THE TEMP
        if(firstClock >= 5) {
            P1OUT ^= 0x01;
            getTime(3);
            firstClock = 0;
        }


        if(flag)
        {
            flag = 0;                   //Setting SysTick Flag to zero

            if(Seconds >= 8) //If four seconds in real time has passed
            {
                if(Magnet_Counter == 0)
                    Calculated_Speed = 0; //If a magnet has not passed in a second, make the calculated speed zero
                else {

                    Calculated_Speed = (((2.04*3.14) * ((Magnet_Counter)*15)) * 60)/5280; //Calculating the display speed
                    Calculated_Rev = (((2.04*3.14) * ((Magnet_Counter)*15)));

                    if(Calculated_Speed > previous_Speed) {
                        int j = abs((Calculated_Speed-previous_Speed)-((Calculated_Speed-previous_Speed)%10));
                        previous_Speed = Calculated_Speed;
                        for(i=0;i<(j*4);i++) {
                            driveMotor(0);
                        }
                        if(pos > 0)
                            pos = pos - (j);


                        if(Calculated_Speed >= 85 && previous_Speed<85) {
                            uint8_t i;                                                                                      // index
                            COMMONCLOCKS_sysTick_delay_48MHZ(msDelay);                                                      // Setting MCLK to 48MHz for faster programming
                            addr_pointer = CALIBRATION_START+4;                                                             // point to address in flash for saving data

                            for(i=0; i<25; i++) {                                                                           // read values in flash before programming
                                timeArr1[i] = *addr_pointer++;
                            }

                            for(i=0; i<25; i++) {                                                                           // read values in flash before programming
                                timeArr2[i] = *addr_pointer++;
                            }

                            for(i=0; i<25; i++) {                                                                           // read values in flash before programming
                                timeArr3[i] = *addr_pointer++;
                            }

                            for(i=0; i<25; i++) {                                                                           // read values in flash before programming
                                timeArr4[i] = *addr_pointer++;
                            }

                            for(i=0; i<25; i++) {                                                                           // read values in flash before programming
                                timeArr5[i] = *addr_pointer++;
                            }

                            addr_pointer = CALIBRATION_START+4;                                                             // point to address in flash for saved data

                            printTime();
                        }

                        previous_Speed = Calculated_Speed;
                    }
                    else if(Calculated_Speed < previous_Speed) {
                        int j = abs((Calculated_Speed-previous_Speed)-((Calculated_Speed-previous_Speed)%10));
                        previous_Speed = Calculated_Speed;
                        for(i=0;i<(j*4);i++) {
                            driveMotor(1);
                        }
                        if(pos < 170)
                            pos = pos + (j);
                    }

                    //TAKING OUT TACHOMETER FOR RIGHT NOW
//                    if(Calculated_Rev > previous_Rev) {
//                        int j = abs((Calculated_Rev-previous_Rev)-((Calculated_Rev-previous_Rev)%10));
//                        previous_Rev = Calculated_Rev;
//                        for(i=0;i<(j*4);i++) {
//                            driveMotor2(0);
//                        }
//                        if(pos > 0)
//                            pos = pos - (j);
//                    }
//                    else if(Calculated_Rev < previous_Rev) {
//                        int j = abs((Calculated_Rev-previous_Rev)-((Calculated_Rev-previous_Rev)%10));
//                        previous_Rev = Calculated_Rev;
//                        for(i=0;i<(j*4);i++) {
//                            driveMotor2(1);
//                        }
//                        if(pos < 170)
//                            pos = pos + (j);
//                    }

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
//    MAP_SysTick_enableModule();
//    MAP_SysTick_setPeriod(3000000);
////    MAP_SysTick_enableInterrupt();

    /* Enabling MASTER interrupts */
//    MAP_Interrupt_enableMaster();

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
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_16);                                            // Setting SMCLK to 12MHz (HFXT/4)

//    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);                                            // Setting SMCLK to 12MHz (HFXT/4)
}

void iicInit() {

    // For example, select Port 6 for I2C
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,    // Set Pin 4, 5 to input Primary Module Function,
    GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);           // P6.4 is UCB1SDA, P6.5 is UCB1SCL

    // Initializing I2C Master (see description in Driver Lib for
    MAP_I2C_initMaster(EUSCI_B1_BASE, &i2cConfig2);                  // proper configuration options)
    MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, SLAVE_ADDRESS);          // Specify slave address
    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);      // Set Master in transmit mode
    MAP_I2C_enableModule(EUSCI_B1_BASE);                            // Enable I2C Module to start operations
}

void port_Init(void) {

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0);                       // Configuring P4.1-.4 as an output
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN1);                       // Configuring P4.1-.4 as an output
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN2);                       // Configuring P4.1-.4 as an output
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN3);                       // Configuring P4.1-.4 as an output
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN4);                       // Configuring P4.1-.4 as an output
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN5);                       // Configuring P4.1-.4 as an output
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN6);                       // Configuring P4.1-.4 as an output
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN7);                       // Configuring P4.1-.4 as an output

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0);                   //
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN1);                   //
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN2);                    //
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN3);                    //
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN4);                   //
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN5);                   //
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN6);                    //
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN7);                    //


    //FOR KEYPAD
    P4SEL0 = 0x00;                                  // Port 4 set for GPIO
    P4SEL1 = 0x00;
    P4DIR  = 0X00;                                  // All bits in port 4 are setup as inputs
    P4REN |= 0b1111000;                             // Enable pull resistor on bits 3-6
    P4OUT |= 0b1111000;                             // Bits 3-6 are pull-up

}

//
//
// WILL HAVE TO ADJUST BECAUSE OF MOTORS
//
//
int getKey()
{
    for(col = 0; col < 3; col++)
    {
        P4DIR &= ~(BIT0 | BIT1 | BIT2);             // Disable all columns
        P4DIR |=  BIT(col);                         // col# is enabled
        P4OUT &= ~BIT(col);                         // Drive col# low
        COMMONCLOCKS_sysTick_delay_3MHZ(10);                          // delay 20 ms.
        row = (P4IN & 0b1111000);                   // Read the rows

        /** Wait for button to be released */
        while(!(P4IN & BIT3) | !(P4IN & BIT4) | !(P4IN & BIT5) | !(P4IN &BIT6));

        P4OUT |= BIT(col);                          // Drive col# high

        if(row != 0b1111000)                        // If a row is low a key has been pressed for the selected col
            break;
     }

    P4OUT |= BIT0 | BIT1 | BIT2;                    // Drive columns high before disabling them
    P4DIR &= ~(BIT0 | BIT1 | BIT2);                 // Disable All Columns

    /** Simple Algebra to return the correct key press  */
    if(col == 3)
        return 0;                                   // No key is pressed
    if(row == 0b0111000)                            // Row 0
        value = col + 1;
    if(row == 0b1011000)                            // Row 1
        value = 3 + col + 1;
    if(row == 0b1101000)                            // Row 2
        value = 6 + col + 1;
    if(row == 0b1110000)                            // Row 3
        value = 9 + col + 1;
    return value;
}

void driveMotor(int right) {
    switch(count) {

    case 1:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0);                   //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN1);                   //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN2);                    //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN3);                    //
        COMMONCLOCKS_sysTick_delay_3MHZ(msDelay);
        break;

    case 2:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN1);                   //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN2);                   //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN3);                    //
        COMMONCLOCKS_sysTick_delay_3MHZ(msDelay);
        break;

    case 3:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN1);                    //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN2);                   //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN3);                   //
        COMMONCLOCKS_sysTick_delay_3MHZ(msDelay);
        break;

    case 4:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0);                   //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN1);                    //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN2);                    //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN3);                   //
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

void driveMotor2(int right) {
    switch(count2) {

    case 1:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN4);                   //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN5);                   //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN6);                    //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN7);                    //
        COMMONCLOCKS_sysTick_delay_3MHZ(msDelay);
        break;

    case 2:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN4);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN5);                   //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN6);                   //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN7);                    //
        COMMONCLOCKS_sysTick_delay_3MHZ(msDelay);
        break;

    case 3:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN4);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN5);                    //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN6);                   //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN7);                   //
        COMMONCLOCKS_sysTick_delay_3MHZ(msDelay);
        break;

    case 4:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN4);                   //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN5);                    //
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN6);                    //
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN7);                   //
        COMMONCLOCKS_sysTick_delay_3MHZ(msDelay);
        break;

    default:
        //printf("Collin doesn't know how to code");
        break;
    }

    if(right) {
        if(count2 == 4)
            count2 = 0;
        else
            count2++;
    }
    else {
        if(count2 == 0)
            count2 = 4;
        else
            count2--;
    }

}

void writeFromMaster() {

    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);      // Set the master in transmit mode
    while(MAP_I2C_isBusBusy(EUSCI_B1_BASE));                        // Wait until bus is no longer busy, Master is ready to write
    MAP_I2C_masterSendMultiByteStart(EUSCI_B1_BASE, 0);             // Start multi-byte transmission from MSP432 to RTC
    MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdSecond);      // Write to seconds register
    MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdMinute);      // Write to minutes register
    MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdHour);        // Write to hours register
    MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdDay);         // Write to days register
    MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdDate);        // Write to date register
    MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdMonth);       // Write to months register
    MAP_I2C_masterSendMultiByteFinish(EUSCI_B1_BASE, bcdYear);      // Write to year register and send stop bit
}

void readFromSlave() {

    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);      // Set Master in transmit mode
    while (MAP_I2C_isBusBusy(EUSCI_B1_BASE));                       // Wait for bus release, ready to write
    MAP_I2C_masterSendSingleByte(EUSCI_B1_BASE,0);                  // set pointer to beginning of RTC registers
    while (MAP_I2C_isBusBusy(EUSCI_B1_BASE));                       // Wait for bus release
    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_RECEIVE_MODE);       // Set Master in receive mode
    while (MAP_I2C_isBusBusy(EUSCI_B1_BASE));                       // Wait for bus release, ready to receive

    int i;  // read from RTC registers (pointer auto increments after each read)

    for(i = 0; i < 19; i++) {
        RTC_registers[i]=MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
    }

    while(MAP_I2C_isBusBusy(EUSCI_B1_BASE));    //Wait

}

void setTime() {

    uint8_t tempVar;

    tempVar   = promptUser("year (0-99)");                  // ask for year
    bcdYear   = convertToBCD(tempVar);                      // assign year
    tempVar   = promptUser("Month (01-12)");                // ask for month
    bcdMonth  = convertToBCD(tempVar);                      // assign month
    tempVar   = promptUser("Date (01-31)");                 // ask for date
    bcdDate   = convertToBCD(tempVar);                      // assign date
    tempVar   = promptUser("Day (1-7)");                    // ask for day
    bcdDay    = convertToBCD(tempVar);                      // assign day
    tempVar   = promptUser("Hour (1-12 + AM/PM, 00-23)");   // ask for hour
    bcdHour   = convertToBCD(tempVar);                      // assign hour
    tempVar   = promptUser("Minute (00-59)");               // ask for minute
    bcdMinute = convertToBCD(tempVar);                      // assign minute
    tempVar   = promptUser("Second (00-59)");               // ask for second
    bcdSecond = convertToBCD(tempVar);                      // set second variable

    writeFromMaster();                                      // send bcd time/date/etc
}

uint16_t promptUser(char* str)
{
    uint16_t tempVar;

    printf("Please enter the %s then press the # key.\n", str);
    fflush(stdout);

    int key    = getKey();
    int number = 0;
    do
    {
        if(key != 0 && key != 10)
        {
            key    = (key==11) ? 0 : key;
            number = number*10 + key;
            printf("%d", key);
            fflush(stdout);
        }

        key = getKey();
    }while(key != 12);

    printf("\n");

    tempVar = number;

    return tempVar;
}

void getTime(uint8_t optionKey) {


    readFromSlave(); // read value from external clock

    uint8_t year, month, date, day, hour, minute, second, tmpInt, tmpFrac;
    uint8_t* addr_pointer;                                      // pointer to address in flash for reading back values

    switch(optionKey)
    {
        // Todays Date
        case 1:
            year   = convertFromBCD(RTC_registers[6]);      // get year
            month  = convertFromBCD(RTC_registers[5]);      // get month
            date   = convertFromBCD(RTC_registers[4]);      // get date
            day    = convertFromBCD(RTC_registers[3]);      // get day
            printf("Todays Date: ");
            printDay(day);
            printf("%d/%d/20%d\n",month,date,year);
            break;
        case 2:
            hour   = convertFromBCD(RTC_registers[2]);      // get hour
            minute = convertFromBCD(RTC_registers[1]);      // get minute
            second = convertFromBCD(RTC_registers[0]);      // get second
            printf("Time: %d:%d:%d\n",hour,minute,second);
            break;
        case 3:
            tmpInt  =  RTC_registers[17];                   // get int of tempature
            tmpFrac = (RTC_registers[18] >> 6) * 25;        // get decimal of tempature
            printf("Degree C : %d.%d\n", tmpInt, tmpFrac);
            break;
        default:
            break;
    }

    //IF TEMP ABOVE 20 THEN SAVE
    if(tmpInt >= 20) {
        uint8_t i;                                                                                      // index
        COMMONCLOCKS_sysTick_delay_48MHZ(msDelay);                                                      // Setting MCLK to 48MHz for faster programming
        addr_pointer = CALIBRATION_START+4;                                                             // point to address in flash for saving data

        for(i=0; i<25; i++) {                                                                           // read values in flash before programming
            timeArr1[i] = *addr_pointer++;
        }

        for(i=0; i<25; i++) {                                                                           // read values in flash before programming
            timeArr2[i] = *addr_pointer++;
        }

        for(i=0; i<25; i++) {                                                                           // read values in flash before programming
            timeArr3[i] = *addr_pointer++;
        }

        for(i=0; i<25; i++) {                                                                           // read values in flash before programming
            timeArr4[i] = *addr_pointer++;
        }

        for(i=0; i<25; i++) {                                                                           // read values in flash before programming
            timeArr5[i] = *addr_pointer++;
        }

        addr_pointer = CALIBRATION_START+4;                                                             // point to address in flash for saved data

        printTime();
    }
}

void printTime() {

    if(firstRead){
        readFromSlave();
        firstRead = 0;
    }
    readFromSlave(); // read value from external clock

    uint8_t year, month, date, day, hour, minute, second, tmpInt, tmpFrac;

    year    = convertFromBCD(RTC_registers[6]);      // get year
    month   = convertFromBCD(RTC_registers[5]);      // get month
    date    = convertFromBCD(RTC_registers[4]);      // get date
    day     = convertFromBCD(RTC_registers[3]);      // get day
    hour    = convertFromBCD(RTC_registers[2]);      // get hour
    minute  = convertFromBCD(RTC_registers[1]);      // get minute
    second  = convertFromBCD(RTC_registers[0]);      // get second
    tmpInt  =  RTC_registers[17];                    // get int of tempature
    tmpFrac = (RTC_registers[18] >> 6) * 25;         // get decimal of tempature

    char iHourtoC[2];
    char iMinuteC[2];
    char iSecondC[2];
    char iMonthC[2];
    char iDateC[2];
    char iYearC[4];

    sprintf(iHourtoC,"%d",hour);
    sprintf(iMinuteC,"%d",minute);
    sprintf(iSecondC,"%d",second);
    sprintf(iMonthC,"%d",month);
    sprintf(iDateC,"%d",date);
    sprintf(iYearC,"%d",year);

    int count = 0;
    char testArray[SIZE_ARRAY]; // array to hold data

    int j = 0;
    for(j = 0; j<sizeof(iHourtoC) / sizeof(uint8_t); j++){
        testArray[j] = iHourtoC[j];
        count ++;
    }

    memcpy (testArray + count, ":", 1);
    count ++;
    memcpy (testArray + count, iMinuteC , sizeof(iMinuteC));
    count += (sizeof(iMinuteC) / sizeof(uint8_t));
    memcpy (testArray + count, ":", 1);
    count ++;
    memcpy (testArray + count, iSecondC , sizeof(iSecondC));
    count += sizeof(iSecondC) / sizeof(uint8_t);
    memcpy (testArray + count, " ", 1);
    count ++;
    memcpy (testArray + count, iMonthC , sizeof(iMonthC));
    count += (sizeof(iMonthC) / sizeof(uint8_t));
    memcpy (testArray + count, "/", 1);
    count ++;
    memcpy (testArray + count, iDateC , sizeof(iDateC));
    count += (sizeof(iDateC) / sizeof(uint8_t));
    memcpy (testArray + count, "/", 1);
    count ++;
    memcpy (testArray + count, iYearC , sizeof(iYearC));
    count += (sizeof(iYearC) / sizeof(uint8_t));

    int i = 0;
    int r = 0;

    while(timeArr4[i] != '\0'){
        timeArr5[i] = timeArr4[i];
        i++;
    }
    i = 0;

    while(timeArr3[i] != '\0'){
        timeArr4[i] = timeArr3[i];
        i++;
    }
    i = 0;

    while(timeArr2[i] != '\0'){
        timeArr3[i] = timeArr2[i];
        i++;
    }
    i = 0;

    while(timeArr1[i] != '\0'){
        timeArr2[i] = timeArr1[i];
        i++;
    }

    for(i = 0; i<(sizeof(testArray) / sizeof(char)); i++) {

        if(testArray[i] != '\0') {
            timeArr1[r] = testArray[i];
            r++;
        }
    }


    MAP_FlashCtl_unprotectSector(FLASH_INFO_MEMORY_SPACE_BANK0,FLASH_SECTOR0);                      // Unprotecting Info Bank 0, Sector 0

    while(!MAP_FlashCtl_eraseSector(CALIBRATION_START));                                            // Erase the flash sector starting CALIBRATION_START.
                                                                                                    // Program the flash with the new data.
    while (!MAP_FlashCtl_programMemory(timeArr1,(void*) CALIBRATION_START+4, 25 ));                 // leave first 4 bytes unprogrammed

    while (!MAP_FlashCtl_programMemory(timeArr2,(void*) CALIBRATION_START+4+25, 25 ));

    while (!MAP_FlashCtl_programMemory(timeArr3,(void*) CALIBRATION_START+4+50, 25 ));

    while (!MAP_FlashCtl_programMemory(timeArr4,(void*) CALIBRATION_START+4+75, 25 ));

    while (!MAP_FlashCtl_programMemory(timeArr5,(void*) CALIBRATION_START+4+100, 25 ));

    MAP_FlashCtl_protectSector(FLASH_INFO_MEMORY_SPACE_BANK0,FLASH_SECTOR0);                        // Setting the sector back to protected

    printf("%s \n", timeArr1);
    printf("%s \n", timeArr2);
    printf("%s \n", timeArr3);
    printf("%s \n", timeArr4);
    printf("%s \n\n\n", timeArr5);
//    ST7735_DrawString(2, 6, timeArr1, textColor);
//    ST7735_DrawString(2, 7, timeArr2, textColor);
//    ST7735_DrawString(2, 8, timeArr3, textColor);
//    ST7735_DrawString(2, 9, timeArr4, textColor);
//    ST7735_DrawString(2, 10, timeArr5, textColor);
}

void printDay(uint8_t day) {
    switch(day) {
        case 1:
            printf("Sunday ");          // 1 = Sunday
            break;
        case 2:
            printf("Monday ");          // 2 = Monday
            break;
        case 3:
            printf("Tuesday ");         // 3 = Tuesday
            break;
        case 4:
            printf("Wednesday ");       // 4 = Wednesday
            break;
        case 5:
            printf("Thursday ");        // 5 = Thursday
            break;
        case 6:
            printf("Friday ");          // 6 = Friday
            break;
        case 7:
            printf("Saturday ");        // 7 = Saturday
            break;
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

    firstFlag = 1;
    firstClock++;

    TIMER32_1->LOAD = 24000000; //Re-load the same interrupt time
}
