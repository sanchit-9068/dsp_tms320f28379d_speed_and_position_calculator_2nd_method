#include "driverlib.h"
#include "device.h"
#define ENCODER_RESOLUTION 4096   // Number of encoder counts per revolution
#define BASE_SPEED_RPM 1800       // Base speed of the motor in RPM


#define GPIO_PIN_SCIRXDA 43
#define mySCI0_SCIRX_GPIO 43
#define mySCI0_SCIRX_PIN_CONFIG GPIO_43_SCIRXDA
//
// SCITXDA - GPIO Settings
//
#define GPIO_PIN_SCITXDA 42
#define mySCI0_SCITX_GPIO 42
#define mySCI0_SCITX_PIN_CONFIG GPIO_42_SCITXDA

//*****************************************************************************
//
// SCI Configurations
//
//*****************************************************************************
#define mySCI0_BASE SCIA_BASE
#define mySCI0_BAUDRATE 9600
#define mySCI0_CONFIG_WLEN SCI_CONFIG_WLEN_8
#define mySCI0_CONFIG_STOP SCI_CONFIG_STOP_ONE
#define mySCI0_CONFIG_PAR SCI_CONFIG_PAR_NONE
#define mySCI0_FIFO_TX_LVL SCI_FIFO_TX0
#define mySCI0_FIFO_RX_LVL SCI_FIFO_RX0

uint16_t cpuTimer0IntCount;
void    PinMux_init();
void mySCI0_init();
void    SCI_init();
__interrupt void cpuTimer0ISR(void);
void initCPUTimers(void);
void configCPUTimer(uint32_t, float, float);

void intToStrPositive(uint32_t value, char* str)
{
    uint32_t i = 0;

    // Convert each digit to a character
    do {
        str[i++] = (char)(value % 10) + '0';
        value /= 10;
    } while (value > 0);

    // Reverse the string
    uint32_t j;
    for (j = 0; j < i / 2; j++) {
        char temp = str[j];
        str[j] = str[i - j - 1];
        str[i - j - 1] = temp;
    }

    // Null-terminate the string
    str[i] = '\0';
}



void setupEQEPModule()
{
    //
     EQEP_setDecoderConfig(EQEP1_BASE, (EQEP_CONFIG_2X_RESOLUTION |
                                        EQEP_CONFIG_QUADRATURE |
                                        EQEP_CONFIG_NO_SWAP));
     EQEP_setEmulationMode(EQEP1_BASE, EQEP_EMULATIONMODE_RUNFREE);

     //
     // Configure the position counter to reset on an index event
     //
     EQEP_setPositionCounterConfig(EQEP1_BASE, EQEP_POSITION_RESET_IDX,
                                   0xFFFFFFFF);

     //
     // Enable the unit timer, setting the frequency to 100 Hz
     //
     EQEP_enableUnitTimer(EQEP1_BASE, (DEVICE_SYSCLK_FREQ / 50));

     //
     // Configure the position counter to be latched on a unit time out
     //
     EQEP_setLatchMode(EQEP1_BASE, EQEP_LATCH_UNIT_TIME_OUT);

     //
     // Enable the eQEP1 module
     //
     EQEP_enableModule(EQEP1_BASE);

     //
     // Configure and enable the edge-capture unit. The capture clock divider is
     // SYSCLKOUT/64. The unit-position event divider is QCLK/32.
     //
     EQEP_setCaptureConfig(EQEP1_BASE, EQEP_CAPTURE_CLK_DIV_64,
                           EQEP_UNIT_POS_EVNT_DIV_32);
     EQEP_enableCapture(EQEP1_BASE);
}
int32_t prev_pos=0;

float calculateSpeed()
{
    uint32_t position = EQEP_getPosition(EQEP1_BASE);
    float deltaTime = 0.001f;  // Time interval between speed calculations (in seconds)
    int32_t y=position-prev_pos;
    // Calculate speed in encoder counts per second
    if(y<0)
        y*=-1;

    float speedCountsPerSec = (float)(y) / deltaTime;
    prev_pos=position;
    // Calculate speed in RPM
    float speedRPM = (speedCountsPerSec /( ENCODER_RESOLUTION*4)) * 60.0f;

    return speedRPM;
}

int main(void)
{
    // Initialize system and EQEP module
    // ...
    Device_init();

        //
        // Disable pin locks and enable internal pullups.
        //
    Device_initGPIO();
    GPIO_setPinConfig(GPIO_20_EQEP1A);
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_STD);

    GPIO_setPinConfig(GPIO_21_EQEP1B);
    GPIO_setPadConfig(21, GPIO_PIN_TYPE_STD);

    GPIO_setPinConfig(GPIO_23_EQEP1I);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_LED1);
    GPIO_setMasterCore(DEVICE_GPIO_PIN_LED1, GPIO_CORE_CPU1);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED1, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED1, GPIO_DIR_MODE_OUT);
    setupEQEPModule();
    Interrupt_initModule();
    PinMux_init();
    mySCI0_init();
    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    initCPUTimers();
    configCPUTimer(CPUTIMER0_BASE, DEVICE_SYSCLK_FREQ, 1000);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_enable(INT_TIMER0);
    CPUTimer_startTimer(CPUTIMER0_BASE);




    EINT;
    ERTM;


    while (1)
    {

    }

    return 0;
}
void PinMux_init()
 {
     //
     // PinMux for modules assigned to CPU1
     //

     //
     // SCIA -> mySCI0 Pinmux
     //
     GPIO_setPinConfig(mySCI0_SCIRX_PIN_CONFIG);
     GPIO_setPadConfig(mySCI0_SCIRX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
     GPIO_setQualificationMode(mySCI0_SCIRX_GPIO, GPIO_QUAL_ASYNC);

     GPIO_setPinConfig(mySCI0_SCITX_PIN_CONFIG);
     GPIO_setPadConfig(mySCI0_SCITX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
     GPIO_setQualificationMode(mySCI0_SCITX_GPIO, GPIO_QUAL_ASYNC);


 }

 //*****************************************************************************
 //
 // SCI Configurations
 //
 //*****************************************************************************
 void SCI_init(){
     mySCI0_init();
 }

 void mySCI0_init(){
     SCI_clearInterruptStatus(mySCI0_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
     SCI_clearOverflowStatus(mySCI0_BASE);
     SCI_resetTxFIFO(mySCI0_BASE);
     SCI_resetRxFIFO(mySCI0_BASE);
     SCI_resetChannels(mySCI0_BASE);
     SCI_setConfig(mySCI0_BASE, DEVICE_LSPCLK_FREQ, mySCI0_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
     SCI_enableLoopback(mySCI0_BASE);
     SCI_performSoftwareReset(mySCI0_BASE);
     SCI_setFIFOInterruptLevel(mySCI0_BASE, SCI_FIFO_TX0, SCI_FIFO_RX0);
     SCI_enableFIFO(mySCI0_BASE);
     SCI_enableModule(mySCI0_BASE);
 }

 void
 initCPUTimers(void)
 {
     //
     // Initialize timer period to maximum
     //
     CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);


     //
     // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
     //
     CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);

     //
     // Make sure timer is stopped
     //
     CPUTimer_stopTimer(CPUTIMER0_BASE);


     //
     // Reload all counter register with period value
     //
     CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);


     //
     // Reset interrupt counter
     //
     cpuTimer0IntCount = 0;

 }


 void
 configCPUTimer(uint32_t cpuTimer, float freq, float period)
 {
     uint32_t temp;

     //
     // Initialize timer period:
     //
     temp = (uint32_t)(freq / 1000000 * period);
     CPUTimer_setPeriod(cpuTimer, temp);

     //
     // Set pre-scale counter to divide by 1 (SYSCLKOUT):
     //
     CPUTimer_setPreScaler(cpuTimer, 0);

     //
     // Initializes timer control register. The timer is stopped, reloaded,
     // free run disabled, and interrupt enabled.
     // Additionally, the free and soft bits are set
     //
     CPUTimer_stopTimer(cpuTimer);
     CPUTimer_reloadTimerCounter(cpuTimer);
     CPUTimer_setEmulationMode(cpuTimer,
                               CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
     CPUTimer_enableInterrupt(cpuTimer);

     //
     // Resets interrupt counters for the three cpuTimers
     //
     if (cpuTimer == CPUTIMER0_BASE)
     {
         cpuTimer0IntCount = 0;
     }

 }

 __interrupt void
 cpuTimer0ISR(void)
 {
     cpuTimer0IntCount++;


     int32_t pos=EQEP_getPosition(EQEP1_BASE);
             pos%=4096;
             float32_t pos_float = (pos*360.00f)/4096.00f;
             int32_t pos2=(int32_t)pos_float;
             int16_t angle=(int16_t)pos2;
             char pos_char[16]={};
             SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("Angle="), sizeof("Angle="));
             intToStrPositive(angle, pos_char);
             SCI_writeCharArray(mySCI0_BASE, (uint16_t *)pos_char, sizeof(pos_char));
             SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("\r\n"), sizeof("\r\n"));







     float speedRPM = calculateSpeed();
            uint32_t j=(int32_t)speedRPM;
            uint16_t m=(int16_t)j;
           char l[16]={};

            SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("SPEED="), sizeof("SPEED="));
            intToStrPositive(m, l);
            SCI_writeCharArray(mySCI0_BASE, (uint16_t *)l, sizeof(l));
            SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("\r\n"), sizeof("\r\n"));
//            uint32_t qq=EQEP_getPosition(EQEP1_BASE);
//            uint16_t qqq=(uint16_t)qq;
//            char tt[16]={};
//            SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("Position="), sizeof("Position="));
//            intToStrPositive(qqq, tt);
//            SCI_writeCharArray(mySCI0_BASE, (uint16_t *)tt, sizeof(tt));
//            SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("\r\n"), sizeof("\r\n"));
//            if(EQEP_getDirection(EQEP1_BASE)>0)
//            {
//                SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("Positive"), sizeof("Positive"));
//                SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("\r\n"), sizeof("\r\n"));
//            }
//            else
//            {
//                SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("Negative"), sizeof("Negative"));
//                SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("\r\n"), sizeof("\r\n"));
//            }



     //
     // Acknowledge this interrupt to receive more interrupts from group 1
     //
     Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
 }
