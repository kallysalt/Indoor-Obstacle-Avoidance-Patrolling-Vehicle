#include "msp.h"
#include <stdio.h>
#include <driverlib.h>
#include <stdint.h>
#include <FinalPIDHeader.h>

static void Delay(uint32_t loop)
{
    volatile uint32_t i;
    for (i = 0 ; i < loop ; i++);
}

void main(void){
    WDT_A_holdTimer();
    FPU_enableModule();
    Interrupt_disableMaster();

    /* Configuring P2.4 as peripheral input for capture */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);
    // Initialize S1 and S2
    GPIO_setAsInputPinWithPullUpResistor(PORT1,S1);
    GPIO_setAsInputPinWithPullUpResistor(PORT1,S2);
    // initialize the GPIO pins for motor
    GPIO_setAsOutputPin(PORT5,PIN6);
    GPIO_setAsOutputPin(PORT5,PIN7);
    GPIO_setOutputLowOnPin(PORT5,PIN6);
    GPIO_setOutputLowOnPin(PORT5,PIN7);
    GPIO_setAsOutputPin(PORT6,PIN6);
    GPIO_setAsOutputPin(PORT6,PIN7);
    GPIO_setOutputLowOnPin(PORT6,PIN6);
    GPIO_setOutputLowOnPin(PORT6,PIN7);

    // Set DCO clock source frequency
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);
    // Tie SMCLK to DCO
    CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_8);

    // Configure Timer A1 for interrupt
    Timer_A_configureUpMode(TIMER_A1_BASE,&upConfig_0);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    Interrupt_enableInterrupt(INT_TA1_0);

    /* Configuring Capture Mode on Timer A0 */
    Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig);
    /* Configuring Continuous Mode on Timer A0 */
    Timer_A_configureContinuousMode(TIMER_A0_BASE, &continuousModeConfig);
    /* Starting the Timer_A0 in continuous mode */
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);

    /* Starting the Timer32 */
    Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_disableInterrupt(TIMER32_0_BASE);
    Timer32_setCount(TIMER32_0_BASE, 1);
    Timer32_startTimer(TIMER32_0_BASE, true);

    // configure Timer A2
    Timer_A_configureUpMode(TIMER_A2_BASE,&upConfig_0);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
    // set Timer A2 period
    TA2CCR0 = base;

    // Enabling interrupts
    Interrupt_enableInterrupt(INT_TA0_N);
    Interrupt_enableMaster();

    // set Timer A0 to have higher priority
    Interrupt_setPriority(24,0);
    Interrupt_setPriority(26,1);

    while (1){
        s1_pressed=GPIO_getInputPinValue(PORT1,S1);
        s2_pressed=GPIO_getInputPinValue(PORT1,S2);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);

        Timer32_setCount(TIMER32_0_BASE, 24 * 10);
        while (Timer32_getValue(TIMER32_0_BASE) > 0); // Wait 10us
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        //software delays
        Delay(40000);
        Delay(40000);
        Delay(40000);
        Delay(40000);
        Delay(40000);
        Delay(40000);
        Delay(40000);
        Delay(40000);
        Delay(40000);
        Delay(40000);
        Delay(40000);
        Delay(40000);
        Delay(40000);
        Delay(40000);
        Delay(40000);

        if (dutyCycle==0) Interrupt_disableInterrupt(26);

        if (forward==1 && dutyCycle~=0){
            Interrupt_enableMaster();
            // set bit 6 of P6SELO and clear bit 6 of P6SEL1 to enable TA2.3 functionality on P6.6
            P6SEL0 |= 0x40;
            P6SEL1 &=~ 0x40;
            // clear bit 7 of P6SELO and set bit 7 of P6SEL1 to disable TA2.4 functionality on P6.7
            P6SEL0 &=~ 0x80;
            P6SEL1 |= 0x80;
            // set bit 6 of P5SELO and clear bit 6 of P5SEL1 to enable TA2.1 functionality on P5.6
            P5SEL0 &=~ 0x40;
            P5SEL1 |= 0x40;
            // clear bit 7 of P5SELO and set bit 7 of P5SEL1 to disable TA2.2 functionality on P5.7
            P5SEL0 |= 0x80;
            P5SEL1 &=~ 0x80;
            for (i=0;i<999999;i++){};
        }

        else if (backward==1 && dutyCycle~=0){
            Interrupt_enableMaster();
            // clear bit 6 of P6SELO and set bit 6 of P6SEL1 to disable TA2.3 functionality on P6.6
            P6SEL0 &=~ 0x40;
            P6SEL1 |= 0x40;
            // set bit 7 of P6SELO and clear bit 7 of P6SEL1 to enable TA2.4 functionality on P6.7
            P6SEL0 |= 0x80;
            P6SEL1 &=~ 0x80;

            // clear bit 6 of P5SELO and set bit 6 of P5SEL1 to disable TA2.1 functionality on P5.6
            P5SEL0 |= 0x40;
            P5SEL1 &=~ 0x40;
            // set bit 7 of P5SELO and clear bit 7 of P5SEL1 to enable TA2.2 functionality on P5.7
            P5SEL0 &=~ 0x80;
            P5SEL1 |= 0x80;
            for (i=0;i<999999;i++){};
        }

        if (turn) {
            // enable
            P6SEL0 |= 0x40;
            P6SEL1 &=~ 0x40;
            // disable
            P6SEL0 &=~ 0x80;
            P6SEL1 |= 0x80;
            // enable
            P5SEL0 |= 0x40;
            P5SEL1 &=~ 0x40;
            // disable
            P5SEL0 &=~ 0x80;
            P5SEL1 |= 0x80;
            Interrupt_enableMaster();
            for(j=0;j<80000;j++){};

            // disable
            P6SEL0 &=~ 0x40;
            P6SEL1 |= 0x40;
            // disable
            P6SEL0 &=~ 0x80;
            P6SEL1 |= 0x80;
            // disable
            P5SEL0 &=~ 0x40;
            P5SEL1 |= 0x40;
            // disable
            P5SEL0 &=~ 0x80;
            P5SEL1 |= 0x80;
            // Interrupt_disableInterrupt(26);
            // now how to resume moving forward ???
            forward = 1;
            dutyCycle = upper*base;
            for (i=0;i<999999;i++){};
        }
    }
}

void TA1_0_IRQHandler(void){
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);

    // update duty cycle for PID controller
    if(u < 0){
        // backward
        forward=0;
        backward=1;
    }
    else{
        // forward;
        forward=1;
        backward=0;
    }

    // set duty cycle
    TA2CCR3 = dutyCycle;  // pin 6.6 (TA 2.3)
    TA2CCR4 = dutyCycle;  // pin 6.7 (TA 2.4)
    TA2CCR1 = dutyCycle;  // pin 5.6(TA 2.1)
    TA2CCR2 = dutyCycle;  // pin 5.7(TA 2.2)

    TA2CCTL3 = OUTMOD_7;  // pin 6.6 (TA 2.3)
    TA2CCTL4 = OUTMOD_7;  // pin 6.7 (TA 2.4)
    TA2CCTL1 = OUTMOD_7;  //pin 5.6 (TA 2.1)
    TA2CCTL2 = OUTMOD_7;  //pin 5.7 (TA 2.2)

    TA2CTL = TASSEL__SMCLK | MC__UP | TACLR;
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


void TA0_N_IRQHandler(void)
{
    int rising = 0;
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

    if(P2IN&0x10) rising=1;
    else rising=0;

    // Start
    if(rising)
    {
        meas1 = Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
    }
    else
    {
        meas2 = Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
        time1 = meas2-meas1;
        //distance1 = 34400*time1/2/375000;
        //distance = time/58/2;
        distance1 = time1/106;
        //printf("Received: %d\n", distance1);
    }

    if (distance1 <0) {
        distance1=distance0;
    }
    distance0=distance1;

    y = distance1;
    error = r - y;
    error_integral += error*dt;
    error_derivative = (error - previous_error)/dt;
    // u is a pwm ratio
    u = Kp*error + Kd*error_derivative;
    // limit max speed
    if(u > upper) u = upper;
    else if(u < -upper) u = -upper;
    previous_error = error;
    dutyCycle = abs(u)*base;
    // stop if error < allowed_error
    if(abs(error) < allowed_error){
        dutyCycle = 0;
        error_integral = 0;
        turn = 1;
        forward = 0;
        backward = 0;
    }

}
