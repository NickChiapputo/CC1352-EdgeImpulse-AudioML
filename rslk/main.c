#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>  //Exact-width integer types
#include <driverlib.h>  // Driver library
#include <motor.h>
#include <tachometer.h>
#include <bumpSensors.h>
#include <uart.h>

#define CLOCK_HF        48000000    // 48MHz
#define CLOCK_LF        32000       // 32kHz

#define HEARTBEAT_FREQ  4           // unit: Hz, heart beat LED blinking frequency for debugging

#define RED_LED         GPIO_PIN0  // heart beat LED
#define GREEN_LED       GPIO_PIN1
#define BLUE_LED        GPIO_PIN2

#define LEFT_MOTOR      0
#define RIGHT_MOTOR     1
#define MOTOR_FORWARD   1
#define MOTOR_STOP      0
#define MOTOR_BACKWARD  -1

#define MOTOR_GO_SPEED	20

#define TRANSMISSION_VALID_PORT GPIO_PORT_P2
#define TRANSMISSION_VALID_PIN  GPIO_PIN4
#define MOTION_CONTROL_PORT     GPIO_PORT_P2
#define MOTION_CONTROL_PIN      GPIO_PIN5

void initDevice_HFXT();
void initHeartBeatLED();
void initGPIOComms();

uint32_t clockMCLK, clockSMCLK;
uint8_t currentLED = RED_LED;

void main(void)
{
    initDevice_HFXT();
    initHeartBeatLED();
    initUART();
    initMotors( clockSMCLK );
    initBumpSensors( clockMCLK );
    initTachometers( clockSMCLK );
    initGPIOComms();

    Interrupt_enableMaster();
    Timer32_startTimer(TIMER32_0_BASE, false);

    // Start timer for tachometer speed measurements
    startTacho();

    while(1)
    {
        // Check status of control LED.
        if( GPIO_getInputPinValue( TRANSMISSION_VALID_PORT, TRANSMISSION_VALID_PIN ) == 1 )
        {
	        if( GPIO_getInputPinValue( MOTION_CONTROL_PORT, MOTION_CONTROL_PIN ) == 1 )
	        {
	            // Stop vehicle.
	            setMotorDutyCycle( LEFT_MOTOR, 0 );
	            setMotorDutyCycle( RIGHT_MOTOR, 0 );
	            currentLED = RED_LED;
	            GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );
	        }
	        else
	        {
	            // Start vehicle.
	            setMotorDutyCycle( LEFT_MOTOR, MOTOR_GO_SPEED );
	            setMotorDutyCycle( RIGHT_MOTOR, MOTOR_GO_SPEED );
	            currentLED = GREEN_LED;
	            GPIO_setOutputHighOnPin( GPIO_PORT_P1, GPIO_PIN0 );
	        }
	    }
	    else
	    {
            // Stop vehicle.
            // setMotorDutyCycle( LEFT_MOTOR, 0 );
            // setMotorDutyCycle( RIGHT_MOTOR, 0 );
            currentLED = BLUE_LED;
            // GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );
	    }

        // True when a bump button is pressed.
        if( bumpStateSet() )
        {
            // Check if bump button is still pressed
            int bumpState = checkBumpState();
            if( bumpState == 0 )
            {
                // Restart motors
                startMotor( LEFT_MOTOR );
                startMotor( RIGHT_MOTOR );
            }
            else
            {
                // Stop motors while bump sensor is active
                pauseMotor( LEFT_MOTOR );
                pauseMotor( RIGHT_MOTOR );
            }
        }
    } //end of while
}

void initDevice_HFXT(void)
{
    WDT_A_holdTimer();  //Stop watchdog timer.

    //Change VCORE to 1 to support a frequency higher than 24MHz.
    //See MSP432 Data Sheet, Section 5.8 for Flash wait-state requirements for active frequency.
    PCM_setPowerState(PCM_AM_DCDC_VCORE1);
    FlashCtl_setWaitState(FLASH_BANK0, 1);
    FlashCtl_setWaitState(FLASH_BANK1, 1);

    FPU_enableModule();
    FPU_enableLazyStacking(); //Required to use FPU within ISR.

    //Configure PJ.2 and PJ.3 in HFXT mode.
    //Initialize external clock sources HFXT.
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN2|GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    CS_setExternalClockSourceFrequency(CLOCK_LF, CLOCK_HF);
    CS_startHFXT(false);

    CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_8);
    CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_16);

    clockMCLK = CS_getMCLK();
    clockSMCLK = CS_getSMCLK();
}

void initHeartBeatLED(void)
{
    //Configure P2.0, P2.1, P2.2 as output.
    //P2.0, P2.1, P2.2 are connected to a RGB tri-color LED on LaunchPad.
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);

    Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_setCount(TIMER32_0_BASE, clockMCLK/HEARTBEAT_FREQ);
    Timer32_enableInterrupt(TIMER32_0_BASE);
    Interrupt_enableInterrupt(INT_T32_INT1); // Enable Timer32_0 interrupt in the interrupt controller.
}

void initGPIOComms()
{
    // Configure P2.4, P2.5 as input.
    // P2.4 -- Transmission valid
    // P2.5 -- Motion control (1 = stop, 0 = go)
    GPIO_setAsInputPin( TRANSMISSION_VALID_PORT, TRANSMISSION_VALID_PIN );
    GPIO_setAsInputPin( MOTION_CONTROL_PORT, MOTION_CONTROL_PIN );

    GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );
}

//Timer32_0 ISR
void T32_INT1_IRQHandler(void)
{
    Timer32_clearInterruptFlag( TIMER32_0_BASE );

    if( GPIO_getInputPinValue( GPIO_PORT_P2, RED_LED | GREEN_LED | BLUE_LED ) )
        GPIO_setOutputLowOnPin( GPIO_PORT_P2, RED_LED | GREEN_LED | BLUE_LED );
    else
        GPIO_setOutputHighOnPin( GPIO_PORT_P2, currentLED );
}
