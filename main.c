/* ---------------------------------------PD Controller------------------------------------------*/
/* The purpose of this application is to control, effectively and reliably,
 * the Position (angle) of a brushed DC motor with a real time PD Controller,
 *  using the QEI module provided by the board(EK-TM4C123GXL). */

//Libraries that we need for our application
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_qei.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/qei.h"
#include "driverlib/timer.h"


#define DownLimit 10         //Minimum PWM output value
#define UpLimit 90           //MAximum PWM output value
//volatile only if variable changes due to input signals(external from encoder)
volatile int32_t Direction;  //Direction of motor shaft
volatile int32_t Velocity;   //Velocity of motor shaft in counts/period
volatile uint32_t Position;  //Position of motor shaft in counts (360 degrees -> 1999 counts)
#define Setpoint 1999*51     //define Setpoint for motor command (1:51 is the transmission ratio of the speed reducer).
                             //With this command I expect a full rotation of the shaft. Tip:Command must be positive value for this code
int error=0;                 //error=Desired Angle(in Counts)-Position(couns-measured Value of Angle from encoder)
#define Kp 0.01              //define Kp gain of PD Controller
#define Kd 10                //define Kd gain
float P_term=0 ;
float D_term=0 ;
float u=0 ;                  //output command(%)
uint32_t PWMGen1Period=1000; //Period in clock ticks for PWM Command

int main(void)
{
    //Run clock at 50MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    //Configure PWM Clock to match system's clock
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlDelay(10); //wait until it is ready


/*  Enable peripherals that we will need for the application(PWM,QEI,GPIOC,GPIOF,GPIOB).*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);   // Enable the GPIOC peripheral for the input signals of the Encoder
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);   // Enable the GPIOB peripheral for the output PWM signal
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);   // Enable the GPIOF peripheral for the output Direction Signal
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);    // Enable the QEI1 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);    // Enable the PWM0 peripheral
    SysCtlDelay(10); //wait until it is ready

/*  Configure GPIO pins.*/
    GPIOPinConfigure(GPIO_PC5_PHA1);                          //Pin PC5 as PhA1
    GPIOPinConfigure(GPIO_PC6_PHB1);                          //Pin PC6 as PhB1
    GPIOPinConfigure(GPIO_PB4_M0PWM2);                        //Pin PB4 as PWM (from datasheet PB4 is mapped as M0PWM2)

/*  Configure type of GPIO pins(enabling the alternative functions of the GPIO Pins).*/
    GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_5 |GPIO_PIN_6);   //Pins PC5 & PC6 for the QEI module
    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_4);               //Pin PB4 as output PWM
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);       //Pin PF2 Direction for Motor(0-false or 5V-true)

/*  Configure QEI.*/
    /*Disable everything first.It is not essential but sometimes can prove beneficial*/
    QEIDisable(QEI1_BASE);
    QEIVelocityDisable(QEI1_BASE);
    QEIIntDisable(QEI1_BASE, (QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX));

    /*Configure the QEI to capture on both A and B, not to reset when there is an index pulse
    *(because we don't have index pulse on this application),configure it as a quadrature encoder and doesn't
    * swap signals PHA0 and PHB0 and set the maximum position as 1999. (MAX_POS_CAPTURE_A_B = (cpr*4)-1.
    *cpr=counts per revolution of the encoder's shaft and because it is a 4 line, we multiply by 4*/
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE |      QEI_CONFIG_NO_SWAP), 2*51*1999);
    SysCtlDelay(10);                                                         //wait until it is ready
    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_16, 40000);                   //40000 is the period at which the velocity will be measured
    SysCtlDelay(10);                                                         //wait until it is ready
    QEIPositionSet(QEI1_BASE, 0);

/*  Configure PWM.*/
    //Configure the PWM generator for count down mode with immediate updates to the parameters.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); //PWM_GEN_1 covers M0PWM2 from datasheet
    SysCtlDelay(10);                                                                //wait until it is ready

    /*Set the period. For a 50 KHz desired PWM frequency, the period = 1/50.000, or 20 microseconds. For a 50 MHz clock, this translates
     into 1000 clock ticks(20microsecs/0.02microsecs) .*/
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, PWMGen1Period);
    SysCtlDelay(10);                                                         //wait until it is ready

    //Set the pulse width of PWM0 for a 0% duty cycle.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0.0*PWMGen1Period);
    SysCtlDelay(10);                                                         //wait until it is ready

    //Start the timers in generator 1.From datasheet one can see that PB4 -> M0PWM2 need Generator 1
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    SysCtlDelay(10);                                                         //wait until it is ready
    // Enable the outputs.
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);                          //Enable PWM signal at the given duty cycle(0% for now)
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);                            //Make PF2 true(5 Volts)-> Direction provided to the motor


/*  Enable QEI1 module*/
    QEIEnable(QEI1_BASE);
    SysCtlDelay(10);                                                         //wait until it is ready
    QEIVelocityEnable(QEI1_BASE);
    SysCtlDelay(10);                                                         //wait until it is ready

/*  Main Loop */
    while(1)
    {
/*      Get direction (1 = forward, -1 = backward).  */
        Direction = QEIDirectionGet(QEI1_BASE);
/*      Get velocity(counts per period specified) and multiply by direction so that it is signed.*/
        Velocity = QEIVelocityGet(QEI1_BASE)*Direction;
/*      Get absolute position in counts.*/
        Position = QEIPositionGet(QEI1_BASE);


/*      PD Control Algorithm */

        error=Setpoint-Position;
        P_term=Kp*error;
        D_term=Kd*(-Velocity);   //Using Velocity from encoder
        u=P_term+D_term; //Output Command
        if (u<0){
            u=-u;
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
        } else{
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
        }

        if (u>=UpLimit){
            u=UpLimit;
        }
        else if (u<=DownLimit){
            u=DownLimit;
        }

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, u/100*PWMGen1Period); //map the PWM signal depending on the output command u
    }
}
/* ------------------------------------- End--------------------------------------- */














