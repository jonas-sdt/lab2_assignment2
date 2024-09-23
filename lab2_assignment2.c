//*****************************************************************************
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "utils/uartstdio.h"

// #include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

// #include "driverlib/interrupt.h"
// #include "driverlib/pin_map.h"
// #include "inc/tm4c129encpdt.h"

//***********************************************************************
//                       Configurations
//***********************************************************************
// Configure the UART.
void ConfigureUART(void) {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//                      Main
//*****************************************************************************
int main(void) {
  ConfigureUART();
  uint32_t duty_cycle = 0;

  float pwm_word;
  uint32_t systemClock;
  systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                    SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                   16000);

  pwm_word = systemClock / 200;

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
  SysCtlPeripheralDisable(SYSCTL_PERIPH_PWM0);
  SysCtlPeripheralReset(SYSCTL_PERIPH_PWM0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
  GPIOPinConfigure(GPIO_PF2_M0PWM2);

  PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                  PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC |
                      PWM_GEN_MODE_DBG_RUN);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwm_word);

  // Joystick
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {
  }
  GPIOPinTypeADC(GPIO_PORTE_AHB_BASE, GPIO_PIN_3);

  // set joystick select to true
  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);

  //
  // Enable the first sample sequencer to capture the value of channel 0 when
  // the processor trigger occurs.
  //
  ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                           ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
  ADCSequenceEnable(ADC0_BASE, 3);

  // Clear the interrupt status flag
  ADCIntClear(ADC0_BASE, 3);

  UARTprintf("Main");

  while (1) {
    // Trigger the sample sequence.
    ADCProcessorTrigger(ADC0_BASE, 3);

    // Wait until the sample sequence has completed.
    while (!ADCIntStatus(ADC0_BASE, 3, false)) {
    }

    ADCIntClear(ADC0_BASE, 3);

    // Read the value from the ADC.
    ADCSequenceDataGet(ADC0_BASE, 3, &duty_cycle);

    duty_cycle = (duty_cycle * 100) / 4095;

    UARTprintf("%d\n", duty_cycle);

    // if (duty_cycle > 100 || duty_cycle < 0) {
    //   UARTprintf("duty cycle must be between 0 and 100\n");
    //   continue;
    // }

    // 2: change value of led
    if (duty_cycle == 0) {
      PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
      GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    } else if (duty_cycle == 100) {
      PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
      GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    } else if (duty_cycle > 0 && duty_cycle < 100) {
      GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                       pwm_word /
                           (100 - duty_cycle)); // NOTE: set pulse width here
      PWMGenEnable(PWM0_BASE, PWM_GEN_1);
      PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    } else {
      // NOTE: invalid input - ignore
    }
  }
}
