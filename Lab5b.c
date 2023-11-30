// Lab 5, uP2 Fall 2023
// Created: 2023-07-31
// Updated: 2023-08-01
// Lab 5 is intended to introduce you to DSP using real-time systems.
// For part a you will be conducting FFT on a test signal once to
// extract its dominant frequency.

/************************************Includes***************************************/
#include "G8RTOS/G8RTOS.h"
#include "./MultimodDrivers/multimod.h"
#include "threads.h"

#include "sample_signals.h"
#include "dsp_defines.h"

#include <driverlib/uartstdio.h>
#include <driverlib/gpio.h>
#include <driverlib/uart.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/interrupt.h>
#include <driverlib/timer.h>

// CMSIS Math includes
#include "arm_math.h"
#include "arm_const_structs.h"

void idle() {
    while(1);
}

void Timer_Init() {
    // Initialize timers
    SYSCTL_RCGCTIMER_R |= 0x00000001; // Timer 0 and timer 1 clock gating control
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // Disable timers
    TIMER0_CTL_R &= ~0x00000001; // clear TAEN bit in GPTMCTL
    TIMER0_CTL_R &= ~0x00000100; // clear TBEN
    //TIMER1_CTL_R &= ~0x00000001; // clear TAEN bit in GPTMCTL
    //TIMER1_CTL_R &= ~0x00000100; // clear TBEN


    // Configure timers as half-width, periodic 16-bit or (32-bit if using 64-bit timers) timers for a total of 4 timers
    TIMER0_CFG_R |= 0x00000004; // 16 bit
    //TIMER1_CFG_R |= 0x00000004; // 16 bit

    TIMER0_TAMR_R |=  0x00000002; // periodic mode
    TIMER0_TBMR_R |=  0x00000002; // periodic mode
    //TIMER1_TAMR_R |=  0x00000002; // periodic mode
    //TIMER1_TBMR_R |=  0x00000002; // periodic mode


    // Set prescalers
    TIMER0_TAPR_R = 159;
    //TIMER1_TAPR_R = 159; // set both prescalers at 160

    // Load initial timer values
        // Sysclock / prescaler * desired seconds = timer period
    // I dunno WHY they call this TIMER0_TAIL_R and GPTMTAILR?
    TIMER0_TAILR_R = 00400; //0.5ms // should we set these in HEX?
    //TIMER0_TBILR_R = 15000; //150ms
    //TIMER1_TAILR_R = 10000; //100ms
    //TIMER1_TBILR_R = 50000; //500ms

    // Enable timer interrupts
    TIMER0_IMR_R |= 0x00000001; // Timer 0A
    //TIMER0_IMR_R |= 0x00000100; // Timer 0B
    //TIMER1_IMR_R |= 0x00000001; // Timer 1A
    //TIMER1_IMR_R |= 0x00000100; // Timer 1B

    // Enable timers
    //TIMER0_CTL_R |= 0x00000001; // Timer 0A
    //TIMER0_CTL_R |= 0x00000100; // Timer 0B
    //TIMER1_CTL_R |= 0x00000001; // Timer 1A
    //TIMER1_CTL_R |= 0x00000100; // Timer 1B

    NVIC_EN0_R |= (1 << 19); // Timer 0A
    NVIC_PRI4_R |= 0x00E00000; // Priority 7

    NVIC_EN0_R |= (1 << 20); // Timer 0B
    NVIC_PRI4_R |= 0xE0000000; // Priority 7

    //NVIC_EN0_R |= (1 << 21); // Timer 1A
    //NVIC_PRI5_R |= 0x00E00000; // Priority 7

    //NVIC_EN0_R |= (1 << 22); // Timer 1B
    //NVIC_PRI5_R |= 0xE0000000; // Priority 7



    // Point to relevant timer handler function
    //TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0_Handler);
    //TimerIntRegister(TIMER0_BASE, TIMER_B, TIMER0B_Handler);
    //TimerIntRegister(TIMER1_BASE, TIMER_A, TIMER1A_Handler);
    //TimerIntRegister(TIMER1_BASE, TIMER_B, TIMER1B_Handler);
}


/************************************MAIN*******************************************/
void main() {

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    G8RTOS_Init();
    multimod_init();
    Timer_Init();

    // your code 
    // Add threads, semaphores, here
    // Semaphores
    G8RTOS_InitSemaphore(&sem_UART, 1);
    G8RTOS_InitSemaphore(&sem_Sample, 1);
    G8RTOS_InitSemaphore(&sem_FFT, 0);
    // Background Threads
    G8RTOS_AddThread(Sample_Thread, 4, "Sample_Thread");
    G8RTOS_AddThread(FFT_Thread, 4, "FFT_Thread");
    G8RTOS_AddThread(idle, 255, "idle");

    // Aperiodic Threads
    G8RTOS_Add_APeriodicEvent(Timer0_Handler, 4, 35);
    // Periodic Threads
    G8RTOS_Add_APeriodicEvent(ADC0S3_Handler, 4, 33);

    G8RTOS_Launch();
    IntMasterEnable();
    while (1);
}

/************************************MAIN*******************************************/
