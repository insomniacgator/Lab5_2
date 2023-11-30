// G8RTOS_Threads.c
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// Defines for thread functions.

/************************************Includes***************************************/

#include "./threads.h"

#include "./MultimodDrivers/multimod.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <driverlib/timer.h>

#include "dsp_defines.h"

// CMSIS Math includes
#include "arm_math.h"
#include "arm_const_structs.h"

/*************************************Defines***************************************/

#define BUCKET_NUM      100

// remember nyquist theorem
#define SAMPLE_FREQ     2000

/*********************************Global Variables**********************************/
uint32_t fftSize = FFT_SAMPLES;
uint32_t ifftFlag = 0;
arm_rfft_fast_instance_f32 S;
uint32_t maxIndex = 0;
arm_status status;
float32_t maxValue;


// Bucket array
int bucketCounts[BUCKET_NUM] = {0};
float bucketLimits[BUCKET_NUM];

// signal array
static float signal_samples[FFT_SAMPLES];

uint32_t sample_index = 0;

// Can't fit this all in one thread (unless you have a very large stack size)
static float32_t complexFFT[FFT_SAMPLES], realFFT[FFT_SAMPLES_HALF],
        imagFFT[FFT_SAMPLES_HALF], angleFFT[FFT_SAMPLES_HALF],
        powerFFT[FFT_SAMPLES_HALF], magnitudeFFT[FFT_SAMPLES_HALF];


/*************************************Threads***************************************/

void Idle_Thread(void) {
    while(1);
}


void DrawDisplay_Thread(void)
{
    // here we have to check an array to see how tall each line will be

    while(1)
    {
        sleep(300);
    }
}

void Sample_Thread(void)
{
    // start timer for conversion at standard rate
    //TIMER0_CTL_R |= 0x00000001; // enable Timer 0A


    // while 1 here?
    while(1)
    {

    // this should wait on a timer-adc semaphore here

        //TIMER0_CTL_R |= 0x00000001; // enable Timer 0A

        TimerLoadSet(TIMER0_BASE, TIMER_A, 50);
        //IntEnable(...);
        TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        TimerEnable(TIMER0_BASE, TIMER_A);
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

        G8RTOS_WaitSemaphore(&sem_Sample);

    // Do 1 sample
        sleep(500);
    }

}

void FFT_Thread(void)
{
    int i;
    // Here we will wait on the fft semaphore which will be signaled by ADC0S1 handler

    // At the end of FFT we need to re-set the timer for Sample_Thread
    // also set the sample index back to 0

    // create an instance of a 1024-point FFT (results are of float32)
            arm_rfft_fast_init_f32(&S, fftSize);

    while(1)
    {
        G8RTOS_WaitSemaphore(&sem_FFT);
        // Do FFT
        //UARTprintf("We got to FFT\n");




        // perform FFT on the sample signal (sample_signals.h)
        arm_rfft_fast_f32(&S, signal_samples, complexFFT, ifftFlag);

        // arm_max_f32
        // arm_cmplx_mag_f32
        // arm_cmplx_mag_sqared_f32
        for (i=1; i<(fftSize/2); i++)
        {
            // Compute magnitudes of each frequency component
            realFFT[i] = complexFFT[2 * i];
            imagFFT[i] = complexFFT[2 * i + 1];
            arm_status result = arm_sqrt_f32(realFFT[i] * realFFT[i] + imagFFT[i] * imagFFT[i], &magnitudeFFT[i]);
            // Compute the power of each frequency component by squaring the magnitude
            powerFFT[i] = magnitudeFFT[i] * magnitudeFFT[i];
            // Get maximum value & index of maximum value within the power vector
            if (powerFFT[i] > maxValue)
            {
                // Remember to offset index to account for DC value
                // SHOULD I DO i-1 FOR BOTH OF THESE?
                maxValue = powerFFT[i];
                maxIndex = i;
            }
        }


        // Compute bucket limits depending on maximum value and resolution
            // Resolution determines how fine (close together) bucket limits are.
        float resolution = maxValue / BUCKET_NUM;
        for (i=0; i<BUCKET_NUM; i++)
        {
            bucketLimits[i] = i * resolution;
        }


        // Increment the counts of each bucket depending on bucket limits and the resulting magnitudes
        for (i=0; i<(fftSize/2); i++)
        {
            for (int j=0; j<BUCKET_NUM-1; j++)
            {
                if (powerFFT[i] >= bucketLimits[j] && powerFFT[i] < bucketLimits[j+1])
                {
                    bucketCounts[j]++;
                    break;
                }
            }
        }


        // Calculate the dominant frequency
        float dominantFrequency = ((float)maxIndex * SAMPLING_FREQUENCY / (float)fftSize);

        int32_t freq = (int32_t)((dominantFrequency));

        //char *text_buffer[10];
        //sprintf(text_buffer, "%f", dominantFrequency);
        UARTprintf("Dominant Frequency: %d\n", freq);

        //TIMER0_CTL_R |= 0x00000001; // enable Timer 0A
        //TimerLoadSet(TIMER0_BASE, TIMER_A, 50);
        //IntEnable(...);
        //TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        //TimerEnable(TIMER0_BASE, TIMER_A);
        //TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

        sleep(300);

        G8RTOS_SignalSemaphore(&sem_Sample);

    }

}

/********************************Periodic Threads***********************************/



/*******************************Aperiodic Threads***********************************/
void ADC0S3_Handler(void)
{
    uint32_t results;
    float results_n;
    // HERE WE HAVE TO ADD THE ADC DATA TO THE ARRAY
    // Wait until conversion is complete
    while(!ADCIntStatus(ADC0_BASE, SEQUENCE_NUM, 0));

    // Clear ADC interrupt flag
    ADCIntClear(ADC0_BASE, SEQUENCE_NUM);

    ADCSequenceDataGet(ADC0_BASE, SEQUENCE_NUM, &results);

    //SysCtlDelay(3);

    //uint32_t results_avg = (results[0] + results[1] + results[2] + results[3])/4;

    //Normalize the data
    results_n = (2.0 * (results - 0) / (4095 - 0)) - 1.0;
    signal_samples[sample_index] = results_n*1000;
    sample_index++;

    //char *text_buffer[10];
    //sprintf(text_buffer, "%f", results_n);

    //results_n *= 1000;
    int32_t temp = (int32_t)(results_n);

    //UARTprintf("results: %d\n", temp);

    // this is actually trigger after every sample
    // Here we can check if we already have 1024 samples.
    // If we do not have 1024 signals in sample array, disable timer 0
    // and signal fft_thread
    //while (1)
    //{
        if (sample_index >= 1024)
        {
            //TIMER0_CTL_R &= ~0x00000001; // disable Timer 0A
            //TimerDisable(...);
            //IntDisable(...);

            TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
            TimerDisable(TIMER0_BASE, TIMER_A);

            sample_index = 0;
            // signal FFT semaphore
            G8RTOS_SignalSemaphore(&sem_FFT);

        }

    //}


}

void Timer0_Handler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Trigger ADC conversion
    // Start conversion
    ADCProcessorTrigger(ADC0_BASE, SEQUENCE_NUM);



}

