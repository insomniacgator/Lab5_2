// multimod_joystick.c
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// Defines for joystick functions

/************************************Includes***************************************/

#include "../multimod_audio.h"

#include "../multimod_i2c.h"
#include "../multimod_spi.h"

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/adc.h>
#include <driverlib/interrupt.h>
#include <driverlib/timer.h>

#include <inc/tm4c123gh6pm.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_i2c.h>
#include <inc/hw_gpio.h>

/************************************Includes***************************************/

/********************************Public Functions***********************************/

// AudioInput_Init
// Initializes relevant audio modules / adc to retrieve audio information
// Return: void
void AudioInput_Init(void) {

    I2C_Init(I2C_B_BASE);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlDelay(3);

    GPIO_PORTE_DIR_R &= ~0x00000002; // set PE1 has input
    GPIO_PORTE_DEN_R &= ~0x00000002; // disable PE1 digital (make it analog)

    // Enable relevant ADC. Set the clock rate to 1 MSPS and enable hardware averaging.
    // Enable Port E
    //HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4;
    // For ADC use port PE1 AIN2
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
    // Determine the correct sequencer and settings to use with the ADC and enable interrupts if relevant.

    ADCSequenceConfigure(ADC0_BASE, SEQUENCE_NUM, ADC_TRIGGER_PROCESSOR, 0);
    ADCHardwareOversampleConfigure(ADC0_BASE, 64);
    ADCSequenceStepConfigure(ADC0_BASE, SEQUENCE_NUM, 0, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, SEQUENCE_NUM);

    //IntRegister(INT_ADC0SS1, ADC0S1_Handler);
    ADCIntEnable(ADC0_BASE, SEQUENCE_NUM);

    IntEnable(INT_ADC0SS3); // redundant

    //ADCIntClear(ADC0_BASE, 3);
    //ADCProcessorTrigger(ADC0_BASE, 3);

    // Enable relevant modules if needed.

}

// AudioOutput_Init
// Initializes SSI2 to communicate with the DAC.
// Return: void
void AudioOutput_Init(void) {
    SPI_Init(SPI_B_BASE);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
}

// AudioDAC_Write
// Writes data to register in DAC
// Return: void
void AudioDAC_Write(uint32_t address, uint32_t data) {
    uint32_t packet = ((address << 19) & 0x1F) | ((DAC_WRITE_CMD << 17) & 0x03) | ((data << 0) & 0xFFFF);
    void AudioDAC_Select(void);

    SPI_WriteSingle(SPI_B_BASE, packet >> 12);
    SPI_WriteSingle(SPI_B_BASE, packet);

    void AudioDAC_Deselect(void);
}

// AudioDAC_Read
// Gets register value from DAC
// Return: uint32_t
uint32_t AudioDAC_Read(uint32_t address) {
    uint32_t packet = ((address << 19) & 0x1F) | ((DAC_READ_CMD << 17) & 0x03);
    uint32_t data;

    AudioDAC_Select();

    SPI_WriteSingle(SPI_B_BASE, packet >> 12);
    SPI_WriteSingle(SPI_B_BASE, 0);

    SSIDataGet(SPI_B_BASE, &data);

    AudioDAC_Deselect();

    return data & 0xFFFF; // extract the 16-bit data
}

// AudioDAC_Select
// Selects DAC chip
// Return: void
void AudioDAC_Select(void) {
    GPIOPinWrite(GPIO_PORTF_BASE, DAC_CS, 0x00);
}

// AudioDAC_Deselect
// Deselects DAC chip
// Return: void
void AudioDAC_Deselect(void) {
    GPIOPinWrite(GPIO_PORTF_BASE, DAC_CS, 0xFF);
}

// Audio_PCA9555_Write
// Writes to audio PCA9555 register
// Return: void
void Audio_PCA9555_Write(uint8_t reg, uint8_t data) {
    uint8_t bytes[] = {reg, data};
    I2C_WriteMultiple(I2C_B_BASE, AUDIO_PCA9555_GPIO_ADDR, bytes, 2);
}

// Audio_PCA9555_Write
// Reads from audio PCA9555 register
// Return: uint8_t
uint8_t Audio_PCA9555_Read(uint8_t reg) {
    I2C_WriteSingle(I2C_B_BASE, AUDIO_PCA9555_GPIO_ADDR, reg);
    return I2C_ReadSingle(I2C_B_BASE, AUDIO_PCA9555_GPIO_ADDR);
}
/********************************Public Functions***********************************/

