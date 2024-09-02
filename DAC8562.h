#ifndef __DAC8562_H__
#define __DAC8562_H__

#include "main.h"  // Ensure this includes the STM32 HAL header files

#define CMD_SETA_UPDATEA          0x18
#define CMD_SETB_UPDATEB          0x19
#define CMD_UPDATE_ALL_DACS       0x0F
#define CMD_GAIN                  0x02
#define DATA_GAIN_B2_A2           0x0000
#define DATA_GAIN_B2_A1           0x0001
#define DATA_GAIN_B1_A2           0x0002
#define DATA_GAIN_B1_A1           0x0003

#define CMD_PWR_UP_A_B            0x20
#define DATA_PWR_UP_A_B           0x0003

#define CMD_RESET_ALL_REG         0x28
#define DATA_RESET_ALL_REG        0x0001

#define CMD_LDAC_DIS              0x30
#define DATA_LDAC_DIS             0x0003

#define CMD_INTERNAL_REF_DIS      0x38
#define DATA_INTERNAL_REF_DIS     0x0000
#define CMD_INTERNAL_REF_EN       0x38
#define DATA_INTERNAL_REF_EN      0x0001

typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    float vref;
} DAC8562;

void DAC8562_Init(DAC8562* dac, SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, float vref);
void DAC8562_WriteRegister(DAC8562* dac, uint8_t cmd_byte, uint16_t data_byte);
void DAC8562_SetValue(DAC8562* dac, uint8_t cmd_byte, uint16_t input);
void DAC8562_WriteVoltageA(DAC8562* dac, float input);
void DAC8562_WriteVoltageB(DAC8562* dac, float input);
void DAC8562_DAC_A(DAC8562* dac, uint16_t value);
void DAC8562_DAC_B(DAC8562* dac, uint16_t value);
void DAC8562_WriteA(DAC8562* dac, float input);
void DAC8562_WriteB(DAC8562* dac, float input);
uint16_t DAC8562_VoltageToValue(DAC8562* dac, float voltage);

#endif
