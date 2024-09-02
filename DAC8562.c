/*
 * DAC8562.c
 *
 *  Created on: Sep 2, 2024
 *      Author: nayim
 */
#include "DAC8562.h"

void DAC8562_Init(DAC8562* dac, SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, float vref) {
    dac->hspi = hspi;
    dac->cs_port = cs_port;
    dac->cs_pin = cs_pin;
    dac->vref = vref;

    // Initialize the DAC8562 with default settings
    DAC8562_WriteRegister(dac, CMD_RESET_ALL_REG, DATA_RESET_ALL_REG);      // reset
    DAC8562_WriteRegister(dac, CMD_PWR_UP_A_B, DATA_PWR_UP_A_B);        // power up
    DAC8562_WriteRegister(dac, CMD_INTERNAL_REF_EN, DATA_INTERNAL_REF_EN);      // enable internal reference
    DAC8562_WriteRegister(dac, CMD_GAIN, DATA_GAIN_B2_A2);            // set multiplier
    DAC8562_WriteRegister(dac, CMD_LDAC_DIS, DATA_LDAC_DIS);          // update the caches

    // Power up DAC-A and DAC-B

}

void DAC8562_WriteRegister(DAC8562* dac, uint8_t cmd_byte, uint16_t data_byte) {
    uint8_t txData[3];
    txData[0] = cmd_byte;
    txData[1] = (data_byte >> 8) & 0xFF;
    txData[2] = data_byte & 0xFF;
    HAL_GPIO_WritePin(LATCH_PIN_GPIO_Port, LATCH_PIN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dac->hspi, txData, sizeof(txData), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LATCH_PIN_GPIO_Port, LATCH_PIN_Pin, GPIO_PIN_SET);

}

void DAC8562_SetValue(DAC8562* dac, uint8_t cmd_byte, uint16_t input) {
    DAC8562_WriteRegister(dac, cmd_byte, input);
}

void DAC8562_WriteVoltageA(DAC8562* dac, float input) {
    uint16_t value = DAC8562_VoltageToValue(dac, input);
    DAC8562_SetValue(dac, CMD_SETA_UPDATEA, value);
}
void DAC8562_WriteVoltageB(DAC8562* dac, float input) {
    uint16_t value = DAC8562_VoltageToValue(dac, input);
    DAC8562_SetValue(dac, CMD_SETB_UPDATEB, value);
}



void DAC8562_DAC_A(DAC8562* dac, uint16_t value) {
    DAC8562_SetValue(dac, CMD_SETA_UPDATEA, value);
}

void DAC8562_DAC_B(DAC8562* dac, uint16_t value) {
    DAC8562_SetValue(dac, CMD_SETB_UPDATEB, value);
}

void DAC8562_WriteA(DAC8562* dac, float input) {
    uint16_t value = DAC8562_VoltageToValue(dac, input);
    DAC8562_SetValue(dac, CMD_SETA_UPDATEA, value);
}

void DAC8562_WriteB(DAC8562* dac, float input) {
    uint16_t value = DAC8562_VoltageToValue(dac, input);
    DAC8562_SetValue(dac, CMD_SETB_UPDATEB, value);
}

uint16_t DAC8562_VoltageToValue(DAC8562* dac, float voltage) {
    // Convert voltage to DAC value
    return (uint16_t)((voltage / (2 * dac->vref)) * 65535);
}


