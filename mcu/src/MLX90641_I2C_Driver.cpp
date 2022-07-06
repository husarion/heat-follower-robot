/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "MLX90641_I2C_Driver.h"

I2C_HandleTypeDef i2c;
GPIO_InitTypeDef i2c_gpio;
void MLX90641_I2CInit()
{
      HAL_Init();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

    i2c_gpio.Mode = GPIO_MODE_AF_OD;
    i2c_gpio.Alternate = GPIO_AF4_I2C1;
    i2c_gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    i2c_gpio.Pull = GPIO_NOPULL;
    i2c_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &i2c_gpio);

    i2c.Instance = I2C1;
    i2c.Init.ClockSpeed = 1000000;
    i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    i2c.Init.OwnAddress1 = 0xFF;
    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c.Init.OwnAddress2 = 0xFF;
    i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&i2c);
}

int MLX90641_I2CGeneralReset(void)
{    
    int ack;
    char cmd[2] = {0,0};
    
    cmd[0] = 0x00;
    cmd[1] = 0x06;    
    HAL_I2C_Master_Transmit(&i2c,cmd[0], (uint8_t*)&cmd[1], 1, HAL_MAX_DELAY);
    return 0;

}

int MLX90641_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
     uint8_t sa;                           
    int ack = 0;                               
    int cnt = 0;
    int i = 0;
    char cmd[2] = {0,0};
    char i2cData[1664] = {0};
    uint16_t *p;
    
    p = data;
    sa = (slaveAddr << 1);
    cmd[0] = startAddress >> 8;
    cmd[1] = startAddress & 0x00FF;
     
    HAL_I2C_Mem_Read(&i2c, sa, startAddress, 2, (uint8_t *)&i2cData, 2*nMemAddressRead, HAL_MAX_DELAY);
          
    for(cnt=0; cnt < nMemAddressRead; cnt++)
    {
        i = cnt << 1;
        *p++ = (uint16_t)i2cData[i]*256 + (uint16_t)i2cData[i+1];
    }
    return 0;
} 

void MLX90641_I2CFreqSet(int freq)
{
    HAL_I2C_DeInit(&i2c);
    i2c.Init.ClockSpeed = freq;
    i2c.Instance = I2C1;
    i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    i2c.Init.OwnAddress1 = 0xFF;
    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c.Init.OwnAddress2 = 0xFF;
    i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&i2c);
}

int MLX90641_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    uint8_t sa;
    int ack = 0;
    char cmd[4] = {0,0,0,0};
    static uint16_t dataCheck;

    sa = (slaveAddr << 1);
    cmd[0] = writeAddress >> 8;
    cmd[1] = writeAddress & 0x00FF;
    cmd[2] = data >> 8;
    cmd[3] = data & 0x00FF;

     HAL_I2C_Master_Transmit(&i2c,sa, (uint8_t*)&cmd, 4, HAL_MAX_DELAY);
   // HAL_I2C_Mem_Write(&i2c, slaveAddr, writeAddress, 1, (uint8_t*)&data, 1,HAL_MAX_DELAY);
    MLX90641_I2CRead(slaveAddr,writeAddress,1, &dataCheck);
    
    if ( dataCheck != data)
    {
        return -2;
    }    
    return -1;

}

