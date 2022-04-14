#include "main.h"
#include "stm32f4xx.h"

#define GPIOBEN				(1U<<1)
#define I2C1EN				(1U<<21)

#define MPU9250_ADDR		0x68<<1U
#define WHO_AM_I_REG		0x75U
#define PWR_MGMT_1_REG		0x6BU
#define SMPLRT_DIV_REG		0x19U
#define ACCEL_CONFIG_REG	0x1CU
#define GYRO_CONFIG_REG		0x1BU
#define ACCEL_XOUT_H_REG	0x43U
#define GYRO_XOUT_H_REG		0x3BU
#define TEMP_OUT_H_REG		0x41U

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

int16_t Temperature_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz, Temperature;

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
}

void TIM5Config (void)
{
	RCC->APB1ENR |= (1<<3);
	TIM5->PSC = 90-1;
	TIM5->ARR = 0xffff;
	TIM5->CR1 |= (1<<0);
	while (!(TIM5->SR & (1<<0)));
}

void Delay_us (uint16_t us)
{

	TIM5->CNT = 0;
	while (TIM5->CNT < us);
}

void Delay_ms (uint16_t ms)
{
	for (uint16_t i=0; i<ms; i++)
	{
		Delay_us (1000); // delay of 1 ms
	}
}

/**** STEPS FOLLOWED  ************
1. Enable the I2C CLOCK and GPIO CLOCK
2. Configure the I2C PINs for ALternate Functions
	a) Select Alternate Function in MODER Register
	b) Select Open Drain Output
	c) Select High SPEED for the PINs
	d) Select Pull-up for both the Pins
	e) Configure the Alternate Function in AFR Register
3. Reset the I2C
4. Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
5. Configure the clock control registers
6. Configure the rise time register
7. Program the I2C_CR1 register to enable the peripheral
*/
//Pins P8 and P9

void I2C_Config (void)
{
/**** STEPS FOLLOWED  ************
1. Enable the I2C CLOCK and GPIO CLOCK
2. Configure the I2C PINs for ALternate Functions
	a) Select Alternate Function in MODER Register
	b) Select Open Drain Output
	c) Select High SPEED for the PINs
	d) Select Pull-up for both the Pins
	e) Configure the Alternate Function in AFR Register
3. Reset the I2C
4. Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
5. Configure the clock control registers
6. Configure the rise time register
7. Program the I2C_CR1 register to enable the peripheral
*/

	// Enable the I2C CLOCK and GPIO CLOCK
	RCC->APB1ENR |= (1<<21);  // enable I2C CLOCK
	RCC->AHB1ENR |= (1<<1);  // Enable GPIOB CLOCK


	// Configure the I2C PINs for ALternate Functions
	GPIOB->MODER |= (2<<12) | (2<<18);  // Bits (17:16)= 1:0 --> Alternate Function for Pin PB8; Bits (19:18)= 1:0 --> Alternate Function for Pin PB9
	GPIOB->OTYPER |= (1<<6) | (1<<9);  //  Bit8=1, Bit9=1  output open drain
	GPIOB->OSPEEDR |= (3<<12) | (3<<18);  // Bits (17:16)= 1:1 --> High Speed for PIN PB8; Bits (19:18)= 1:1 --> High Speed for PIN PB9
	GPIOB->PUPDR |= (1<<12) | (1<<18);  // Bits (17:16)= 0:1 --> Pull up for PIN PB8; Bits (19:18)= 0:1 --> pull up for PIN PB9
	GPIOB->AFR[1] |= (4<<4);  // Bits (3:2:1:0) = 0:1:0:0 --> AF4 for pin PB8;  Bits (7:6:5:4) = 0:1:0:0 --> AF4 for pin PB9
	GPIOB->AFR[0] |= (4<<24);  // Bits (3:2:1:0) = 0:1:0:0 --> AF4 for pin PB8;  Bits (7:6:5:4) = 0:1:0:0 --> AF4 for pin PB9


	// Reset the I2C
	I2C1->CR1 |= (1<<15);
	I2C1->CR1 &= ~(1<<15);

	// Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
	I2C1->CR2 |= (36<<0);  // PCLK1 FREQUENCY in MHz

	// Configure the clock control registers
	I2C1->CCR = 180<<0;  // check calculation in PDF

	// Configure the rise time register
	I2C1->TRISE = 37;  // check PDF again

	// Program the I2C_CR1 register to enable the peripheral
	I2C1->CR1 |= (1<<0);  // Enable I2C
}

void I2C_Start (void)
{
/**** STEPS FOLLOWED  ************
1. Send the START condition
2. Wait for the SB ( Bit 0 in SR1) to set. This indicates that the start condition is generated
*/

	I2C1->CR1 |= (1<<10);  // Enable the ACK
	I2C1->CR1 |= (1<<8);  // Generate START
	while (!(I2C1->SR1 & (1<<0)));  // Wait for SB bit to set
}

void I2C_Write (uint8_t data)
{
/**** STEPS FOLLOWED  ************
1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
2. Send the DATA to the DR Register
3. Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
*/
	while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
	I2C1->DR = data;
	while (!(I2C1->SR1 & (1<<2)));  // wait for BTF bit to set
}

void I2C_Address (uint8_t Address)
{
/**** STEPS FOLLOWED  ************
1. Send the Slave Address to the DR Register
2. Wait for the ADDR (bit 1 in SR1) to set. This indicates the end of address transmission
3. clear the ADDR by reading the SR1 and SR2
*/
	I2C1->DR = Address;  //  send the address
	while (!(I2C1->SR1 & (1<<1U)));  // wait for ADDR bit to set
	uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
}

void I2C_Stop (void)
{
	I2C1->CR1 |= (1<<9);  // Stop I2C
}

void I2C_WriteMulti (uint8_t *data, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
2. Keep Sending DATA to the DR Register after performing the check if the TXE bit is set
3. Once the DATA transfer is complete, Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
*/
	while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
	while (size)
	{
		while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
		I2C1->DR = (uint32_t )*data++;  // send data
		size--;
	}

	while (!(I2C1->SR1 & (1<<2)));  // wait for BTF to set
}

void I2C_Read (uint8_t Address, uint8_t *buffer, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. If only 1 BYTE needs to be Read
	a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
	c) Wait for the RXNE (Receive Buffer not Empty) bit to set
	d) Read the data from the DR

2. If Multiple BYTES needs to be read
  a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) Clear the ADDR bit by reading the SR1 and SR2 Registers
	c) Wait for the RXNE (Receive buffer not empty) bit to set
	d) Read the data from the DR
	e) Generate the Acknowlegment by settint the ACK (bit 10 in SR1)
	f) To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the
		 second last data byte (after second last RxNE event)
	g) In order to generate the Stop/Restart condition, software must set the STOP/START bit
	   after reading the second last data byte (after the second last RxNE event)
*/

	int remaining = size;

/**** STEP 1 ****/
	if (size == 1)
	{
		/**** STEP 1-a ****/
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set

		/**** STEP 1-b ****/
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit
		uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition
		I2C1->CR1 |= (1<<9);
		while (!(I2C1->SR1 & (1<<6)));
		buffer[size-remaining] = I2C1->DR;

	}

	else
	{
		I2C1->DR = Address;
		while (!(I2C1->SR1 & (1<<1)));
		uint8_t temp = I2C1->SR1 | I2C1->SR2;

		while (remaining>2)
		{
			while (!(I2C1->SR1 & (1<<6)));
			buffer[size-remaining] = I2C1->DR;
			I2C1->CR1 |= 1<<10;

			remaining--;
		}
		while (!(I2C1->SR1 & (1<<6)));
		buffer[size-remaining] = I2C1->DR;

		I2C1->CR1 &= ~(1<<10);
		I2C1->CR1 |= (1<<9);

		remaining--;

		while (!(I2C1->SR1 & (1<<6)));
		buffer[size-remaining] = I2C1->DR;
	}

}


void MPU_Write (uint8_t Address, uint8_t Reg, uint8_t Data)
{
	I2C_Start ();
	I2C_Address (Address);
	I2C_Write (Reg);
	I2C_Write (Data);
	I2C_Stop ();
}

void MPU_Read (uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size)
{
	I2C_Start ();
	I2C_Address (Address);
	I2C_Write (Reg);
	I2C_Start ();
	I2C_Read (Address+0x01, buffer, size);
	I2C_Stop ();
}

void MPU9250_Init (void)
{
	uint8_t check;
	uint8_t Data;

	MPU_Read (MPU9250_ADDR,WHO_AM_I_REG, &check, 1);

	if (check == 104)
	{
		Data = 0;
		MPU_Write (MPU9250_ADDR, PWR_MGMT_1_REG, Data);

		Data = 0x07;
		MPU_Write(MPU9250_ADDR, SMPLRT_DIV_REG, Data);

		Data = 0x00;
		MPU_Write(MPU9250_ADDR, ACCEL_CONFIG_REG, Data);

		Data = 0x00;
		MPU_Write(MPU9250_ADDR, GYRO_CONFIG_REG, Data);
	}

}

void MPU9250_Read_Accel(int16_t *Accel_X_RAW, int16_t *Accel_Y_RAW, int16_t *Accel_Z_RAW, float *Ax, float *Ay, float *Az){
	uint8_t Rx_data[6];

	MPU_Read(MPU9250_ADDR, ACCEL_XOUT_H_REG, Rx_data, 6);

	*Accel_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data [1]);
	*Accel_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data [3]);
	*Accel_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data [5]);

	*Ax = *Accel_X_RAW/16384.0;
	*Ay = *Accel_Y_RAW/16384.0;
	*Az = *Accel_Z_RAW/16384.0;
}

void MPU9250_Read_Gyro(int16_t *Gyro_X_RAW, int16_t *Gyro_Y_RAW, int16_t *Gyro_Z_RAW, float *Gx, float *Gy, float *Gz){
	uint8_t Rx_data[6];

	MPU_Read(MPU9250_ADDR, GYRO_XOUT_H_REG, Rx_data, 6);

	*Gyro_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);
	*Gyro_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data[3]);
	*Gyro_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data[5]);

	*Gx = *Gyro_X_RAW / 178;
	*Gy = *Gyro_Y_RAW / 178;
	*Gz = *Gyro_Z_RAW / 178;
}

void MPU9250_Read_Temp(int16_t *Temperature_RAW, float *Temperature){
	uint8_t Rx_data[2];

	MPU_Read(MPU9250_ADDR, TEMP_OUT_H_REG, Rx_data, 2);

	*Temperature_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);

	*Temperature = (*Temperature_RAW-21)/333.87 + 17;
}

int main(void)
{
	SystemClock_Config ();
	TIM5Config ();
	I2C_Config ();

	MPU9250_Init ();
	while (1)
	{
	  	MPU9250_Read_Accel(&Accel_X_RAW, &Accel_Y_RAW, &Accel_Z_RAW, &Ax, &Ay, &Az);
	  	MPU9250_Read_Gyro(&Gyro_X_RAW, &Gyro_Y_RAW, &Gyro_Z_RAW, &Gx, &Gy, &Gz);
	  	MPU9250_Read_Temp(&Temperature_RAW, &Temperature);
		Delay_ms (1);
	}
}


