#include "Functions.h"
char str[20];

uint8_t DRV_read(SPI_HandleTypeDef *hspi1,  UART_HandleTypeDef *huart1){
		uint8_t serialData[5] = {0};
		uint8_t regData[5] = {0};
		char buf[50];
	
		serialData[0] = 0x80;
	  serialData[1] = 0x00;
	  serialData[2] = 0x00;
	 
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  HAL_SPI_Transmit (hspi1, serialData , 1, 10);

	  HAL_SPI_Receive (hspi1, regData, 2, 10);	
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		
		sprintf(buf, "%X %X\n", regData[0], regData[1] );

		HAL_UART_Transmit_IT(huart1, (uint8_t*) buf, 10);	
		HAL_Delay(300);

    ////////////////////////////		
		serialData[0] = 0x81;
	  serialData[1] = 0x00;
	  serialData[2] = 0x00;
	 
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  HAL_SPI_Transmit (hspi1, serialData , 1, 10);

	  HAL_SPI_Receive (hspi1, regData, 2, 10);	
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		
		sprintf(buf, "%X %X\n", regData[0], regData[1] );

		HAL_UART_Transmit_IT(huart1, (uint8_t*) buf, 10);	
		HAL_Delay(300);
		
		///////////////////////////////
		serialData[0] = 0x82;
	  serialData[1] = 0x00;
	  serialData[2] = 0x00;
	 
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  HAL_SPI_Transmit (hspi1, serialData , 1, 10);

	  HAL_SPI_Receive (hspi1, regData, 2, 10);	
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		
		sprintf(buf, "%X %X\n", regData[0], regData[1] );

		HAL_UART_Transmit_IT(huart1, (uint8_t*) buf, 10);	
		HAL_Delay(300);
    /////////////////////////////////////
		serialData[0] = 0x83;
	  serialData[1] = 0x00;
	  serialData[2] = 0x00;
	 
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  HAL_SPI_Transmit (hspi1, serialData , 1, 10);

	  HAL_SPI_Receive (hspi1, regData, 2, 10);	
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		
		sprintf(buf, "%X %X\n", regData[0], regData[1] );

		HAL_UART_Transmit_IT(huart1, (uint8_t*) buf, 10);	
		HAL_Delay(300);
		
		////////////////////////////////////
		serialData[0] = 0x84;
	  serialData[1] = 0x00;
	  serialData[2] = 0x00;
	 
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  HAL_SPI_Transmit (hspi1, serialData , 1, 10);

	  HAL_SPI_Receive (hspi1, regData, 2, 10);	
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		
		sprintf(buf, "%X %X\n", regData[0], regData[1] );

		HAL_UART_Transmit_IT(huart1, (uint8_t*) buf, 10);	
		HAL_Delay(300);
		
		////////////////////////////////////////////
		serialData[0] = 0x85;
	  serialData[1] = 0x00;
	  serialData[2] = 0x00;
	 
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  HAL_SPI_Transmit (hspi1, serialData , 1, 10);

	  HAL_SPI_Receive (hspi1, regData, 2, 10);	
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		
		sprintf(buf, "%X %X\n", regData[0], regData[1] );

		HAL_UART_Transmit_IT(huart1, (uint8_t*) buf, 10);	
		HAL_Delay(300);
		
		////////////////////////////////////////////////////
		serialData[0] = 0x86;
	  serialData[1] = 0x00;
	  serialData[2] = 0x00;
	 
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  HAL_SPI_Transmit (hspi1, serialData , 1, 10);

	  HAL_SPI_Receive (hspi1, regData, 2, 10);	
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		
		sprintf(buf, "%X %X\n", regData[0], regData[1] );

		HAL_UART_Transmit_IT(huart1, (uint8_t*) buf, 10);	
		HAL_Delay(300);
		
		///////////////////////////////////////////////////
		serialData[0] = 0x87;
	  serialData[1] = 0x00;
	  serialData[2] = 0x00;
	 
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  HAL_SPI_Transmit (hspi1, serialData , 1, 10);

	  HAL_SPI_Receive (hspi1, regData, 2, 10);	
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		
		sprintf(buf, "%X %X\n", regData[0], regData[1] );

		HAL_UART_Transmit_IT(huart1, (uint8_t*) buf, 10);	
		HAL_Delay(300);
		
		/////////////////////////////////////////////////
		serialData[0] = 0x88;
	  serialData[1] = 0x00;
	  serialData[2] = 0x00;
	 
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  HAL_SPI_Transmit (hspi1, serialData , 1, 10);

	  HAL_SPI_Receive (hspi1, regData, 2, 10);	
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		
		sprintf(buf, "%X %X\n", regData[0], regData[1] );

		HAL_UART_Transmit_IT(huart1, (uint8_t*) buf, 10);	
		HAL_Delay(300);

		///////////////////////////////////
		serialData[0] = 0x89;
	  serialData[1] = 0x00;
	  serialData[2] = 0x00;
	 
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  HAL_SPI_Transmit (hspi1, serialData , 1, 10);

	  HAL_SPI_Receive (hspi1, regData, 2, 10);	
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		
		sprintf(buf, "%X %X\n", regData[0], regData[1] );

		HAL_UART_Transmit_IT(huart1, (uint8_t*) buf, 10);	
		HAL_Delay(300);
		
		////////////////////////////////////
		serialData[0] = 0x8A;
	  serialData[1] = 0x00;
	  serialData[2] = 0x00;
	 
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  HAL_SPI_Transmit (hspi1, serialData , 1, 10);

	  HAL_SPI_Receive (hspi1, regData, 2, 10);	
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		
		sprintf(buf, "%X %X\n", regData[0], regData[1] );

		HAL_UART_Transmit_IT(huart1, (uint8_t*) buf, 10);	
		HAL_Delay(300);
		
		////////////////////////////////
		serialData[0] = 0x8B;
	  serialData[1] = 0x00;
	  serialData[2] = 0x00;
	 
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  HAL_SPI_Transmit (hspi1, serialData , 1, 10);

	  HAL_SPI_Receive (hspi1, regData, 2, 10);	
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		
		sprintf(buf, "%X %X\n", regData[0], regData[1] );

		HAL_UART_Transmit_IT(huart1, (uint8_t*) buf, 10);	
		HAL_Delay(300);
		
	return 1;

}

uint8_t DRV_setup(SPI_HandleTypeDef *hspi1){
	uint32_t SPI_TIMEOUT =  5;
	uint8_t serialData[3];
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	serialData[0] = 0x00;
	serialData[1] = 0x91;      //91
	serialData[2] = 0X51;	     //51
	HAL_SPI_Transmit (hspi1, serialData , 3, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	HAL_Delay(100);
	
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	serialData[0] = 0x01;
	serialData[1] = 0x00;
	serialData[2] = 0X38;	
	HAL_SPI_Transmit (hspi1, serialData , 3, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	HAL_Delay(100);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	serialData[0] = 0x02;
	serialData[1] = 0x03;
	serialData[2] = 0XB4;	
	HAL_SPI_Transmit (hspi1, serialData , 3, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	HAL_Delay(100);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	serialData[0] = 0x03;
	serialData[1] = 0x68;
	serialData[2] = 0X00;	
	HAL_SPI_Transmit (hspi1, serialData , 3, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	HAL_Delay(100);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	serialData[0] = 0x04;
	serialData[1] = 0x02;
	serialData[2] = 0X40;	
	HAL_SPI_Transmit (hspi1, serialData , 3, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	HAL_Delay(100);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	serialData[0] = 0x05;
	serialData[1] = 0x30;
	serialData[2] = 0X07;	
	HAL_SPI_Transmit (hspi1, serialData , 3, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	HAL_Delay(100);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	serialData[0] = 0x06;
	serialData[1] = 0x84;
	serialData[2] = 0XB1;	
	HAL_SPI_Transmit (hspi1, serialData , 3, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	HAL_Delay(100);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	serialData[0] = 0x07;
	serialData[1] = 0x03;
	serialData[2] = 0XBA;	
	HAL_SPI_Transmit (hspi1, serialData , 3, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	HAL_Delay(100);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	serialData[0] = 0x08;
	serialData[1] = 0x01;
	serialData[2] = 0X2C;	
	HAL_SPI_Transmit (hspi1, serialData , 3, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	HAL_Delay(100);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	serialData[0] = 0x09;
	serialData[1] = 0x02;
	serialData[2] = 0X50;	
	HAL_SPI_Transmit (hspi1, serialData , 3, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	HAL_Delay(100);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	serialData[0] = 0x0A;
	serialData[1] = 0xF0;
	serialData[2] = 0X64;	
	HAL_SPI_Transmit (hspi1, serialData , 3, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	HAL_Delay(100);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	serialData[0] = 0x0B;
	serialData[1] = 0x05;
	serialData[2] = 0XDC;	
	HAL_SPI_Transmit (hspi1, serialData , 3, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	HAL_Delay(100);

	
	return 1;
}
uint8_t DRV_start(int input_PWM_DRV){ //return direction
		uint8_t direction;
	  uint16_t PWM_abs;
		
	  PWM_abs = abs(input_PWM_DRV);
	
		if (PWM_abs >= 20 && PWM_abs <= 80){
			_ENABLE_(1);
			HAL_Delay(100);
		}else{
			_ENABLE_(0);		
		}
		
		if (input_PWM_DRV < 0){
			DIR(1);
			direction = 1;
		}else{
			DIR(0);
			direction = 0;
		}																																																																							
		
		if (input_PWM_DRV < 70){
			while (TIM10->CCR1 != PWM_abs){
				if (TIM10->CCR1 < PWM_abs){
					TIM10->CCR1 += 1;
					HAL_Delay(10);
				}
				else{
					TIM10->CCR1 -= 1;
					HAL_Delay(10);
				}
		  }
		}else{
			while(TIM10->CCR1 != 70){
				TIM10->CCR1 += 1;
				HAL_Delay(10);
			}
		}
		
		return direction;
}

uint8_t DRV_start_reverse(int input_PWM_DRV){
		uint8_t direction;
	  uint16_t PWM_abs;
		
	  PWM_abs = abs(input_PWM_DRV);
	
		if (PWM_abs >= 20 && PWM_abs <= 80){
			_ENABLE_(1);
			HAL_Delay(100);
		}else{
			_ENABLE_(0);		
		}
		
		if (input_PWM_DRV < 0){
			DIR(0);
			direction = 1;
		}else{
			DIR(1);
			direction = 0;
		}																																																																							
		
		if (input_PWM_DRV < 70){
			while (TIM10->CCR1 != PWM_abs){
				if (TIM10->CCR1 < PWM_abs){
					TIM10->CCR1 += 1;
					HAL_Delay(10);
				}
				else{
					TIM10->CCR1 -= 1;
					HAL_Delay(10);
				}
		  }
		}else{
			while(TIM10->CCR1 != 70){
				TIM10->CCR1 += 1;
				HAL_Delay(10);
			}
		}
		
		return direction;
}
uint8_t DRV_stack_start(int input_PWM_DRV, struct Stack *stack, UART_HandleTypeDef *huart1){
		uint8_t direction;
	  uint16_t PWM_abs;
		
	  PWM_abs = abs(input_PWM_DRV);
	
		if (PWM_abs >= 20 && PWM_abs <= 80){
			_ENABLE_(1);
			HAL_Delay(100);
		}else{
			_ENABLE_(0);		
		}
		
		if (input_PWM_DRV < 0){
			DIR(1);
			direction = 1;
		}else{
			DIR(0);
			direction = 0;
		}																																																																							
		
		if (input_PWM_DRV < 70){
			while (TIM10->CCR1 != PWM_abs){
				if (TIM10->CCR1 < PWM_abs){
					TIM10->CCR1 += 1;
					push(stack, TIM10->CCR1, huart1);
					HAL_Delay(10);
				}
				else{
					TIM10->CCR1 -= 1;
					push(stack, TIM10->CCR1, huart1);
					HAL_Delay(10);
				}
		  }
		}else{
			while(TIM10->CCR1 != 70){
				TIM10->CCR1 += 1;
				push(stack, TIM10->CCR1, huart1);
				HAL_Delay(10);
			}
		}
		
		return direction;
}

uint8_t SERVO_start(int input_PWM_servo){
			if (input_PWM_servo > 950 && input_PWM_servo < 1500){
			while (TIM11->CCR1 != input_PWM_servo){
				if (TIM11->CCR1 > input_PWM_servo){
					TIM11->CCR1 -= 1;
					HAL_Delay(MAX_DELAY);
				}
				else{
					TIM11->CCR1 += 1;
					HAL_Delay(MAX_DELAY);
				}			
			}
	  }	else{
			if (input_PWM_servo < 950){
				while (TIM11->CCR1 != 950){
					TIM11->CCR1 -= 1;
					HAL_Delay(MAX_DELAY);
				}
			}
			if (input_PWM_servo > 1500){
				while (TIM11->CCR1 != 1500){
					TIM11->CCR1 += 1;
					HAL_Delay(MAX_DELAY);
				}			
			}			
				
		}	
		return 1;

}

uint16_t getAngle(I2C_HandleTypeDef *hi2c1){
	uint8_t I2C_ADDRESS = 0x81;
	uint8_t I2C_TIMEOUT = 100;
	
	uint8_t regData[1];
	uint8_t regAddress[2];
	
	uint8_t low_byte  = 0;
	uint8_t high_byte = 0;
	uint16_t angle    = 0;
	
	regAddress[0] = 0xFE;
	HAL_I2C_Master_Transmit(hi2c1, I2C_ADDRESS, regAddress, 1, I2C_TIMEOUT);
	HAL_I2C_Master_Receive(hi2c1, I2C_ADDRESS, regData, 1, I2C_TIMEOUT);
	low_byte = regData[0];
	
	regAddress[0] = 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_ADDRESS, regAddress, 1, I2C_TIMEOUT);
	HAL_I2C_Master_Receive(hi2c1, I2C_ADDRESS, regData, 1, I2C_TIMEOUT);
	high_byte = regData[0];
	
	angle = ((uint16_t) high_byte << 6) | ((uint16_t) low_byte);
	
	return angle;
}

uint8_t getCount(uint16_t angle){
	uint16_t old_angle;
	uint16_t new_angle;
	uint8_t count = 0;
	
	new_angle = angle;
	
	if (abs(old_angle - new_angle) > 8192){
		if (new_angle > old_angle){
			count += 1;			
		}else{
			count -= 1;
		}
	}
	

	old_angle = new_angle;
		
	return count;	
}
	
uint8_t DIR(uint8_t flag){
	if (flag == 0)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
	else
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
	
	return 1;
} 

uint8_t _ENABLE_(uint8_t flag){
	if (flag == 0)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0);
	else
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1);
	
	return 1;
} 

uint8_t _RESET_(uint8_t flag){
	if (flag == 0)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
	else
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
	
	return 1;
} 

uint8_t push(struct Stack* stack, uint16_t value, UART_HandleTypeDef *huart1) {
		if (stack->size < stack->capacity) {
			stack->data[stack->size++] = value;
			
    }else{
			sprintf (str, "Stack Overflow!!!");
			HAL_UART_Transmit(huart1, (uint8_t*) str, 50, 10);
			HAL_Delay(100);
		}   

		return 1;
}

short  pop(struct Stack* stack, UART_HandleTypeDef *huart1) {
    if (stack->size == 0) {
				sprintf(str, "Cannot pop from empty stack");
				HAL_UART_Transmit(huart1, (uint8_t*) str, 50, 10);
				HAL_Delay(100);
    }

    uint16_t del_element = stack->data[stack->size - 1];
    stack->data[--stack->size] = 0;
    return del_element;
}

uint8_t Stack_ctor_(struct Stack* stack) {
    stack->size     = 0;
    stack->capacity = CAPACITY;
	
    return 1;
}


