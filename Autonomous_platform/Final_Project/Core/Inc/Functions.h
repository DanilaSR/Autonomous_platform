#include <stdio.h>
#include <cstdlib>
#include "stm32f4xx_hal.h"

#define CAPACITY                   350        //Capacity of Stack
#define MAX_DELAY                    2

uint16_t getAngle(I2C_HandleTypeDef *hi2c1);

uint8_t getCount(uint16_t angle);

uint8_t DRV_setup(SPI_HandleTypeDef *hspi1);

uint8_t DRV_read(SPI_HandleTypeDef *hspi1,  UART_HandleTypeDef *huart1);

uint8_t DRV_start(int input_PWM_DRV);

uint8_t DRV_start_reverse(int input_PWM_DRV); 

uint8_t SERVO_start(int input_PWM_servo);

uint8_t print_uart(UART_HandleTypeDef *huart1, char *arr);

uint8_t DIR(uint8_t flag);    //variable flag control state: if flag = 0 => false; else => true

uint8_t _ENABLE_(uint8_t flag);

uint8_t _RESET_(uint8_t flag);

struct Stack {

    short data[CAPACITY];
    uint16_t capacity;
    uint16_t size;
};

uint8_t Stack_ctor_(struct Stack* stack);
	
uint8_t push(struct Stack* stack, uint16_t value, UART_HandleTypeDef *huart1);

short  pop(struct Stack* stack, UART_HandleTypeDef *huart1);

uint8_t DRV_stack_start(int input_PWM_DRV, struct Stack *stack, UART_HandleTypeDef *huart1);


