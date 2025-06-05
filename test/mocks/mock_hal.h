/**
 * @file mock_hal.h
 * @brief Mock HAL layer for testing
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Mock HAL Status */
typedef enum {
    HAL_OK       = 0x00U,
    HAL_ERROR    = 0x01U,
    HAL_BUSY     = 0x02U,
    HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

/* Mock GPIO */
typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET
} GPIO_PinState;

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
    uint32_t Alternate;
} GPIO_InitTypeDef;

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

/* Mock CAN */
typedef struct {
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
    uint8_t Data[8];
} CAN_TxHeaderTypeDef;

typedef struct {
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
    uint32_t FilterMatchIndex;
} CAN_RxHeaderTypeDef;

typedef struct {
    volatile uint32_t MCR;
    volatile uint32_t MSR;
    volatile uint32_t TSR;
    volatile uint32_t RF0R;
    volatile uint32_t RF1R;
    volatile uint32_t IER;
    volatile uint32_t ESR;
    volatile uint32_t BTR;
} CAN_TypeDef;

typedef struct {
    CAN_TypeDef* Instance;
    /* Other fields... */
} CAN_HandleTypeDef;

/* Mock UART */
typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

typedef struct {
    USART_TypeDef* Instance;
    /* Other fields... */
} UART_HandleTypeDef;

/* Mock SPI */
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} SPI_TypeDef;

typedef struct {
    SPI_TypeDef* Instance;
    /* Other fields... */
} SPI_HandleTypeDef;

/* Mock ADC */
typedef struct {
    volatile uint32_t SR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t JOFR1;
    volatile uint32_t JOFR2;
    volatile uint32_t JOFR3;
    volatile uint32_t JOFR4;
    volatile uint32_t HTR;
    volatile uint32_t LTR;
    volatile uint32_t SQR1;
    volatile uint32_t SQR2;
    volatile uint32_t SQR3;
    volatile uint32_t JSQR;
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t JDR4;
    volatile uint32_t DR;
} ADC_TypeDef;

typedef struct {
    ADC_TypeDef* Instance;
    /* Other fields... */
} ADC_HandleTypeDef;

/* Mock peripheral instances */
extern GPIO_TypeDef* GPIOA;
extern GPIO_TypeDef* GPIOB;
extern GPIO_TypeDef* GPIOC;
extern CAN_TypeDef* CAN1;
extern USART_TypeDef* USART1;
extern USART_TypeDef* USART2;
extern SPI_TypeDef* SPI1;
extern SPI_TypeDef* SPI2;
extern ADC_TypeDef* ADC1;

/* Mock function declarations */
HAL_StatusTypeDef HAL_Init(void);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);

/* GPIO Mock Functions */
void HAL_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_Init);
void HAL_GPIO_DeInit(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/* CAN Mock Functions */
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* pHeader, 
                                       uint8_t aData[], uint32_t* pTxMailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef* hcan, uint32_t RxFifo, 
                                       CAN_RxHeaderTypeDef* pHeader, uint8_t aData[]);

/* UART Mock Functions */
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef* huart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef* huart, uint8_t* pData, 
                                    uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef* huart, uint8_t* pData, 
                                   uint16_t Size, uint32_t Timeout);

/* SPI Mock Functions */
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef* hspi);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef* hspi, uint8_t* pData, 
                                   uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef* hspi, uint8_t* pData, 
                                  uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* pTxData, 
                                          uint8_t* pRxData, uint16_t Size, uint32_t Timeout);

/* ADC Mock Functions */
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc);

/* Mock control functions for testing */
void mock_hal_reset(void);
void mock_hal_set_tick(uint32_t tick);
void mock_hal_set_gpio_pin_state(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState state);
void mock_hal_inject_can_message(uint32_t id, uint8_t* data, uint8_t length);
void mock_hal_set_adc_value(uint32_t value);
void mock_hal_set_next_status(HAL_StatusTypeDef status);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_HAL_H */