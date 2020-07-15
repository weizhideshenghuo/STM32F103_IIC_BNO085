#include "i2c_hal.h"

/*
PB6->SCL
PB7->SDA
PA15->BNO_RST
PB4->HINT
PB5->BOOTN

IIC Address:0x4A(when SA0_MOSI is low)
CLKSEL0 set high to use internal clock
*/

#define RSTN_PORT GPIOA
#define RSTN_PIN  GPIO_PIN_15

#define INTN_PORT GPIOB
#define INTN_PIN GPIO_PIN_4

#define BOOTN_PORT GPIOB
#define BOOTN_PIN  GPIO_PIN_5

// Keep reset asserted this long.
// (Some targets have a long RC decay on reset.)
#define RESET_DELAY_US (10000)

// Wait up to this long to see first interrupt from SH
#define START_DELAY_US (2000000)

// Wait this long before assuming bootloader is ready
#define DFU_BOOT_DELAY_US (50000)

// How many bytes to read when reading the length field
#define READ_LEN (2)

// ------------------------------------------------------------------------
// Private types

enum BusState_e {
    BUS_INIT,
    BUS_IDLE,
    BUS_READING_LEN,
    BUS_GOT_LEN,
    BUS_READING_TRANSFER,
    BUS_WRITING,
    BUS_READING_DFU,
    BUS_WRITING_DFU,
};

#define ADDR_SH2_0 (0x4A)
#define ADDR_SH2_1 (0x4B)

#define ADDR_DFU_0 (0x28)
#define ADDR_DFU_1 (0x29)

// ------------------------------------------------------------------------
// Private data

static bool isOpen = false;

enum BusState_e i2cBusState;

volatile uint32_t rxTimestamp_uS;            // timestamp of INTN event

// Receive Buffer
static uint8_t rxBuf[SH2_HAL_MAX_TRANSFER_IN];      // data
static uint32_t rxBufLen;   // valid bytes stored in rxBuf (0 when buf empty)
static uint16_t payloadLen;

// Transmit buffer
static uint8_t txBuf[SH2_HAL_MAX_TRANSFER_OUT];

// True after INTN observed, until read starts
static bool rxDataReady;
static uint32_t discards = 0;

// I2C Addr (in 7 MSB positions)
static uint16_t i2cAddr;

// True between asserting reset and seeing first INTN assertion
static volatile bool inReset;

static sh2_Hal_t sh2Hal;

// ------------------------------------------------------------------------
// Private methods

void enableInts(void)
{
    // Enable INTN interrupt
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    // Enable I2C interrupts
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void disableInts(void)
{
    // Disable I2C interrupts
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);

    // Disable INTN interrupt line
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
}

static void enableI2cInts(void)
{
    // Enable I2C interrupts
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

static void disableI2cInts(void)
{
    // Disable I2C interrupts
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
}

static void bootn(bool state)
{
    HAL_GPIO_WritePin(BOOTN_PORT, BOOTN_PIN, 
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void rstn(bool state)
{
    HAL_GPIO_WritePin(RSTN_PORT, RSTN_PIN, 
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t timeNowUs(void)
{
    return __HAL_TIM_GET_COUNTER(&htim1);
}

static void delay_us(uint32_t t)
{
    uint32_t now = timeNowUs();
    uint32_t start = now;
    while ((now - start) < t)
    {
        now = timeNowUs();
    }
}

static void reset_delay_us(uint32_t t)
{
    uint32_t now = timeNowUs();
    uint32_t start = now;
    while (((now - start) < t) && (inReset))
    {
        now = timeNowUs();
    }
}

// ----------------------------------------------------------------------------------
// Callbacks for ISR, I2C Operations
// ----------------------------------------------------------------------------------

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *pI2c)
{
    // Read completed
    if (i2cBusState == BUS_READING_LEN)
    {
        // Len of payload is available, decide how long to do next read
        uint16_t len = (rxBuf[0] + (rxBuf[1] << 8)) & ~0x8000;
        if (len > sizeof(rxBuf))
        {
            // read only what will fit in rxBuf
            payloadLen = sizeof(rxBuf);
        }
        else
        {
            payloadLen = len;
        }
        i2cBusState = BUS_GOT_LEN;
    }
    else if (i2cBusState == BUS_READING_TRANSFER)
    {
        // rxBuf is now ready for client.
        rxBufLen = payloadLen;

        // Nothing left to do
        i2cBusState = BUS_IDLE;
    }
    else if (i2cBusState == BUS_READING_DFU)
    {
        // Transition back to idle state
        rxBufLen = payloadLen;
        i2cBusState = BUS_IDLE;
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *i2c)
{
    if (i2cBusState == BUS_WRITING)
    {
        // Switch back to bus idle
        i2cBusState = BUS_IDLE;
    }
    else if (i2cBusState == BUS_WRITING_DFU)
    {
        // Switch back to bus idle
        i2cBusState = BUS_IDLE;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t n)
{
    if (i2cBusState == BUS_INIT)
    {
        // No active hal, ignore this call, don't crash.
        return;
    }
    
    rxTimestamp_uS = timeNowUs();
    inReset = false;

    // Start read, if possible
    if (i2cBusState == BUS_IDLE)
    {
        if (rxBufLen > 0)
        {
            // Discard earlier payload!
            discards++;
            rxBufLen = 0;
        }
        
        // Read payload len
        i2cBusState = BUS_READING_LEN;
        HAL_I2C_Master_Receive_IT(&hi2c1, i2cAddr, rxBuf, READ_LEN);
    }
    else if (i2cBusState == BUS_GOT_LEN)
    {
        // Read payload
        i2cBusState = BUS_READING_TRANSFER;
        HAL_I2C_Master_Receive_IT(&hi2c1, i2cAddr, rxBuf, payloadLen);
    }
    else
    {
        // We can't start read immediately, set flag so it gets done later.
        rxDataReady = true;
    }
}

/*
// Handle INTN Interrupt through STM32 HAL
// (It, in turn, calls HAL_GPIO_EXTI_Callback, above)
void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

// Handle I2C1 EV IRQ, passing it to STM32 HAL library
void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

// Handle I2C1 ER IRQ, passing it to STM32 HAL library
void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}
*/

// ------------------------------------------------------------------------
// SH2 HAL Methods

static int sh2_i2c_hal_open(sh2_Hal_t *self)
{
    if (isOpen)
    {
        return SH2_ERR;
    }

    i2cBusState = BUS_INIT;
    i2cAddr = ADDR_SH2_0 << 1;

    isOpen = true;

    // Hold in reset, not for DFU
    rstn(false);

    inReset = true;  // will change back to false when INTN serviced

    enableInts();

    // Delay for RESET_DELAY_US to ensure reset takes effect
    delay_us(RESET_DELAY_US);
    
    // transition to idle state
    i2cBusState = BUS_IDLE;

    // Clear rx, tx buffers
    rxBufLen = 0;
    rxDataReady = false;

    // Deassert BOOT, don't go into bootloader
    bootn(true);
    
    // Deassert reset
    rstn(true);

    // Wait for INTN to be asserted
    reset_delay_us(START_DELAY_US);

    return SH2_OK;
}

static void sh2_i2c_hal_close(sh2_Hal_t *self)
{
    // Hold sensor hub in reset
    rstn(false);
    bootn(true);

    i2cBusState = BUS_INIT;

    // Disable interrupts
    disableInts();
    
    // Deinit I2C peripheral
    HAL_I2C_MspDeInit(&hi2c1);
    
    // Deinit timer
    HAL_TIM_Base_MspDeInit(&htim1);
    
    isOpen = false;
}

static int sh2_i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{
    int retval = 0;
    
    disableInts();
    if (rxBufLen > 0)
    {
        // There is data to be had.
        if (len < rxBufLen)
        {
            // Client buffer too small!
            // Discard what was read
            rxBufLen = 0;
            retval = SH2_ERR_BAD_PARAM;
        }
        else
        {
            // Copy data to the client buffer
            memcpy(pBuffer, rxBuf, rxBufLen);
            retval = rxBufLen;
            rxBufLen = 0;
            *t = rxTimestamp_uS;
        }
    }
    enableInts();

    // if more data is ready, start reading it
    if (rxDataReady)
    {
        if ((i2cBusState == BUS_IDLE))
        {
            rxDataReady = false;
            i2cBusState = BUS_READING_LEN;
            HAL_I2C_Master_Receive_IT(&hi2c1, i2cAddr, rxBuf, READ_LEN);
        }
        else if ((i2cBusState == BUS_GOT_LEN))
        {
            rxDataReady = false;
            i2cBusState = BUS_READING_TRANSFER;
            HAL_I2C_Master_Receive_IT(&hi2c1, i2cAddr, rxBuf, payloadLen);
        }
    }

    return retval;
}

static int sh2_i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    int retval = 0;
    
    // Validate parameters
    if ((pBuffer == 0) || (len == 0) || (len > sizeof(txBuf)))
    {
        return SH2_ERR_BAD_PARAM;
    }

    // Disable I2C Interrupt for a moment so busState can't change
    disableI2cInts();
    
    if (i2cBusState == BUS_IDLE)
    {
        i2cBusState = BUS_WRITING;

        // Set up write operation
        memcpy(txBuf, pBuffer, len);
        HAL_I2C_Master_Transmit_IT(&hi2c1, i2cAddr, txBuf, len);

        retval = len;
    }

    // re-enable interrupts
    enableI2cInts();
    
    return retval;
}

static uint32_t sh2_i2c_hal_getTimeUs(sh2_Hal_t *self)
{
    return timeNowUs();
}

// ------------------------------------------------------------------------
// Public methods

sh2_Hal_t *sh2_hal_init(void)
{
    // Set up the HAL reference object for the client
    sh2Hal.open = sh2_i2c_hal_open;
    sh2Hal.close = sh2_i2c_hal_close;
    sh2Hal.read = sh2_i2c_hal_read;
    sh2Hal.write = sh2_i2c_hal_write;
    sh2Hal.getTimeUs = sh2_i2c_hal_getTimeUs;

    return &sh2Hal;
}
