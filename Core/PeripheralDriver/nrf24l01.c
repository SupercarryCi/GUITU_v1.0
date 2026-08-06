#include "nrf24l01.h"

#include <string.h>

#define NRF24_CMD_R_REGISTER      0x00U
#define NRF24_CMD_W_REGISTER      0x20U
#define NRF24_CMD_R_RX_PAYLOAD    0x61U
#define NRF24_CMD_W_TX_PAYLOAD    0xA0U
#define NRF24_CMD_FLUSH_TX        0xE1U
#define NRF24_CMD_FLUSH_RX        0xE2U
#define NRF24_CMD_NOP             0xFFU

#define NRF24_REG_CONFIG          0x00U
#define NRF24_REG_EN_AA           0x01U
#define NRF24_REG_EN_RXADDR       0x02U
#define NRF24_REG_SETUP_AW        0x03U
#define NRF24_REG_SETUP_RETR      0x04U
#define NRF24_REG_RF_CH           0x05U
#define NRF24_REG_RF_SETUP        0x06U
#define NRF24_REG_STATUS          0x07U
#define NRF24_REG_OBSERVE_TX      0x08U
#define NRF24_REG_RPD             0x09U
#define NRF24_REG_RX_ADDR_P0      0x0AU
#define NRF24_REG_RX_ADDR_P1      0x0BU
#define NRF24_REG_RX_ADDR_P2      0x0CU
#define NRF24_REG_TX_ADDR         0x10U
#define NRF24_REG_RX_PW_P0        0x11U
#define NRF24_REG_RX_PW_P1        0x12U
#define NRF24_REG_RX_PW_P2        0x13U
#define NRF24_REG_FIFO_STATUS     0x17U
#define NRF24_REG_DYNPD           0x1CU
#define NRF24_REG_FEATURE         0x1DU

#define NRF24_CONFIG_EN_CRC       0x08U
#define NRF24_CONFIG_CRCO         0x04U
#define NRF24_CONFIG_PWR_UP       0x02U
#define NRF24_CONFIG_PRIM_RX      0x01U

#define NRF24_STATUS_RX_DR        0x40U
#define NRF24_STATUS_TX_DS        0x20U
#define NRF24_STATUS_MAX_RT       0x10U
#define NRF24_STATUS_IRQ_MASK     0x70U
#define NRF24_STATUS_RX_P_NO_MASK 0x0EU
#define NRF24_FIFO_RX_EMPTY       0x01U

#define NRF24_SETUP_AW_5_BYTES    0x03U
#define NRF24_SETUP_RETR_VALUE    0x5AU
#define NRF24_RF_SETUP_VALUE      0x06U
#define NRF24_TX_TIMEOUT_MS       40U
#define NRF24_POWER_ON_DELAY_MS   100U

static Nrf24l01Config_t s_config;
static uint8_t s_bound;
static uint8_t s_ready;

static void nrf24_ce_write(uint8_t level)
{
    HAL_GPIO_WritePin(s_config.ce_port,
                      s_config.ce_pin,
                      (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void nrf24_csn_write(uint8_t level)
{
    HAL_GPIO_WritePin(s_config.csn_port,
                      s_config.csn_pin,
                      (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int32_t nrf24_exchange(uint8_t command,
                              const uint8_t *tx_data,
                              uint8_t *rx_data,
                              uint8_t length,
                              uint8_t *status)
{
    uint8_t tx[NRF24L01_FRAME_SIZE + 1U];
    uint8_t rx[NRF24L01_FRAME_SIZE + 1U];
    uint8_t i;

    if ((s_bound == 0U) || (length > NRF24L01_FRAME_SIZE))
    {
        return NRF24L01_ERROR_ARGUMENT;
    }

    tx[0] = command;
    for (i = 0U; i < length; i++)
    {
        tx[i + 1U] = (tx_data != NULL) ? tx_data[i] : NRF24_CMD_NOP;
    }

    nrf24_csn_write(0U);
    if (HAL_SPI_TransmitReceive(s_config.hspi,
                                tx,
                                rx,
                                (uint16_t)(length + 1U),
                                s_config.spi_timeout_ms) != HAL_OK)
    {
        nrf24_csn_write(1U);
        return NRF24L01_ERROR_SPI;
    }
    nrf24_csn_write(1U);

    if (status != NULL)
    {
        *status = rx[0];
    }
    if (rx_data != NULL)
    {
        memcpy(rx_data, &rx[1], length);
    }

    return NRF24L01_OK;
}

static int32_t nrf24_read_register(uint8_t reg, uint8_t *data, uint8_t length)
{
    return nrf24_exchange((uint8_t)(NRF24_CMD_R_REGISTER | (reg & 0x1FU)),
                          NULL,
                          data,
                          length,
                          NULL);
}

static int32_t nrf24_write_register(uint8_t reg, const uint8_t *data, uint8_t length)
{
    return nrf24_exchange((uint8_t)(NRF24_CMD_W_REGISTER | (reg & 0x1FU)),
                          data,
                          NULL,
                          length,
                          NULL);
}

static int32_t nrf24_write_register_u8(uint8_t reg, uint8_t value)
{
    return nrf24_write_register(reg, &value, 1U);
}

static int32_t nrf24_command(uint8_t command)
{
    return nrf24_exchange(command, NULL, NULL, 0U, NULL);
}

static int32_t nrf24_read_status(uint8_t *status)
{
    if (status == NULL)
    {
        return NRF24L01_ERROR_ARGUMENT;
    }

    return nrf24_exchange(NRF24_CMD_NOP, NULL, NULL, 0U, status);
}

static int32_t nrf24_enter_rx(void)
{
    int32_t result;
    uint8_t config;

    nrf24_ce_write(0U);
    if (nrf24_write_register_u8(NRF24_REG_EN_RXADDR,
                                s_config.rx_pipe_mask) != NRF24L01_OK)
    {
        return NRF24L01_ERROR_SPI;
    }

    config = NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO |
             NRF24_CONFIG_PWR_UP;
    if (s_config.rx_pipe_mask != 0U)
    {
        config |= NRF24_CONFIG_PRIM_RX;
    }
    result = nrf24_write_register_u8(NRF24_REG_CONFIG, config);
    if (result != NRF24L01_OK)
    {
        return result;
    }

    HAL_Delay(2U);
    if (s_config.rx_pipe_mask != 0U)
    {
        nrf24_ce_write(1U);
    }
    return NRF24L01_OK;
}

void Nrf24l01_GetDefaultConfig(Nrf24l01Config_t *config)
{
    static const uint8_t default_address[NRF24L01_ADDRESS_WIDTH] =
    {
        0xA1U, 0xC2U, 0xC2U, 0xC2U, 0xC2U
    };

    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->spi_timeout_ms = 20U;
    config->channel = 76U;
    config->rx_pipe_mask = NRF24L01_RX_PIPE_1;
    config->pipe2_lsb = 0xA3U;
    memcpy(config->address, default_address, sizeof(default_address));
}

int32_t Nrf24l01_Init(const Nrf24l01Config_t *config)
{
    GPIO_InitTypeDef gpio;
    uint8_t value;

    if ((config == NULL) ||
        (config->hspi == NULL) ||
        (config->ce_port == NULL) ||
        (config->ce_pin == 0U) ||
        (config->csn_port == NULL) ||
        (config->csn_pin == 0U) ||
        (config->channel > 125U) ||
        ((config->rx_pipe_mask &
          (uint8_t)~(NRF24L01_RX_PIPE_1 | NRF24L01_RX_PIPE_2)) != 0U))
    {
        return NRF24L01_ERROR_ARGUMENT;
    }

    s_config = *config;
    if (s_config.spi_timeout_ms == 0U)
    {
        s_config.spi_timeout_ms = 20U;
    }
    s_bound = 1U;
    s_ready = 0U;

    /* CE默认拉低、CSN默认拉高，保证与同一SPI4上的LoRa互不选中。 */
    nrf24_ce_write(0U);
    nrf24_csn_write(1U);
    memset(&gpio, 0, sizeof(gpio));
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = s_config.ce_pin;
    HAL_GPIO_Init(s_config.ce_port, &gpio);
    gpio.Pin = s_config.csn_pin;
    HAL_GPIO_Init(s_config.csn_port, &gpio);
    /* 模块上电后留足内部上电复位时间，避免冷启动偶发读写失败。 */
    HAL_Delay(NRF24_POWER_ON_DELAY_MS);

    if ((nrf24_write_register_u8(NRF24_REG_CONFIG,
                                  NRF24_CONFIG_EN_CRC |
                                  NRF24_CONFIG_CRCO) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_EN_AA,
                                 (uint8_t)(0x01U |
                                           s_config.rx_pipe_mask)) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_EN_RXADDR,
                                 s_config.rx_pipe_mask) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_SETUP_AW, NRF24_SETUP_AW_5_BYTES) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_SETUP_RETR, NRF24_SETUP_RETR_VALUE) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_RF_CH, s_config.channel) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_RF_SETUP, NRF24_RF_SETUP_VALUE) != NRF24L01_OK) ||
        (nrf24_write_register(NRF24_REG_RX_ADDR_P0, s_config.address, sizeof(s_config.address)) != NRF24L01_OK) ||
        (nrf24_write_register(NRF24_REG_RX_ADDR_P1, s_config.address, sizeof(s_config.address)) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_RX_ADDR_P2, s_config.pipe2_lsb) != NRF24L01_OK) ||
        (nrf24_write_register(NRF24_REG_TX_ADDR, s_config.address, sizeof(s_config.address)) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_RX_PW_P0, NRF24L01_FRAME_SIZE) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_RX_PW_P1, NRF24L01_FRAME_SIZE) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_RX_PW_P2, NRF24L01_FRAME_SIZE) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_DYNPD, 0x00U) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_FEATURE, 0x00U) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_STATUS, NRF24_STATUS_IRQ_MASK) != NRF24L01_OK) ||
        (nrf24_command(NRF24_CMD_FLUSH_TX) != NRF24L01_OK) ||
        (nrf24_command(NRF24_CMD_FLUSH_RX) != NRF24L01_OK))
    {
        return NRF24L01_ERROR_SPI;
    }

    if ((nrf24_read_register(NRF24_REG_RF_CH, &value, 1U) != NRF24L01_OK) ||
        (value != s_config.channel))
    {
        return NRF24L01_ERROR_NOT_FOUND;
    }
    if ((nrf24_read_register(NRF24_REG_RF_SETUP, &value, 1U) != NRF24L01_OK) ||
        (value != NRF24_RF_SETUP_VALUE))
    {
        return NRF24L01_ERROR_NOT_FOUND;
    }

    if (nrf24_enter_rx() != NRF24L01_OK)
    {
        return NRF24L01_ERROR_SPI;
    }

    s_ready = 1U;
    return NRF24L01_OK;
}

int32_t Nrf24l01_SendTo(const uint8_t address[NRF24L01_ADDRESS_WIDTH],
                        const uint8_t *data,
                        uint8_t length,
                        uint8_t *retry_count)
{
    uint8_t frame[NRF24L01_FRAME_SIZE];
    uint8_t config;
    uint8_t observe_tx;
    uint8_t status;
    uint32_t start_tick;
    int32_t result;

    if ((s_ready == 0U) || (address == NULL) || (data == NULL) ||
        (length == 0U) || (length > NRF24L01_MAX_PAYLOAD_LEN))
    {
        return NRF24L01_ERROR_ARGUMENT;
    }

    memset(frame, 0, sizeof(frame));
    frame[0] = length;
    memcpy(&frame[1], data, length);
    if (retry_count != NULL)
    {
        *retry_count = 0U;
    }

    nrf24_ce_write(0U);
    config = NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO |
             NRF24_CONFIG_PWR_UP;
    if ((nrf24_write_register_u8(NRF24_REG_EN_AA,
                                 (uint8_t)(0x01U |
                                           s_config.rx_pipe_mask)) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_EN_RXADDR,
                                 (uint8_t)(0x01U |
                                           s_config.rx_pipe_mask)) != NRF24L01_OK) ||
        (nrf24_write_register(NRF24_REG_RX_ADDR_P0,
                              address,
                              NRF24L01_ADDRESS_WIDTH) != NRF24L01_OK) ||
        (nrf24_write_register(NRF24_REG_TX_ADDR,
                              address,
                              NRF24L01_ADDRESS_WIDTH) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_CONFIG, config) != NRF24L01_OK) ||
        (nrf24_write_register_u8(NRF24_REG_STATUS,
                                 NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT) != NRF24L01_OK) ||
        (nrf24_command(NRF24_CMD_FLUSH_TX) != NRF24L01_OK))
    {
        (void)nrf24_enter_rx();
        return NRF24L01_ERROR_SPI;
    }

    HAL_Delay(1U);
    if (nrf24_exchange(NRF24_CMD_W_TX_PAYLOAD,
                       frame,
                       NULL,
                       NRF24L01_FRAME_SIZE,
                       NULL) != NRF24L01_OK)
    {
        (void)nrf24_enter_rx();
        return NRF24L01_ERROR_SPI;
    }

    /* 手册要求CE高电平不少于10us；1ms脉冲避免额外建立微秒延时组件。 */
    nrf24_ce_write(1U);
    HAL_Delay(1U);
    nrf24_ce_write(0U);

    result = NRF24L01_ERROR_TIMEOUT;
    start_tick = HAL_GetTick();
    for (;;)
    {
        if (nrf24_read_status(&status) != NRF24L01_OK)
        {
            result = NRF24L01_ERROR_SPI;
            break;
        }
        if ((status & NRF24_STATUS_TX_DS) != 0U)
        {
            result = NRF24L01_OK;
            break;
        }
        if ((status & NRF24_STATUS_MAX_RT) != 0U)
        {
            result = NRF24L01_ERROR_MAX_RETRY;
            break;
        }
        if ((HAL_GetTick() - start_tick) >= NRF24_TX_TIMEOUT_MS)
        {
            break;
        }
        HAL_Delay(1U);
    }

    if ((retry_count != NULL) &&
        (nrf24_read_register(NRF24_REG_OBSERVE_TX, &observe_tx, 1U) == NRF24L01_OK))
    {
        *retry_count = (uint8_t)(observe_tx & 0x0FU);
    }

    (void)nrf24_write_register_u8(NRF24_REG_STATUS,
                                  NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    if (result != NRF24L01_OK)
    {
        (void)nrf24_command(NRF24_CMD_FLUSH_TX);
    }
    if (nrf24_enter_rx() != NRF24L01_OK)
    {
        return NRF24L01_ERROR_SPI;
    }

    return result;
}

int32_t Nrf24l01_Send(const uint8_t *data,
                      uint8_t length,
                      uint8_t *retry_count)
{
    return Nrf24l01_SendTo(s_config.address,
                           data,
                           length,
                           retry_count);
}

int32_t Nrf24l01_PollReceive(uint8_t *data,
                             uint8_t capacity,
                             uint8_t *length,
                             uint8_t *pipe_number)
{
    uint8_t fifo_status;
    uint8_t frame[NRF24L01_FRAME_SIZE];
    uint8_t payload_length;
    uint8_t pipe;
    uint8_t status;

    if ((s_ready == 0U) || (data == NULL) || (length == NULL))
    {
        return NRF24L01_ERROR_ARGUMENT;
    }
    *length = 0U;
    if (pipe_number != NULL)
    {
        *pipe_number = 0x07U;
    }

    if (nrf24_read_register(NRF24_REG_FIFO_STATUS, &fifo_status, 1U) != NRF24L01_OK)
    {
        return NRF24L01_ERROR_SPI;
    }
    if ((fifo_status & NRF24_FIFO_RX_EMPTY) != 0U)
    {
        return 0;
    }

    if (nrf24_read_status(&status) != NRF24L01_OK)
    {
        return NRF24L01_ERROR_SPI;
    }
    pipe = (uint8_t)((status & NRF24_STATUS_RX_P_NO_MASK) >> 1U);
    if ((pipe < 1U) || (pipe > 2U))
    {
        (void)nrf24_command(NRF24_CMD_FLUSH_RX);
        (void)nrf24_write_register_u8(NRF24_REG_STATUS,
                                      NRF24_STATUS_RX_DR);
        return NRF24L01_ERROR_PAYLOAD;
    }

    if (nrf24_exchange(NRF24_CMD_R_RX_PAYLOAD,
                       NULL,
                       frame,
                       NRF24L01_FRAME_SIZE,
                       NULL) != NRF24L01_OK)
    {
        return NRF24L01_ERROR_SPI;
    }
    (void)nrf24_write_register_u8(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);

    payload_length = frame[0];
    if ((payload_length == 0U) ||
        (payload_length > NRF24L01_MAX_PAYLOAD_LEN) ||
        (payload_length > capacity))
    {
        (void)nrf24_command(NRF24_CMD_FLUSH_RX);
        return NRF24L01_ERROR_PAYLOAD;
    }

    memcpy(data, &frame[1], payload_length);
    *length = payload_length;
    if (pipe_number != NULL)
    {
        *pipe_number = pipe;
    }
    return 1;
}

uint8_t Nrf24l01_IsReady(void)
{
    return s_ready;
}
