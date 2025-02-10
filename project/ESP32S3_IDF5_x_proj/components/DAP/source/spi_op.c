#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "main/include/dap_configuration.h"

#include "components/DAP/include/cmsis_compiler.h"
#include "components/DAP/include/spi_op.h"
#include "components/DAP/include/spi_switch.h"
#include "components/DAP/include/gpio_common.h"

#define DAP_SPI GPSPI2

#define SET_MOSI_BIT_LEN(x) DAP_SPI.ms_dlen.ms_data_bitlen = x
#define SET_MISO_BIT_LEN(x) DAP_SPI.ms_dlen.ms_data_bitlen = x
#define START_AND_WAIT_SPI_TRANSMISSION_DONE() \
    do {                                       \
        DAP_SPI.cmd.update = 1;                \
        while (DAP_SPI.cmd.update) continue;   \
        DAP_SPI.cmd.usr = 1;                   \
        while (DAP_SPI.cmd.usr) continue;      \
    } while(0)

/**
 * @brief Calculate integer division and round up
 *
 * @param A
 * @param B
 * @return result
 */
__STATIC_FORCEINLINE int div_round_up(int A, int B)
{
    return (A + B - 1) / B;
}


/**
 * @brief Write bits. LSB & little-endian
 *        Note: No check. The pointer must be valid.
 * @param count Number of bits to be written (<= 64 bits, no length check)
 * @param buf Data Buf
 */
void DAP_SPI_WriteBits(const uint8_t count, const uint8_t *buf)
{
    uint32_t data[16];
    int nbytes, i;

    DAP_SPI.user.usr_command = 0;
    DAP_SPI.user.usr_addr = 0;

    // have data to send
    DAP_SPI.user.usr_mosi = 1;
    DAP_SPI.user.usr_miso = 0;
    SET_MOSI_BIT_LEN(count - 1);

    nbytes = div_round_up(count, 8);
    memcpy(data, buf, nbytes);

    for (i = 0; i < nbytes; i++) {
        DAP_SPI.data_buf[i] = data[i];
    }

    START_AND_WAIT_SPI_TRANSMISSION_DONE();
}



/**
 * @brief Read bits. LSB & little-endian
 *        Note: No check. The pointer must be valid.
 * @param count Number of bits to be read (<= 64 bits, no length check)
 * @param buf Data Buf
 */
void DAP_SPI_ReadBits(const uint8_t count, uint8_t *buf) {
    int i;
    uint32_t data_buf[2];

    uint8_t * pData = (uint8_t *)data_buf;

    DAP_SPI.user.usr_mosi = 0;
    DAP_SPI.user.usr_miso = 1;

#if (USE_SPI_SIO == 1)
    DAP_SPI.user.sio = true;
#endif

    SET_MISO_BIT_LEN(count - 1U);

    START_AND_WAIT_SPI_TRANSMISSION_DONE();

#if (USE_SPI_SIO == 1)
    DAP_SPI.user.sio = false;
#endif

    data_buf[0] = DAP_SPI.data_buf[0];
    data_buf[1] = DAP_SPI.data_buf[1];

    for (i = 0; i < div_round_up(count, 8); i++)
    {
        buf[i] = pData[i];
    }
    // last byte use mask:
    buf[i-1] = buf[i-1] & ((2 >> (count % 8)) - 1);
}

__FORCEINLINE void DAP_SPI_Send_Header(const uint8_t packetHeaderData, uint8_t *ack, uint8_t TrnAfterACK)
{
    uint32_t dataBuf;

    // have data to send
    DAP_SPI.user.usr_mosi = 0;
    DAP_SPI.user.usr_command = 1;
    DAP_SPI.user.usr_miso = 1;

    // 8bits Header + 1 bit Trn(Before ACK) - 1(prescribed)
    DAP_SPI.user2.usr_command_bitlen = 8U + 1U - 1U;
    DAP_SPI.user2.usr_command_value = packetHeaderData;


#if (USE_SPI_SIO == 1)
    DAP_SPI.user.sio = true;
#endif

    // 3bits ACK + TrnAferACK  - 1(prescribed)
    SET_MISO_BIT_LEN(3U + TrnAfterACK - 1U);

    START_AND_WAIT_SPI_TRANSMISSION_DONE();

#if (USE_SPI_SIO == 1)
    DAP_SPI.user.sio = false;
#endif

    DAP_SPI.user.usr_command = 0;

    dataBuf = DAP_SPI.data_buf[0];
    *ack = dataBuf & 0b111;
}


/**
 * @brief Step2: Read Data
 *
 * @param resData data from target
 * @param resParity parity from target
 */
__FORCEINLINE void DAP_SPI_Read_Data(uint32_t *resData, uint8_t *resParity)
{
    volatile uint64_t dataBuf;
    uint32_t *pU32Data = (uint32_t *)&dataBuf;

    DAP_SPI.user.usr_mosi = 0;
    DAP_SPI.user.usr_miso = 1;

#if (USE_SPI_SIO == 1)
    DAP_SPI.user.sio = true;
#endif

    // 1 bit Trn(End) + 32bis data + 1bit parity - 1(prescribed)
    SET_MISO_BIT_LEN(1U + 32U + 1U - 1U);

    START_AND_WAIT_SPI_TRANSMISSION_DONE();

#if (USE_SPI_SIO == 1)
    DAP_SPI.user.sio = false;
#endif

    pU32Data[0] = DAP_SPI.data_buf[0];
    pU32Data[1] = DAP_SPI.data_buf[1];

    *resData = (dataBuf >> 0U) & 0xFFFFFFFFU;  // 32bits Response Data
    *resParity = (dataBuf >> (0U + 32U)) & 1U; // 1bit parity
}

__FORCEINLINE void DAP_SPI_Write_Data(uint32_t data, uint8_t parity)
{
    DAP_SPI.user.usr_mosi = 1;
    DAP_SPI.user.usr_miso = 0;

    // esp32c3 can not send 33 bits of data correctly, we need to send an additional bit
    // that will not be recognized as the start bit.
    SET_MOSI_BIT_LEN(32U + 1U + 1U - 1U);
    DAP_SPI.data_buf[0] = data;
    DAP_SPI.data_buf[1] = parity == 0 ? 0b00 : 0b01;

    START_AND_WAIT_SPI_TRANSMISSION_DONE();
}


/**
 * @brief Generate Clock Cycle
 *
 * @param num Cycle Num
 */
__FORCEINLINE void DAP_SPI_Generate_Cycle(uint8_t num)
{
    //// TODO: It may take long time to generate just one clock
    DAP_SPI.user.usr_mosi = 1;
    DAP_SPI.user.usr_miso = 0;
    SET_MOSI_BIT_LEN(num - 1U);

    DAP_SPI.data_buf[0] = 0x00000000U;

    START_AND_WAIT_SPI_TRANSMISSION_DONE();
}

/**
 * @brief Quickly generate 1 clock
 *
 */
__FORCEINLINE void DAP_SPI_Fast_Cycle()
{
    DAP_SPI_Release();
    DAP_SPI_Acquire();
}

/**
 * @brief Generate Protocol Error Cycle
 *
 */
__FORCEINLINE void DAP_SPI_Protocol_Error_Read()
{
    DAP_SPI.user.usr_mosi = 1;
    DAP_SPI.user.usr_miso = 0;
    SET_MOSI_BIT_LEN(32U + 1U - 1); // 32bit ignore data + 1 bit - 1(prescribed)

    DAP_SPI.data_buf[0] = 0xFFFFFFFFU;
    DAP_SPI.data_buf[1] = 0xFFFFFFFFU;

    START_AND_WAIT_SPI_TRANSMISSION_DONE();
}


/**
 * @brief Generate Protocol Error Cycle
 *
 */
__FORCEINLINE void DAP_SPI_Protocol_Error_Write()
{
    DAP_SPI.user.usr_mosi = 1;
    DAP_SPI.user.usr_miso = 0;
    SET_MOSI_BIT_LEN(1U + 32U + 1U - 1); // 1bit Trn + 32bit ignore data + 1 bit - 1(prescribed)

    DAP_SPI.data_buf[0] = 0xFFFFFFFFU;
    DAP_SPI.data_buf[1] = 0xFFFFFFFFU;

    START_AND_WAIT_SPI_TRANSMISSION_DONE();
}
