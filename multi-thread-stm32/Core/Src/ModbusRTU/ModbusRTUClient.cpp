#include <ModbusRTUClient.h>
#include <string.h>
#include "printf.h";

#define WRITE_REQ_HEAD_LEN 4u
#define READ_REQ_HEAD_LEN 6u
#define READ_RESP_HEAD_LEN 3u
#define EXCEP_HEAD_LEN 2u
#define EXCEP_BASE_FCODE 0x80u
#define CRC_LEN 2u
#define EXCEP_FRAME_LEN (EXCEP_HEAD_LEN + 1u + CRC_LEN)
#define SINGLE_WRITE_FRAME_LEN (2u + WRITE_REQ_HEAD_LEN + CRC_LEN)

using namespace ModbusRTU;

bool Client::connect(DeviceDriver::Config *conf)
{
    return m_dev.connect(conf);
}

bool Client::disconnect()
{
    return m_dev.disconnect();
}

Client::Client(uint8_t *comBuffer, uint32_t comBufferLen, millis_t defaultTimeoutMs)
{
    m_comBuff = comBuffer;
    m_comBuffLen = comBufferLen;
    m_timeoutMs = defaultTimeoutMs;
}

void Client::setTimeoutMs(millis_t value)
{
    m_timeoutMs = value;
}

Exception Client::writeRegister(uint8_t serverID, uint16_t regAddr, uint16_t val)
{
    if(SINGLE_WRITE_FRAME_LEN > m_comBuffLen)
    {
        return EXC_INSUFFICIENT_COM_BUFFER;
    }

    Exception excep = sendWriteRequest(FC_WRITE_SINGLE_REG, serverID, regAddr, val);
    if(excep != EXC_NONE)
    {
        return excep;
    }

    return waitWriteResponse(FC_WRITE_SINGLE_REG, serverID, regAddr, val);
}

Exception Client::writeCoil(uint8_t serverID, uint16_t coilAddr, bool val)
{
    if(SINGLE_WRITE_FRAME_LEN > m_comBuffLen)
    {
        return EXC_INSUFFICIENT_COM_BUFFER;
    }

    uint16_t regVal = val ? 0xFF00 : 0x0000;
    Exception excep = sendWriteRequest(FC_WRITE_SINGLE_COIL, serverID, coilAddr, regVal);
    if(excep != EXC_NONE)
    {
        return excep;
    }

    return waitWriteResponse(FC_WRITE_SINGLE_COIL, serverID, coilAddr, regVal);
}

Exception Client::readInputRegs(uint8_t serverID,  uint16_t startAddr, 
    uint16_t *resultBuff, uint16_t numReg)
{
    return readRegisters(FC_READ_INPUT_REG, serverID, startAddr,
        resultBuff, numReg);
}

Exception Client::readHoldingRegs(uint8_t serverID, uint16_t startAddr,
    uint16_t *resultBuff, uint16_t numReg)
{
    return readRegisters(FC_READ_HOLDING_REG, serverID, startAddr,
        resultBuff, numReg);
}

/* Will return the discrete input values as big endian bitmasks */
Exception Client::readDiscretes(uint8_t serverID, uint16_t startAddr,
    uint8_t *resultBuff, uint16_t numCoil)
{
    return readBits(FC_READ_DISCRETE, serverID, startAddr, resultBuff, numCoil);
}

/* Will return the coil values as bitmasks */
Exception Client::readCoils(uint8_t serverID, uint16_t startAddr,
    uint8_t *resultBuff, uint16_t numCoil)
{
    return readBits(FC_READ_COIL, serverID, startAddr, resultBuff, numCoil);
}

Exception Client::readRegisters(FunctionCode fCode, uint8_t serverID, 
    uint16_t startAddr, uint16_t *resultBuff, uint16_t numReg)
{
    if(numReg * 2 + READ_REQ_HEAD_LEN + CRC_LEN > m_comBuffLen)
    {
        return EXC_INSUFFICIENT_COM_BUFFER;
    }

    Exception excep = sendReadRequest(fCode, serverID, startAddr, numReg);
    if(excep != EXC_NONE)
    {
        return excep;
    }

    uint32_t expectedBytes = READ_RESP_HEAD_LEN + CRC_LEN + numReg * 2;
    excep = waitReadResponse(fCode, serverID, expectedBytes);
    printf("\nWait Respon Modbus , Except : %d\n ",excep);
    if(excep == EXC_NONE)
    {
        for(uint32_t i = 0; i < numReg; i++)
        {
            resultBuff[i] = m_comBuff[READ_RESP_HEAD_LEN + (i * 2)] << 8;
            resultBuff[i] += m_comBuff[READ_RESP_HEAD_LEN + (i * 2) + 1];
        }
    }

    return excep;
}

Exception Client::readBits(FunctionCode fCode, int16_t serverID, 
    uint16_t startAddr, uint8_t *resultBuff, uint16_t numCoil)
{
    /* rounding up division */
    uint32_t numDataByte =  (numCoil + 7) / 8;

    if(numDataByte + READ_REQ_HEAD_LEN + CRC_LEN > m_comBuffLen)
    {
        return EXC_INSUFFICIENT_COM_BUFFER;
    }

    Exception excep = sendReadRequest(fCode, serverID, startAddr, numCoil);
    if(excep != EXC_NONE)
    {
        return excep;
    }

    uint32_t expectedBytes = READ_RESP_HEAD_LEN + CRC_LEN + numDataByte;
    excep = waitReadResponse(fCode, serverID, expectedBytes);

    if(excep == EXC_NONE)
    {
        memcpy(resultBuff, m_comBuff + READ_RESP_HEAD_LEN, numDataByte);
    }

    return excep;
}

Exception Client::sendWriteRequest(FunctionCode fCode, uint8_t serverID,
    uint16_t addr, uint16_t val)
{
    m_comBuff[0] = serverID;
    m_comBuff[1] = static_cast<uint8_t>(fCode);
    m_comBuff[2] = (addr >> 8) & 0xFF;
    m_comBuff[3] = addr & 0xFF;
    m_comBuff[4] = (val >> 8) & 0xFF;
    m_comBuff[5] = val & 0xFF;

    uint16_t crc = crc16_be(m_comBuff, 6);
    m_comBuff[6] = (crc >> 8) & 0xFF;
    m_comBuff[7] = crc & 0xFF;

    m_dev.flushReceive();
    if(!m_dev.transmit(m_comBuff, 8))
    {
        return EXC_UART_ERROR;
    }
    
    return EXC_NONE;
}

Exception Client::waitWriteResponse(FunctionCode fCode, uint8_t serverID,
    uint16_t addr, uint16_t val)
{
    uint32_t receivedBytes = 0; 
    millis_t startMs = m_dev.millis();

    while(m_dev.elapsedMillis(startMs) < m_timeoutMs)
    {
        uint32_t rxLen;
        if(!m_dev.receive(m_comBuff + receivedBytes,
            m_comBuffLen - receivedBytes, &rxLen))
        {
            return EXC_UART_ERROR;
        }

        receivedBytes += rxLen;

        if(receivedBytes > 0 && m_comBuff[0] != serverID)
        {
            return EXC_CORRUPT_RESP;
        }


        if(receivedBytes >= SINGLE_WRITE_FRAME_LEN)
        {
            uint16_t receivedAddr = (m_comBuff[2] << 8) + m_comBuff[3];
            uint16_t receivedVal = (m_comBuff[4] << 8) + m_comBuff[5];

            if(m_comBuff[1] != fCode || receivedAddr != addr || receivedVal != val)
            {
                return EXC_CORRUPT_RESP;
            }

            uint16_t calculatedCRC = crc16_be(m_comBuff, SINGLE_WRITE_FRAME_LEN - CRC_LEN);
            uint16_t receivedCRC = m_comBuff[SINGLE_WRITE_FRAME_LEN - CRC_LEN] << 8;
            receivedCRC += m_comBuff[SINGLE_WRITE_FRAME_LEN - CRC_LEN + 1];

            return (calculatedCRC == receivedCRC) ? EXC_NONE: EXC_CORRUPT_RESP;
        }
        else if(receivedBytes >= EXCEP_FRAME_LEN
            && m_comBuff[1] == (EXCEP_BASE_FCODE + fCode)) /* Check if exception response is received */
        {
            uint16_t calculatedCRC = crc16_be(m_comBuff, EXCEP_FRAME_LEN - CRC_LEN);
            uint16_t receivedCRC = m_comBuff[EXCEP_FRAME_LEN - CRC_LEN] << 8;
            receivedCRC += m_comBuff[EXCEP_FRAME_LEN - CRC_LEN + 1];

            return (calculatedCRC == receivedCRC) ?
                static_cast<Exception>(m_comBuff[EXCEP_HEAD_LEN]) : EXC_CORRUPT_RESP;
        }
    }

    return EXC_TIMEOUT;
}

Exception Client::sendReadRequest(FunctionCode fCode, uint8_t serverID, 
    uint16_t startAddr, uint16_t numRead)
{
    m_comBuff[0] = serverID;
    m_comBuff[1] = static_cast<uint8_t>(fCode);
    m_comBuff[2] = (startAddr >> 8) & 0xFF;
    m_comBuff[3] = startAddr & 0xFF;
    m_comBuff[4] = (numRead >> 8) & 0xFF;
    m_comBuff[5] = numRead & 0xFF;

    uint16_t crc = crc16_be(m_comBuff, 6);
    m_comBuff[6] = (crc >> 8) & 0xFF;
    m_comBuff[7] = crc & 0xFF;

    m_dev.flushReceive();
    if(!m_dev.transmit(m_comBuff, 8))
    {
        return EXC_UART_ERROR;
    }
    
    return EXC_NONE;
}

Exception Client::waitReadResponse(FunctionCode fCode, uint8_t serverID,
        uint32_t expectedBytes)
{
    uint32_t receivedBytes = 0; 
    millis_t startMs = m_dev.millis();

    while(m_dev.elapsedMillis(startMs) < m_timeoutMs)
    {
        uint32_t rxLen;
        if(!m_dev.receive(m_comBuff + receivedBytes,
            m_comBuffLen - receivedBytes, &rxLen))
        {
            return EXC_UART_ERROR;
        }

        receivedBytes += rxLen;

        if(receivedBytes > 0 && m_comBuff[0] != serverID)
        {
            return EXC_CORRUPT_RESP;
        }

        if(receivedBytes >= expectedBytes)
		{
			if(m_comBuff[1] != fCode)
			{
				return EXC_CORRUPT_RESP;
			}

			uint16_t calculatedCRC = crc16_be(m_comBuff, expectedBytes - CRC_LEN);
			uint16_t receivedCRC = m_comBuff[expectedBytes - CRC_LEN] << 8;
			receivedCRC += m_comBuff[expectedBytes - CRC_LEN + 1];

			return (calculatedCRC == receivedCRC) ? EXC_NONE: EXC_CORRUPT_RESP;
		}
        else if(receivedBytes >= EXCEP_FRAME_LEN
            && m_comBuff[1] == (EXCEP_BASE_FCODE + fCode)) /* Check if exception response is received */
        {
            uint16_t calculatedCRC = crc16_be(m_comBuff, EXCEP_FRAME_LEN - CRC_LEN);
            uint16_t receivedCRC = m_comBuff[EXCEP_FRAME_LEN - CRC_LEN] << 8;
            receivedCRC += m_comBuff[EXCEP_FRAME_LEN - CRC_LEN + 1];

            return (calculatedCRC == receivedCRC) ? 
                static_cast<Exception>(m_comBuff[EXCEP_HEAD_LEN]) : EXC_CORRUPT_RESP;
        }
    }

    return EXC_TIMEOUT;
}
