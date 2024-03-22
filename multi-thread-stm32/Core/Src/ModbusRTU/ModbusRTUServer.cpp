#include <ModbusRTUServer.h>
#include <string.h>

#define CRC_BYTE 2u
#define CRC_WORD 1u
#define REPLY_READ_HEAD_BYTE 3u
#define REPLY_READ_HEAD_WORD 2u

using namespace ModbusRTU;

Server::Server(uint8_t id, uint32_t baud,
    uint16_t *buffer, uint32_t bufferLen,
    bool isLittleEndArch)
{
    m_id = id;
    m_errorCount = 0;
    m_correctID = false;
    m_funcExist = false;
    m_archLE = isLittleEndArch;
    /*timeout of  3.5 character before the current frame progress 
    is ignored. Byte require 10-bit transmission in uart*/
    m_byteTimeoutMs = (35000 / baud) & 0xFFFF;
    m_byteTimeoutMs = m_byteTimeoutMs < 1 ? 1 : m_byteTimeoutMs;
    m_lastByteMs = 0;

    m_buff = buffer;
    m_buffLen = bufferLen;
    m_state = PSTATE_SERVER_ID;

    OnCoilRead = nullptr;
    OnSingleCoilWrite = nullptr;
    OnDiscreteRead = nullptr;

    OnInputRead = nullptr;
    OnHoldRead = nullptr;
    OnRegWrite = nullptr;

    m_respDelayMs = DEFAULT_RESP_DELAY_MS;
}

void Server::setRespDelay(millis_t delayMs)
{
    m_respDelayMs = static_cast<millis_t>(delayMs);
}

void Server::setID(uint8_t id)
{
    m_id = id;
}

bool Server::connect(DeviceDriver::Config *conf)
{
    return m_dev.connect(conf);
}

bool Server::disconnect()
{
    return m_dev.disconnect();
}

uint32_t Server::getErrorCount()
{
    return m_errorCount;
}

void Server::poll()
{
    uint8_t data;
    uint32_t rxLen;

    if(!m_dev.receive(&data, 1, &rxLen))
    {
        m_errorCount++;
        return;
    }

    if(rxLen)
    {
        parse(data);
        m_lastByteMs = m_dev.millis();
    }
    else if(m_dev.elapsedMillis(m_lastByteMs) > m_byteTimeoutMs)
    {
        /*timeout of  3.5 character (mobdus spec), frame is assumed
        broken, parsing progress is reset*/
        m_state = PSTATE_SERVER_ID;
    }
}

void Server::parse(uint8_t byte)
{
    switch(m_state)
    {
    case PSTATE_SERVER_ID:
        m_correctID = false;
        m_funcExist = false;
        m_targetAddr = 0;
        m_buffIndex = 0;
        m_numWord = 0;
        m_crc = 0xFFFF;

        if(byte == m_id)
        {
            m_correctID = true;
            m_crc = crc16_update_le(m_crc, byte);
        }
        m_state = PSTATE_FUNC_CODE;
        break;

    case PSTATE_FUNC_CODE:
        m_targetFunc = (FunctionCode) byte;
        m_funcExist = isFuncSupported(m_targetFunc);
        if(m_correctID)
        {
            m_crc = crc16_update_le(m_crc, byte);
        }
        m_state = PSTATE_ADDR1;
        break;

    case PSTATE_ADDR1:
        if(m_correctID)
        {
            m_crc = crc16_update_le(m_crc, byte);
        }

        m_targetAddr = byte << 8;
        m_state = PSTATE_ADDR2;
        break;

    case PSTATE_ADDR2:
        if(m_correctID)
        {
            m_crc = crc16_update_le(m_crc, byte);
        }

        m_targetAddr += byte;
        if(m_targetFunc == FC_WRITE_MULT_REG)
        {
            m_state = PSTATE_BYTE_COUNT;
        }
        else
        {
            m_numWord = 1;
            m_state = PSTATE_VALUE1;
        }
        break;

    case PSTATE_BYTE_COUNT:
        if(m_correctID)
        {
            m_crc = crc16_update_le(m_crc, byte);
        }

        /* check register & byte count (total 3 bytes) */
        m_buff[m_buffIndex++] = byte;

        if(m_buffIndex == 3)
        {
            m_buffIndex = 0;
            m_numWord = (m_buff[0] & 0xFF) << 8;
            m_numWord += m_buff[1] & 0xFF;
            m_state = PSTATE_VALUE1;
        }
        break;

    case PSTATE_VALUE1:
        if(m_correctID)
        {
            m_crc = crc16_update_le(m_crc, byte);
        }

        if(m_buffIndex < m_buffLen)
        {
            m_buff[m_buffIndex] = byte << 8;
        }
        m_state = PSTATE_VALUE2;
        break;

    case PSTATE_VALUE2:
        if(m_correctID)
        {
            m_crc = crc16_update_le(m_crc, byte);
        }

        if(m_buffIndex < m_buffLen)
        {
            m_buff[m_buffIndex] += byte;
        }
        
        m_state = (++m_buffIndex == m_numWord) ? 
            PSTATE_CRC_CHECK1 : PSTATE_VALUE1;
        break;

    case PSTATE_CRC_CHECK1:
        /* using little endian crc to match the 
        calculated crc16 */
        m_receivedCrc = byte;
        m_state = PSTATE_CRC_CHECK2;
        break;

    case PSTATE_CRC_CHECK2:
    	m_state = PSTATE_SERVER_ID;
        m_receivedCrc += byte << 8;
        if(!m_correctID || m_receivedCrc != m_crc)
        {
            return;
        }

        if(m_numWord > m_buffLen)
        {
            sendException(m_targetFunc, EXC_ILLEGAL_ADDR);
            return;
        }

        if(!m_funcExist)
        {
            sendException(m_targetFunc, EXC_ILLEGAL_FUNC);
            return;
        }

        procFunc(m_targetFunc);
        break;

    }
}

void Server::procDiscreteRead(FunctionCode fc, DiscreteReadCb cb)
{
    Exception except;
    uint16_t numCoils = m_buff[0];
    uint32_t dataBytes = (numCoils + 7) / 8;
    
    if(numCoils > 2000 
        || (REPLY_READ_HEAD_BYTE + CRC_BYTE + dataBytes) > (m_buffLen * 2))
    {
        except = EXC_ILLEGAL_ADDR;
    }
    else 
    {
        BitStorageBE bitmasks(((uint8_t *) m_buff) + REPLY_READ_HEAD_BYTE,
            sizeof(m_buff), numCoils);
        except = cb(m_targetAddr, &bitmasks, numCoils);
    }
    if(except == EXC_NONE)
    {
        replyReadDiscrete(fc, m_targetAddr, dataBytes);
    }
    else
    {
        sendException(fc, except);
    }
}

void Server::procRegisterRead(FunctionCode fc, RegisterReadCb cb)
{
    Exception except;
    uint16_t numRegister = m_buff[0];
    if(numRegister > 125 || 
        (REPLY_READ_HEAD_WORD + CRC_WORD + numRegister) > m_buffLen)
    {
        except = EXC_ILLEGAL_ADDR;
    }
    else
    {
        uint16_t *regBuff = m_buff + REPLY_READ_HEAD_WORD;
        except = cb(m_targetAddr, regBuff, numRegister);
        if(m_archLE) swapEndianness(regBuff, numRegister);
    }

    if(except == EXC_NONE)
    {
        replyReadRegister(fc, m_targetAddr, numRegister);
    }
    else
    {
        sendException(fc, except);
    }
}

void Server::procRegisterWrite(FunctionCode fc, uint16_t data)
{
    Exception except = OnRegWrite(m_targetAddr, m_buff, m_buffIndex);

    if(except == EXC_NONE)
    {
        replyWriteRegister(fc, m_targetAddr, data);
    }
    else
    {
        sendException(fc, except);
    }
        
}

void Server::procFunc(FunctionCode fc)
{
    switch(fc)
    {
    case FC_READ_COIL:
        procDiscreteRead(fc, OnCoilRead);
        break;

    case FC_READ_DISCRETE:
        procDiscreteRead(fc, OnDiscreteRead);
        break;

    case FC_READ_INPUT_REG:
        procRegisterRead(fc, OnInputRead);
        break;

    case FC_READ_HOLDING_REG:
        procRegisterRead(fc, OnHoldRead);
        break;

    case FC_WRITE_SINGLE_REG:
        procRegisterWrite(fc, m_buff[0]);
        break;
    case FC_WRITE_MULT_REG:
        procRegisterWrite(fc, m_buffIndex);
        break;

    case FC_WRITE_SINGLE_COIL:
    {
        Exception except = OnSingleCoilWrite(m_targetAddr, m_buff[0]);

        if(except == EXC_NONE)
        {
            replyWriteRegister(fc, m_targetAddr, m_buff[0]);
        }
        else
        {
            sendException(fc, except);
        }
        break;
    }

    default:
    	break;
    }
}

bool Server::isFuncSupported(FunctionCode fc)
{
    switch(fc)
    {
    case FC_READ_COIL:
        return OnCoilRead == nullptr ? false : true;

    case FC_READ_DISCRETE:
        return OnDiscreteRead == nullptr ? false : true;

    case FC_READ_HOLDING_REG:
        return OnHoldRead == nullptr ? false : true;

    case FC_READ_INPUT_REG:
        return OnInputRead == nullptr ? false : true;
 
    case FC_WRITE_SINGLE_COIL:
        return OnSingleCoilWrite == nullptr ? false : true;

    case FC_WRITE_SINGLE_REG:
    case FC_WRITE_MULT_REG:
        return OnRegWrite == nullptr ? false : true;
    
    default:
        return false;
    }
}

void Server::replyReadDiscrete(FunctionCode fc, uint16_t startAddr,
    uint16_t numDataBytes)
{
    uint8_t *frame = (uint8_t *) m_buff;
    uint32_t numBytes = 0;

    frame[numBytes++] = m_id;
    frame[numBytes++] = (uint8_t) fc;
    frame[numBytes++] = numDataBytes & 0xFF;

    /* data is already in buffer */
    numBytes += numDataBytes;
    
    uint16_t crc = crc16_be(frame, numBytes);
    frame[numBytes++] = (crc >> 8) & 0xFF;
    frame[numBytes++] = crc & 0xFF;

    if(!m_dev.transmit(frame, numBytes, m_respDelayMs))
    {
        m_errorCount++;
    }
}

void Server::replyReadRegister(FunctionCode fc, uint16_t startAddr,
    uint16_t numRegister)
{
    uint8_t *frame = ((uint8_t *) m_buff) + 1;
    uint32_t numBytes = 0;

    frame[numBytes++] = m_id;
    frame[numBytes++] = (uint8_t) fc;
    frame[numBytes++] = (numRegister * 2) & 0xFF;

    /* data is already in buffer */
    numBytes += numRegister * 2;
    
    uint16_t crc = crc16_be(frame, numBytes);
    frame[numBytes++] = (crc >> 8) & 0xFF;
    frame[numBytes++] = crc & 0xFF;

    if(!m_dev.transmit(frame, numBytes, m_respDelayMs))
    {
        m_errorCount++;
    }
}

void Server::replyWriteRegister(FunctionCode fc, uint16_t startAddr,
    uint16_t value)
{
    uint8_t *frame = (uint8_t *) m_buff;
    frame[0] = m_id;
    frame[1] = (uint8_t) fc;
    frame[2] = (startAddr >> 8) & 0xFF;
    frame[3] = startAddr & 0xFF;
    frame[4] = (value >> 8) & 0xFF;
    frame[5] = value & 0xFF;
    
    uint16_t crc = crc16_be(frame, 6);
    frame[6] = (crc >> 8) & 0xFF;
    frame[7] = crc & 0xFF;

    if(!m_dev.transmit(frame, 8, m_respDelayMs))
    {
        m_errorCount++;
    }
}

void Server::sendException(FunctionCode fc, Exception excep)
{
    uint8_t *frame = (uint8_t *) m_buff;
    frame[0] = m_id;
    frame[1] = 0x80 + (uint8_t) fc;
    frame[2] = (uint8_t) excep;
    
    uint16_t crc = crc16_be(frame, 3);
    frame[3] = (crc >> 8) & 0xFF;
    frame[4] = crc & 0xFF;

    if(!m_dev.transmit(frame, 5, m_respDelayMs))
    {
        m_errorCount++;
    }
}
