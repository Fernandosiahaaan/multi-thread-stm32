#include <ModbusRTUDevice.h>
#include <cstring>
#include "stm32f0xx_ll_gpio.h"
#include "cmsis_os2.h"

using namespace ModbusRTU;

extern osMessageQueueId_t queueEsp;

DeviceDriver::DeviceDriver()
{
	m_errorCount = 0;
}

bool DeviceDriver::connect(Config *conf)
{
	m_usart = conf->usart;
    flushReceive();
    return true;
}

bool DeviceDriver::connectEsp(ConfigEsp *conf)
{
	m_usart = conf->usart;
    m_fifo = conf->queue;

    m_fifo->flush();
    flushReceive();
    return true;
}

bool DeviceDriver::disconnect()
{
	/*unimplemented in stm32*/
    return true;
}

bool DeviceDriver::transmit(uint8_t *data, uint32_t datalen)
{
	for(uint32_t i = 0; i < datalen; ++i)
	{
		while(!LL_USART_IsActiveFlag_TXE(m_usart));
		LL_USART_TransmitData8(m_usart, data[i]);
	}

    return true;
}

bool DeviceDriver::transmit(uint8_t *data, uint32_t datalen, millis_t delay)
{
    millis_t start = millis();
    while(((millis() - start) & 0xFFFF) < delay) {}

    for(uint32_t i = 0; i < datalen; ++i)
    {
        while(!LL_USART_IsActiveFlag_TXE(m_usart));
        LL_USART_TransmitData8(m_usart, data[i]);
    }
    return true;
}

bool DeviceDriver::isError()
{
	if(READ_BIT(m_usart->ISR, USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE | USART_ISR_PE) > 0)
	{
		/* Clear buffer */
    	__IO uint32_t tmpreg = m_usart->ISR;
    	(void) tmpreg;
    	tmpreg = m_usart->RDR;
    	(void) tmpreg;

    	m_errorCount++;
		return true;
	}

	return false;
}

void DeviceDriver::flushReceive()
{
	if(!isError() && LL_USART_IsActiveFlag_RXNE(m_usart))
	{
	    __IO uint32_t tmpreg = m_usart->RDR;
	}
}

bool DeviceDriver::receive(uint8_t *buff, uint32_t bufflen, uint32_t *receivedLen)
{
	*receivedLen = 0;
//
//	/* Any UART error is intentionally ignored */
//    if(!isError() &&
    if(LL_USART_IsActiveFlag_RXNE(m_usart))
    {
    	*receivedLen = 1;
        buff[0] = LL_USART_ReceiveData8(m_usart);
    }

    return true;
}

//bool DeviceDriver::receive(uint8_t *buff, uint32_t bufflen, uint32_t *receivedLen)
//{
//	*receivedLen = 0;
////
////	/* Any UART error is intentionally ignored */
////    if(!isError() &&
//    if(LL_USART_IsActiveFlag_RXNE(m_usart))
//    {
//    	*receivedLen = 1;
//        buff[0] = LL_USART_ReceiveData8(m_usart);
//    }
//
//    return true;
//}

bool DeviceDriver::receiveEsp(uint8_t *buff, uint32_t bufflen, uint32_t *receivedLen)
{
	uint8_t result;
	if(osMessageQueueGet(queueEsp, &result, 0, osWaitForever) == osOK)
    {
        if(result >= 0)
        {
            buff[0] = result & 0xFF;
            *receivedLen = 1;
        }
        else
        {
            *receivedLen = 0;
        }
    }
    return true;
}

millis_t DeviceDriver::millis()
{
	return BSP::millisOs();
}

millis_t DeviceDriver::elapsedMillis(millis_t start)
{
	return BSP::elapsedMillisOs(start);
}

uint32_t DeviceDriver::getErrorCount()
{
	return m_errorCount;
}
