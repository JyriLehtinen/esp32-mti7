
//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "mtssp_driver_spi.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "init_spi.h"
#include "wait.h"

#include <string.h>

/*!	\class MtsspDriverSpi
	\brief MtsspDriver for the SPI bus
*/
#define MTI_SPI_BUS VSPI_HOST
#define SPI_LOCK_TIMEOUT 0
#define MTI_SPI_CLOCK_SPEED_HZ	2*1000*1000 //Clock out at 2 MHz
#define MTI_SPI_MODE 3
#define CS_EN_PRE_WAIT_CYCLES 1
#define CS_EN_POST_WAIT_CYCLES 0
#define MTI_SPI_INPUT_DELAY_NS 10


/*!	\brief Constructs a MtsspDriverSpi
	\param[in] cs_pin The GPIO pin of the nCS pin
*/
MtsspDriverSpi::MtsspDriverSpi(gpio_num_t cs_pin)
	: m_cs_pin(cs_pin)
{

	spi_device_interface_config_t devcfg = {
        .mode=MTI_SPI_MODE,
		.cs_ena_pretrans=CS_EN_PRE_WAIT_CYCLES,
		.cs_ena_posttrans=CS_EN_POST_WAIT_CYCLES,
		.clock_speed_hz=MTI_SPI_CLOCK_SPEED_HZ,
		.input_delay_ns=MTI_SPI_INPUT_DELAY_NS,
        .spics_io_num=m_cs_pin,             //CS pin
		//.spics_io_num=-1,             		//CS Manual control of CS pin
        .queue_size=5,                      //We want to be able to queue 5 transactions at a time
	};

	spi_bus_add_device(MTI_SPI_BUS, &devcfg, &spi_dev);
}

/*!	\brief Perform a blocking write transfer on the SPI bus
	\param[in] opcode Opcode to use
	\param[in] data Pointer to data to be written
	\param[in] dataLength Number of data bytes to write
*/
void MtsspDriverSpi::write(uint8_t opcode, uint8_t const* data, int dataLength)
{
	if (dataLength==0) return;             //no need to send anything

	//esp_err_t ret;
    spi_transaction_t t;
	memset(&t, 0, sizeof(t));     //Zero out the transaction

	memset(m_tx_buffer, 0, sizeof(m_tx_buffer));
	m_tx_buffer[0] = opcode;
	
	memcpy((m_tx_buffer+4), data, dataLength);
	/*
    t.length=4*8;                 //Len is in bytes, transaction length is in bits.
	mempcpy(t.tx_data, buffer, 4); // Four bytes can be sent with tx_data
	//spi_device_acquire_bus(spi_dev, SPI_LOCK_TIMEOUT);
	//ret=spi_device_polling_transmit(spi_dev, &t);  //Transmit the op command
	spi_device_polling_transmit(spi_dev, &t);  //Transmit the op command
	*/

	//memset(&t, 0, sizeof(t));     	//Zero out the transaction
	t.length=(4+dataLength)*8;          //Len is in bytes, transaction length is in bits.
    t.tx_buffer=m_tx_buffer;               //Data
    
    //ret=spi_device_polling_transmit(spi_dev, &t);  //Transmit!
	spi_device_polling_transmit(spi_dev, &t);  //Transmit!
	//spi_device_release_bus(spi_dev);
}


/*!	\brief Perform a blocking read transfer on the SPI bus
	\param[in] opcode Opcode to use
	\param[out] data Pointer to result buffer
	\param[in] dataLength Number of data bytes to read
*/
void MtsspDriverSpi::read(uint8_t opcode, uint8_t* dest, int dataLength)
{
	//esp_err_t ret;
    spi_transaction_t t;
	memset(&t, 0, sizeof(t));     //Zero out the transaction

	memset(m_tx_buffer, 0, sizeof(m_tx_buffer));
	m_tx_buffer[0] = opcode;
	
    t.length=(4+dataLength)*8;                 //Len is in bytes, transaction length is in bits.
	t.rxlength=(4+dataLength)*8;               //Len is in bytes, transaction length is in bits.
	t.tx_buffer = m_tx_buffer;
	t.rx_buffer = m_rx_buffer;
	//ret=spi_device_polling_transmit(spi_dev, &t);  //Transmit the op command
	spi_device_polling_transmit(spi_dev, &t);  //Transmit the op command
	//spi_device_release_bus(spi_dev);

	if(m_rx_buffer[0] != 0xFA) {
		printf("SPI Read Error, 1st byte != 0xFA!\n");
		printf(" %02X %02X %02X %02X\n", m_rx_buffer[0], m_rx_buffer[1], m_rx_buffer[2], m_rx_buffer[3] );
		memset(dest, 0, dataLength);
		return;
	}

	memcpy(dest, (m_rx_buffer+4), dataLength);

}


/*!	\brief Perform a blocking write transfer on the SPI bus
	\param[in] data Pointer to data to be written
	\param[in] dataLength Number of data bytes to write
*/
void MtsspDriverSpi::writeRaw(uint8_t const* data, int dataLength)
{
	
	//esp_err_t ret;
    spi_transaction_t t;
	memset(&t, 0, sizeof(t));     //Zero out the transaction
	
	memset(&t, 0, sizeof(t));     	//Zero out the transaction
	t.length=dataLength*8;          //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    
	//spi_device_acquire_bus(spi_dev, SPI_LOCK_TIMEOUT);
    //ret=spi_device_polling_transmit(spi_dev, &t);  //Transmit!
	spi_device_polling_transmit(spi_dev, &t);  //Transmit!
	//spi_device_release_bus(spi_dev);

}





