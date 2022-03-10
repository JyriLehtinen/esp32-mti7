
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

#include "hostinterface.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdarg.h>
#include <cstdlib>

/*!	\class HostInterface
	\brief Implements the command line interface with the host (PC) via uart communication
*/

#define TIMEOUT_MS 1

static HostInterface* thisPtr = 0;


/*!	\brief Constructs a HostInterface
*/
HostInterface::HostInterface()
{
	thisPtr = this;
}


/*!	\brief Wait until the user has entered an integer
	\return The integer which was received
*/
int HostInterface::readInteger()
{
	const int bufferSize = 32;
	char buffer[bufferSize];
	buffer[bufferSize - 1] = 0;
	for (int n = 0; n < bufferSize - 1; n++)
	{
		buffer[n] = getch();
		if (buffer[n] == '\n' || buffer[n] == '\r')
		{
			buffer[n] = 0;
			break;
		}
	}
	int result = std::atoi(buffer);
	return result;
}


/*!	\brief Returns the next character in the input buffer or waits until a character can be read
	\return The next character in the input buffer
*/
char HostInterface::getch()
{
	int length = 0;
	uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length);
	while (length == 0) {
		uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length);
		vTaskDelay(1);
	}

	char result;
	length = uart_read_bytes(UART_NUM_0, &result, 1, TIMEOUT_MS / portTICK_PERIOD_MS);
	return result;
}


/*!	\brief Returns true is a character can be read from the input buffer, false otherwise
*/
bool HostInterface::kbhit()
{
	int length = 0;
	uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length);
	return (length > 0);
}


/*!	\brief Formated printing a string to the host terminal
*/
void HostInterface::printf(const char* formatString, ...)
{
	va_list args;
	va_start (args, formatString);
	vprintf(formatString, args);
	va_end (args);
}


/*!	\brief Formated printing a string to the host terminal
*/
void HostInterface::vprintf(const char* formatString, va_list v)
{
	int len = vsnprintf((char*)m_txBuffer, sizeof(m_txBuffer), formatString, v);
	uart_write_bytes(UART_NUM_0, (const char*)m_txBuffer, len);
}