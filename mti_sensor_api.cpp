#include "mti_sensor_api.h"

/*
void goToConfigurationMode(void);
void goToMeasurementMode(void);
*/

static inline uint8_t readUint8(const uint8_t* data, int& index) {
	uint8_t result = data[index++];
	return result;
}

static inline uint16_t readUint16(const uint8_t* data, int& index) {
	uint16_t result = 0;
	result |= data[index++] << 8;
	result |= data[index++] << 0;
	return result;
}

static inline uint32_t readUint32(const uint8_t* data, int& index) {
	uint32_t result = 0;
	result |= data[index++] << 24;
	result |= data[index++] << 16;
	result |= data[index++] << 8;
	result |= data[index++] << 0;
	return result;
}

static inline float readFloat(const uint8_t* data, int& index) {
	uint32_t temp = readUint32(data, index);
	float result;
	memcpy(&result, &temp, 4);
	return result;
}





/*!	\brief Constructs an Application
	\param hostInterface The HostInterface for communicating with the PC
	\param driver The MtsspDriver for communicating with the MTi
*/
MTI7::MTI7(MtsspDriver* driver, uint8_t cs_pin, uint8_t drdy_pin, uint8_t reset_pin)
	: m_state(STATE_Idle)
	, m_driver(driver)
{

	m_device = new MtsspInterface(m_driver);
	m_cs_pin = (gpio_num_t) cs_pin;
	m_drdy_pin = (gpio_num_t) drdy_pin;
	m_reset_pin = (gpio_num_t) reset_pin;
}


/*!	\brief Returns the value of the DataReady line
*/
bool MTI7::checkDataReadyLine()
{
	return gpio_get_level(m_drdy_pin);
}


/*!	\brief Defines the main loop of the program which handles user commands
*/
void MTI7::run()
{
	handleEvent(EVT_Start);
	
	while(m_state != STATE_Ready) {
		if (checkDataReadyLine()) {
			readDataFromDevice();
		}
		vTaskDelay(100);
	}
}


/*!	\brief Implements the state machine which defines the program
*/
void MTI7::handleEvent(Event event, const uint8_t* data)
{
	switch (m_state)
	{
		case STATE_Idle:
		{
			if (event == EVT_Start)
			{
				//resetDevice();
				//m_state = STATE_WaitForWakeUp;
				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_GotoConfig, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);
				m_state = STATE_Ready;
			}
		} break;

		case STATE_WaitForWakeUp:
		{
			if (event == EVT_XbusMessage && Xbus_getMessageId(data) == XMID_Wakeup)
			{
				printf("Got Wakeup message\n");
				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_GotoConfig, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);
				m_state = STATE_Ready;
			}
		} break;

		case STATE_WaitForConfigMode:
		{
			Xbus_message(m_xbusTxBuffer, 0xFF, XMID_ReqDid, 0);
			m_device->sendXbusMessage(m_xbusTxBuffer);
			m_state = STATE_WaitForDeviceId;
		} break;

		case STATE_WaitForDeviceId:
		{
			if (event == EVT_XbusMessage && Xbus_getMessageId(data) == XMID_DeviceId)
			{
				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_ReqFirmwareRevision, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);
				m_state = STATE_WaitForFirmwareRevision;
			}
		} break;

		case STATE_WaitForFirmwareRevision:
		{
			if (event == EVT_XbusMessage && Xbus_getMessageId(data) == XMID_FirmwareRevision)
			{
				set_default_outputs(1);			
				m_state = STATE_WaitForSetOutputConfigurationAck;
			}
		} break;

		case STATE_WaitForSetOutputConfigurationAck:
		{
			if (event == EVT_XbusMessage && Xbus_getMessageId(data) == XMID_OutputConfig)
			{
				m_state = STATE_Ready;
                Xbus_message(m_xbusTxBuffer, 0xFF, XMID_GotoMeasurement, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);
			}
		} break;

		case STATE_Ready:
		{
			if (event == EVT_GotoConfig)
			{
				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_GotoConfig, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);

			}

			if (event == EVT_GotoMeasuring)
			{
				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_GotoMeasurement, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);

			}

			if (event == EVT_RequestDeviceId)
			{
				Xbus_message(m_xbusTxBuffer, 0xFF, XMID_ReqDid, 0);
				m_device->sendXbusMessage(m_xbusTxBuffer);
			}

			if (event == EVT_XbusMessage)
			{
                processXbusData(data);
                //printf("Roll:%.2f\tPitch:%.2f\tYaw:%.2f\n", euler.roll, euler.pitch, euler.yaw);
                //printf("RollRate:%.2f\tPitchRate:%.2f\tYawRate:%.2f\n", angularRate.roll, angularRate.pitch, angularRate.yaw);
			}
			
		} break;
	}
}


/*!	\brief Resets the MTi
*/
void MTI7::resetDevice()
{
	return; // Clearly this isn't working
	/*
	gpio_set_direction(m_reset_pin, GPIO_MODE_OUTPUT);
	vTaskDelay(10);
	gpio_set_level(m_reset_pin, 0);
	vTaskDelay(1);

	// Reset the pin and set it to floating
	gpio_config_t cfg = {
        .pin_bit_mask = BIT64(m_reset_pin),
        .mode = GPIO_MODE_DISABLE,
        //for powersave reasons, the GPIO should not be floating, select pullup
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
	*/

}


/*!	\brief Read data from the Notification and Control pipes of the device
*/
void MTI7::readDataFromDevice()
{
	uint16_t notificationMessageSize;
	uint16_t measurementMessageSize;
	m_device->readPipeStatus(notificationMessageSize, measurementMessageSize);

	m_dataBuffer[0] = XBUS_PREAMBLE;
	m_dataBuffer[1] = XBUS_MASTERDEVICE;

	if (notificationMessageSize && notificationMessageSize < sizeof(m_dataBuffer))
	{
		m_device->readFromPipe(&m_dataBuffer[2], notificationMessageSize, XBUS_NOTIFICATION_PIPE);
		handleEvent(EVT_XbusMessage, m_dataBuffer);
	}

	if (measurementMessageSize && measurementMessageSize < sizeof(m_dataBuffer))
	{
		m_device->readFromPipe(&m_dataBuffer[2], measurementMessageSize, XBUS_MEASUREMENT_PIPE);
		handleEvent(EVT_XbusMessage, m_dataBuffer);
	}
}

void MTI7::setOutputConfiguration(XsDataIdentifier XDA_identifier, uint16_t output_freq) {
    uint8_t tx_buf[8];
    
    Xbus_message(tx_buf, 0xFF, XMID_SetOutputConfig, 4);
    tx_buf[4] = (uint8_t)(XDA_identifier >> 8);
    tx_buf[5] = (uint8_t)(XDA_identifier & 0xFF);
    tx_buf[6] = (uint8_t)(output_freq >> 8);
    tx_buf[7] = (uint8_t)(output_freq & 0xFF);

    //Xbus_message(tx_buffer, 0xFF, XMID_SetOutputConfig, 4);
    //driver->m_device->sendXbusMessage(m_xbusTxBuffer);
    
    m_device->sendXbusMessage(tx_buf);

    return;
}

void MTI7::set_default_outputs(uint16_t output_freq) {

    //setOutputConfiguration(m_device, XDI_EulerAngles, output_freq); 
    //setOutputConfiguration(m_device, XDI_RateOfTurn, output_freq); 
    //setOutputConfiguration(m_device, XDI_VelocityXYZ, output_freq); 
    //setOutputConfiguration(m_device, XDI_Acceleration, output_freq); 

    const uint8_t configArrayLen = 4*4;
    uint8_t tx_buf[4+configArrayLen];


    
    Xbus_message(tx_buf, 0xFF, XMID_SetOutputConfig, configArrayLen);
    // first 4 bytes is the header
    uint8_t idx = 4;
    tx_buf[idx++] = (uint8_t)(XDI_EulerAngles >> 8);
    tx_buf[idx++] = (uint8_t)(XDI_EulerAngles & 0xFF);
    tx_buf[idx++] = (uint8_t)(output_freq >> 8);
    tx_buf[idx++] = (uint8_t)(output_freq & 0xFF);

    tx_buf[idx++] = (uint8_t)(XDI_RateOfTurn >> 8);
    tx_buf[idx++] = (uint8_t)(XDI_RateOfTurn & 0xFF);
    tx_buf[idx++] = (uint8_t)(output_freq >> 8);
    tx_buf[idx++] = (uint8_t)(output_freq & 0xFF);

    tx_buf[idx++] = (uint8_t)(XDI_VelocityXYZ >> 8);
    tx_buf[idx++] = (uint8_t)(XDI_VelocityXYZ & 0xFF);
    tx_buf[idx++] = (uint8_t)(output_freq >> 8);
    tx_buf[idx++] = (uint8_t)(output_freq & 0xFF);

    tx_buf[idx++] = (uint8_t)(XDI_Acceleration >> 8);
    tx_buf[idx++] = (uint8_t)(XDI_Acceleration & 0xFF);
    tx_buf[idx++] = (uint8_t)(output_freq >> 8);
    tx_buf[idx++] = (uint8_t)(output_freq & 0xFF);
    
    m_device->sendXbusMessage(tx_buf);

    return;
}

void MTI7::goToConfigurationMode(void) {
	handleEvent(EVT_GotoConfig, m_dataBuffer);
}

void MTI7::goToMeasurementMode(void) {
	handleEvent(EVT_GotoMeasuring, m_dataBuffer);
}

// Return 0 for success, Errcode otherwise
uint8_t MTI7::processXbusData(const uint8_t* xbusData) {
	if (!Xbus_checkPreamble(xbusData))
		return 101;

	uint8_t messageId = Xbus_getMessageId(xbusData);
	uint16_t payloadLength = Xbus_getPayloadLength(xbusData);
	int index = 4;
	switch (messageId)
	{
		case XMID_Wakeup:
		{
			printf("Got Wakeup MSG while in ready state\n");
			goToConfigurationMode();
			vTaskDelay(5);
			set_default_outputs(100);
			vTaskDelay(5);
			goToMeasurementMode();
			return 0;
		} break;

		case XMID_DeviceId:
		{
			//uint32_t deviceId = readUint32(xbusData, index);
			return 0;
		} break;

		case XMID_GotoConfigAck:
		{
			return 0;
		} break;

		case XMID_GotoMeasurementAck:
		{
			return 0;
		} break;

		case XMID_MtData2:
		{
			
			while(index < payloadLength + 4) {
				uint16_t dataId = readUint16(xbusData, index);
				uint8_t dataSize = readUint8(xbusData, index);

				if (dataId == XDI_EulerAngles && dataSize == 12) {
					euler.roll = readFloat(xbusData, index);
					euler.pitch = readFloat(xbusData, index);
					euler.yaw = readFloat(xbusData, index);
					//printf("XMID_MtData2: roll = %.2f, pitch = %.2f, yaw = %.2f\n", euler.roll , euler.pitch, euler.yaw);
				} else if (dataId == XDI_RateOfTurn && dataSize == 12) {
					angularRate.roll = readFloat(xbusData, index);
					angularRate.pitch = readFloat(xbusData, index);
					angularRate.yaw = readFloat(xbusData, index);
					//printf("XMID_MtData2: roll_rate = %.2f, pitch_rate = %.2f, yaw_rate = %.2f\n", angular_rate.roll , angular_rate.pitch, angular_rate.yaw_rate);
				} else if (dataId == XDI_Acceleration && dataSize == 12) {
                    acceleration.x = readFloat(xbusData, index);
                    acceleration.y = readFloat(xbusData, index);
                    acceleration.z = readFloat(xbusData, index);
				} else if (dataId == XDI_VelocityXYZ && dataSize == 12) {
					velocity.N = readFloat(xbusData, index);
                    velocity.E = readFloat(xbusData, index);
                    velocity.U = readFloat(xbusData, index);
					//printf("XMID_MtData2: E-Vel = %.2f, N-Vel = %.2f, U-Vel = %.2f\n", vel_E, vel_N, vel_U);
				}
				else
				{
					return 102;
				}
			}

			return 0;
		} break;

		case XMID_FirmwareRevision:
		{
			//uint8_t major = readUint8(xbusData, index);
			//uint8_t minor = readUint8(xbusData, index);
			//uint8_t patch = readUint8(xbusData, index);
			//snprintf(g_textBuffer, sizeof(g_textBuffer), "Firmware revision: %d.%d.%d", major, minor, patch);
		} break;

		case XMID_GotoBootLoaderAck:
            break;
		case XMID_FirmwareUpdate:
            break;

		case XMID_ResetAck:
            break;

		default:
		{
            return 104;
		}
	}
    return 0;
}


