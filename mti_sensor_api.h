#ifndef __MTI_SENSOR_API_H__
#define __MTI_SENSOR_API_H__

#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"

#include "xbusmessageid.h"
#include "xsdataidentifier.h"
#include "xbus.h"
#include "mtssp.h"
#include "mtssp_interface.h"

#include "mtssp_driver_spi.h"


class MtsspDriver;
class MtsspInterface;


enum Event
{
	EVT_Start,
	EVT_GotoConfig,
	EVT_GotoMeasuring,
	EVT_Reset,
	EVT_XbusMessage,
	EVT_RequestDeviceId,
};

enum State
{
	STATE_Idle,
	STATE_WaitForWakeUp,
	STATE_WaitForConfigMode,
	STATE_WaitForDeviceId,
	STATE_WaitForFirmwareRevision,
	STATE_WaitForSetOutputConfigurationAck,
	STATE_Ready,
};

struct Euler {
    float roll;
    float pitch;
    float yaw;
};
struct AngularRate {
    float roll;
    float pitch;
    float yaw;
};
struct Acceleration {
    float x;
    float y;
    float z;
};
struct Velocity {
    float N;
    float E;
    float U;
};

class MTI7
{
	public:
		MTI7(MtsspDriver* driver, uint8_t cs_pin, uint8_t drdy_pin, uint8_t reset_pin);
        bool checkDataReadyLine(void);
		void run();
		
        void resetDevice();
		void readDataFromDevice();
		
		void setOutputConfiguration(XsDataIdentifier XDA_identifier, uint16_t output_freq);
		void set_default_outputs(uint16_t output_freq);
		void goToConfigurationMode();
		void goToMeasurementMode();

        // Measurements
        Euler           euler;
        AngularRate     angularRate;
        Acceleration    acceleration;
        Velocity        velocity;
        
	private:
        void handleEvent(Event event, const uint8_t* data = 0);
        uint8_t processXbusData(const uint8_t *xbusData);
		
		State m_state;

		// Pins
		gpio_num_t m_cs_pin;
		gpio_num_t m_drdy_pin;
		gpio_num_t m_reset_pin;

		MtsspDriver* m_driver;
		MtsspInterface* m_device;

		uint8_t m_xbusTxBuffer[256];
		uint8_t m_dataBuffer[256];

};



#endif