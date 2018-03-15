/*
* Licensee agrees that the example code provided to Licensee has been developed and released by Bosch solely as an example to be used as a potential reference for Licensee�s application development. 
* Fitness and suitability of the example code for any use within Licensee�s applications need to be verified by Licensee on its own authority by taking appropriate state of the art actions and measures (e.g. by means of quality assurance measures).
* Licensee shall be responsible for conducting the development of its applications as well as integration of parts of the example code into such applications, taking into account the state of the art of technology and any statutory regulations and provisions applicable for such applications. Compliance with the functional system requirements and testing there of (including validation of information/data security aspects and functional safety) and release shall be solely incumbent upon Licensee. 
* For the avoidance of doubt, Licensee shall be responsible and fully liable for the applications and any distribution of such applications into the market.
* 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are 
* met:
* 
*     (1) Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer. 
* 
*     (2) Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.  
*     
*     (3)The name of the author may not be used to
*     endorse or promote products derived from this software without
*     specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR 
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
*  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*----------------------------------------------------------------------------*/
/**
* @ingroup APPS_LIST
*
* @defgroup SEND_ACCELEROMETER_DATA_OVER_BLE Send Accelerometer Data Over BLE
* @{
*
* @brief Demo application of Transmitting BMA280 Accelerometer data on BLE(Bluetooth Low Energy) every one second, initiated by auto reloaded timer(freeRTOS)
*
* @details This example demonstrates how to read sensor values from the BMA280 Acceleration sensor and streams them over Bluetooth Low Energy via custom Bi-Directional Service.<br>
* Either use your Android or iOS mobile phone (see [Android](https://play.google.com/store/apps/details?id=com.macdom.ble.blescanner&hl=en) or
* [iOS](https://itunes.apple.com/in/app/lightblue-explorer-bluetooth-low-energy/id557428110?mt=8) App Store) to connect to XDK and receive the data.
 * Send command <b>start (HEX: 0x7374617274)</b>  to XDK via Bluetooth Low Energy, so that streaming of data begins.
 * To stop the streaming send command <b>end (HEX: 656e64)</b>
 *
 * This Application enables the bi-directional service in ble and sends Accelerometer Data over ble and UDP. <br>
 * <b> Bi-Directional Service : </b>
 *
 *  Service             |  Characteristic                |         Attribute-Type             |               UUID                   |
 * ---------------------|--------------------------------|------------------------------------|--------------------------------------|
 * BidirectionalService |    NA                          |         uint8_t                    | b9e875c0-1cfa-11e6-b797-0002a5d5c51b |
 * NA                   |    Rx                          |         uint8_t X[20]              | 0c68d100-266f-11e6-b388-0002a5d5c51b |
 * NA                   |    Tx                          |         uint8_t X[20]              | 1ed9e2c0-266f-11e6-850b-0002a5d5c51b |
 *
 * @file
**/

/* module includes ********************************************************** */
#include "XDKAppInfo.h"
#undef BCDS_MODULE_ID  /* Module ID define before including Basics package*/
#define BCDS_MODULE_ID XDK_APP_MODULE_ID_ACCEL_OVER_BLE

/* own header files */
#include "SendAccelerometerDataOverBle.h"
#include "XdkSensorHandle.h"
#include "XdkUsbResetUtility.h"

/* system header files */
#include <stdio.h>
#include <math.h>
#include "BCDS_Basics.h"

/* additional interface header files */
#include "BCDS_Assert.h"
#include "BCDS_BidirectionalService.h"
#include "BCDS_Ble.h"
#include "BCDS_BlePeripheral.h"
#include "BCDS_CmdProcessor.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"
#include "BCDS_BSP_LED.h"
#include "BSP_BoardType.h"

/* constant definitions ***************************************************** */
#define SECONDS(x) ((portTickType) (x * 1000) / portTICK_RATE_MS)

/* local variables ********************************************************** */

/* global variables ********************************************************* */
CmdProcessor_T *AppCmdProcessorHandle;
static uint8_t bleTransmitStatus = NUMBER_ZERO; /**< Validate the repeated start flag */
static xTimerHandle bleTransmitTimerHandle; /**< variable to store timer handle*/
static SemaphoreHandle_t BleStartSyncSemphr = NULL;
static SemaphoreHandle_t BleWakeUpSyncSemphr = NULL;
static SemaphoreHandle_t SendCompleteSync = NULL;
static int readType = MAGNETOMETER;

/* inline functions ********************************************************* */

/* local functions ********************************************************** */

/**
 * @brief Callback function called on BLE event
 *
 * @param [in]  event : event to be send by BLE during communication.
 *
 * @param [in]  data : void pointer pointing to a data type based on event.
 *                     currently reserved for future use
 *
 * event                                    |   data type                   |
 * -----------------------------------------|-------------------------------|
 * BLE_PERIPHERAL_STARTED                   |   Retcode_T                   |
 * BLE_PERIPHERAL_SERVICES_REGISTERED       |   unused                      |
 * BLE_PERIPHERAL_SLEEP_SUCCEEDED           |   Retcode_T                   |
 * BLE_PERIPHERAL_WAKEUP_SUCCEEDED          |   Retcode_T                   |
 * BLE_PERIPHERAL_CONNECTED                 |   Ble_RemoteDeviceAddress_T   |
 * BLE_PERIPHERAL_DISCONNECTED              |   Ble_RemoteDeviceAddress_T   |
 * BLE_PERIPHERAL_ERROR                     |   Retcode_T                   |
 */
static void BleEventCallBack(BlePeripheral_Event_T event, void * data)
{

    BlePeripheral_Event_T Event = event;
    BCDS_UNUSED(data);
    switch (Event)
    {
    case BLE_PERIPHERAL_SERVICES_REGISTERED:
        break;
    case BLE_PERIPHERAL_STARTED:
        printf("BLE powered ON successfully \r\n");
        if ( xSemaphoreGive( BleStartSyncSemphr ) != pdTRUE)
        {
            /* We would not expect this call to fail because we must have obtained the semaphore to get here.*/
            Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_GIVE_ERROR));
        }
        break;

    case BLE_PERIPHERAL_SLEEP_SUCCEEDED:
        printf("BLE successfully entered into sleep mode \r\n");
        ledsManagement(DISCONNECTED);
        break;

    case BLE_PERIPHERAL_WAKEUP_SUCCEEDED:
        printf("Device Wake up succceded  : \r\n");
        if ( xSemaphoreGive( BleWakeUpSyncSemphr ) != pdTRUE)
        {
            /*We would not expect this call to fail because we must have obtained the semaphore to get here.*/
            Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_GIVE_ERROR));
        }
        ledsManagement(DISCONNECTED);
        break;

    case BLE_PERIPHERAL_CONNECTED:
        printf("Device connected  : \r\n");
        ledsManagement(CONNECTED);
        break;

    case BLE_PERIPHERAL_DISCONNECTED:
        printf("Device Disconnected   : \r\n");
        ledsManagement(DISCONNECTED);
        break;
    case BLE_PERIPHERAL_ERROR:
        printf("BLE Error Event  : \r\n");
        ledsManagement(DISCONNECTED);
        break;

    default:
        /* assertion reason : invalid status of Bluetooth Device */
        assert(false);
        break;
    }
}

/**
 * @brief Callback function called on BLE send completion
 *
 * @param [in]  sendStatus : event to be send by BLE during communication.
 *
 */
static void BleAccelDataSentCallback(Retcode_T sendStatus)
{
    if (RETCODE_OK != sendStatus)
    {
        printf("Error in transmitting the Accel Data over BLE. ERROR Code %ui  : \r\n", (unsigned int) sendStatus);
    }
    if ( xSemaphoreGive( SendCompleteSync ) != pdTRUE)
    {
        /*We would not expect this call to fail because we must have obtained the semaphore to get here.*/
        Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_GIVE_ERROR));
    }
}

/** The function to send start or stop message to Bidirectional DataExchange service
 *  @param [in]  param1 : Unused, Reserved for future use
 *  @param [in]  param2 : Differentiates start and stop command
 */
static void BleStartEndMsgSend(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    Retcode_T bleRetval = RETCODE_OK;

    if (param2 == BLE_TRIGGER_START_CMD)
    {
        bleRetval = BidirectionalService_SendData(((uint8_t*) "X      Y      Z"), ((uint8_t) sizeof("X      Y      Z") - 1));
    }
    if (param2 == BLE_TRIGGER_END_CMD)
    {
        bleRetval = BidirectionalService_SendData(((uint8_t*) "Transfer Terminated!"), ((uint8_t) sizeof("Transfer Terminated!") - 1));
    }
    if (RETCODE_OK == bleRetval)
    {
        if (pdFALSE == xSemaphoreTake(SendCompleteSync, BLE_SEND_TIMEOUT))
        {
            bleRetval = RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_TIME_OUT_ERROR);
        }
    }
    if (RETCODE_OK != bleRetval)
    {
        printf("Not able to send response to start or stop command  on BLE  : \r\n");
    }
}

/**
 * @brief Callback function called on data reception over BLE
 *
 * @param [in]  rxBuffer : Buffer in which received data to be stored.
 *
 * @param [in]  rxDataLength : Length of received data.
 */

static void BleDataReceivedCallBack(uint8_t *rxBuffer, uint8_t rxDataLength)
{
    Retcode_T retVal = RETCODE_OK;
    uint8_t bleReceiveBuff[BLE_RECEIVE_BUFFER_SIZE];
    if (rxDataLength >= BLE_RECEIVE_BUFFER_SIZE)
    {
        printf("Data length received is invalid \n");
    }
    else
    {
        memset(bleReceiveBuff, 0, sizeof(bleReceiveBuff));
        memcpy(bleReceiveBuff, rxBuffer, rxDataLength);
        /* make sure that the received string is null-terminated */
        bleReceiveBuff[rxDataLength] = '\0';

        /* validate received data */
        printf("%s \r\n", (const char *) bleReceiveBuff);
        int compare = strcmp((const char *) bleReceiveBuff, "magneto") == 0 || strcmp((const char *) bleReceiveBuff, "light") == 0;
        printf("compare = %d \n\r", compare);
        printf("%s \r\n", (const char *) bleReceiveBuff);
        if ((NUMBER_ZERO == !compare) && (NUMBER_ZERO == bleTransmitStatus))
        {
        	ledsManagement(DISCONNECTED);
        	readType = strcmp((const char *) bleReceiveBuff, "magneto") == 0 ? MAGNETOMETER : LIGHT;
            retVal = CmdProcessor_enqueue(AppCmdProcessorHandle, BleStartEndMsgSend, NULL, UINT32_C(1));
            if (RETCODE_OK != retVal)
            {
                printf("Failed to Enqueue BleStartEndMsgSend to Application Command Processor \r\n");
            }
            /* start accelerometer data transmission timer */
            if (pdTRUE != xTimerStart(bleTransmitTimerHandle, (ONESECONDDELAY/portTICK_RATE_MS)))
            {
                /* Assertion Reason : Failed to start software timer. Check command queue size of software timer service*/
                assert(false);
            }
            else
            {
                bleTransmitStatus = NUMBER_ONE;
            }
        }
        else if ((NUMBER_ZERO == strcmp((const char *) bleReceiveBuff, "end"))
                && (NUMBER_ONE == bleTransmitStatus))
        {
        	ledsManagement(CONNECTED);

            /* stop accelerometer data transmission timer */
            if (pdTRUE != xTimerStop(bleTransmitTimerHandle, NUMBER_ZERO))
            {
                /* Assertion Reason: Failed to start software timer. Check command queue size of software timer service. */
                assert(false);
            }
            else
            {
                bleTransmitStatus = NUMBER_ZERO;
                retVal = CmdProcessor_enqueue(AppCmdProcessorHandle, BleStartEndMsgSend, NULL, UINT32_C(0));
                if (RETCODE_OK != retVal)
                {
                    printf("Failed to Enqueue BleStartEndMsgSend to Application Command Processor \r\n");
                }
            }

        }
    }
}

/**
 *  The function to register the bidirectional service
 */

static Retcode_T BiDirectionalServiceRegistryCallback(void)
{
    Retcode_T retval = RETCODE_OK;

    retval = BidirectionalService_Init(BleDataReceivedCallBack, BleAccelDataSentCallback);
    if (RETCODE_OK == retval)
    {
        retval = BidirectionalService_Register();
    }

    return (retval);
}

/** The function to get and transfer the accel data using BLE alpwise DataExchange service
 *
 * @brief        Gets the raw data from BMA280 Accel and transfer through the alphwise DataExchange service on BLE
 *
 * @param[in]   *param1: a generic pointer to any context data structure which will be passed to the function when it is invoked by the command processor.
 *
 * @param[in]    param2: a generic 32 bit value  which will be passed to the function when it is invoked by the command processor.
 */
static void BleAccelDataTransmit(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);

    Retcode_T bleRetval = RETCODE_OK;
    /* Return value for software timer */
    int8_t timerReturnVal;

    Retcode_T advancedApiRetValue = (Retcode_T) RETCODE_FAILURE;
    int lightValue = 0;

    /* buffer for accel data receive function */
    uint8_t accelDataRec[ACCEL_RECEIVELENGTH] = { 0 };

    /** structure variable to hold the accel raw data*/
    //Accelerometer_XyzData_T getaccelData = { INT32_C(0), INT32_C(0), INT32_C(0) };
    Magnetometer_XyzData_T getaccelData = { INT32_C(0), INT32_C(0), INT32_C(0), INT32_C(0) };
    if (readType == MAGNETOMETER) {
    	printf("Read MAGNETOMETER\n\r");
    	advancedApiRetValue = Magnetometer_readXyzTeslaData(xdkMagnetometer_BMM150_Handle, &getaccelData);
    }
    else {
    	printf("Read Light Sensor\n\r");
    	advancedApiRetValue = readLightSensor(&lightValue);
    }

    if (RETCODE_OK == advancedApiRetValue)
    {
        /*Copying the Accel value into BLE-Buffer*/
        //sprintf((char*) accelDataRec, "%ld %ld %ld", (long int) getaccelData.xAxisData,
                //(long int) getaccelData.yAxisData, (long int) getaccelData.zAxisData);

        /*print chip id and Accel data of BMA280 on serialport for validation*/
    	if (readType == MAGNETOMETER) {
    		printf("BMA280 Magnetometer Raw Data :\n\rx =%ld\n\ry =%ld\n\rz =%ld\n\rresistance =%d\n\r",
					(long int) getaccelData.xAxisData, (long int) getaccelData.yAxisData, (long int) getaccelData.zAxisData, (int) getaccelData.resistance);
			double mag = calculateMagnitude((long int) getaccelData.xAxisData, (long int) getaccelData.yAxisData, (long int) getaccelData.zAxisData);
			printf("Magnitude = %f", mag);
			lightValue = handleMagnetometerResponse(mag);
    	}
        /*Transmitting the Accel value into target device via Alphwise DataExchange service */
    	sprintf((char*) accelDataRec, "%d", (int) lightValue);

        bleRetval = BidirectionalService_SendData((uint8_t*) accelDataRec, (uint8_t) BLETRANSMITLENGTH);
        if (RETCODE_OK == bleRetval)
        {
            if (pdFALSE == xSemaphoreTake(SendCompleteSync, BLE_SEND_TIMEOUT))
            {
                bleRetval = RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_TIME_OUT_ERROR);
            }
        }
        if (RETCODE_OK != bleRetval)
        /*Device Disconnect and data are discarded by Alphwise DataExchange Service */
        {
            /* clearing the flag */
            bleTransmitStatus = NUMBER_ZERO;

            printf("Not able to send accel data over BLE so Stpping the enqueue timer : \r\n");

            /* Terminating the Accel data transmission timer */
            timerReturnVal = xTimerStop(bleTransmitTimerHandle,
                    NUMBER_ZERO);

            /* BLE timer stop fail case */
            if (TIMER_NOT_ENOUGH_MEMORY == timerReturnVal)
            {
                printf("Failed to stop the enqueue timer : \r\n");

                /* Assertion Reason : "This software timer was not stopped, Due to Insufficient heap memory" */
                assert(false);

            }
        }
    }
    else
    {
        printf("BMA280 XYZ Data read FAILED\n\r");
    }
}

double calculateMagnitude(long int x, long int y, long int z)
{
	double magnitude = sqrt((x*x) + (y*y) + (z*z));

	return magnitude;
}

/**
 * @brief        This is a application timer callback function used to enqueue BleAccelDataTransmit function
 *               to the command processor.
 *
 * @param[in]    pvParameters unused parameter.
 */
static void EnqueueAccelDatatoBLE(void *pvParameters)
{
    BCDS_UNUSED(pvParameters);

    Retcode_T retVal = CmdProcessor_enqueue(AppCmdProcessorHandle, BleAccelDataTransmit, NULL, UINT32_C(0));
    if (RETCODE_OK != retVal)
    {
        printf("Failed to Enqueue BleAccelDataTransmit to Application Command Processor \r\n");
    }
}

/* global functions ********************************************************* */

/**
 * @brief To initializes SendDataOverBLE application for its proper operation
 */
Retcode_T init(void)
{
    Retcode_T retval = RETCODE_OK;

    static_assert((portTICK_RATE_MS != 0), "Tick rate MS is zero");

    BleStartSyncSemphr = xSemaphoreCreateBinary();
    if (NULL == BleStartSyncSemphr)
    {
        return (RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_CREATE_ERROR));
    }
    BleWakeUpSyncSemphr = xSemaphoreCreateBinary();
    if (NULL == BleWakeUpSyncSemphr)
    {
        return (RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_CREATE_ERROR));
    }
    SendCompleteSync = xSemaphoreCreateBinary();
    if (NULL == SendCompleteSync)
    {
        return (RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_CREATE_ERROR));
    }

    /*initialize accel sensor*/
    retval = Accelerometer_init(xdkAccelerometers_BMA280_Handle);
    initLightSensor();
    initMagnetometer();
    LedInitialize();

    if (RETCODE_OK == retval)
    {
        bleTransmitTimerHandle = xTimerCreate((char * const ) "bleTransmitTimerHandle", pdMS_TO_TICKS(BLE_TX_FREQ), TIMER_AUTORELOAD_ON, NULL, EnqueueAccelDatatoBLE);
        if (NULL != bleTransmitTimerHandle)
        {
            retval = BlePeripheral_Initialize(BleEventCallBack, BiDirectionalServiceRegistryCallback);
            if (RETCODE_OK == retval)
            {
                retval = BlePeripheral_SetDeviceName((uint8_t*) XDK_BLE_DEVICE_NAME);
            }
            /* Powering on BLE module*/
            if (RETCODE_OK == retval)
            {
                retval = BlePeripheral_Start();
            }
            if (RETCODE_OK == retval)
            {
                if (pdTRUE != xSemaphoreTake(BleStartSyncSemphr, BLE_START_SYNC_TIMEOUT))
                {
                    printf("Failed to Start BLE before timeout, Ble Initialization failed \n");
                    retval = RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_TIME_OUT_ERROR);
                }
            }
            if (RETCODE_OK == retval)
            {
                retval = BlePeripheral_Wakeup();
            }
            /* Wake up BLE module*/
            if (RETCODE_OK == retval)
            {
                if (pdTRUE != xSemaphoreTake(BleWakeUpSyncSemphr, BLE_WAKEUP_SYNC_TIMEOUT))
                {
                    printf("Failed to Wake up BLE before timeout, Ble Initialization failed \n");
                    retval = RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_TIME_OUT_ERROR);
                }
            }
            if (RETCODE_OK == retval)
            {
                printf(" Ble Initialization succeded \n");
            }
            else
            {
                printf("Ble Initialization Failed \r\n");
            }
        }
        else
        {
            retval = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_FAILURE);
            printf("Failed to create BleTransmitTimerHandle,so ble connection not enabled\n");
        }
    }
    return retval;
}

int handleMagnetometerResponse(double value)
{
	int valueInt = (int) value;
	int retVal = LOW;

	if (valueInt >= 0 && valueInt <= 100) {
		retVal = LOW;
		handleLeds(LOW);
	}
	else if (valueInt > 100 && valueInt <= 250) {
		retVal = MEDIUM;
		handleLeds(MEDIUM);
	}
	else {
		retVal = HIGH;
		handleLeds(HIGH);
	}

	return retVal;
}

Retcode_T readLightSensor(int *value)
{
	uint32_t milliLuxData = UINT32_C(0);
	Retcode_T returnValue = RETCODE_FAILURE;

	returnValue = LightSensor_readLuxData(xdkLightSensor_MAX44009_Handle, &milliLuxData);

	if (RETCODE_OK == returnValue) {
		printf("Light Sensor data: %d \r\n", (unsigned int) milliLuxData);
		if (milliLuxData < 500) {
			*value = LOW;
			handleLeds(LOW);
		}
		else if (milliLuxData > 500 && milliLuxData < 1200) {
			*value = MEDIUM;
			handleLeds(MEDIUM);
		}
		else {
			*value = HIGH;
			handleLeds(HIGH);
		}
	}

	return returnValue;
}

void handleLeds(int range)
{
	printf("Range %d \r\n", range);
	switch(range) {
		case LOW:
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_R, (uint32_t) BSP_LED_COMMAND_ON);
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_O, (uint32_t) BSP_LED_COMMAND_OFF);
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_Y, (uint32_t) BSP_LED_COMMAND_OFF);
		break;
		case MEDIUM:
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_R, (uint32_t) BSP_LED_COMMAND_OFF);
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_O, (uint32_t) BSP_LED_COMMAND_ON);
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_Y, (uint32_t) BSP_LED_COMMAND_OFF);
		break;
		case HIGH:
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_R, (uint32_t) BSP_LED_COMMAND_OFF);
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_O, (uint32_t) BSP_LED_COMMAND_OFF);
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_Y, (uint32_t) BSP_LED_COMMAND_ON);
		break;
	}
}

void ledsManagement(int mode)
{
	switch(mode) {
		case DISCONNECTED:
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_R, (uint32_t) BSP_LED_COMMAND_OFF);
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_O, (uint32_t) BSP_LED_COMMAND_OFF);
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_Y, (uint32_t) BSP_LED_COMMAND_OFF);
		break;
		case CONNECTED:
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_R, (uint32_t) BSP_LED_COMMAND_ON);
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_O, (uint32_t) BSP_LED_COMMAND_ON);
			BSP_LED_Switch((uint32_t) BSP_XDK_LED_Y, (uint32_t) BSP_LED_COMMAND_ON);
		break;
	}
}

Retcode_T initLightSensor(void)
{
	Retcode_T retval = RETCODE_FAILURE, retBrigVal = RETCODE_FAILURE;

	retval = LightSensor_init(xdkLightSensor_MAX44009_Handle);
	if (RETCODE_OK != retval)
		printf("Error init light sensor");

	retBrigVal = LightSensor_setBrightness(xdkLightSensor_MAX44009_Handle, LIGHTSENSOR_NORMAL_BRIGHTNESS);
	if (RETCODE_OK != retBrigVal)
		printf("Error setting brightness");

	printf("Light Sensor succeed");
	return retval;
}

Retcode_T initMagnetometer()
{
	Retcode_T retval = RETCODE_FAILURE, retRange = RETCODE_FAILURE, retMode = RETCODE_FAILURE;

	retval = Magnetometer_init(xdkMagnetometer_BMM150_Handle);
	if (RETCODE_OK != retval)
		printf("Error init magnetometer");

	retRange = Magnetometer_setDataRate(xdkMagnetometer_BMM150_Handle, MAGNETOMETER_BMM150_DATARATE_10HZ);
	if (RETCODE_OK != retRange)
		printf("Magnetometer Error setting range");

	retMode = Magnetometer_setPresetMode(xdkMagnetometer_BMM150_Handle, MAGNETOMETER_BMM150_PRESETMODE_REGULAR);
	if (RETCODE_OK != retMode)
		printf("Magnetometer Error setting preset mode");

	return retval;
}

/* Routine to Initialize the LED */
Retcode_T LedInitialize(void)
{
    Retcode_T returnVal = RETCODE_OK;
    returnVal = BSP_LED_Connect();
    if (RETCODE_OK == returnVal)
    {
        returnVal = BSP_LED_Enable((uint32_t) BSP_XDK_LED_R);
    }
    if (RETCODE_OK == returnVal)
    {
        returnVal = BSP_LED_Enable((uint32_t) BSP_XDK_LED_O);
    }
    if (RETCODE_OK == returnVal)
    {
        returnVal = BSP_LED_Enable((uint32_t) BSP_XDK_LED_Y);
    }
    if (RETCODE_OK == returnVal)
    {
        printf("LED Initialization succeed without error %u \n", (unsigned int) returnVal);
    }
    else
    {
        printf(" Error occurred in LED Initialization routine %u \n", (unsigned int) returnVal);
    }
    return returnVal;
}

/**
 * @brief This is a template function where the user can write his custom application.
 *
 */
void appInitSystem(void * CmdProcessorHandle, uint32_t param2)
{
    if (CmdProcessorHandle == NULL)
    {
        printf("Command processor handle is null \n\r");
        assert(false);
    }
    BCDS_UNUSED(param2);
    AppCmdProcessorHandle = (CmdProcessor_T *) CmdProcessorHandle;
    Retcode_T retVal = RETCODE_OK;
    retVal = init();
    if (RETCODE_OK == retVal)
    {
        printf("SendAccelerometerDataOverBle App Initialization completed successfully \r\n ");
    }
    else
    {
        printf("SendAccelerometerDataOverBle App Initialization failed \r\n ");
    }

}
/**@} */

/** ************************************************************************* */
