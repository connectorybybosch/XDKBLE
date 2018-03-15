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
/* @file
 * @brief  This Module is Configuration header for SendAccelerometerDataOverBle configurations
 *
 */

/* header definition ******************************************************** */
#ifndef XDK110_SENDACCELEROMETERDATAOVERBLE_H_
#define XDK110_SENDACCELEROMETERDATAOVERBLE_H_

/* local interface declaration ********************************************** */
#include "BCDS_Basics.h"

/* local type and macro definitions */
#define BLE_TX_FREQ                  UINT32_C(1000)             /**< Macro to represent One second time unit*/
#define ONESECONDDELAY 			      UINT16_C(1000)	               /**< one second is represented by this macro */
#define NUMBER_ZERO 			      UINT8_C(0)                       /**< Zero value for BLE variables */
#define NUMBER_ONE 			          UINT8_C(1)                       /**< One value for BLE variables */
#define BLETRANSMITLENGTH  		      UINT8_C(16)                      /**<Transmit length for BLE */
#define ACCEL_RECEIVELENGTH 	      UINT8_C(30)                      /**< Receive length for BLE */
#define ENABLE_FLAG                   UINT8_C(1)                       /**< macro to represent the "enable" */
#define DISABLE_FLAG                  UINT8_C(0)                       /**< macro to represent the "Disable" */
#define REMOTE_ADDRESS_LENGTH         UINT8_C(6)                       /**< The length of remote device address */
#define MAX_DEVICE_LENGTH             UINT8_C(20)                      /**< The Maximum length of bluetooth device name */
#define TIMER_NOT_ENOUGH_MEMORY       (-1L)							   /**<Macro to define not enough memory error in timer*/
#define TIMER_AUTORELOAD_ON           UINT32_C(1)            		   /**< Auto reload of timer is enabled*/
#define TIMER_AUTORELOAD_OFF          UINT32_C(0)             		   /**< Auto reload of timer is disabled*/
#define XDK_BLE_DEVICE_NAME           "XDK_CASTORENA"              		   /**< Name of the BLE device*/
#define XDK_BLE_DEVICE_NAME_LENGTH    UINT8_C(sizeof(XDK_BLE_DEVICE_NAME))/**< length of the BLE device name*/
#define USB_CMD_BUFFER_SIZE           UINT8_C(10)                      /**< Buffer size for commands received over usb*/
#define BLE_RECEIVE_BUFFER_SIZE       UINT8_C(32)                      /**< Buffer size for commands received over BLE*/
#define BLE_START_SYNC_TIMEOUT        UINT32_C(5000)
#define BLE_WAKEUP_SYNC_TIMEOUT       UINT32_C(5000)
#define BLE_SEND_TIMEOUT       UINT32_C(1000)
#define BLE_TRIGGER_START_CMD      UINT32_C(1)
#define BLE_TRIGGER_END_CMD      UINT32_C(0)

/* local function prototype declarations */
/**
 * @brief This is a template function where the user can write his custom application.
 *
 * @param[in] CmdProcessorHandle Handle of the main commandprocessor
 *
 * @param[in] param2  Currently not used will be used in future
 *
 */
void appInitSystem(void * CmdProcessorHandle, uint32_t param2);
Retcode_T initLightSensor(void);
Retcode_T readLightSensor(int *value);
Retcode_T initMagnetometer(void);
void handleLeds(int range);
Retcode_T LedInitialize(void);
double calculateMagnitude(long int x, long int y, long int z);
int handleMagnetometerResponse(double value);
void ledsManagement(int mode);
#endif /* XDK110_SENDACCELEROMETERDATAOVERBLE_H_ */

/** ************************************************************************* */
