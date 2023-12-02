/**
  ******************************************************************************
  * @file    L6206_def.h 
  * @author  IPC Rennes
  * @version V1.2.0
  * @date    March 30, 2016
  * @brief   Header for L6206 driver (dual full bridge driver)
  * @note    (C) COPYRIGHT 2015 STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __L6206_H
#define __L6206_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "L6206_config.h"
#include "../Common/motor_def.h"

/** @addtogroup BSP
  * @{
  */   
  
/** @addtogroup Components
  * @{
  */  
   
/** @addtogroup L6206
  * @{
  */   
   
/* Exported Constants --------------------------------------------------------*/

/** @defgroup L6206_Exported_Defines L6206 Exported Defines
  * @{
  */   

/// Current FW version
#define L6206_FW_VERSION (0)
///Max number of Brush DC motors
#define L6206_NB_MAX_MOTORS (4)
///Max number of Bridges
#define L6206_NB_MAX_BRIDGES (2)
///Max number of input of Bridges
#define L6206_NB_MAX_BRIDGE_INPUT (2 * L6206_NB_MAX_BRIDGES)
/// MCU wait time in ms after power bridges are enabled
#define BSP_MOTOR_CONTROL_BOARD_BRIDGE_TURN_ON_DELAY    (20)
   
 /**
  * @}
  */

     
/* Exported Types  -------------------------------------------------------*/

/** @defgroup L6206_Exported_Types L6206 Exported Types
  * @{
  */   




/** @defgroup Initialization_Structure Initialization Structure
  * @{
  */

typedef struct
{
    /// Bridge configuration structure.
    dualFullBridgeConfig_t config;

    /// PWM frequency
    uint32_t pwmFreq[L6206_NB_MAX_BRIDGE_INPUT];

    /// Motor speed
    uint16_t speed[L6206_NB_MAX_MOTORS];

    /// Motor direction
    motorDir_t direction[L6206_NB_MAX_MOTORS];

    /// Current motor state
    motorState_t motionState[L6206_NB_MAX_MOTORS];

    /// Bridge enabled (true) or not (false)
    bool bridgeEnabled[L6206_NB_MAX_BRIDGES];
} deviceParams_t;




typedef deviceParams_t L6206_init_t;



/**
  * @}
  */


/** @defgroup Data_Structure Data Structure
  * @{
  */

/* ACTION --------------------------------------------------------------------*
 * Declare here the structure of component's data, if any, one variable per   *
 * line without initialization.                                               *
 *                                                                            *
 * Example:                                                                   *
 *   typedef struct                                                           *
 *   {                                                                        *
 *       int T0_out;                                                          *
 *       int T1_out;                                                          *
 *       float T0_degC;                                                       *
 *       float T1_degC;                                                       *
 *   } COMPONENT_Data_t;                                                      *
 *----------------------------------------------------------------------------*/
typedef struct
{
  /// Function pointer to flag interrupt call back
  void (*flagInterruptCallback)(void);

  /// Function pointer to error handler call back
  void (*errorHandlerCallback)(uint16_t error);

  /// number of L6206 devices
  uint8_t numberOfDevices;

  /// L6206 driver instance
  uint8_t deviceInstance;

  /// L6206 Device Paramaters structure
  deviceParams_t devicePrm;
} L6206_Data_t;

/**
  * @}
  */



/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/


/** @defgroup MotorControl_Board_Linked_Functions MotorControl Board Linked Functions
  * @{
  */   
///Delay of the requested number of milliseconds
extern void L6206_Board_Delay(void *handle, uint32_t delay);
///Disable the specified bridge
extern void L6206_Board_DisableBridge(void *handle, uint8_t bridgeId);
///Enable the specified bridge
extern void L6206_Board_EnableBridge(void *handle, uint8_t bridgeId, uint8_t addDelay);
//Get the status of the flag and enable Pin
extern uint32_t L6206_Board_GetFlagPinState(void *handle, uint8_t bridgeId);
///Initialise GPIOs used for L6206s
extern void L6206_Board_GpioInit(void *handle);
///Set Briges Inputs PWM frequency and start it
extern void L6206_Board_PwmSetFreq(void *handle, uint8_t bridgeInput, uint32_t newFreq, uint8_t duty);
///Deinitialise the PWM of the specified bridge input
extern void L6206_Board_PwmDeInit(void *handle, uint8_t bridgeInput);
///init the PWM of the specified bridge input
extern void L6206_Board_PwmInit(void *handle, uint8_t bridgeInput);
///Stop the PWM of the specified bridge input
extern void L6206_Board_PwmStop(void *handle, uint8_t bridgeInput);
/**
  * @}
  */



/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
  }
#endif

#endif /* #ifndef __L6206_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


