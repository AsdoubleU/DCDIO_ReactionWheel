/**
  ******************************************************************************
  * @file    L6206.cpp
  * @author  IPC Rennes
  * @version V1.1.0
  * @date    March 02, 2016
  * @brief   L6206 driver (dual full bridge driver)
  * @note     (C) COPYRIGHT 2015 STMicroelectronics
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


/* Generated with STM32CubeTOO -----------------------------------------------*/


/* Revision ------------------------------------------------------------------*/
/*
	Repository:       http://svn.x-nucleodev.codex.cro.st.com/svnroot/X-NucleoDev
	Branch/Trunk/Tag: trunk
	Based on:         X-CUBE-SPN4/trunk/Drivers/BSP/Components/L6206/L6206.c
	Revision:         0
*/

/* Includes ------------------------------------------------------------------*/
#include "L6206.h"
#include "string.h"


/** @addtogroup BSP
  * @{
  */   
  
/** @addtogroup Components
  * @{
  */  
   
/** @addtogroup L6206
  * @{
  */   

/* Private constants  ---------------------------------------------------------*/
    
    
/* Private variables ---------------------------------------------------------*/

/** @defgroup L6206_Private_Variables L6206 Private Variables
  * @{
  */   
static uint8_t l6206ArrayNbMaxMotorsByConfig[PARALLELING_END_ENUM] = {2,3,3,4,2,3,2,3,2,1,2,1,1};
 /**
  * @}
  */

/* Private constant ---------------------------------------------------------*/


/* Public Function prototypes -----------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/

/** @defgroup L6206_Private_functions L6206 Private functions
  * @{
  */  
  

/******************************************************//**
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library 
 * detects an error
 * @param[in] callback Name of the callback to attach 
 * to the error Hanlder
 * @retval None
 **********************************************************/
void L6206::L6206_AttachErrorHandler(void (*callback)(uint16_t error))
{
	errorHandlerCallback = (void (*)(uint16_t error)) callback;
}

/******************************************************//**
 * @brief  Attaches a user callback to the flag Interrupt
 * The call back will be then called each time the status 
 * flag pin will be pulled down due to the occurrence of 
 * a programmed alarms ( OCD, thermal alert)
 * @param[in] callback Name of the callback to attach 
 * to the Flag Interrupt
 * @retval None
 **********************************************************/
void L6206::L6206_attach_flag_interrupt(void (*callback)(void))
{
	flagInterruptCallback = (void (*)(void))callback;
}

/******************************************************//**
 * @brief Disable the specified bridge
 * @param[in] bridgeId (from 0 for bridge A to 1 for bridge B)
 * @retval None
 * @note  When input of different brigdes are parallelized 
 * together, the disabling of one bridge leads to the disabling
 * of the second one
 **********************************************************/
void L6206::L6206_DisableBridge(uint8_t bridgeId)
{
  L6206_Board_DisableBridge(bridgeId);

  devicePrm.bridgeEnabled[bridgeId] = FALSE;
  if (devicePrm.config >= PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR)
  {
    if (bridgeId == BRIDGE_A) 
    {
      L6206_Board_DisableBridge(BRIDGE_B);
      devicePrm.bridgeEnabled[BRIDGE_B] = FALSE;
    }
    else 
    {
      L6206_Board_DisableBridge(BRIDGE_A);
      devicePrm.bridgeEnabled[BRIDGE_A] = FALSE;
    }    
  }  
}

/******************************************************//**
 * @brief Enable the specified bridge
 * @param[in] bridgeId (from 0 for bridge A to 1 for bridge B)
 * @retval None
 * @note  When input of different brigdes are parallelized 
 * together, the enabling of one bridge leads to the enabling
 * of the second one
 **********************************************************/
void L6206::L6206_EnableBridge(uint8_t bridgeId)
{
  devicePrm.bridgeEnabled[bridgeId] = TRUE;
  if (devicePrm.config >= PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR)
  {
    L6206_Board_EnableBridge(bridgeId, 0);
    if (bridgeId == BRIDGE_A) 
    {
      L6206_Board_EnableBridge(BRIDGE_B, 1);
      devicePrm.bridgeEnabled[BRIDGE_B] = TRUE;
    }
    else 
    {
      L6206_Board_EnableBridge(BRIDGE_A, 1);
      devicePrm.bridgeEnabled[BRIDGE_A] = TRUE;
    }    
  }
  else
  {
    L6206_Board_EnableBridge(bridgeId, 1);
  }
}

/******************************************************//**
 * @brief Start the L6206 library
 * @param[in] init pointer to the initialization data
 * @retval None
 **********************************************************/
status_t L6206::L6206_Init(void *init)
{
	deviceInstance++;

	/* Initialise the GPIOs */
	L6206_Board_GpioInit();

	if (init == NULL)
	{
		/* Set context variables to the predefined values from l6206_target_config.h */
		/* Set GPIO according to these values */
		L6206_SetDeviceParamsToPredefinedValues();
	}
	else
	{
		L6206_SetDeviceParamsToGivenValues((L6206_init_t*) init);
	}

	/* Initialise input bridges PWMs */
	L6206_SetDualFullBridgeConfig(devicePrm.config);

	return COMPONENT_OK;
}

/******************************************************//**
 * @brief  Get the PWM frequency of the specified bridge 
 * @param[in] bridgeId 0 for bridge A, 1 for bridge B
 * @retval Freq in Hz
 **********************************************************/
uint32_t L6206::L6206_GetBridgeInputPwmFreq(uint8_t bridgeId)
{
	return (devicePrm.pwmFreq[(bridgeId << 1)]);
}

/******************************************************//**
 * @brief  Returns the current speed of the specified motor
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 * @retval current speed in % from 0 to 100
 **********************************************************/
uint16_t L6206::L6206_GetCurrentSpeed(uint8_t motorId)
{                                                  
	uint16_t speed = 0;

  if (motorId > l6206ArrayNbMaxMotorsByConfig[devicePrm.config])
  {
    L6206_ErrorHandler(L6206_ERROR_1);
  }
  else
  {
  	if (devicePrm.motionState[motorId] != INACTIVE)
    {
      speed = devicePrm.speed[motorId];
    }
  }
  
  return (speed);
}

/******************************************************//**
 * @brief Returns the device state
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 * @retval State (STEADY or INACTIVE)
 **********************************************************/
motorState_t L6206::L6206_get_device_state(uint8_t motorId)
{
	motorState_t state =  INACTIVE;

  if (motorId > l6206ArrayNbMaxMotorsByConfig[devicePrm.config])
  {
    L6206_ErrorHandler(L6206_ERROR_1);
  }
  else
  {
    state =  devicePrm.motionState[motorId];
  }
  return (state);  
}

/******************************************************//**
 * @brief Returns the FW version of the library
 * @retval L6206_FW_VERSION
 **********************************************************/
uint8_t L6206::L6206_GetFwVersion(void)
{
  return (L6206_FW_VERSION);
}

/******************************************************//**
 * @brief  Returns the max  speed of the specified motor
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 * @retval maxSpeed in % from 0 to 100
 **********************************************************/
uint16_t L6206::L6206_GetMaxSpeed(uint8_t motorId)
{
  uint16_t speed = 0;
  if (motorId > l6206ArrayNbMaxMotorsByConfig[devicePrm.config])
  {
    L6206_ErrorHandler(L6206_ERROR_1);
  }
  else
  {
    speed =  devicePrm.speed[motorId];
  }
  return (speed);
}


/******************************************************//**
 * @brief  Get the status of the bridge enabling of the corresponding bridge
 * @param[in] bridgeId from 0 for bridge A to 1 for bridge B
 * @retval State of the Enable&Flag pin of the corresponding bridge (1 set, 0 for reset)
  **********************************************************/
uint16_t L6206::L6206_GetBridgeStatus(uint8_t bridgeId)
{
  uint16_t status = L6206_Board_GetFlagPinState(bridgeId);
  
  return (status);
}

/******************************************************//**
 * @brief  Immediatly stops the motor and disable the power bridge
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 * @retval None
 * @note  if two motors uses the same power bridge, the 
 * power bridge will be disable only if the two motors are
 * stopped
 **********************************************************/
void L6206::L6206_HardHiz(uint8_t motorId)
{
  if (motorId > l6206ArrayNbMaxMotorsByConfig[devicePrm.config])
  {
    L6206_ErrorHandler(L6206_ERROR_1);
  }
  else
  {
     /* Get bridge Id of the corresponding motor */
    uint8_t bridgeId = L6206_GetBridgeIdUsedByMotorId(motorId);
    
    if (devicePrm.bridgeEnabled[bridgeId] != FALSE)
    {
      bool skip = FALSE;

      /* Check if another motor is currently running by using the same bridge */
      switch (devicePrm.config)
      {
        case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
          if ((motorId > 0) && (devicePrm.motionState[1] == STEADY) && (devicePrm.motionState[2] == STEADY))
          {
            skip = TRUE;
          }
          break;
        case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
          if ((motorId < 2) && (devicePrm.motionState[0] == STEADY) && (devicePrm.motionState[1] == STEADY))
          {
            skip = TRUE;
          }
          break;          
        case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
          if (((motorId < 2) && (devicePrm.motionState[0] == STEADY) && (devicePrm.motionState[1] == STEADY))||
              ((motorId > 1) && (devicePrm.motionState[2] == STEADY) && (devicePrm.motionState[3] == STEADY)))
          {
            skip = TRUE;
          }
          break;          
        case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
          if ((motorId > 0) && (devicePrm.motionState[1] == STEADY) && (devicePrm.motionState[2] == STEADY))
          {
            skip = TRUE;
          }
          break;          
        case PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
          if ((motorId < 2) && (devicePrm.motionState[0] == STEADY) && (devicePrm.motionState[1] == STEADY))
          {
            skip = TRUE;
          }
          break;          
        case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A:
          if ((devicePrm.motionState[0] == STEADY) && (devicePrm.motionState[1] == STEADY))
          {
            skip = TRUE;
          }
          break;          
        default:
          break;
      }      

      if (skip == FALSE)
      {
        /* Disable the bridge */
        L6206_DisableBridge(bridgeId);
      }  
    }
    /* Disable the PWM */
    L6206_HardStop(motorId);
  }
}

/******************************************************//**
 * @brief  Stops the motor without disabling the bridge
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 * @retval none
 **********************************************************/
void L6206::L6206_HardStop(uint8_t motorId)
{	
	if (motorId > l6206ArrayNbMaxMotorsByConfig[devicePrm.config])
	{
		L6206_ErrorHandler(L6206_ERROR_1);
	}
	else
	{
		if (devicePrm.motionState[motorId] != INACTIVE)
		{
			uint8_t bridgeInput;
    
			/* Get bridge input of the corresponding motor */
			bridgeInput = L6206_GetBridgeInputUsedByMotorId(motorId);
    
			/* Disable corresponding PWM */
			L6206_Board_PwmStop(bridgeInput);

			/* for bidirectionnal motor, disable second PWM*/
			if (L6206_IsBidirectionnalMotor(motorId))
			{
				bridgeInput = L6206_GetSecondBridgeInputUsedByMotorId(motorId);
				L6206_Board_PwmStop(bridgeInput);
			}
			/* Set inactive state */
			devicePrm.motionState[motorId] = INACTIVE;
		}
	}
}

/******************************************************//**
 * @brief Read id
 * @retval Id of the l6206 Driver Instance
 **********************************************************/
status_t L6206::L6206_ReadId(uint8_t *id)
{
	*id = deviceInstance;

	return COMPONENT_OK;
}

/******************************************************//**
 * @brief  Runs the motor
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 * @note  For unidirectionnal motor, direction parameter has 
 * no effect
 **********************************************************/
void L6206::L6206_Run(uint8_t motorId, motorDir_t direction)
{
  if (motorId > l6206ArrayNbMaxMotorsByConfig[devicePrm.config])
  {
    L6206_ErrorHandler(L6206_ERROR_1);
  }  
  else
  {
    if ((devicePrm.motionState[motorId] == INACTIVE) ||
        (devicePrm.direction[motorId] != direction))
    {
      uint8_t bridgeId;
      uint8_t bridgeInput;
    
      /* Eventually deactivate motor */
      if (devicePrm.motionState[motorId] != INACTIVE)
      {
        L6206_HardStop(motorId);
      }
   
      /* Store new direction */
      devicePrm.direction[motorId] = direction;
    
      /* Switch to steady state */
      devicePrm.motionState[motorId] = STEADY;
    
     /* Get bridge Id of the corresponding motor */
      bridgeId = L6206_GetBridgeIdUsedByMotorId(motorId);
        
      /* Get bridge input of the corresponding motor */
      bridgeInput = L6206_GetBridgeInputUsedByMotorId(motorId);
    
      /* Enable bridge */
      L6206_EnableBridge(bridgeId);
    
      /* Set PWM */
      if (L6206_IsBidirectionnalMotor(motorId))
      {
        /* for bidirectionnal motor */
        L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput],(100 - devicePrm.speed[motorId]));
        bridgeInput = L6206_GetSecondBridgeInputUsedByMotorId(motorId);
        L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput],100);
      }
      else
      {
        /* for unidirectionnal motor */
        L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput],devicePrm.speed[motorId]);
      }
    }
  }
}
/******************************************************//**
 * @brief Set dual full bridge parallelling configuration
 * @param[in] newConfig bridge configuration to apply from 
 * dualFullBridgeConfig_t enum
 * @retval None
 **********************************************************/
void L6206::L6206_SetDualFullBridgeConfig(uint8_t newConfig)
{
  devicePrm.config = (dualFullBridgeConfig_t)newConfig;

  /* Start to reset all else if several inits are used successively */
  /* they will fail */
  L6206_Board_PwmDeInit(INPUT_1A);
  L6206_Board_PwmDeInit(INPUT_2A);
  L6206_Board_PwmDeInit(INPUT_1B);
  L6206_Board_PwmDeInit(INPUT_2B);
  
  
  /* Initialise the bridges inputs PWMs --------------------------------------*/
  switch (devicePrm.config)
  {
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:   
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:  
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:      
        L6206_Board_PwmInit(INPUT_1A);
        L6206_Board_PwmInit(INPUT_2A);
        L6206_Board_PwmInit(INPUT_1B);
        L6206_Board_PwmInit(INPUT_2B);
      break;
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
        L6206_Board_PwmDeInit(INPUT_2A);
        L6206_Board_PwmInit(INPUT_1A);
        L6206_Board_PwmInit(INPUT_1B);
        L6206_Board_PwmInit(INPUT_2B);
      break;         
    case PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
        L6206_Board_PwmDeInit(INPUT_2B);
        L6206_Board_PwmInit(INPUT_1A);
        L6206_Board_PwmInit(INPUT_2A);
        L6206_Board_PwmInit(INPUT_1B);
      break;      
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR:
        L6206_Board_PwmDeInit(INPUT_2A);
        L6206_Board_PwmDeInit(INPUT_2B);
        L6206_Board_PwmInit(INPUT_1A);
        L6206_Board_PwmInit(INPUT_1B);
      break;       
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A:
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR:
        L6206_Board_PwmDeInit(INPUT_1B);
        L6206_Board_PwmDeInit(INPUT_2B);
        L6206_Board_PwmInit(INPUT_1A);
        L6206_Board_PwmInit(INPUT_2A);
      break;      
    case PARALLELING_ALL_WITH_IN1A___1_UNDIR_MOTOR:
        L6206_Board_PwmDeInit(INPUT_2A);
        L6206_Board_PwmDeInit(INPUT_1B);
        L6206_Board_PwmDeInit(INPUT_2B);
        L6206_Board_PwmInit(INPUT_1A);
      break;
    default:
    break;       
  }  
}
/******************************************************//**
 * @brief  Changes the max speed of the specified device
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 * @param[in] newMaxSpeed in % from 0 to 100
 * @retval true if the command is successfully executed, else false
 **********************************************************/
bool L6206::L6206_SetMaxSpeed(uint8_t motorId, uint16_t newMaxSpeed)
{
  bool cmdExecuted = FALSE;

  if (motorId > l6206ArrayNbMaxMotorsByConfig[devicePrm.config])
  {
    L6206_ErrorHandler(L6206_ERROR_1);
  }
  else
  {
    devicePrm.speed[motorId] = newMaxSpeed;
    if (devicePrm.motionState[motorId] != INACTIVE)
    {
      uint8_t bridgeInput;
          
      /* Get Bridge input of the corresponding motor */
      bridgeInput = L6206_GetBridgeInputUsedByMotorId(motorId);
      
      /* Set PWM frequency*/
      if (L6206_IsBidirectionnalMotor(motorId))
      {
        /* for bidirectionnal motor */
        L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput],(100 - devicePrm.speed[motorId]));
      }
      else
      {
        /* for unidirectionnal motor */
        L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput],devicePrm.speed[motorId]);
      }      
    }
    cmdExecuted = TRUE;
  }
  return cmdExecuted;
}                                                     

/******************************************************//**
 * @brief  Changes the PWM frequency of the bridge input
 * @param[in] bridgeId 0 for bridge A, 1 for bridge B
 * @param[in] newFreq in Hz
 * @retval None
 **********************************************************/
void L6206::L6206_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq)
{
  uint8_t loop;
  
  if (newFreq > L6206_MAX_PWM_FREQ)
  {
    newFreq = L6206_MAX_PWM_FREQ;
  }
  for (loop = 0; loop < 2;loop ++)
  {
    uint8_t motorId;
    uint8_t bridgeInput = (bridgeId << 1) + loop; 
    devicePrm.pwmFreq[bridgeInput] = newFreq;
    
    /* Get motor Id using this bridge */
    motorId = L6206_GetMotorIdUsingbridgeInput(bridgeInput);

    /* Immediatly update frequency if motor is running */
    if (devicePrm.motionState[motorId] != INACTIVE)
    {
      /* Test if motor is bidir */
      if (L6206_IsBidirectionnalMotor(motorId))
      {
        if (bridgeInput !=  L6206_GetSecondBridgeInputUsedByMotorId(motorId))
        {
          /* Set PWM frequency for bidirectionnal motor of the first bridge*/
          L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput],(100 - devicePrm.speed[motorId]));
        }
        else
        {
          /* Set PWM frequency for bidirectionnal motor of the second bridge */
          L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput],100);
        }
      }
      else
      {
        /* Set PWM frequency  for unidirectionnal motor */
        L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput],devicePrm.speed[motorId]);
      }
    }
  }
}
/******************************************************//**
 * @brief  Sets the number of devices to be used
 * @param[in] nbDevices (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval TRUE if successfull, FALSE if failure, attempt to set a number of
 * devices greater than MAX_NUMBER_OF_DEVICES
 **********************************************************/
bool L6206::L6206_SetNbDevices(uint8_t nbDevices)
{
  if (nbDevices <= MAX_NUMBER_OF_DEVICES)
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}


/******************************************************//**
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void L6206::L6206_ErrorHandler(uint16_t error)
{
  if (errorHandlerCallback != 0)
  {
    (void) errorHandlerCallback(error);
  }
  else   
  {
    while(1)
    {
      /* Infinite loop */
    }
  }
}

/******************************************************//**
 * @brief  Handlers of the flag interrupt which calls the user callback (if defined)
 * @retval None
 **********************************************************/
void L6206::L6206_FlagInterruptHandler(void)
{
  bool status;
  
  status = L6206_GetBridgeStatus(BRIDGE_A);
  if (status != devicePrm.bridgeEnabled[BRIDGE_A])
  {
    devicePrm.bridgeEnabled[BRIDGE_A] = status;
  }
  
  status = L6206_GetBridgeStatus(BRIDGE_B);
  if (status != devicePrm.bridgeEnabled[BRIDGE_B])
  {
    devicePrm.bridgeEnabled[BRIDGE_B] = status;
  }  
  
  if (flagInterruptCallback != 0)
  {
	  flagInterruptCallback();
  }
}

/******************************************************//**
 * @brief  Get the bridges Id used by a given motor
 * @param motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 * @retval bridgeId 0 for bridge A , 1 for bridge B
 **********************************************************/
uint8_t L6206::L6206_GetBridgeIdUsedByMotorId(uint8_t motorId)
{
	uint8_t bridgeId;

  switch (devicePrm.config)
  {
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:        
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if (motorId == 0)
      {
        bridgeId = 0;
      }
      else
      {
        bridgeId = 1;
      }
      break;  
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if (motorId < 2)
      {
        bridgeId = 0;
      }
      else
      {
        bridgeId = 1;
      }
      break;          
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR:
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A:
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR:
    case PARALLELING_ALL_WITH_IN1A___1_UNDIR_MOTOR:
    default:
        bridgeId = 0;
      break;        
  }  
  return (bridgeId);
}

/******************************************************//**
 * @brief  Get the motor Id which is using the specified bridge input
 * @param bridgeInput 0 for bridgeInput 1A, 1 for 2A, 2 for 1B, 3 for 3B
 * @retval bridgeId 0 for bridge A , 1 for bridge B
 **********************************************************/
uint8_t L6206::L6206_GetMotorIdUsingbridgeInput(uint8_t bridgeInput)
{
	uint8_t motorId;
  
  switch (devicePrm.config)
  {
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:      
    case PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:      
      if (bridgeInput >= INPUT_1B) 
      {
        motorId = 1;
      }
      else
      {
        motorId = 0;
      }
      break;  
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
    case   PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:      
      if (bridgeInput == INPUT_2B) 
      {
        motorId = 2;
      }
      else
      {
      	if (bridgeInput == INPUT_1B) 
        {
          motorId = 1;
        }
        else
        {
          motorId = 0;
        }
      }
      break;        
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:      
      if (bridgeInput >= INPUT_1B) 
      {
        motorId = 2;
      }
      else
      {
      	if (bridgeInput == INPUT_2A) 
        {
          motorId = 1;
        }
        else
        {
          motorId = 0;
        }
      }
      break;           
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      if (bridgeInput == INPUT_2B) 
      {
        motorId = 3;
      }
      else
      {
      	if (bridgeInput == INPUT_1B) 
        {
          motorId = 2;
        }
        else
        {
          if (bridgeInput == INPUT_2A) 
          {
            motorId = 1;
          }
          else
          {
            motorId = 0;
          }
        }
      }
      break;           
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A:
      if ((bridgeInput == INPUT_2A) || (bridgeInput == INPUT_2B))
      {
        motorId = 1;
      }
      else
      {
        motorId = 0;
      }    
      break;
    case   PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR:
    case   PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR:
    case   PARALLELING_ALL_WITH_IN1A___1_UNDIR_MOTOR:
    default:
      motorId = 0;  
      break;        
  }
  
  return (motorId);
}
/******************************************************//**
 * @brief  Get the PWM input used by a given motor
 * @param motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 * @retval PWM input 0 for 1A, 1 for 2A, 2 for 1B, 3 for 3B
 **********************************************************/
uint8_t L6206::L6206_GetBridgeInputUsedByMotorId(uint8_t motorId)
{
	uint8_t bridgeInput;

  switch (devicePrm.config)
  {
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
         if (devicePrm.direction[0] == FORWARD)
         {
           bridgeInput = INPUT_1A;
         }
         else
         {
           bridgeInput = INPUT_2A;
         }
      }
      else
      { 
         if (devicePrm.direction[1] == FORWARD)
         {
           bridgeInput = INPUT_1B;
         }
         else
         {
           bridgeInput = INPUT_2B;
         }
      }  
      break;
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
         if (devicePrm.direction[0] == FORWARD)
         {
           bridgeInput = INPUT_1A;
         }
         else
         {
           bridgeInput = INPUT_2A;
         }
      }
      else
      {
      	if  (motorId == 1)
        {
          bridgeInput = INPUT_1B;
        }
        else
        {
          bridgeInput = INPUT_2B;        
        }
      }
      break;        
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
        bridgeInput = INPUT_1A;
      }
      else
      {
      	if (motorId == 1)
        {
          bridgeInput = INPUT_2A;
        }
        else 
        {
          if (devicePrm.direction[2] == FORWARD)
          {
            bridgeInput = INPUT_1B;
           }
           else
           {
             bridgeInput = INPUT_2B;
           }
        }
      }
      break;   
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
        bridgeInput = INPUT_1A;
      }
      else
      {
          if (motorId == 1)
	      {   
	        bridgeInput = INPUT_2A;
	      }
	      else
	      {
			  if  (motorId == 2)
		      { 
		        bridgeInput = INPUT_1B;
		      }
		      else
		      {
		        bridgeInput = INPUT_2B;
		      }        
	      }
      }
      break;     
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
        bridgeInput = INPUT_1A;
      }
      else 
      {  
        if (devicePrm.direction[1] == FORWARD)
        {   
          bridgeInput = INPUT_1B;
        }
        else
        {   
          bridgeInput = INPUT_2B;
        }
      }
      break;     
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
        bridgeInput = INPUT_1A;
      }
      else
      {
	      if  (motorId == 1)
	      {  
	        bridgeInput = INPUT_1B;
	      }
	      else
	      {   
	        bridgeInput = INPUT_2B;
	      }
      }
      break;
    case PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
         if (devicePrm.direction[0] == FORWARD)
         {
           bridgeInput = INPUT_1A;
         }
         else
         {
           bridgeInput = INPUT_2A;
         }
      }
      else
      { 
        bridgeInput = INPUT_1B;
      }  
      break;
    case PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
        bridgeInput = INPUT_1A;        
      }
      else
      {
	      if  (motorId == 1)
	      { 
	        bridgeInput = INPUT_2A;        
	      }        
	      else
	      {
	        bridgeInput = INPUT_1B;        
	      }
      }
      break;
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
        bridgeInput = INPUT_1A;        
      }
      else
      {
        bridgeInput = INPUT_1B;        
      }
      break;      
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR:
      if (devicePrm.direction[0] == FORWARD)
      {
        bridgeInput = INPUT_1A;
      }
      else
      {
        bridgeInput = INPUT_1B;
      }
      break;       
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A:
      if (motorId == 0)
      {
        bridgeInput = INPUT_1A;
      }
      else
      {
        bridgeInput = INPUT_2A;
      }      
      break;
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR:
      if (devicePrm.direction[0] == FORWARD)
      {
        bridgeInput = INPUT_1A;
      }
      else
      {
        bridgeInput = INPUT_2A;
      }  
      break;      
    case PARALLELING_ALL_WITH_IN1A___1_UNDIR_MOTOR:
    default:
      bridgeInput = INPUT_1A;
      break;       
  }  
  return (bridgeInput);
}

/******************************************************//**
 * @brief  Get the second PWM input used by a given bidirectionnal motor
 * @param motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 * @retval PWM input 0 for 1A, 1 for 2A, 2 for 1B, 3 for 3B
 **********************************************************/
uint8_t L6206::L6206_GetSecondBridgeInputUsedByMotorId(uint8_t motorId)
{
  uint8_t bridgeInput = 0xFF;
  
  switch (devicePrm.config)
  {
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
         if (devicePrm.direction[0] == FORWARD)
         {
           bridgeInput = INPUT_2A;
         }
         else
         {
           bridgeInput = INPUT_1A;
         }
      }
      else
      { 
         if (devicePrm.direction[1] == FORWARD)
         {
           bridgeInput = INPUT_2B;
         }
         else
         {
           bridgeInput = INPUT_1B;
         }
      }  
      break;
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
         if (devicePrm.direction[0] == FORWARD)
         {
           bridgeInput = INPUT_2A;
         }
         else
         {
           bridgeInput = INPUT_1A;
         }
      }
      break;        
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if (motorId == 2)
      { 
         if (devicePrm.direction[2] == FORWARD)
         {
           bridgeInput = INPUT_2B;
         }
         else
         {
           bridgeInput = INPUT_1B;
         }        
      }
      break;   

    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if  (motorId == 1)
      {  
        if (devicePrm.direction[1] == FORWARD)
        {   
          bridgeInput = INPUT_2B;
        }
        else
        {   
          bridgeInput = INPUT_1B;
        }
      }
      break;     

    case PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
         if (devicePrm.direction[0] == FORWARD)
         {
           bridgeInput = INPUT_2A;
         }
         else
         {
           bridgeInput = INPUT_1A;
         }
      }
      break;
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR:
      if (devicePrm.direction[0] == FORWARD)
       {
         bridgeInput = INPUT_1B;
       }
       else
       {
         bridgeInput = INPUT_1A;
       }
      break;       
    
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR:
      if (devicePrm.direction[0] == FORWARD)
      {
        bridgeInput = INPUT_2A;
      }
      else
      {
        bridgeInput = INPUT_1A;
      }  
      break;      
    default:
      bridgeInput = 0XFF;
      break;       
  }  
  if (bridgeInput == 0XFF)
  {
    L6206_ErrorHandler(L6206_ERROR_2);
  }
  
  return (bridgeInput);
}        
        
/******************************************************//**
 * @brief  Test if motor is bidirectionnal
 * @param motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 * @retval True if motor is bidirectionnal, else false
 **********************************************************/
bool L6206::L6206_IsBidirectionnalMotor(uint8_t motorId)
{
  bool isBiDir = FALSE;

  switch (devicePrm.config)
  {
      case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR:
      case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR:
        isBiDir = TRUE;
      break;      
  
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if  (motorId == 0)
      { 
        isBiDir = TRUE;
      }
      break;        
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if  (motorId == 2)
      { 
        isBiDir = TRUE;
      }
      break;   
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if  (motorId == 1)
      { 
        isBiDir = TRUE;
      }
      break;     
  
    default:
      break;       
  }    
  
  return (isBiDir);
}


/******************************************************//**
 * @brief  Sets the parameters of the device to predefined values
 * from l6206_target_config.h
 * @retval None
 **********************************************************/
void L6206::L6206_SetDeviceParamsToPredefinedValues(void)
{
  uint32_t i;

  memset(&devicePrm, 0, sizeof(devicePrm));

  devicePrm.config = L6206_CONF_PARAM_PARALLE_BRIDGES;

  devicePrm.pwmFreq[INPUT_1A] = L6206_CONF_PARAM_FREQ_PWM1A;
  devicePrm.pwmFreq[INPUT_2A] = L6206_CONF_PARAM_FREQ_PWM2A;
  devicePrm.pwmFreq[INPUT_1B] = L6206_CONF_PARAM_FREQ_PWM1B;
  devicePrm.pwmFreq[INPUT_2B] = L6206_CONF_PARAM_FREQ_PWM2B;
  
  for (i = 0; i < MAX_NUMBER_OF_BRUSH_DC_MOTORS; i++)
  {
    devicePrm.speed[i] = 100;
    devicePrm.direction[i] = FORWARD;
    devicePrm.motionState[i] = INACTIVE;
  }
  for (i = 0; i < L6206_NB_MAX_BRIDGES; i++)
  {  
    devicePrm.bridgeEnabled[i] = FALSE;
  }
}


/******************************************************//**
 * @brief  Set the parameters of the device to values of initDevicePrm structure
 * Set GPIO according to these values
 * @param initDevicePrm structure containing values to initialize the device
 * parameters
 * @retval None
 **********************************************************/
void L6206::L6206_SetDeviceParamsToGivenValues(L6206_init_t* initDevicePrm)
{
	memcpy(&devicePrm, initDevicePrm, sizeof(devicePrm));
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
