/**
  ******************************************************************************
  * @file    L6206.h
  * @author  IPC Rennes
  * @version V1.1.0
  * @date    March 02, 2016
  * @brief   L6206 driver (dual full bridge driver)
  * @note     (C) COPYRIGHT 2015 STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
	Based on:         X-CUBE-SPN4/trunk/Drivers/BSP/Components/L6206/L6206.h
	Revision:         0
*/


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __L6206_CLASS_H
#define __L6206_CLASS_H


/* Includes ------------------------------------------------------------------*/

/* ACTION 1 ------------------------------------------------------------------*
 * Include here platform specific header files.                               *
 *----------------------------------------------------------------------------*/		
#include "mbed.h"
/* ACTION 2 ------------------------------------------------------------------*
 * Include here component specific header files.                              *
 *----------------------------------------------------------------------------*/		
#include "L6206_def.h"
/* ACTION 3 ------------------------------------------------------------------*
 * Include here interface specific header files.                              *
 *                                                                            *
 * Example:                                                                   *
 *   #include "HumiditySensor.h"                                              *
 *   #include "TemperatureSensor.h"                                           *
 *----------------------------------------------------------------------------*/
#include "BDCMotor.h"



/* Private constants ---------------------------------------------------------*/

/** @defgroup L6206_Private_Constants L6206 Private Constants
  * @{
  */

/// The Number of L6206 devices required for initialisation is not supported
#define L6206_ERROR_0   (0x8000)
/// Error: Access a motor index greater than the one of the current brigde configuration
#define L6206_ERROR_1   (0x8001)
/// Error: Access a motor index which is not bidirectionnal
#define L6206_ERROR_2   (0x8002)

/// Maximum frequency of the PWMs in Hz
#define L6206_MAX_PWM_FREQ   (100000)

/// Minimum frequency of the PWMs in Hz
#define L6206_MIN_PWM_FREQ   (2)

/// Bridge Input 1A
#define INPUT_1A         (0)
/// Bridge Input 2A
#define INPUT_2A         (1)
/// Bridge Input 1B
#define INPUT_1B         (2)
/// Bridge Input 2B
#define INPUT_2B         (3)

/// Bridge A
#define BRIDGE_A         (0)
/// Bridge B
#define BRIDGE_B         (1)





/* Classes -------------------------------------------------------------------*/

/**
 * @brief Class representing a L6206 component.
 */
class L6206 : public BDCMotor
{
public:

	/*** Constructor and Destructor Methods ***/

	/**
	 * @brief Constructor.
	 */
	L6206(PinName EN_flag_A, PinName EN_flag_B, PinName pwm_1A, PinName pwm_2A, PinName pwm_1B, PinName pwm_2B) : BDCMotor(), flag_A_irq(EN_flag_A), flag_B_irq(EN_flag_B), EN_flag_A(EN_flag_A), EN_flag_B(EN_flag_B), pwm_1A(pwm_1A), pwm_2A(pwm_2A), pwm_1B(pwm_1B), pwm_2B(pwm_2B)
	{
		/* ACTION 4 ----------------------------------------------------------*
		 * Initialize here the component's member variables, one variable per *
		 * line.                                                              *
		 *                                                                    *
		 * Example:                                                           *
		 *   measure = 0;                                                     *
		 *   instance_id = number_of_instances++;                             *
		 *--------------------------------------------------------------------*/
		 
		flagInterruptCallback = 0;
		errorHandlerCallback = 0;
		numberOfDevices = 0;
		deviceInstance = 0;
	}
	
	/**
	 * @brief Destructor.
	 */
	virtual ~L6206(void) {}
	

	/*** Public Component Related Methods ***/

	/* ACTION 5 --------------------------------------------------------------*
	 * Implement here the component's public methods, as wrappers of the C    *
	 * component's functions.                                                 *
	 * They should be:                                                        *
	 *   + Methods with the same name of the C component's virtual table's    *
	 *     functions (1);                                                     *
	 *   + Methods with the same name of the C component's extended virtual   *
	 *     table's functions, if any (2).                                     *
	 *                                                                        *
	 * Example:                                                               *
	 *   virtual int get_value(float *f)  //(1)                               *
	 *   {                                                                    *
	 *     return COMPONENT_get_value(float *f);                              *
	 *   }                                                                    *
	 *                                                                        *
	 *   virtual int enable_feature(void) //(2)                               *
	 *   {                                                                    *
	 *     return COMPONENT_enable_feature();                                 *
	 *   }                                                                    *
	 *------------------------------------------------------------------------*/
	 
    /**
     * @brief  Initializing the component.
     * @param  init Pointer to device specific initalization structure.
     * @retval "0" in case of success, an error code otherwise.
     */
	virtual int init(void *init = NULL)
	{
		return (int) L6206_Init((void *) init);
	}
	
    /**
     * @brief  Getting the ID of the component.
     * @param  id Pointer to an allocated variable to store the ID into.
     * @retval "0" in case of success, an error code otherwise.
     */
	virtual int read_id(uint8_t *id = NULL)
	{
		return (int) L6206_ReadId((uint8_t *) id);
	}


	/**
 	* @brief  Attaches a user callback to the error Handler.
 	* The call back will be then called each time the library detects an error
 	* @param[in] callback Name of the callback to attach to the error Hanlder
 	* @retval None
 	*/
	virtual void attach_error_handler(void (*callback)(uint16_t error))
	{
		L6206_AttachErrorHandler((void (*)(uint16_t error)) callback);
	}


	/**
 	* @brief  Attaches a user callback to the flag Interrupt
 	* The call back will be then called each time the status 
 	* flag pin will be pulled down due to the occurrence of 
 	* a programmed alarms ( OCD, thermal alert)
 	* @param[in] callback Name of the callback to attach 
 	* to the Flag Interrupt
 	* @retval None
 	*/
	virtual void attach_flag_interrupt(void (*callback)(void))
	{
		L6206_attach_flag_interrupt((void (*)(void)) callback);
	}

	/**
 	* @brief  Returns the current speed of the specified motor
 	* @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 	* @retval current speed in % from 0 to 100
 	*/
	virtual unsigned int get_speed(unsigned int motorId)
	{
		return (unsigned int) L6206_GetCurrentSpeed((uint8_t) motorId);
	}

	/**
 	* @brief Returns the device state
 	* @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 	* @retval State (STEADY or INACTIVE)
 	*/
	virtual unsigned int get_device_state(unsigned int motorId)
	{
		return (motorState_t) L6206_get_device_state((uint8_t) motorId);
	}

	/**
 	* @brief Returns the FW version of the library
 	* @retval L6206_FW_VERSION
 	*/
	virtual uint8_t get_fw_version(void)
	{
		return (uint8_t) L6206_GetFwVersion();
	}

	/**
 	* @brief  Returns the max  speed of the specified motor
 	* @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 	* @retval maxSpeed in % from 0 to 100
 	*/
	virtual uint16_t get_max_speed(unsigned int motorId)
	{
		return (uint16_t) L6206_GetMaxSpeed((uint8_t) motorId);
	}

	/**
 	* @brief  Stops the motor without disabling the bridge
 	* @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 	* @retval none
 	*/
	virtual void hard_stop(unsigned int motorId)
	{
		L6206_HardStop((uint8_t) motorId);
	}

	/**
 	* @brief  Runs the motor
 	* @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 	* @param[in] direction FORWARD or BACKWARD
 	* @retval None
 	* @note  For unidirectionnal motor, direction parameter has no effect
 	*/
	virtual void run(unsigned int motorId, direction_t direction)
	{
		L6206_Run((uint8_t) motorId, (motorDir_t) direction);
	}

	/**
 	* @brief  Changes the max speed of the specified device
 	* @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 	* @param[in] newMaxSpeed in % from 0 to 100
 	* @retval true if the command is successfully executed, else false
 	*/
	virtual bool set_speed(unsigned int motorId, unsigned int newMaxSpeed)
	{
		return (bool) L6206_SetMaxSpeed((uint8_t) motorId, (uint16_t) newMaxSpeed);
	}

	/**
 	* @brief Disable the specified bridge
 	* @param[in] bridgeId (from 0 for bridge A to 1 for bridge B)
 	* @retval None
 	* @note  When input of different brigdes are parallelized 
 	* together, the disabling of one bridge leads to the disabling
 	* of the second one
 	*/
	virtual void disable_bridge(unsigned int bridgeId)
	{
		L6206_DisableBridge((uint8_t) bridgeId);
	}

	/**
 	* @brief Enable the specified bridge
 	* @param[in] bridgeId (from 0 for bridge A to 1 for bridge B)
 	* @retval None
 	* @note  When input of different brigdes are parallelized 
 	* together, the enabling of one bridge leads to the enabling
 	* of the second one
 	*/
	virtual void enable_bridge(unsigned int bridgeId)
	{
		L6206_EnableBridge((uint8_t) bridgeId);
	}

	/**
 	* @brief  Get the status of the bridge enabling of the corresponding bridge
 	* @param[in] bridgeId from 0 for bridge A to 1 for bridge B
 	* @retval State of the Enable&Flag pin of the corresponding bridge (1 set, 0 for reset)
  	*/
	virtual unsigned int get_bridge_status(unsigned int bridgeId)
	{
		return (unsigned int) L6206_GetBridgeStatus((uint8_t) bridgeId);
	}

	/**
 	* @brief  Immediatly stops the motor and disable the power bridge
 	* @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS 
 	* @retval None
 	* @note  if two motors uses the same power bridge, the 
 	* power bridge will be disable only if the two motors are stopped
 	*/
	virtual void hard_hiz(unsigned int motorId)
	{
		L6206_HardHiz((uint8_t) motorId);
	}
	
	/**
 	* @brief Error handler which calls the user callback (if defined)
 	* @param[in] error Number of the error
 	* @retval None
 	*/
	virtual void error_handler(uint16_t error)
	{
		L6206_ErrorHandler((uint16_t) error);
	}

	/**
 	* @brief Set dual full bridge parallelling configuration
 	* @param[in] newConfig bridge configuration to apply from 
 	* dualFullBridgeConfig_t enum
 	* @retval None
 	*/
	virtual void set_dual_full_bridge_config(unsigned int newConfig)
	{
		L6206_SetDualFullBridgeConfig((uint8_t) newConfig);
	}

	/**
 	* @brief  Get the PWM frequency of the specified bridge 
 	* @param[in] bridgeId 0 for bridge A, 1 for bridge B
 	* @retval Freq in Hz
 	*/
	virtual unsigned int  get_bridge_input_pwm_freq(unsigned int bridgeId)
	{
		return (unsigned int) L6206_GetBridgeInputPwmFreq((uint8_t) bridgeId);
	}

	/**
 	* @brief  Changes the PWM frequency of the bridge input
 	* @param[in] bridgeId 0 for bridge A, 1 for bridge B
 	* @param[in] newFreq in Hz
 	* @retval None
 	*/
	virtual void set_bridge_input_pwm_freq(unsigned int bridgeId, unsigned int newFreq)
	{
		L6206_SetBridgeInputPwmFreq((uint8_t) bridgeId, (uint32_t) newFreq);
	}

	/**
 	* @brief  Sets the number of devices to be used
 	* @param[in] nbDevices (from 1 to MAX_NUMBER_OF_DEVICES)
 	* @retval TRUE if successfull, FALSE if failure, attempt to set a number of
 	* devices greater than MAX_NUMBER_OF_DEVICES
 	*/
	virtual bool set_nb_devices(uint8_t nbDevices)
	{
		return (bool) L6206_SetNbDevices((uint8_t) nbDevices);
	}


	/*** Public Interrupt Related Methods ***/

	/* ACTION 6 --------------------------------------------------------------*
	 * Implement here interrupt related methods, if any.                      *
	 * Note that interrupt handling is platform dependent, e.g.:              *
	 *   + mbed:                                                              *
	 *     InterruptIn feature_irq(pin);           //Interrupt object.        *
	 *     feature_irq.fall(callback);             //Attach a callback.       *
	 *     feature_irq.mode(PullNone);             //Set interrupt mode.      *
	 *     feature_irq.enable_irq();               //Enable interrupt.        *
	 *     feature_irq.disable_irq();              //Disable interrupt.       *
	 *   + Arduino:                                                           *
	 *     attachInterrupt(pin, callback, RISING); //Attach a callback.       *
	 *     detachInterrupt(pin);                   //Detach a callback.       *
	 *                                                                        *
	 * Example (mbed):                                                        *
	 *   void attach_feature_irq(void (*fptr) (void))                         *
	 *   {                                                                    *
	 *     feature_irq.fall(fptr);                                            *
	 *   }                                                                    *
	 *                                                                        *
	 *   void enable_feature_irq(void)                                        *
	 *   {                                                                    *
	 *     feature_irq.enable_irq();                                          *
	 *   }                                                                    *
	 *                                                                        *
	 *   void disable_feature_irq(void)                                       *
	 *   {                                                                    *
	 *     feature_irq.disable_irq();                                         *
	 *   }                                                                    *
	 *------------------------------------------------------------------------*/


	/**
     * @brief  Enabling the FLAG interrupt handling.
     * @param[in]  bridgeId 0 for bridge A, 1 for bridge B
     * @retval None.
     */
    virtual void enable_flag_irq(uint8_t bridgeId)
    {
		if (bridgeId == BRIDGE_A)
		{
			flag_A_irq.mode(PullUp);
			flag_A_irq.fall(this, &L6206::L6206_FlagInterruptHandler);
		}
		else
		{
			flag_B_irq.mode(PullUp);
			flag_B_irq.fall(this, &L6206::L6206_FlagInterruptHandler);
		}
    }

    /**
     * @brief  Disabling the FLAG interrupt handling.
     * @param[in]  bridgeId 0 for bridge A, 1 for bridge B
     * @retval None.
     */
    virtual void disable_flag_irq(uint8_t bridgeId)
    {
    	if (bridgeId == BRIDGE_A)
    	{
    		flag_A_irq.fall(NULL);
		}
    	else
    	{
    		flag_B_irq.fall(NULL);
		}
    }


	/*** Public In/Out Related Methods ***/



protected:

	/*** Protected Component Related Methods ***/

	/* ACTION 7 --------------------------------------------------------------*
	 * Declare here the component's specific methods.                         *
	 * They should be:                                                        *
	 *   + Methods with the same name of the C component's virtual table's    *
	 *     functions (1);                                                     *
	 *   + Methods with the same name of the C component's extended virtual   *
	 *     table's functions, if any (2);                                     *
	 *   + Helper methods, if any, like functions declared in the component's *
	 *     source files but not pointed by the component's virtual table (3). *
	 *                                                                        *
	 * Example:                                                               *
	 *   status_t COMPONENT_get_value(float *f);   //(1)                      *
	 *   status_t COMPONENT_enable_feature(void);  //(2)                      *
	 *   status_t COMPONENT_compute_average(void); //(3)                      *
	 *------------------------------------------------------------------------*/
	status_t L6206_Init(void *init);
	status_t L6206_ReadId(uint8_t *id);
	void L6206_TickHandler(uint8_t deviceId);                                    //Handle the device state machine at each tick timer pulse end
	void L6206_AttachErrorHandler(void (*callback)(uint16_t error));  //Attach a user callback to the error handler
	void L6206_attach_flag_interrupt(void (*callback)(void));                  //Attach a user callback to the flag Interrupt
	void L6206_DisableBridge(uint8_t bridgeId);                              //Disable the specified bridge
	void L6206_EnableBridge(uint8_t bridgeId);                               //Enable the specified bridge
	uint16_t L6206_GetBridgeStatus(uint8_t deviceId);                        //Get bridge status
	uint16_t L6206_GetCurrentSpeed(uint8_t motorId);                         //Return the current speed in pps
	motorState_t L6206_get_device_state(uint8_t motorId);                      //Return the device state
	uint8_t L6206_GetFwVersion(void);                                        //Return the FW version
	uint16_t L6206_GetMaxSpeed(uint8_t motorId);                             //Return the max speed in pps
	void L6206_HardHiz(uint8_t motorId);                                     //Stop the motor and disable the power bridge
	void L6206_HardStop(uint8_t motorId);                                    //Stop the motor without disabling the power bridge
	void L6206_Run(uint8_t motorId, motorDir_t direction);                   //run the motor
	uint32_t L6206_GetBridgeInputPwmFreq(uint8_t bridgeId);                  // Get the PWM frequency of the bridge input
	void L6206_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq);    // Set the PWM frequency of the bridge input
	void L6206_SetDualFullBridgeConfig(uint8_t newConfig);                   // Set dual full bridge configuration
	bool L6206_SetMaxSpeed(uint8_t motorId,uint16_t newMaxSpeed);            //Set the max speed in pps
	bool L6206_SetNbDevices(uint8_t nbDevices);                              //Set the number of driver devices
	void L6206_ErrorHandler(uint16_t error);
	void L6206_FlagInterruptHandler(void);
	uint8_t L6206_GetBridgeIdUsedByMotorId(uint8_t motorId);
	uint8_t L6206_GetBridgeInputUsedByMotorId(uint8_t motorId);
	uint8_t L6206_GetMotorIdUsingbridgeInput(uint8_t bridgeInput);
	uint8_t L6206_GetSecondBridgeInputUsedByMotorId(uint8_t motorId);
	bool L6206_IsBidirectionnalMotor(uint8_t motorId);
	void L6206_SetDeviceParamsToPredefinedValues(void);
	void L6206_SetDeviceParamsToGivenValues(L6206_init_t* initDevicePrm);

	/*** Component's I/O Methods ***/

	/* ACTION 8 --------------------------------------------------------------*
	 * Implement here other I/O methods beyond those already implemented      *
	 * above, which are declared extern within the component's header file.   *
	 *------------------------------------------------------------------------*/
	 
	/**
     * @brief      Utility function to set or unset EN pin for Bridge A or Bridge B.
     * @param[out] none
     * @param[in]  bridgeId 0 for bridge A, 1 for bridge B
     * @retval     none
     */
	void L6206_OutVal( uint8_t bridgeId, uint8_t val)
    {
    	if( bridgeId == BRIDGE_A)
    	{
    		EN_flag_A.output();
    		EN_flag_A.mode(PullNone);
    		EN_flag_A.write(val);
    	}
    	else
    	{
    		EN_flag_B.output();
    		EN_flag_B.mode(PullNone);
    		EN_flag_B.write(val);
    	}
    }
	 
    /**
     * @brief  Making the CPU wait.
     * @param  None.
     * @retval None.
     */
	void L6206_Board_Delay(uint32_t delay)
	{
		wait_ms(delay);
	}

	/**
 	* @brief Disable the specified bridge
 	* @param[in] bridgeId (from 0 for bridge A to 1 for bridge B)
 	* @retval None
 	* @note  When input of different brigdes are parallelized 
 	* together, the disabling of one bridge leads to the disabling
 	* of the second one
 	*/
	void L6206_Board_DisableBridge(uint8_t bridgeId)
	{
		disable_flag_irq(BRIDGE_A);
		disable_flag_irq(BRIDGE_B);

		__disable_irq();
		L6206_OutVal( bridgeId, 0);
		__enable_irq();
	}

	/**
 	* @brief Enable the specified bridge
 	* @param[in] bridgeId (from 0 for bridge A to 1 for bridge B)
 	* @retval None
 	* @note  When input of different brigdes are parallelized 
 	* together, the enabling of one bridge leads to the enabling
 	* of the second one
 	*/
	void L6206_Board_EnableBridge(uint8_t bridgeId, uint8_t addDelay)
	{
		L6206_OutVal( bridgeId, 1);

  		if (addDelay != 0)
  		{
			wait_ms(BSP_MOTOR_CONTROL_BOARD_BRIDGE_TURN_ON_DELAY);
		}

  		enable_flag_irq( bridgeId);
	}
	
	/**
 	* @brief  Returns the FLAG pin state.
 	* @param[in]  bridgeId (from 0 for bridge A to 1 for bridge B)
 	* @retval The FLAG pin value.
 	*/
	uint32_t L6206_Board_GetFlagPinState(uint8_t bridgeId)
	{
  		if (bridgeId == 0)
  		{
  			EN_flag_A.input();
  			return EN_flag_A.read();
  		}
  		else
  		{
  			EN_flag_B.input();
  			return EN_flag_B.read(); 
  		}
	}

	/**
 	* @brief  Initiliases the GPIOs used by the L6206s
 	* @retval None
  	*/
	void L6206_Board_GpioInit(void)
	{
		/* init bridge Enable */
		EN_flag_A.output();
		EN_flag_A.write(0);
		EN_flag_A.input();

		EN_flag_B.output();
		EN_flag_B.write(0);
		EN_flag_B.input();
		

		/* init flag Irq */
		disable_flag_irq(BRIDGE_A);
		disable_flag_irq(BRIDGE_B);

	}

	/**
	* @brief  Sets the frequency of PWM used for bridges inputs
 	* @param[in] bridgeInput 0 for input 1A, 1 for input 2A,
  	* 2 for input 1B,  3 for input 2B
 	* @param[in] newFreq in Hz
 	* @param[in] duty Duty cycle
 	* @retval None
 	* @note The frequency is directly the current speed of the device
 	*/
	void L6206_Board_PwmSetFreq(uint8_t bridgeInput, uint32_t newFreq, uint8_t duty)
	{
        /* Computing the period of PWM. */
        float period = 1.0f / newFreq;
        float duty_cycle;
        int period_us = (int)(period * 1000000);

        if (duty > 100)
        {
        	duty = 100;
    	}
        duty_cycle = (float)duty / 100.0f;

        switch (bridgeInput)
  		{
    		case 0:
    		default:
        		/* Setting the period and the duty-cycle of PWM. */
        		pwm_1A.period_us(period_us);
        		pwm_1A.write(duty_cycle);
      			break;

    		case  1:
        		/* Setting the period and the duty-cycle of PWM. */
        		pwm_2A.period_us(period_us);
        		pwm_2A.write(duty_cycle);
      			break;

    		case 2:
        		/* Setting the period and the duty-cycle of PWM. */
        		pwm_1B.period_us(period_us);
        		pwm_1B.write(duty_cycle);
      			break;

    		case 3:
        		/* Setting the period and the duty-cycle of PWM. */
        		pwm_2B.period_us(period_us);
        		pwm_2B.write(duty_cycle);
      			break;    
  		}
	}

	/**
 	* @brief  Reset the PWM for the specified brigde input
 	* @param[in] bridgeInput 0 for input 1A, 1 for input 2A,
 	* 2 for input 1B, 3 for input 2B
 	* @retval None
  	*/
	void L6206_Board_PwmDeInit(uint8_t bridgeInput)
	{
		  switch (bridgeInput)
		  {
		    case 0:
		    default:
		    	//timer_pwm_1A.detach();
		    	break;

		    case  1:
		    	//timer_pwm_2A.detach();
		    	break;

		    case 2:
		    	//timer_pwm_1B.detach();
		    	break;

		    case 3:
		    	//timer_pwm_2B.detach();
		    	break;
		  }
	}

	/**
 	* @brief  Set the PWM frequency the for the specified bridge input
 	* @param[in] bridgeInput 0 for input 1A, 1 for input 2A,
 	* 2 for input 1B, 3 for input 2B
 	* @retval None
  	*/
	void L6206_Board_PwmInit(uint8_t bridgeInput)
	{
	}

	/**
 	* @brief  Stops the PWM uses for the specified brige input
 	* @param[in] bridgeInput 0 for input 1A, 1 for input 2A,
 	* 2 for input 1B, 3 for input 2B
 	* @retval None
 	*/
	void L6206_Board_PwmStop(uint8_t bridgeInput)
	{
		  switch (bridgeInput)
		  {
		    case 0:
		    default:
        		pwm_1A.write(0.0);
		    	break;

		    case  1:
        		pwm_2A.write(0.0);
		    	break;

		    case 2:
        		pwm_1B.write(0.0);
		    	break;

		    case 3:
        		pwm_2B.write(0.0);
		    	break;
		  }
	}


	/*** Component's Instance Variables ***/

	/* ACTION 9 --------------------------------------------------------------*
	 * Declare here interrupt related variables, if needed.                   *
	 * Note that interrupt handling is platform dependent, see                *
	 * "Interrupt Related Methods" above.                                     *
	 *                                                                        *
	 * Example:                                                               *
	 *   + mbed:                                                              *
	 *     InterruptIn feature_irq;                                           *
	 *------------------------------------------------------------------------*/

	/* Flag Interrupt. */
    InterruptIn flag_A_irq;
    InterruptIn flag_B_irq;

	/* ACTION 10 -------------------------------------------------------------*
	 * Declare here other pin related variables, if needed.                   *
	 *                                                                        *
	 * Example:                                                               *
	 *   + mbed:                                                              *
	 *     DigitalOut standby_reset;                                          *
	 *------------------------------------------------------------------------*/

    /* Digital In/Out for Flag EN pin */
    DigitalInOut EN_flag_A;
	DigitalInOut EN_flag_B;
	
    /* PWM Out  pin */
	PwmOut pwm_1A;
	PwmOut pwm_2A;
	PwmOut pwm_1B;
	PwmOut pwm_2B;

	/* ACTION 11 -------------------------------------------------------------*
	 * Declare here communication related variables, if needed.               *
	 *                                                                        *
	 * Example:                                                               *
	 *   + mbed:                                                              *
	 *     DigitalOut address;                                                *
	 *     DevI2C &dev_i2c;                                                   *
	 *------------------------------------------------------------------------*/

	/* ACTION 12 -------------------------------------------------------------*
	 * Declare here identity related variables, if needed.                    *
	 * Note that there should be only a unique identifier for each component, *
	 * which should be the "who_am_i" parameter.                              *
	 *------------------------------------------------------------------------*/
	/* Identity */
	uint8_t who_am_i;

	/* ACTION 13 -------------------------------------------------------------*
	 * Declare here the component's static and non-static data, one variable  *
	 * per line.                                                              *
	 *                                                                        *
	 * Example:                                                               *
	 *   float measure;                                                       *
	 *   int instance_id;                                                     *
	 *   static int number_of_instances;                                      *
	 *------------------------------------------------------------------------*/
	void (*flagInterruptCallback)(void);
	
	void (*errorHandlerCallback)(uint16_t error);
	
	uint8_t numberOfDevices;
	
	uint8_t deviceInstance;
	
	deviceParams_t devicePrm;


	/** PWM timer variables */

	bool pwm_1A_activated;
	bool pwm_2A_activated;
	bool pwm_1B_activated;
	bool pwm_2B_activated;
};

#endif /* __L6206_CLASS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



