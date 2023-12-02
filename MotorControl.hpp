#include "L6206.h"

#define MAX_MOTOR (4)

static volatile uint16_t gLastError;

L6206_init_t init =
{
    L6206_CONF_PARAM_PARALLE_BRIDGES,
    {L6206_CONF_PARAM_FREQ_PWM1A, L6206_CONF_PARAM_FREQ_PWM2A, L6206_CONF_PARAM_FREQ_PWM1B, L6206_CONF_PARAM_FREQ_PWM2B},
    {100,100,100,100},
    {FORWARD,FORWARD,BACKWARD,FORWARD},
    {INACTIVE,INACTIVE,INACTIVE,INACTIVE},
    {FALSE,FALSE}
};

L6206 *motor;
InterruptIn my_button_irq(USER_BUTTON);

void my_error_handler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;
  
  /* Enter your own code here */
}

void my_flag_irq_handler(void)
{
  /* Code to be customised */
  /************************/
  /* Get the state of bridge A */
  uint16_t bridgeState  = motor->get_bridge_status(0);
  
  if (bridgeState == 0) {
    if ((motor->get_device_state(0) != INACTIVE)||
        (motor->get_device_state(1) != INACTIVE)) {
      /* Bridge A was disabling due to overcurrent or over temperature */
      /* When at least on of its  motor was running */
        my_error_handler(0XBAD0);
    }
  }
  
  /* Get the state of bridge B */
  bridgeState  = motor->get_bridge_status(1);
  
  if (bridgeState == 0)  {
    if ((motor->get_device_state(2) != INACTIVE)||
        (motor->get_device_state(3) != INACTIVE)) {
      /* Bridge A was disabling due to overcurrent or over temperature */
      /* When at least on of its  motor was running */
        my_error_handler(0XBAD1);
    }
  }  
}

void motor_control(float input)
{
    float voltage;
    static bool flag, pre_flag;

    if(input < 0) {
        flag = false;
        input = -input;
    }
    else flag = true;

    if (flag != pre_flag) {
        motor->hard_hiz(0);
        motor->hard_hiz(1);
    }
    pre_flag = flag;

    voltage = input*(100/24);

    if(flag == true){
        motor->set_speed(0,voltage);
        motor->run(0, BDCMotor::FWD);
    }
    else if(flag == false){
        motor->set_speed(1,voltage);
        motor->run(1, BDCMotor::FWD);
    }
}