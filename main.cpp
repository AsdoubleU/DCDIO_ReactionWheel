#include <cmath>
#include <iostream>
#include "mbed.h"
#include "MotorControl.hpp"
#include "QEncoder.hpp"
#include "Eigen/Dense.h"
#include "Parameters.hpp"

DigitalOut led1(LED1);
Timer measure_time;
QEncoder  enco(D8,D9);
double theta, pre_theta, theta_dot, u;
Eigen::Vector2d state;

int main(void)
{
    motor = new L6206(D2, A4, D5, D4, A0, A1);

    enco.init();
    enco.setCount(0);

    if (motor->init(&init) != COMPONENT_OK) {
        exit(EXIT_FAILURE);
    }

    motor->attach_flag_interrupt(my_flag_irq_handler);
    motor->attach_error_handler(my_error_handler);
    motor->set_dual_full_bridge_config(PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B);
    motor->set_bridge_input_pwm_freq(0,1000);
    motor->set_bridge_input_pwm_freq(1,2000);

    motor->set_speed(0,100);
    motor->run(0, BDCMotor::FWD);
    wait(0.5);
    motor->hard_hiz(1);
    motor->hard_hiz(0);

    while (true) {

        measure_time.reset();
        measure_time.start();

        theta = (0.8)*enco.getCount()*(2*PI/8200) + (0.2)*pre_theta;

        measure_time.stop();

        theta_dot = (theta - pre_theta)/measure_time.read();
        pre_theta = theta;
        state << theta,theta_dot;

        double energy = (1/2)*Jp*(theta_dot)*(theta_dot) + M*G*lp*(1-cos(theta));

        u = (10)*(energy - E_ref)*theta_dot;

        motor_control(u);

        std::cout<<"theta : "<<theta<<" , theta_dot : "<<theta_dot<<std::endl;

        }
}
