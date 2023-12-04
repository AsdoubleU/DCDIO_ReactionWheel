#include <cmath>
#include <iostream>
#include "mbed.h"
#include "MotorControl.hpp"
#include "QEncoder.hpp"
#include "Eigen/Dense.h"
#include "Parameters.hpp"

#define Kp 214.3207
#define Kd -100
// #define Kp 91524
// #define Kd -300
#define Krd -100
// #define Kp 71160
// #define Kd -7598.1
// #define Krd -31.733


DigitalOut led1(LED1);
Timer measure_time;
QEncoder  motor_enco(D3,D7);
QEncoder  enco(D8,D9);
double theta, pre_theta, theta_dot, theta_r, pre_theta_r, theta_r_dot, u;
Eigen::Vector4d state, Kg;

bool flag;

int main(void)
{
    motor = new L6206(D2, A4, D5, D4, A0, A1);

    enco.init(true);
    enco.setCount(0);
    motor_enco.init(false);
    motor_enco.setCount(0);

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

    state << 0,0,0,0;
    // Kg << -294.4453  , 32.0773   , 0.2618 ,   0.0550;
     Kg <<  -18.1711,   -1.7264,    0.0031,    0.0015;

    while (true) {

        measure_time.reset();
        measure_time.start();

        // if((theta < 3.3 && theta > 3.0) || (theta > -3.3 && theta < -3.0)){
        if((theta < 3.16 && theta > 3.12) || (theta > -3.16 && theta < -3.12)){
            if(theta < 0) {theta = 2*PI + theta;}
            u = Kg.transpose()*state;
        }
        else{
            double energy = (1/2)*Jp*(theta_dot)*(theta_dot) + M*G*lp*(1-cos(theta));
            u = (8)*(energy - E_ref)*theta_dot;
        }

        motor_control(u);

        pre_theta_r = theta_r;
        pre_theta = theta;
        theta_r = motor_enco.getCount()*(2*PI/800);
        theta = enco.getCount()*(2*PI/8200);
        std::cout<<"theta : "<<theta*(180/PI)<<" , theta_dot : "<<theta_dot*(180/PI) << " , theta_r_dot : " << theta_r_dot*(180/PI)<<std::endl;
        state << theta-PI,theta_dot,theta_r,theta_r_dot;

        measure_time.stop();

        // Update velocity
        theta_r_dot = (theta_r - pre_theta_r)/measure_time.read();
        theta_dot = (theta - pre_theta)/measure_time.read();

        }
}
