#include "mbed.h"
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

#define A_BUTTON    0
#define B_BUTTON    1
#define X_BUTTON    2
#define Y_BUTTON    3
#define LEFT_BUMP   4
#define RIGHT_BUMP  5
#define BACK        6
#define START       7
#define CENTRE      8
#define LEFT_STICK  9
#define RIGHT_STICK 10

#define LEFT_STICK_LR_INDEX     0
#define LEFT_STICK_UD_INDEX     1
#define RIGHT_STICK_LR_INDEX    2
#define RIGHT_STICK_UD_INDEX    3
#define RIGHT_TRIGGER_INDEX     4
#define LEFT_TRIGGER_INDEX      5
#define DPAD_LR_INDEX           6
#define DPAD_UP_INDEX           7

#define lMotorControlPin PA_4
#define rMotorControlPin PA_5

#define lMotorDirectionPin PF_13
#define rMotorDirectionPin PE_9

#define lMotorEnablePin PB_10
#define rMotorEnablePin PB_11

DigitalOut led(LED1);

/*==================
Speed Control (DACS)
==================*/
AnalogOut Left_Wheel(lMotorControlPin);
AnalogOut Right_Wheel(rMotorControlPin);

/*=============
Motor Direction
=============*/
DigitalOut lMotorDirection(lMotorDirectionPin);
DigitalOut rMotorDirection(rMotorDirectionPin);

/*==========
Motor Enable
==========*/
DigitalOut lMotorEnable(lMotorEnablePin);
DigitalOut rMotorEnable(rMotorEnablePin);

/*==================
Controller Variables
==================*/
float left_vel = 1.0f;          //left motor velocity, start at max because ESC speed control is difference between Vref and control input.
float right_vel = 1.0f;         //right motor velocity
float right_trig = 0.0f;        //right trigger value
float turn = 0.0f;              //turn value
float turn_tmp = 0.0f;          //temp turn
float remapped_trig = 0.0f;     //remapped trigger
float turn_mag = 0.0f;          //magnitude of the turn

bool a_butt = false;            //enable boolean linked to button A

//Remap number from one range to another
float Remap(float value, float from1, float to1, float from2, float to2) {
    return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
}

//ROS Callback
void controllerCB(const sensor_msgs::Joy &joy_msg) {
    
    //split up joy_msg and take the right trigger and left stick components.
    right_trig = joy_msg.axes[RIGHT_TRIGGER_INDEX];
    turn = joy_msg.axes[LEFT_STICK_LR_INDEX];
    
    //take button input from joy_msg to enable or disable the motors.
    if(joy_msg.buttons[A_BUTTON] == 1){
        a_butt = true;
    }else if(joy_msg.buttons[B_BUTTON] == 1){
        a_butt = false;
    }
}


ros::NodeHandle nh;
ros::Subscriber<sensor_msgs::Joy> sub("joy", &controllerCB);


int main() {
    
    
    //initialise ros node and subscribe to "joy" topic
    nh.initNode();
    nh.subscribe(sub);
    
    //set initial motor directions
    lMotorDirection = 0;
    rMotorDirection = 1;
    
    while(1){
        nh.spinOnce();
        //remap right trigger input
        remapped_trig = Remap(right_trig, -1.0f, 1.0f, 0.0f, 1.0f);
        turn_tmp = turn;
        
        //if the a button is pressed, motors are enabled. 
        if(a_butt == true){
            left_vel = remapped_trig;           //set left wheel velocity to remapped trigger value
            right_vel = 1 - remapped_trig;      //set right wheel velocity to inverted remapped trigger value
            lMotorEnable = 1;                   //enable left motor
            rMotorEnable = 1;                   //enable right motor
            Left_Wheel.write(left_vel);         //set left control pin to left wheel velocity
            Right_Wheel.write(right_vel);       //set right control pin to right wheel velocity
            led = 1;                            //turn onboard LED on to show motors enabled.
        }else{
            lMotorEnable = 0;                   //disable motors
            rMotorEnable = 0;                   
            led = 0;                            //turn onboard led off
        }

        wait_ms(1);
    }

}


