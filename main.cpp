#include "mbed.h"
#include "Motor.h"
#include "rtos.h"
#include "XNucleo53L0A1.h"
#include <stdio.h>

Serial blue(p9,p10); // Bluetooth 

DigitalOut trigger(p11); // Trigger Pin for Sonar
DigitalIn  echo(p12); // Echo Pin for Sonar

Motor mL(p21, p19, p20); // pwm, fwd, rev (Left Motor)
Motor mR(p22, p16, p18); // pwm, fwd, rev (Right Motor)

PwmOut myservo(p24); // Servo Control via direct PWM

DigitalOut shdn(p26); // Distance Sensor Shutdown pin

DigitalOut myled(LED1); //monitor trigger
DigitalOut myled2(LED2); //monitor echo

static XNucleo53L0A1 *board=NULL;

int s_distance = 0; // Distance Measurement for Sonar
int correction = 0; // Correction value for Sonar

uint32_t distance; // Distance Measuremnt for Lidar
int status; // Status signal for Lidar Init

int servo_state = 1; // Servo state variable to control turn angle

bool no_wall = true; // Wall Detection Flag
bool adjusting = false; // Flag for automatic adjustment phase (to avoid 
                        // motor overwrites from controller)
bool auto_go = false;  // Autonomous Mode Flag

bool sonar_detect = false; // Sonar Detection Threshold Flag
bool lidar_detect = false; // Lidar Detection Threshold Flag

Timer sonar; // Sonar timer

//I2C sensor pins
#define VL53L0_I2C_SDA   p28
#define VL53L0_I2C_SCL   p27

void Servo_Thread() {     //Thread for controlling the servo
    
    int servo_direction = 1; //State variable for direction
    myservo.period(0.020); //Servo needs this exact period value to work
    
    while(1) {                           //Cycles the servo through a set of 6
        if (servo_state == 1) {          //positions to increase sensor coverage
            myservo.pulsewidth(0.0007);
        } else if (servo_state == 2) {
            myservo.pulsewidth(0.0009);
        } else if (servo_state == 3) {
            myservo.pulsewidth(0.0011);
        } else if (servo_state == 4) {
            myservo.pulsewidth(0.0016);
        } else if (servo_state == 5) {
            myservo.pulsewidth(0.0018);
        } else {
            myservo.pulsewidth(0.0020);
        }
        if (servo_direction == 1) {                //State update
                servo_state = servo_state + 1;
            } else {
                servo_state = servo_state - 1;
            }
        if (servo_state == 1 || servo_state == 6){ //Changes servo direction 
            servo_direction = !servo_direction;    //at boundaries
        }
        Thread::wait(250);              //Change state 4 times per second
    }
}
void remote()           //Thread for reading inputs from phone via bluetooth
{
    char bnum=0;        //Usual bluetooth setup for app data
    char bhit=0;
    while(1) {
        if(blue.readable() && !adjusting){
            if (blue.getc()=='!') {
                if (blue.getc()=='B') { //button data packet
                    bnum = blue.getc(); //button number
                    bhit = blue.getc(); //1=hit, 0=release
                    if (blue.getc()==char(~('!' + 'B' + bnum + bhit))) { 
                        switch (bnum) {
                            case '1': //number button 1 - Autonomous mode on
                                if (bhit=='1') {
                                    mL.speed(0.6);
                                    mR.speed(0.6);
                                    auto_go = true;
                                }
                                break;
                            case '2': //number button 2 - Autonomous mode off
                                if (bhit=='1') {
                                    mL.speed(0);
                                    mR.speed(0);
                                    auto_go = false;
                                }
                                break;
                            case '5': //button 5 up arrow - go forwards
                                if (!auto_go) {if (bhit=='1') {
                                    mL.speed(0.8);
                                    mR.speed(0.8);
                                } else {
                                    mL.speed(0);
                                    mR.speed(0);
                                }}
                                break;
                            case '6': //button 6 down arrow - go backwards
                                if (!auto_go) {if (bhit=='1') {
                                    mL.speed(-0.8);
                                    mR.speed(-0.8);
                                } else {
                                    mL.speed(0);
                                    mR.speed(0);
                                }}
                                break;
                            case '7': //button 7 left arrow - turn left
                                if (!auto_go) {if (bhit=='1') {
                                    mL.speed(-0.8);
                                    mR.speed(0.8);
                                } else {
                                    mL.speed(0);
                                    mR.speed(0);
                                }}
                                break;
                            case '8': //button 8 right arrow - turn right
                                if (!auto_go) {if (bhit=='1') {
                                    mL.speed(0.8);
                                    mR.speed(-0.8);
                                } else {
                                    mL.speed(0);
                                    mR.speed(0);
                                }}
                                break;
                            default:
                                break;
                            }
                        }
                    }
                }
            }
        Thread::yield();
    }
}
void Dist_Thread() { //Thread for acquiring Lidar distance measurements
    while(1) {
    status = board->sensor_centre->get_distance(&distance);
    if (status == VL53L0X_ERROR_NONE) {
         printf("D=%ld mm\r\n", distance);
         if (distance < 150) { // Set distance threshold here
             no_wall = false;      //Toggle relevant flags for other threads
             lidar_detect = true;
         }
    }
    Thread::wait(200); //Run 5 times a second
    }
}
void Sonar_Thread() { //Thread for acquiring Sonar measurements
    //Loop to read Sonar distance values, scale, and print
    while(1) {
        // trigger sonar to send a ping
        trigger = 1;
        myled = 1;
        myled2 = 0;
        sonar.reset();
        wait_us(10.0);
        trigger = 0;
        myled = 0;
        //wait for echo high
        while (echo==0) {};
        myled2=echo;
        //echo high, so start timer
        sonar.start();
        //wait for echo low
        while (echo==1) {};
        //stop timer and read value
        sonar.stop();
        //subtract software overhead timer delay and scale to cm
        s_distance = (sonar.read_us()-correction)/58.0;
        myled2 = 0;
        printf(" %d cm \n\r",s_distance);
        if (s_distance < 25) {  //Set distance threshold here
             no_wall = false;   //Toggle relevant flags for other threads
             sonar_detect = true;
         }
        //wait so that any echo(s) return before sending another ping
        Thread::wait(500);
    }
}
int main() {
    sonar.reset();
    // measure actual software polling timer delays
    // delay used later in time correction
    // start timer
    sonar.start();
    // min software polling delay to read echo pin
    while (echo==2) {};
    myled2 = 0;
    // stop timer
    sonar.stop();
    // read timer
    correction = sonar.read_us();
    //printf("Approximate software overhead timer delay is %d uS\n\r",correction);
    
   
    DevI2C *device_i2c = new DevI2C(VL53L0_I2C_SDA, VL53L0_I2C_SCL);
    /* creates the 53L0A1 expansion board singleton obj */
    board = XNucleo53L0A1::instance(device_i2c, A2, D8, D2);
    shdn = 0; //must reset sensor for an mbed reset to work
    Thread::wait(100);
    shdn = 1;
    Thread::wait(100);
    /* init the 53L0A1 board with default values */
    status = board->init_board();
    while (status) {
        status = board->init_board();
    }
    
    Thread dist; //Distance Sensor
    Thread sonar; //Sonar Sensor
    Thread controller; //Remote Control
    Thread serv;  //Servo
    serv.start(callback(Servo_Thread));
    controller.start(callback(remote));
    sonar.start(callback(Sonar_Thread));
    dist.start(callback(Dist_Thread));
    
    while(1){
        if (!no_wall & !adjusting) {
            adjusting = true;
            if (sonar_detect) { //If sonar detects something, back up and turn
                mL.speed(-0.6); 
                mR.speed(-0.6);
                Thread::wait(500);
                mL.speed(0.8); 
                mR.speed(-0.8);
                Thread::wait(500);
                mL.speed(0); 
                mR.speed(0);
                sonar_detect = false;
            } else if (lidar_detect) { //turn depending on state of servo
                if (servo_state == 1) {
                    mL.speed(0.7); 
                    mR.speed(-0.7);
                    Thread::wait(200);
                    mL.speed(0); 
                    mR.speed(0);
                    lidar_detect = false;
                } else if (servo_state == 2) {
                    mL.speed(0.7); 
                    mR.speed(-0.7);
                    Thread::wait(400);
                    mL.speed(0); 
                    mR.speed(0);
                    lidar_detect = false;
                } else if (servo_state == 3) {
                    mL.speed(0.7); 
                    mR.speed(-0.7);
                    Thread::wait(600);
                    mL.speed(0); 
                    mR.speed(0);
                    lidar_detect = false;
                } else if (servo_state == 4) {
                    mL.speed(-0.7); 
                    mR.speed(0.7);
                    Thread::wait(600);
                    mL.speed(0); 
                    mR.speed(0);
                    lidar_detect = false;
                } else if (servo_state == 5) {
                    mL.speed(-0.7); 
                    mR.speed(0.7);
                    Thread::wait(400);
                    mL.speed(0); 
                    mR.speed(0);
                    lidar_detect = false;
                } else if (servo_state == 6) {
                    mL.speed(-0.7); 
                    mR.speed(0.7);
                    Thread::wait(200);
                    mL.speed(0); 
                    mR.speed(0);
                    lidar_detect = false;
                }
                
            }
            adjusting = false;
            if (auto_go) {  //autonomous forward movement
                mL.speed(0.6); 
                mR.speed(0.6);
            }
        }
        Thread::wait(100);
    }
    
}
