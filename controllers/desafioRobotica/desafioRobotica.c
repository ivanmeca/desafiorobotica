#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define MAX_SPEED 5.24 // maximal speed allowed
#define MAX_SENSOR_NUMBER 8 // how many sensors are on the robot
#define DELAY 70 // delay used for the blinking leds
#define MAX_SENSOR_VALUE 1024 // maximal value returned by the sensors
#define MIN_DISTANCE 1.0 // minimal distance, in meters, for an obstacle to be considered
#define WHEEL_WEIGHT_THRESHOLD 100 // minimal weight for the robot to turn

#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1

// structure to store the data associated to one sensor
typedef struct {
    WbDeviceTag device_tag;
    double wheel_weight[2];
} SensorData;
// structure to store the data associated to the robot
typedef struct {
    double LeftWheelSpeed;
    double LeftWheelWeight;
    double RightWheelSpeed;
    double RightWheelWeight;
}Robot;

// enum to represent the state of the robot
typedef enum { FORWARD, LEFT, RIGHT } State;

// how much each sensor affects the direction of the robot
static SensorData sensors[MAX_SENSOR_NUMBER] = {
        {.wheel_weight = {150, 0}}, {.wheel_weight = {200, 0}}, {.wheel_weight = {300, 0}}, {.wheel_weight = {600, 0}},
        {.wheel_weight = {0, 600}}, {.wheel_weight = {0, 300}}, {.wheel_weight = {0, 200}}, {.wheel_weight = {0, 150}}};

State DefineDirection(State ActualDirection,Robot *robot);
void TreatSensors(Robot *robot);

int main() {
    // necessary to initialize Webots
    wb_robot_init();

    // stores simulation time step
    int time_step = wb_robot_get_basic_time_step();

    // stores device IDs for the wheels
    WbDeviceTag left_wheel = wb_robot_get_device("left wheel");
    WbDeviceTag right_wheel = wb_robot_get_device("right wheel");

    char sensor_name[5] = "";
    int i;

    // sets up sensors and stores some info about them
    for (i = 0; i < MAX_SENSOR_NUMBER; ++i) {
        sprintf(sensor_name, "so%d", i);
        sensors[i].device_tag = wb_robot_get_device(sensor_name);
        wb_distance_sensor_enable(sensors[i].device_tag, time_step);
    }

    // sets up wheels
    wb_motor_set_position(left_wheel, INFINITY);
    wb_motor_set_position(right_wheel, INFINITY);
    wb_motor_set_velocity(left_wheel, 0.0);
    wb_motor_set_velocity(right_wheel, 0.0);

    // by default, the robot goes forward
    State state = FORWARD;
    Robot robot1;

    // run simulation
    while (wb_robot_step(time_step) != -1) {
        TreatSensors(&robot1);
        state = DefineDirection(state,&robot1);
        // sets the motor speeds
        wb_motor_set_velocity(left_wheel, robot1.LeftWheelSpeed);
        wb_motor_set_velocity(right_wheel, robot1.RightWheelSpeed);
    }

    wb_robot_cleanup();

    return 0;
}

State DefineDirection(State ActualDirection,Robot *robot){
    switch (ActualDirection) {
        case FORWARD:
            if (robot->LeftWheelWeight > WHEEL_WEIGHT_THRESHOLD) {
                robot->LeftWheelSpeed = 0.7 * MAX_SPEED;
                robot->RightWheelSpeed = -0.7 * MAX_SPEED;
                ActualDirection = LEFT;
            } else if (robot->RightWheelWeight > WHEEL_WEIGHT_THRESHOLD) {
                robot->LeftWheelSpeed = -0.7 * MAX_SPEED;
                robot->RightWheelSpeed = 0.7 * MAX_SPEED;
                ActualDirection = RIGHT;
            } else {
                robot->LeftWheelSpeed = MAX_SPEED;
                robot->RightWheelSpeed = MAX_SPEED;
            }
            break;
        case LEFT:
            if (robot->LeftWheelWeight > WHEEL_WEIGHT_THRESHOLD || robot->RightWheelWeight > WHEEL_WEIGHT_THRESHOLD) {
                robot->LeftWheelSpeed = 0.7 * MAX_SPEED;
                robot->RightWheelSpeed = -0.7 * MAX_SPEED;
            } else {
                robot->LeftWheelSpeed = MAX_SPEED;
                robot->RightWheelSpeed = MAX_SPEED;
                ActualDirection = FORWARD;
            }
            break;
        case RIGHT:
            if (robot->LeftWheelWeight > WHEEL_WEIGHT_THRESHOLD || robot->RightWheelWeight > WHEEL_WEIGHT_THRESHOLD) {
                robot->LeftWheelSpeed = -0.7 * MAX_SPEED;
                robot->RightWheelSpeed = 0.7 * MAX_SPEED;
            } else {
                robot->LeftWheelSpeed = MAX_SPEED;
                robot->RightWheelSpeed = MAX_SPEED;
                ActualDirection = FORWARD;
            }
            break;
    }
    return ActualDirection;
}

void TreatSensors(Robot *robot) {
    int i;
    double distance,speed_modifier, sensor_value;

    robot->LeftWheelWeight =0;
    robot->RightWheelWeight=0;

    for (i = 0; i < MAX_SENSOR_NUMBER; ++i) {
        sensor_value = wb_distance_sensor_get_value(sensors[i].device_tag);
        if (sensor_value == 0.0)
            speed_modifier = 0.0;
        else {
            distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE));  // lookup table inverse.
            if (distance < MIN_DISTANCE)
                speed_modifier = 1 - (distance / MIN_DISTANCE);
            else
                speed_modifier = 0.0;
        }
        robot->LeftWheelWeight += sensors[i].wheel_weight[LEFT_WHEEL] * speed_modifier;
        robot->RightWheelWeight += sensors[i].wheel_weight[RIGHT_WHEEL] * speed_modifier;
    }
}