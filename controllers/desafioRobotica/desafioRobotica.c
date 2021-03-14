#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define MAX_SPEED 5.24 // maximal speed allowed
#define MAX_SENSOR_NUMBER 16 // how many sensors are on the robot
#define DELAY 70 // delay used for the blinking leds
#define MAX_SENSOR_VALUE 1024 // maximal value returned by the sensors
#define MIN_DISTANCE 1.0 // minimal distance, in meters, for an obstacle to be considered
#define WHEEL_WEIGHT_THRESHOLD 100 // minimal weight for the robot to turn
#define PI 3.1415

#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1
#define FRONT_SENSORS 4
#define SIDE_SENSORS 2

// structure to store the data associated to one sensor
typedef struct {
    WbDeviceTag device_tag;
    double wheel_weight[2];
    double radAngleToFront;
} SensorData;

typedef struct {
    float x, y, radAngle;
} Vector;

// structure to store the data associated to the robot
typedef struct {
    double LeftWheelWeight, LeftWheelSpeed;
    double RightWheelWeight, RightWheelSpeed;
    int LeftBorder, RightBorder;
    Vector DistanceReadFromSensor[MAX_SENSOR_NUMBER];
} Robot;

// enum to represent the state of the robot
typedef enum {
    FORWARD, LEFT, RIGHT
} State;

// how much each sensor affects the direction of the robot
static SensorData sensors[MAX_SENSOR_NUMBER] = {
        {.radAngleToFront = -1.570796327},
        {.radAngleToFront = -0.872664626},
        {.radAngleToFront =-0.5235987756},
        {.radAngleToFront =-0.1745329252},
        {.radAngleToFront = 0.1745329252},
        {.radAngleToFront =0.5235987756},
        {.radAngleToFront =0.872664626},
        {.radAngleToFront =1.570796327},
        {.radAngleToFront = 1.570796327},
        {.radAngleToFront = 2.268928028},
        {.radAngleToFront =2.617993878},
        {.radAngleToFront =2.967059728},
        {.radAngleToFront = -2.967059728},
        {.radAngleToFront = -2.617993878},
        {.radAngleToFront = -2.268928028},
        {.radAngleToFront = -1.570796327}};

static int FrontSensors[4] = {2,3,4,5};
static int LeftSensors[2] = {0,1};
static int RightSensors[2] = {6,7};

State DefineDirection(State ActualDirection, Robot *robot);
void TreatSensors(Robot *robot);
void TreatSensors2(Robot *robot);

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
        printf("Sensor %s enabled\n\r",sensor_name);
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
        /*
        TreatSensors(&robot1);
         /*/
        TreatSensors2(&robot1);
        //*/
        state = DefineDirection(state, &robot1);
        // sets the motor speeds
        wb_motor_set_velocity(left_wheel, robot1.LeftWheelSpeed);
        wb_motor_set_velocity(right_wheel, robot1.RightWheelSpeed);
    }

    wb_robot_cleanup();

    return 0;
}

State DefineDirection(State ActualDirection, Robot *robot) {
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
    double distance, sensor_value;
    Vector FreeDirection;

    robot->LeftWheelWeight = 0;
    robot->RightWheelWeight = 0;

    FreeDirection.x = 0;
    FreeDirection.y = 0;

    for(i=0;i<FRONT_SENSORS;i++){
        sensor_value = wb_distance_sensor_get_value(sensors[FrontSensors[i]].device_tag);
        distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE));
        printf("D%d: %2.2f(v) %2.2f(d)\t",FrontSensors[i],sensor_value,distance);
        FreeDirection.y += distance * cos(sensors[FrontSensors[i]].radAngleToFront);
        FreeDirection.x += distance * sin(sensors[FrontSensors[i]].radAngleToFront);
    }
    printf("D: %2.2fx,%2.2fy\n\r",FreeDirection.x,FreeDirection.y);
}

void TreatSensors2(Robot *robot) {
    int i;
    double distance, speed_modifier, sensor_value;
    static SensorData sensorsw[MAX_SENSOR_NUMBER] = {
        {.wheel_weight = {150, 0}}, {.wheel_weight = {200, 0}}, {.wheel_weight = {300, 0}}, {.wheel_weight = {600, 0}},
        {.wheel_weight = {0, 600}}, {.wheel_weight = {0, 300}}, {.wheel_weight = {0, 200}}, {.wheel_weight = {0, 150}},
        {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},
        {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}}};

    robot->LeftWheelWeight = 0;
    robot->RightWheelWeight = 0;

    printf("\n\r");
    for (i = 0; i < MAX_SENSOR_NUMBER; ++i) {
        sensor_value = wb_distance_sensor_get_value(sensors[i].device_tag);
        printf("D%d %2.2f   ",i,sensor_value);
        if (sensor_value == 0.0)
            speed_modifier = 0.0;
        else {
            distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE));  // lookup table inverse.
            if (distance < MIN_DISTANCE)
                speed_modifier = 1 - (distance / MIN_DISTANCE);
            else
                speed_modifier = 0.0;
        }
        robot->LeftWheelWeight += sensorsw[i].wheel_weight[LEFT_WHEEL] * speed_modifier;
        robot->RightWheelWeight -= sensorsw[i].wheel_weight[RIGHT_WHEEL] * speed_modifier;
    }
}