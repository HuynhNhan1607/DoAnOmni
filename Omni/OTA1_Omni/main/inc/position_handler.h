#ifndef POSITION_HANDLER_H
#define POSITION_HANDLER_H

typedef struct RobotPos_t
{
    float pos_x;
    float pos_y;
    float pos_theta;
} RobotPos_t;

typedef struct RobotVel_t
{
    float vel_x;
    float vel_y;
    float omega;
} RobotVel_t;

typedef struct RobotAccel_t
{
    float accel_x;
    float accel_y;
    float accel_z;
} RobotAccel_t;

void get_robot_position(RobotPos_t *pos);

void set_robot_position(RobotPos_t *pos);

void set_control_velocity(float vel_x, float vel_y);

void start_forward_kinematics(int *socket);

void stop_forward_kinematics(void);

#endif