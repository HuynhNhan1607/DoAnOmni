#ifndef POSITION_HANDLER_H
#define POSITION_HANDLER_H

typedef struct RobotPos_t
{
    float pos_x;     // Tọa độ x của robot (mét)
    float pos_y;     // Tọa độ y của robot (mét)
    float pos_theta; // Góc quay của robot (radian)
} RobotPos_t;

void get_robot_position(RobotPos_t *pos);

void set_robot_position(RobotPos_t *pos);

void start_forward_kinematics(int *socket);

void stop_forward_kinematics(void);

#endif