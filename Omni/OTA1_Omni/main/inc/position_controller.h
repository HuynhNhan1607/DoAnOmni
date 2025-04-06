#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <stdbool.h>

void set_target_position(float x, float y);

bool is_target_reached(void);

void start_position_controller(void);

void stop_position_controller(void);

#endif // POSITION_CONTROLLER_H