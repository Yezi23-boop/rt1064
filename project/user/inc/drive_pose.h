#ifndef _drive_pose_h_
#define _drive_pose_h_

#include "drive_config.h"

typedef struct
{
    float x_cm;
    float y_cm;
    float yaw_deg;
    float body_vx_cm;
    float body_vy_cm;
} drive_pose_struct;

void drive_pose_init(void);
void drive_pose_reset(float x_cm, float y_cm, float yaw_deg);
void drive_pose_reset_origin(float current_yaw_deg);
void drive_pose_update_20ms(const float encoder_count[WHEEL_COUNT], float yaw_deg);
const drive_pose_struct *drive_pose_get(void);

#endif
