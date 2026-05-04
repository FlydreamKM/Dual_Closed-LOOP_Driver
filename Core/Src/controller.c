/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    controller.c
  * @brief   Motion controller with trapezoidal trajectory and cascaded PID.
  *
  * Control period: 1 ms (1 kHz).
  ******************************************************************************
  */
/* USER CODE END Header */

#include "controller.h"
#include <math.h>

#define CONTROL_DT_MS   1.0f
#define CONTROL_DT_S    0.001f
#define POS_DEADBAND    0.01f   /* rad, ~0.57 deg */
#define SPEED_DEADBAND  0.1f    /* rad/s */

static void TrajectoryPlanner(MotorController_t *ctrl)
{
    if (ctrl->mode == MODE_POSITION) {
        float pos_error = ctrl->target_angle - ctrl->traj_angle;
        float dir = (pos_error >= 0.0f) ? 1.0f : -1.0f;
        float abs_error = fabsf(pos_error);

        /* Deceleration distance: v^2 / (2*a) */
        float decel_dist = (ctrl->traj_speed * ctrl->traj_speed) / (2.0f * ctrl->decel);

        float target_speed;
        if (abs_error > decel_dist) {
            /* Far enough: accelerate / cruise toward target_speed */
            target_speed = dir * fabsf(ctrl->target_speed);
        } else {
            /* Near target: decelerate */
            target_speed = dir * sqrtf(2.0f * ctrl->decel * abs_error);
        }

        /* Rate-limit speed change (acceleration limit) */
        float dv = target_speed - ctrl->traj_speed;
        float max_dv = ctrl->accel * CONTROL_DT_S;
        if (dv > max_dv)       dv = max_dv;
        else if (dv < -max_dv) dv = -max_dv;
        ctrl->traj_speed += dv;

        /* Update trajectory position */
        ctrl->traj_angle += ctrl->traj_speed * CONTROL_DT_S;

        /* Lock if very close and nearly stopped */
        if (abs_error < POS_DEADBAND && fabsf(ctrl->traj_speed) < SPEED_DEADBAND) {
            ctrl->traj_speed = 0.0f;
            ctrl->traj_angle = ctrl->target_angle;
        }
    } else {
        /* Speed mode: ramp target_speed */
        float target_sp = ctrl->target_speed;
        float dv = target_sp - ctrl->traj_speed;
        float max_dv = ctrl->accel * CONTROL_DT_S;
        if (dv > max_dv)       dv = max_dv;
        else if (dv < -max_dv) dv = -max_dv;
        ctrl->traj_speed += dv;
    }
}

void Controller_Init(MotorController_t *ctrl, Encoder_t *enc, Motor_t *motor)
{
    ctrl->encoder = enc;
    ctrl->motor = motor;
    ctrl->mode = MODE_SPEED;
    ctrl->state = STATE_IDLE;
    ctrl->target_speed = 0.0f;
    ctrl->target_angle = 0.0f;
    ctrl->accel = 10.0f;    /* default 10 rad/s^2 */
    ctrl->decel = 10.0f;
    ctrl->traj_speed = 0.0f;
    ctrl->traj_angle = 0.0f;
    ctrl->actual_speed = 0.0f;
    ctrl->actual_angle = 0.0f;
    ctrl->pwm_output = 0;
    ctrl->fault_code = FAULT_NONE;
    ctrl->enabled = 0;
    ctrl->homing_done = 0;

    PID_Init(&ctrl->speed_pid, 2.0f, 0.5f, 0.0f, 1000.0f, 500.0f);
    PID_Init(&ctrl->pos_pid,  5.0f, 0.0f, 0.1f, 100.0f,  50.0f);
}

void Controller_SetMode(MotorController_t *ctrl, ControlMode_t mode)
{
    if (ctrl->state == STATE_EMERGENCY) return;
    ctrl->mode = mode;
    PID_Reset(&ctrl->speed_pid);
    PID_Reset(&ctrl->pos_pid);
    ctrl->traj_speed = 0.0f;
}

void Controller_SetTarget(MotorController_t *ctrl, float speed, float angle, float accel, float decel)
{
    if (ctrl->state == STATE_EMERGENCY) return;
    ctrl->target_speed = speed;
    if (ctrl->mode == MODE_POSITION) {
        ctrl->target_angle = angle;
    }
    if (accel > 0.0f) ctrl->accel = accel;
    if (decel > 0.0f) ctrl->decel = decel;
}

void Controller_SetSpeedPID(MotorController_t *ctrl, float kp, float ki, float kd)
{
    PID_Init(&ctrl->speed_pid, kp, ki, kd, 1000.0f, 500.0f);
}

void Controller_SetPosPID(MotorController_t *ctrl, float kp, float ki, float kd)
{
    PID_Init(&ctrl->pos_pid, kp, ki, kd, 100.0f, 50.0f);
}

void Controller_Enable(MotorController_t *ctrl)
{
    ctrl->enabled = 1;
    Motor_Enable(ctrl->motor);
    if (ctrl->state == STATE_IDLE || ctrl->state == STATE_FAULT) {
        ctrl->state = STATE_RUNNING;
        ctrl->fault_code = FAULT_NONE;
    }
}

void Controller_Disable(MotorController_t *ctrl)
{
    ctrl->enabled = 0;
    Motor_Disable(ctrl->motor);
    ctrl->state = STATE_IDLE;
    ctrl->traj_speed = 0.0f;
    PID_Reset(&ctrl->speed_pid);
    PID_Reset(&ctrl->pos_pid);
}

void Controller_EmergencyStop(MotorController_t *ctrl)
{
    Motor_EmergencyStop(ctrl->motor);
    ctrl->state = STATE_EMERGENCY;
    ctrl->traj_speed = 0.0f;
    ctrl->enabled = 0;
}

void Controller_ClearEmergency(MotorController_t *ctrl)
{
    Motor_ClearEmergency(ctrl->motor);
    ctrl->state = STATE_IDLE;
    ctrl->fault_code = FAULT_NONE;
    PID_Reset(&ctrl->speed_pid);
    PID_Reset(&ctrl->pos_pid);
}

void Controller_Home(MotorController_t *ctrl)
{
    if (ctrl->state == STATE_EMERGENCY) return;
    Encoder_Reset(ctrl->encoder);
    ctrl->traj_angle = 0.0f;
    ctrl->traj_speed = 0.0f;
    ctrl->target_angle = 0.0f;
    ctrl->actual_angle = 0.0f;
    ctrl->homing_done = 1;
    PID_Reset(&ctrl->speed_pid);
    PID_Reset(&ctrl->pos_pid);
    Motor_SetOutput(ctrl->motor, 0);
}

void Controller_Update(MotorController_t *ctrl)
{
    /* Update encoder -> actual_speed / actual_angle */
    Encoder_Update(ctrl->encoder);
    ctrl->actual_speed = ctrl->encoder->actual_speed;
    ctrl->actual_angle = ctrl->encoder->actual_angle;

    if (ctrl->state == STATE_EMERGENCY || ctrl->state == STATE_IDLE || !ctrl->enabled) {
        Motor_SetOutput(ctrl->motor, 0);
        ctrl->pwm_output = 0;
        return;
    }

    /* Trajectory generation */
    TrajectoryPlanner(ctrl);

    float speed_setpoint = ctrl->traj_speed;

    if (ctrl->mode == MODE_POSITION) {
        /* Outer position loop -> speed correction */
        float pos_out = PID_Update(&ctrl->pos_pid, ctrl->traj_angle, ctrl->actual_angle);
        speed_setpoint += pos_out;
    }

    /* Inner speed loop -> PWM */
    float pwm = PID_Update(&ctrl->speed_pid, speed_setpoint, ctrl->actual_speed);

    ctrl->pwm_output = (int16_t)pwm;
    Motor_SetOutput(ctrl->motor, ctrl->pwm_output);
}
