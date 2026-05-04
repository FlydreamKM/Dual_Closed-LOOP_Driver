/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    controller.h
  * @brief   Motion controller: state machine, trajectory planner, cascaded PID.
  *
  * Control modes:
  *   MODE_SPEED    : single-loop speed PID, angle parameter ignored.
  *   MODE_POSITION : cascaded position + speed PID with trapezoidal trajectory.
  *
  * Commands:
  *   ENABLE, DISABLE, HOME (reset zero), EMERGENCY_STOP, CLEAR_FAULT.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include <stdint.h>

typedef enum {
    MODE_SPEED = 0,
    MODE_POSITION = 1
} ControlMode_t;

typedef enum {
    STATE_IDLE = 0,
    STATE_RUNNING = 1,
    STATE_HOMING = 2,
    STATE_EMERGENCY = 3,
    STATE_FAULT = 4
} MotorState_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_OVERCURRENT = 1,
    FAULT_OVERSPEED = 2,
    FAULT_STALL = 3
} FaultCode_t;

typedef struct {
    ControlMode_t mode;
    MotorState_t state;

    float target_speed;     /* rad/s, signed for direction */
    float target_angle;     /* rad */
    float accel;            /* rad/s^2 (>0) */
    float decel;            /* rad/s^2 (>0) */

    /* Trajectory planner output */
    float traj_speed;       /* rad/s */
    float traj_angle;       /* rad */

    PID_t speed_pid;
    PID_t pos_pid;

    Encoder_t *encoder;
    Motor_t *motor;

    /* Status outputs */
    float actual_speed;
    float actual_angle;
    int16_t pwm_output;
    FaultCode_t fault_code;

    uint8_t enabled;
    uint8_t homing_done;
} MotorController_t;

void Controller_Init(MotorController_t *ctrl, Encoder_t *enc, Motor_t *motor);
void Controller_SetMode(MotorController_t *ctrl, ControlMode_t mode);
void Controller_SetTarget(MotorController_t *ctrl, float speed, float angle, float accel, float decel);
void Controller_SetSpeedPID(MotorController_t *ctrl, float kp, float ki, float kd);
void Controller_SetPosPID(MotorController_t *ctrl, float kp, float ki, float kd);
void Controller_Enable(MotorController_t *ctrl);
void Controller_Disable(MotorController_t *ctrl);
void Controller_EmergencyStop(MotorController_t *ctrl);
void Controller_ClearEmergency(MotorController_t *ctrl);
void Controller_Home(MotorController_t *ctrl);
void Controller_Update(MotorController_t *ctrl);  /* call at 1 ms period */

#ifdef __cplusplus
}
#endif

#endif /* __CONTROLLER_H */
