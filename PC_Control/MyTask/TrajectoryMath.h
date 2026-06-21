#ifndef __MATH_H__
#define __MATH_H__
#include "Run.h"
#include "LiftSystem.h"
void LiftMotor_UpdateTrajectory(LiftMotor_t *motor, uint32_t current_tick_ms);
void LiftMotor_StopTrajectory(LiftMotor_t *motor);
void LiftMotor_SetTrajectoryTarget(LiftMotor_t *motor, float target_distance_m, float duration_ms);
#endif
