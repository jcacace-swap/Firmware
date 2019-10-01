/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file SMPositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#include <matrix/matrix/math.hpp>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>


#include <px4_module_params.h>
#pragma once

struct SMPositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

struct VEHICLE_ATTITUDE {
	//Attitude
	double qw;
	double qx;
	double qy;
	double qz;

	//dot Attotide
	double droll;
	double dpitch;
	double dyaw;

	double roll;
	double pitch;
	double yaw;
};

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 */
class SMPositionControl : public ModuleParams
{
public:

	SMPositionControl(ModuleParams *parent);
	~SMPositionControl() = default;

	/**
	 *	Overwrites certain parameters.
	 *	Overwrites are required for unit-conversion.
	 *	This method should only be called if parameters
	 *	have been updated.
	 */
	void overwriteParams();

	/**
	 * Update the current vehicle state.
	 * @param SMPositionControlStates structure
	 */
	void updateState(const SMPositionControlStates &states);

	/**
	 * Update the desired setpoints.
	 * @param setpoint a vehicle_local_position_setpoint_s structure
	 */
	void updateSetpoint(const vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Set constraints that are stricter than the global limits.
	 * @param constraints a SMPositionControl structure with supported constraints
	 */
	void updateConstraints(const vehicle_constraints_s &constraints);


	void set_vehicle_attitude( float qw,  float qx,  float qy,  float qz, float droll, float dpitch, float dyaw ); 



	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt the delta-time
	 */
	void generateThrustYawSetpoint(const float dt);

	/**
	 * 	Set the integral term in xy to 0.
	 * 	@see _thr_int
	 */
	void resetIntegralXY() { _thr_int(0) = _thr_int(1) = 0.0f; }

	/**
	 * 	Set the integral term in z to 0.
	 * 	@see _thr_int
	 */
	void resetIntegralZ() { _thr_int(2) = 0.0f; }

	/**
	 * 	Get the
	 * 	@see _thr_sp
	 * 	@return The thrust set-point member.
	 */
	const matrix::Vector3f &getThrustSetpoint() { return _thr_sp; }

	/**
	 * 	Get the
	 * 	@see _yaw_sp
	 * 	@return The yaw set-point member.
	 */
	const float &getYawSetpoint() { 
		//printf("Yaw setpoint: %f\n", (double)_yaw_sp);
		return _yaw_sp; }

	/**
	 * 	Get the
	 * 	@see _yawspeed_sp
	 * 	@return The yawspeed set-point member.
	 */
	const float &getYawspeedSetpoint() { return _yawspeed_sp; }

	/**
	 * 	Get the
	 * 	@see _vel_sp
	 * 	@return The velocity set-point member.
	 */
	const matrix::Vector3f &getVelSp() { return _vel_sp; }

	/**
	 * 	Get the
	 * 	@see _pos_sp
	 * 	@return The position set-point member.
	 */
	const matrix::Vector3f &getPosSp() { return _pos_sp; }


	void geometric_tracking_control(const float dt); /**my own cartsian position control (to thrust) **/


	
protected:

	void updateParams() override;

private:
	void _interfaceMapping(); /** maps set-points to internal member set-points */
	void _SMPositionController(); /** applies the P-position-controller */
	void _velocityController(const float &dt); /** applies the PID-velocity-controller */

	void controller(float dt);

	matrix::Vector3f _pos{}; /**< MC position */
	matrix::Vector3f _vel{}; /**< MC velocity */
	matrix::Vector3f _vel_dot{}; /**< MC velocity derivative */
	matrix::Vector3f _acc{}; /**< MC acceleration */
	float _yaw{0.0f}; /**< MC yaw */
	matrix::Vector3f _pos_sp{}; /**< desired position */
	matrix::Vector3f _vel_sp{}; /**< desired velocity */
	matrix::Vector3f _acc_sp{}; /**< desired acceleration: not supported yet */
	matrix::Vector3f _thr_sp{}; /**< desired thrust */
	float _yaw_sp{}; /**< desired yaw */
	float _yawspeed_sp{}; /** desired yaw-speed */
	matrix::Vector3f _thr_int{}; /**< thrust integral term */
	vehicle_constraints_s _constraints{}; /**< variable constraints */
	bool _skip_controller{false}; /**< skips position/velocity controller. true for stabilized mode */
	
	/*
	matrix::Vector3f _e_p{};
	matrix::Vector3f _e_dp{};
	matrix::Vector3f _kp{};
	matrix::Vector3f _kv{};
	matrix::Vector3f _ki{};
	matrix::Vector3f _mu{};
	matrix::Vector3f _e_pp{};
	*/

	matrix::Vector3f _b1d_p{};
	matrix::Vector3f _b1d_1dot_p{};
	matrix::Vector3f _vel_d_p{}; 
	matrix::Vector3f _acc_d_p{};
	matrix::Vector3f _acc_p{};
	matrix::Vector3f _jerk_d_p{};


	VEHICLE_ATTITUDE _att;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_THR_MAX>) MPC_THR_MAX,
		(ParamFloat<px4::params::MPC_THR_HOVER>) MPC_THR_HOVER,
		(ParamFloat<px4::params::MPC_THR_MIN>) MPC_THR_MIN,
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) MPC_MANTHR_MIN,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) MPC_XY_VEL_MAX,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) MPC_Z_VEL_MAX_DN,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) MPC_Z_VEL_MAX_UP,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>)
		MPC_TILTMAX_AIR_rad, // maximum tilt for any position controlled mode in radians
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) MPC_MAN_TILT_MAX_rad, // maximum til for stabilized/altitude mode in radians
		(ParamFloat<px4::params::MPC_Z_P>) MPC_Z_P,
		(ParamFloat<px4::params::MPC_Z_VEL_P>) MPC_Z_VEL_P,
		(ParamFloat<px4::params::MPC_Z_VEL_I>) MPC_Z_VEL_I,
		(ParamFloat<px4::params::MPC_Z_VEL_D>) MPC_Z_VEL_D,
		(ParamFloat<px4::params::MPC_XY_P>) MPC_XY_P,
		(ParamFloat<px4::params::MPC_XY_VEL_P>) MPC_XY_VEL_P,
		(ParamFloat<px4::params::MPC_XY_VEL_I>) MPC_XY_VEL_I,
		(ParamFloat<px4::params::MPC_XY_VEL_D>) MPC_XY_VEL_D
	)
};

float dot_prod( matrix::Vector3f a, matrix::Vector3f b );
matrix::Vector3f cross_prod( matrix::Vector3f a, matrix::Vector3f b );
matrix::SquareMatrix<float, 3> point_prod( matrix::SquareMatrix<float, 3> A, matrix::SquareMatrix<float, 3> B);
matrix::Vector3f vee( matrix::SquareMatrix<float, 3>  R );
matrix::SquareMatrix<float, 3> hat( matrix::Vector3f v);
float trace(  matrix::SquareMatrix<float, 3>  R );