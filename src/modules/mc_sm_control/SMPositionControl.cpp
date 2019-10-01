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
 * @file SMPositionControl.cpp
 */

#include "SMPositionControl.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include "Utility/SMControlMath.hpp"

#include <iostream>

using namespace std;

using namespace matrix;

SMPositionControl::SMPositionControl(ModuleParams *parent) :
	ModuleParams(parent) {

	_b1d_p(0) = _b1d_p(1) = _b1d_p(2) = 0.0;
	_b1d_1dot_p(0) = _b1d_1dot_p(1) = _b1d_1dot_p(2) = 0.0; 
	_vel_d_p(0) = _vel_d_p(1) = _vel_d_p(2) = 0.0;  
	_acc_d_p(0) = _acc_d_p(1) = _acc_d_p(2) = 0.0; 
	_acc_p(0) = _acc_p(1) = _acc_p(2) = 0.0; 
	_jerk_d_p(0) = _jerk_d_p(1) = _jerk_d_p(2) = 0.0;




}

void SMPositionControl::updateState(const SMPositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_acc = states.acceleration;
	_vel_dot = states.acceleration;

}

void SMPositionControl::updateSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	// If full manual is required (thrust already generated), don't run position/velocity
	// controller and just return thrust.
	_skip_controller = false;

	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acc_x, setpoint.acc_y, setpoint.acc_z);

	_thr_sp = Vector3f(setpoint.thrust);
	_yaw_sp = setpoint.yaw;
	
	_yawspeed_sp = setpoint.yawspeed;
	_interfaceMapping();

	if (PX4_ISFINITE(setpoint.thrust[0]) && PX4_ISFINITE(setpoint.thrust[1]) && PX4_ISFINITE(setpoint.thrust[2])) {
		_skip_controller = true;
	}
}


void SMPositionControl::set_vehicle_attitude( float qw,  float qx,  float qy,  float qz, float droll, float dpitch, float dyaw ) {

	_att.qw = qw;
	_att.qx = qx;
	_att.qy = qy;
	_att.qz = qz;

	_att.droll = droll;
	_att.dpitch = dpitch;
	_att.dyaw = dyaw;
	
	_att.roll =  atan2(2.0*(_att.qy*_att.qz + _att.qw*_att.qx), _att.qw*_att.qw - _att.qx*_att.qx - _att.qy*_att.qy + _att.qz*_att.qz);
	_att.pitch = asin(-2.0*(_att.qx*_att.qz - _att.qw*_att.qy));
	_att.yaw = atan2(2.0*(_att.qx*_att.qy + _att.qw*_att.qz), _att.qw*_att.qw + _att.qx*_att.qx - _att.qy*_att.qy - _att.qz*_att.qz);

	//cout << "Roll: " << _att.roll << " Pitch: " << _att.pitch << " Yaw: " << _att.yaw << endl;
}

void SMPositionControl::generateThrustYawSetpoint(const float dt)
{
	if (_skip_controller) {
		// Already received a valid thrust set-point.
		// Limit the thrust vector.
		float thr_mag = _thr_sp.length();

		if (thr_mag > MPC_THR_MAX.get()) {
			_thr_sp = _thr_sp.normalized() * MPC_THR_MAX.get();

		} else if (thr_mag < MPC_MANTHR_MIN.get() && thr_mag > FLT_EPSILON) {
			_thr_sp = _thr_sp.normalized() * MPC_MANTHR_MIN.get();
		}

		// Just set the set-points equal to the current vehicle state.
		_pos_sp = _pos;
		_vel_sp = _vel;
		_acc_sp = _acc;

	} else {
		_SMPositionController();
		_velocityController(dt);
	}
}


void SMPositionControl::geometric_tracking_control(const float dt)
{
	if (_skip_controller) {
		// Already received a valid thrust set-point.
		// Limit the thrust vector.
		float thr_mag = _thr_sp.length();

		if (thr_mag > MPC_THR_MAX.get()) {
			_thr_sp = _thr_sp.normalized() * MPC_THR_MAX.get();

		} else if (thr_mag < MPC_MANTHR_MIN.get() && thr_mag > FLT_EPSILON) {
			_thr_sp = _thr_sp.normalized() * MPC_MANTHR_MIN.get();
		}

		// Just set the set-points equal to the current vehicle state.
		_pos_sp = _pos;
		_vel_sp = _vel;
		_acc_sp = _acc;

	} //Manually operated 
	else {
		controller( dt );
	}
}

float dot_prod( matrix::Vector3f a, matrix::Vector3f b ) {
	return a(0)*b(0) + a(1)*b(1) + a(2)*b(2);
}

//metodo che calcola la matrice skew
matrix::SquareMatrix<float, 3> hat( matrix::Vector3f v) {
  matrix::SquareMatrix<float, 3> S;

  S(0,0) = 0.0;
  S(0,1) =  v(2);
  S(0,2) = -v(1);

  S(1,0) = -v(2);
  S(1,1) = 0.0;
  S(1,2) = v(0);

  S(2,0) = v(1);
  S(2,1) = -v(0);
  S(2,2) = 0.0;

  return S;
}


matrix::SquareMatrix<float, 3> point_prod( matrix::SquareMatrix<float, 3> A, matrix::SquareMatrix<float, 3> B) {
	matrix::SquareMatrix<float, 3> R;
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			R(i,j) = A(0,j)*B(i,j);
		}
	}
	return R;
}

matrix::Vector3f vee( matrix::SquareMatrix<float, 3>  R ) {
	matrix::Vector3f v;
	v(0) = R(2,1);
	v(1) = R(0,2);
	v(2) = R(1,0);

	return v;
}

matrix::Vector3f cross_prod( matrix::Vector3f a, matrix::Vector3f b ) {
	matrix::Vector3f v{};

	v(0) = a(1)*b(2) - a(2)*b(1);
	v(1) = a(2)*b(0) - a(0)*b(2);
	v(2) = a(0)*b(1) - a(1)*b(0);

	return v;
}

float trace(  matrix::SquareMatrix<float, 3>  R ) {

	return R(0,0)+R(1,1)+R(2,2);

}

void SMPositionControl::controller( float dt) {

	//---State
	float UAV_MASS = 2.0;
	float SM_kx = float(4)*UAV_MASS;
	float SM_kv = float(5.6)*UAV_MASS;


	float SM_Jxx = 0.0820;
	float SM_Jyy = 0.0845;
	float SM_Jzz = 0.1377;

	float SM_kR = float(8.81);
	float SM_kOmega = float(2.54);
	

	matrix::Vector3f x_{};
	matrix::Vector3f v_{};
	matrix::Vector3f a_{};
	matrix::Vector3f j_{}; 
	
	matrix::Vector3f omega{};
	matrix::Vector3f e3{};
	matrix::Vector3f vel_d{}; 
	matrix::Vector3f acc_d{};
	matrix::Vector3f jerk_d{};
	matrix::Vector3f b1d_1dot{};
	matrix::Vector3f b1d_2dot{};



	matrix::SquareMatrix<float, 3> J = eye<float, 3>();
	J(0,0) = SM_Jxx;
	J(1,1) = SM_Jyy;
	J(2,2) = SM_Jzz;

	matrix::SquareMatrix<float, 3> R = eye<float, 3>();
	
	//Current
	x_ = _pos;
	v_ = _vel;
	a_ = _acc;
	j_ = ( a_ - _acc_p)*dt;

	// inertial frame 3-axis
	e3(0) = 0.0; e3(1) = 0.0; e3(2) = 1.0;

	//---Quat to mat
	double s = _att.qw;
  	double x = _att.qx;
  	double y = _att.qy;
  	double z = _att.qz;

  	R(0,0) = 1-2*(y*y+z*z);
  	R(0,1) = 2*(x*y-s*z);
  	R(0,2) = 2*(x*z+s*y);

  	R(1,0) = 2*(x*y+s*z);
  	R(1,1) = 1-2*(x*x+z*z);
  	R(1,2) = 2*(y*z-s*x);


  	R(2,0) = 2*(x*z-s*y);
  	R(2,1) = 2*(y*z+s*x);
  	R(2,2) = 1-2*(x*x+y*y);

	omega(0) = _att.droll;
	omega(1) = _att.dpitch;
	omega(2) = _att.dyaw;
	//---


	/*
	0.2500    0.0000    1.5873  -31.2344
    0.2500   -1.5873         0   31.2344
    0.2500    0.0000   -1.5873  -31.2344
    0.2500    1.5873         0   31.2344
	*/

	//---Velocity Setpoint
	//Velocity error
	Vector3f vel_sp_position = (_pos_sp - x_).emult(Vector3f(MPC_XY_P.get(), MPC_XY_P.get(), MPC_Z_P.get()));
	vel_d = vel_sp_position + vel_d;
	const Vector2f vel_sp_xy = SMControlMath::constrainXY(Vector2f(vel_sp_position(0), vel_sp_position(1)), Vector2f(&(_vel_sp - vel_sp_position)(0)), _constraints.speed_xy);
	vel_d(0) = vel_sp_xy(0);
	vel_d(1) = vel_sp_xy(1);
	// Constrain velocity in z-direction.
	vel_d(2) = math::constrain(vel_d(2), -_constraints.speed_up, _constraints.speed_down);	
	//---

	//---Acceleration setpoint
	acc_d = (vel_d - _vel_d_p)*dt;
	jerk_d = ( jerk_d - _jerk_d_p )*dt;
	//---

	//---b1d
	matrix::Vector3f b1d{};
	b1d(0) = cos( _yaw_sp );
	b1d(1) = sin( _yaw_sp );
	b1d(2) = 0.0;
	//---

	//b1d_1dot
	b1d_1dot = (b1d - _b1d_1dot_p)*dt;	
	b1d_2dot = ( b1d_1dot - _b1d_1dot_p)*dt;

	
	matrix::Vector3f ep = x_ - _pos_sp;
	matrix::Vector3f ev = v_ - vel_d;
	matrix::Vector3f ea = a_ - acc_d;
	matrix::Vector3f ej = j_ - jerk_d;


	float g = 9.81;
	matrix::Vector3f A = - ( -SM_kx*ep - SM_kv*ev - UAV_MASS*g*e3 + UAV_MASS*acc_d);//(R*e3);

	matrix::Vector3f Re = R*e3;
	//float f = (f_tot(0)*Re(0) + f_tot(1)*Re(1) + f_tot(2)*Re(2) );	
	float f = dot_prod(A, Re );
	matrix::Vector3f b3c = -A / A.norm();

	matrix::Vector3f C = cross_prod( b3c, b1d );
	matrix::Vector3f b1c = -(1/C.norm() )*cross_prod( b3c, C );
	matrix::Vector3f b2c = C / C.norm();

	matrix::SquareMatrix<float, 3> Rc = eye<float, 3>();

	Rc(0,0) = b1c(0);
	Rc(1,0) = b1c(1);
	Rc(2,0) = b1c(2);

	Rc(0,1) = b2c(0);
	Rc(1,1) = b2c(1);
	Rc(2,1) = b2c(2);

	Rc(0,2) = b3c(0);
	Rc(1,2) = b3c(1);
	Rc(2,2) = b3c(2);


	//First time derivative dynamics
	matrix::Vector3f A_1dot = SM_kx*ev - SM_kv*ea + UAV_MASS*acc_d;
	matrix::Vector3f b3c_1dot = (-A_1dot / A.norm()) + (dot_prod(A, A_1dot) / (A.norm()*A.norm()*A.norm()));
	matrix::Vector3f C_1dot = cross_prod( b3c_1dot, b1d ) + cross_prod( b3c, b1d_1dot );
	matrix::Vector3f b2c_1dot = C / C.norm() - ( dot_prod( C, C_1dot) / (C.norm()*C.norm()*C.norm()))*C;
	matrix::Vector3f b1c_1dot = cross_prod( b2c_1dot, b3c) + cross_prod( b2c, b3c_1dot);

	//Second time derivative dynamics
	matrix::Vector3f A_2dot = -SM_kx*ea - SM_kv*ej + UAV_MASS*jerk_d;
	matrix::Vector3f b3c_2dot = -A_2dot / A.norm() + ( 2 / A.norm()*A.norm()*A.norm())*A_1dot + 
	( ( A_1dot.norm()*A_1dot.norm() ) + dot_prod( A, A_2dot)) / (A.norm()*A.norm()*A.norm())*A -
	( 3 / (A.norm()*A.norm()*A.norm()*A.norm()*A.norm())) * (  dot_prod(A,A_2dot)  )*A;

	matrix::Vector3f b3c_1dot_b1d_1dot = cross_prod( b3c_1dot, b1d_1dot);
	b3c_1dot_b1d_1dot(0) = 2*b3c_1dot_b1d_1dot(0);
	b3c_1dot_b1d_1dot(1) = 2*b3c_1dot_b1d_1dot(1);
	b3c_1dot_b1d_1dot(2) = 2*b3c_1dot_b1d_1dot(2);
	matrix::Vector3f C_2dot = cross_prod( b3c_2dot, b1d) + cross_prod( b3c, b1d_2dot) + b3c_1dot_b1d_1dot;

	matrix::Vector3f b2c_2dot = C_2dot/C.norm() - ( 2 / (C.norm()*C.norm()*C.norm())) * dot_prod( C, C_1dot)*C_1dot -
	( ( (C_2dot.norm()*C_2dot.norm()) + dot_prod(C, C_2dot)) / (C.norm()*C.norm()*C.norm()) )*C;

	matrix::Vector3f b2c_1dot_b3c_1dot = cross_prod( b2c_1dot, b3c_1dot );
	b2c_1dot_b3c_1dot(0) = 2*b2c_1dot_b3c_1dot(0);
	b2c_1dot_b3c_1dot(1) = 2*b2c_1dot_b3c_1dot(1);
	b2c_1dot_b3c_1dot(2) = 2*b2c_1dot_b3c_1dot(2);
	matrix::Vector3f b1c_2dot = cross_prod( b2c_2dot, b3c) + cross_prod( b2c, b3c_2dot) + b2c_1dot_b3c_1dot; 
	

	// Extract calculated angular velocities and their time-derivatives
	matrix::SquareMatrix<float, 3> Rc_1dot;
	Rc_1dot(0,0) = b1c_1dot(0);
	Rc_1dot(1,0) = b1c_1dot(1);
	Rc_1dot(2,0) = b1c_1dot(2);

	Rc_1dot(0,1) = b2c_1dot(0);
	Rc_1dot(1,1) = b2c_1dot(1);
	Rc_1dot(2,1) = b2c_1dot(2);

	Rc_1dot(0,2) = b3c_1dot(0);
	Rc_1dot(1,2) = b3c_1dot(1);
	Rc_1dot(2,2) = b3c_1dot(2);

	matrix::SquareMatrix<float, 3> Rc_2dot;
	Rc_2dot(0,0) = b1c_2dot(0);
	Rc_2dot(1,0) = b1c_2dot(1);
	Rc_2dot(2,0) = b1c_2dot(2);

	Rc_2dot(0,1) = b2c_2dot(0);
	Rc_2dot(1,1) = b2c_2dot(1);
	Rc_2dot(2,1) = b2c_2dot(2);

	Rc_2dot(0,2) = b3c_2dot(0);
	Rc_2dot(1,2) = b3c_2dot(1);
	Rc_2dot(2,2) = b3c_2dot(2);

	matrix::Vector3f Omegac = vee( point_prod( Rc.T(), Rc_1dot ));
	matrix::Vector3f Omegac_1dot = vee( point_prod( Rc.T(), Rc_2dot) - hat(Omegac)*hat(Omegac) );


	matrix::Vector3f eR = vee( point_prod( Rc.T(), R) - point_prod (R.T(), Rc) );
	matrix::Vector3f eOmega = omega - R.T()*Rc*Omegac;

	matrix::Vector3f M = -SM_kR*eR - SM_kOmega*eOmega + cross_prod(omega, J*omega) - J*(hat(omega) * point_prod (R.T(),Rc)*Omegac  - point_prod( R, Rc)*Omegac_1dot );
	float Psi = float(0.5)*trace( eye<float, 3>() - point_prod (Rc.T(), R ) );
	cout << "Psi: " << Psi << endl;


	float tauM[4];
	tauM[0] = f;
	tauM[1] = M(0);
	tauM[2] = M(1);
	tauM[3] = M(2);

	
	matrix::SquareMatrix<float, 4> Mix;
	Mix(0,0) = 0.2500; 
	Mix(0,1) = 0.2500;
	Mix(0,2) = 0.2500;
	Mix(0,3) = 0.2500;

	Mix(1,0) = 0.0;
	Mix(1,1) = -1.5873;
	Mix(1,2) = 0.0;
	Mix(1,3) = 1.5873;

	Mix(2,0) = 1.5873;
	Mix(2,1) = 0;
	Mix(2,2) = -1.587;
	Mix(2,3) = 0.0;

	Mix(3,0) = -31.234;
	Mix(3,1) = 31.234;
	Mix(3,2) = -31.234;
	Mix(3,3) = 31.234;

	float p[4];
	p[0] = p[1] = p[2] = p[3] = 0.0;
	for ( int i = 0; i < 4; i++ ) {
		p[i] = 0;                     
		for ( int j = 0; j < 4; j++ ) {
			p[i] += Mix(i,j) * tauM[j];       // <===== Add terms to sum for ith element
		}
	}

	cout << "v: " << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << endl;
	

	_vel_d_p = vel_d;
	_b1d_p = b1d;
	_b1d_1dot_p = b1d_1dot;
	_acc_p = a_;

}




void SMPositionControl::_interfaceMapping() {

	// if noting is valid, then apply failsafe landing
	bool failsafe = false;

	// Respects FlightTask interface, where NAN-set-points are of no interest
	// and do not require control. A valide position and velocity setpoint will
	// be mapped to a desired position setpoint with a feed-forward term.
	for (int i = 0; i <= 2; i++) {

		if (PX4_ISFINITE(_pos_sp(i))) {
			// Position control is required

			if (!PX4_ISFINITE(_vel_sp(i))) {
				// Velocity is not used as feedforward term.
				_vel_sp(i) = 0.0f;
			}

			// thrust setpoint is not supported in position control
			_thr_sp(i) = 0.0f;

			// to run position control, we require valid position and velocity
			if (!PX4_ISFINITE(_pos(i)) || !PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} 
		else if (PX4_ISFINITE(_vel_sp(i))) {

			// Velocity controller is active without position control.
			// Set the desired position set-point equal to current position
			// such that error is zero.
			if (PX4_ISFINITE(_pos(i))) {
				_pos_sp(i) = _pos(i);

			} else {
				_pos_sp(i) = _pos(i) = 0.0f;
			}

			// thrust setpoint is not supported in position control
			_thr_sp(i) = 0.0f;

			// to run velocity control, we require valid velocity
			if (!PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_thr_sp(i))) {

			// Thrust setpoint was generated from sticks directly.
			// Set all other set-points equal MC states.
			if (PX4_ISFINITE(_pos(i))) {
				_pos_sp(i) = _pos(i);

			} else {
				_pos_sp(i) = _pos(i) = 0.0f;
			}

			if (PX4_ISFINITE(_vel(i))) {
				_vel_sp(i) = _vel(i);

			} else {
				_vel_sp(i) = _vel(i) = 0.0f;
			}

			// Reset the Integral term.
			_thr_int(i) = 0.0f;
			// Don't require velocity derivative.
			_vel_dot(i) = 0.0f;

		} else {
			// nothing is valid. do failsafe
			failsafe = true;
		}
	}

	// ensure that vel_dot is finite, otherwise set to 0
	if (!PX4_ISFINITE(_vel_dot(0)) || !PX4_ISFINITE(_vel_dot(1))) {
		_vel_dot(0) = _vel_dot(1) = 0.0f;
	}

	if (!PX4_ISFINITE(_vel_dot(2))) {
		_vel_dot(2) = 0.0f;
	}

	if (!PX4_ISFINITE(_yawspeed_sp)) {
		// Set the yawspeed to 0 since not used.
		_yawspeed_sp = 0.0f;
	}

	if (!PX4_ISFINITE(_yaw_sp)) {
		// Set the yaw-sp equal the current yaw.
		// That is the best we can do and it also
		// agrees with FlightTask-interface definition.
		if (PX4_ISFINITE(_yaw)) {
			_yaw_sp = _yaw;
		} else {
			failsafe = true;
		}
	}

	// check failsafe
	if (failsafe) {
		_skip_controller = true;

		// point the thrust upwards
		_thr_sp(0) = _thr_sp(1) = 0.0f;
		// throttle down such that vehicle goes down with
		// 70% of throttle range between min and hover
		_thr_sp(2) = -(MPC_THR_MIN.get() + (MPC_THR_HOVER.get() - MPC_THR_MIN.get()) * 0.7f);
	}
}

void SMPositionControl::_SMPositionController() {
	// P-position controller
	const Vector3f vel_sp_position = (_pos_sp - _pos).emult(Vector3f(MPC_XY_P.get(), MPC_XY_P.get(), MPC_Z_P.get()));
	_vel_sp = vel_sp_position + _vel_sp;

	//cout << "vel_sp_position: " << vel_sp_position(0) << " "<< vel_sp_position(1) << " "<< vel_sp_position(2) << endl;
	
	//cout << "_constraints.speed_xy: " << _constraints.speed_xy << endl;
	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	const Vector2f vel_sp_xy = SMControlMath::constrainXY(Vector2f(vel_sp_position(0), vel_sp_position(1)),
				   Vector2f(&(_vel_sp - vel_sp_position)(0)), _constraints.speed_xy);
	_vel_sp(0) = vel_sp_xy(0);
	_vel_sp(1) = vel_sp_xy(1);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_constraints.speed_up, _constraints.speed_down);	


}

void SMPositionControl::_velocityController(const float &dt)
{

	// Generate desired thrust setpoint.
	// PID
	// u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)
	// Umin <= u_des <= Umax
	//
	// Anti-Windup:
	// u_des = _thr_sp; r = _vel_sp; y = _vel
	// u_des >= Umax and r - y >= 0 => Saturation = true
	// u_des >= Umax and r - y <= 0 => Saturation = false
	// u_des <= Umin and r - y <= 0 => Saturation = true
	// u_des <= Umin and r - y >= 0 => Saturation = false
	//
	// 	Notes:
	// - PID implementation is in NED-frame
	// - control output in D-direction has priority over NE-direction
	// - the equilibrium point for the PID is at hover-thrust
	// - the maximum tilt cannot exceed 90 degrees. This means that it is
	// 	 not possible to have a desired thrust direction pointing in the positive
	// 	 D-direction (= downward)
	// - the desired thrust in D-direction is limited by the thrust limits
	// - the desired thrust in NE-direction is limited by the thrust excess after
	// 	 consideration of the desired thrust in D-direction. In addition, the thrust in
	// 	 NE-direction si also limited by the maximum tilt.

	const Vector3f vel_err = _vel_sp - _vel;


	
	// Consider thrust in D-direction.
	float thrust_desired_D = MPC_Z_VEL_P.get() * vel_err(2) +  MPC_Z_VEL_D.get() * _vel_dot(2) + _thr_int(
					 2) - MPC_THR_HOVER.get();
	

	// The Thrust limits are negated and swapped due to NED-frame.
	float uMax = -MPC_THR_MIN.get();
	float uMin = -MPC_THR_MAX.get();

	// Apply Anti-Windup in D-direction.
	bool stop_integral_D = (thrust_desired_D >= uMax && vel_err(2) >= 0.0f) ||
			       (thrust_desired_D <= uMin && vel_err(2) <= 0.0f);

	if (!stop_integral_D) {
		_thr_int(2) += vel_err(2) * MPC_Z_VEL_I.get() * dt;

		// limit thrust integral
		_thr_int(2) = math::min(fabsf(_thr_int(2)), MPC_THR_MAX.get()) * math::sign(_thr_int(2));
	}
	else {
		cout << "stop: " << _thr_int(2) << endl;	
	}
	// Saturate thrust setpoint in D-direction.
	_thr_sp(2) = math::constrain(thrust_desired_D, uMin, uMax);

	if (fabsf(_thr_sp(0)) + fabsf(_thr_sp(1))  > FLT_EPSILON) {
		// Thrust set-point in NE-direction is already provided. Only
		// scaling by the maximum tilt is required.
		float thr_xy_max = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		_thr_sp(0) *= thr_xy_max;
		_thr_sp(1) *= thr_xy_max;

	} else {
		// PID-velocity controller for NE-direction.
		Vector2f thrust_desired_NE;
		thrust_desired_NE(0) = MPC_XY_VEL_P.get() * vel_err(0) + MPC_XY_VEL_D.get() * _vel_dot(0) + _thr_int(0);
		thrust_desired_NE(1) = MPC_XY_VEL_P.get() * vel_err(1) + MPC_XY_VEL_D.get() * _vel_dot(1) + _thr_int(1);

		// Get maximum allowed thrust in NE based on tilt and excess thrust.
		float thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		float thrust_max_NE = sqrtf(MPC_THR_MAX.get() * MPC_THR_MAX.get() - _thr_sp(2) * _thr_sp(2));
		thrust_max_NE = math::min(thrust_max_NE_tilt, thrust_max_NE);

		// Saturate thrust in NE-direction.
		_thr_sp(0) = thrust_desired_NE(0);
		_thr_sp(1) = thrust_desired_NE(1);

		if (thrust_desired_NE * thrust_desired_NE > thrust_max_NE * thrust_max_NE) {
			float mag = thrust_desired_NE.length();
			_thr_sp(0) = thrust_desired_NE(0) / mag * thrust_max_NE;
			_thr_sp(1) = thrust_desired_NE(1) / mag * thrust_max_NE;
		}


		// Get the direction of (r-y) in NE-direction.
		float direction_NE = Vector2f(vel_err(0), vel_err(1)) * Vector2f(_vel_sp(0), _vel_sp(1));

		// Apply Anti-Windup in NE-direction.
		bool stop_integral_NE = (thrust_desired_NE * thrust_desired_NE >= thrust_max_NE * thrust_max_NE &&
					 direction_NE >= 0.0f);

		if (!stop_integral_NE) {
			_thr_int(0) += vel_err(0) * MPC_XY_VEL_I.get() * dt;
			_thr_int(1) += vel_err(1) * MPC_XY_VEL_I.get() * dt;

			// magnitude of thrust integral can never exceed maximum throttle in NE
			float integral_mag_NE = Vector2f(&_thr_int(0)).length();

			if (integral_mag_NE > 0.0f && integral_mag_NE > thrust_max_NE) {
				_thr_int(0) = _thr_int(0) / integral_mag_NE * thrust_max_NE;
				_thr_int(1) = _thr_int(1) / integral_mag_NE * thrust_max_NE;
			}
		}
	}
}

void SMPositionControl::updateConstraints(const vehicle_constraints_s &constraints)
{
	_constraints = constraints;

	// For safety check if adjustable constraints are below global constraints. If they are not stricter than global
	// constraints, then just use global constraints for the limits.

	if (!PX4_ISFINITE(constraints.tilt)
	    || !(constraints.tilt < math::max(MPC_TILTMAX_AIR_rad.get(), MPC_MAN_TILT_MAX_rad.get()))) {
		_constraints.tilt = math::max(MPC_TILTMAX_AIR_rad.get(), MPC_MAN_TILT_MAX_rad.get());
	}

	if (!PX4_ISFINITE(constraints.speed_up) || !(constraints.speed_up < MPC_Z_VEL_MAX_UP.get())) {
		_constraints.speed_up = MPC_Z_VEL_MAX_UP.get();
	}

	if (!PX4_ISFINITE(constraints.speed_down) || !(constraints.speed_down < MPC_Z_VEL_MAX_DN.get())) {
		_constraints.speed_down = MPC_Z_VEL_MAX_DN.get();
	}

	if (!PX4_ISFINITE(constraints.speed_xy) || !(constraints.speed_xy < MPC_XY_VEL_MAX.get())) {
		_constraints.speed_xy = MPC_XY_VEL_MAX.get();
	}
}

void SMPositionControl::updateParams()
{
	ModuleParams::updateParams();

	// Tilt needs to be in radians
	MPC_TILTMAX_AIR_rad.set(math::radians(MPC_TILTMAX_AIR_rad.get()));
	MPC_MAN_TILT_MAX_rad.set(math::radians(MPC_MAN_TILT_MAX_rad.get()));
}
