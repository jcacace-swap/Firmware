/****************************************************************************
 *
 *   Copyright (c) 2013 - 2017 PX4 Development Team. All rights reserved.
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
 * @file mc_sm_control_main.cpp
 * Multicopter position controller.
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that is split to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module_params.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <systemlib/hysteresis/hysteresis.h>
#include <commander/px4_custom_mode.h>

#include <uORB/topics/home_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>

#include <float.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

#include <controllib/blocks.hpp>

#include <iostream>

using namespace std;

/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int sm_ctrl_test_main(int argc, char *argv[]);

class SmCtrlTest : public control::SuperBlock, public ModuleParams
{
public:
	/**
	 * Constructor
	 */
	SmCtrlTest();

	/**
	 * Destructor, also kills task.
	 */
	~SmCtrlTest();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool		_task_should_exit = false;			/**<true if task should exit */

	orb_advert_t	_att_sp_pub{nullptr};			/**< attitude setpoint publication */
	orb_advert_t	_local_sm_sp_pub{nullptr};		/**< vehicle local position setpoint publication */
	orb_advert_t _traj_wp_avoidance_desired_pub{nullptr}; /**< trajectory waypoint desired publication */
	orb_advert_t _pub_vehicle_command{nullptr};           /**< vehicle command publication */
	orb_id_t _attitude_setpoint_id{nullptr};
	orb_advert_t	_outputs_pub{nullptr};

	int		_control_task{-1};			/**< task handle for task */
	int		_vehicle_status_sub{-1};		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub{-1};	/**< vehicle land detected subscription */
	int		_control_mode_sub{-1};		/**< vehicle control mode subscription */
	int		_params_sub{-1};			/**< notification of parameter updates */
	int		_local_sm_sub{-1};			/**< vehicle local position */
	int		_home_sm_sub{-1}; 			/**< home position */
	int		_traj_wp_avoidance_sub{-1};	/**< trajectory waypoint */
		
	actuator_outputs_s _actuator_outputs = {};
	vehicle_status_s 			_vehicle_status{};		/**< vehicle status */
	vehicle_land_detected_s 		_vehicle_land_detected{};	/**< vehicle land detected */
	vehicle_attitude_setpoint_s		_att_sp{};			/**< vehicle attitude setpoint */
	vehicle_control_mode_s			_control_mode{};		/**< vehicle control mode */
	vehicle_local_position_s		_local_pos{};			/**< vehicle local position */
	home_position_s				_home_pos{};			/**< home position */
	vehicle_trajectory_waypoint_s		_traj_wp_avoidance{};		/**< trajectory waypoint */
	vehicle_trajectory_waypoint_s		_traj_wp_avoidance_desired{};	/**< desired waypoints, inputs to an obstacle avoidance module */

	hrt_abstime _last_warn = 0; /**< timer when the last warn message was sent out */

	/** Timeout in us for trajectory data to get considered invalid */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500000;
	/**< number of tries before switching to a failsafe flight task */
	static constexpr int NUM_FAILURE_TRIES = 10;
	/**< If Flighttask fails, keep 0.2 seconds the current setpoint before going into failsafe land */
	static constexpr uint64_t LOITER_TIME_BEFORE_DESCEND = 200000;

	static int	task_main_trampoline(int argc, char *argv[]);
	
	void		poll_subscriptions();
	void		task_main();
};

namespace sm_test
{
SmCtrlTest	*g_control;
}



SmCtrlTest::SmCtrlTest() :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr)
{
	// fetch initial parameter values
	//parameters_update(true);
	// set failsafe hysteresis
	//_failsafe_land_hysteresis.set_hysteresis_time_from(false, LOITER_TIME_BEFORE_DESCEND);
}

SmCtrlTest::~SmCtrlTest()
{
	if (_control_task != -1) {
		// task wakes up every 100ms or so at the longest
		_task_should_exit = true;

		// wait for a second for the task to quit at our request
		unsigned i = 0;

		do {
			// wait 20ms
			usleep(20000);

			// if we have given up, kill it
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	sm_test::g_control = nullptr;
}


int
SmCtrlTest::task_main_trampoline(int argc, char *argv[]) {
	sm_test::g_control->task_main();
	return 0;
}



void
SmCtrlTest::poll_subscriptions()
{
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		// set correct uORB ID, depending on if vehicle is VTOL or not
		if (!_attitude_setpoint_id) {
			if (_vehicle_status.is_vtol) {
				_attitude_setpoint_id = ORB_ID(mc_virtual_attitude_setpoint);

			} else {
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
		

		
	}

	orb_check(_local_sm_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_sm_sub, &_local_pos);
	}

	orb_check(_home_sm_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(home_position), _home_sm_sub, &_home_pos);
	}

	orb_check(_traj_wp_avoidance_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_trajectory_waypoint), _traj_wp_avoidance_sub, &_traj_wp_avoidance);
	}
}

void
SmCtrlTest::task_main() {

	
	// do subscriptions
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_local_sm_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_home_sm_sub = orb_subscribe(ORB_ID(home_position));

	/*
	parameters_update(true);
	*/
	// get an initial update for all sensor and status data
	poll_subscriptions();

	// We really need to know from the beginning if we're landed or in-air.
	orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);

	hrt_abstime t_prev = 0;

	/*
	// Let's be safe and have the landing gear down by default
	_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN;
	*/

	// wakeup source
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _local_sm_sub;
	fds[0].events = POLLIN;
	


	/* advertise the mixed control outputs, insist on the first group output */
	_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_actuator_outputs);
	_actuator_outputs.output[0] = _actuator_outputs.output[1] = _actuator_outputs.output[2] = _actuator_outputs.output[3] = 1000;   


	int m = 0;
	while ( m < 4 ) {

		if ( m==0 ) {
			_actuator_outputs.output[1] = _actuator_outputs.output[2] = _actuator_outputs.output[3] = 1000; 
		}
		else if ( m == 1 ) {
			_actuator_outputs.output[0] = _actuator_outputs.output[2] = _actuator_outputs.output[3] = 1000; 
		}
		else if( m == 2 ) {
			_actuator_outputs.output[0] = _actuator_outputs.output[1] = _actuator_outputs.output[3] = 1000; 

		}
		else if ( m == 3 ) {

			_actuator_outputs.output[0] = _actuator_outputs.output[1] = _actuator_outputs.output[2] = 1000;   

		}
		
		// wait for up to 20ms for data
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

		// timed out - periodic check for _task_should_exit
		if (pret == 0) {
			// Go through the loop anyway to copy manual input at 50 Hz.
		}

		// this is undesirable but not much we can do
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		poll_subscriptions();
		/*
		parameters_update(false);
		*/
		hrt_abstime t = hrt_absolute_time();
		const float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.004f;
		t_prev = t;

		// set dt for control blocks
		setDt(dt);

		_actuator_outputs.noutputs = 4;
		for (size_t i = 0; i < sizeof(_actuator_outputs.output) / sizeof(_actuator_outputs.output[0]); i++) {
			if (i >= 4) {
					_actuator_outputs.output[i] = NAN;
			}
		}


		

		if (_control_mode.flag_armed) {
			_actuator_outputs.output[m] += 1;
			_actuator_outputs.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_actuator_outputs);





			if ( double(_actuator_outputs.output[m]) > 1700.0 ) m++;







		}
		/*
		if (_control_mode.flag_armed) {
			// as soon vehicle is armed check for flighttask
			start_flight_task();
			// arm hysteresis prevents vehicle to takeoff
			// before propeller reached idle speed.
			_arm_hysteresis.set_state_and_update(true);

		} else {
			// disable flighttask
			_flight_tasks.switchTask(FlightTaskIndex::None);
			// reset arm hysteresis
			_arm_hysteresis.set_state_and_update(false);
		}

		// check if any task is active
		if (_flight_tasks.isAnyTaskActive()) {

			// setpoints from flighttask
			vehicle_local_position_setpoint_s setpoint;

			// update task
			if (!_flight_tasks.update()) {
				// FAILSAFE
				// Task was not able to update correctly. Do Failsafe.
				failsafe(setpoint, _states, false);

			} else {
				setpoint = _flight_tasks.getPositionSetpoint();
				_failsafe_land_hysteresis.set_state_and_update(false);

				// Check if position, velocity or thrust pairs are valid -> trigger failsaife if no pair is valid
				if (!(PX4_ISFINITE(setpoint.x) && PX4_ISFINITE(setpoint.y)) &&
				    !(PX4_ISFINITE(setpoint.vx) && PX4_ISFINITE(setpoint.vy)) &&
				    !(PX4_ISFINITE(setpoint.thrust[0]) && PX4_ISFINITE(setpoint.thrust[1]))) {
					failsafe(setpoint, _states, true);
				}

				// Check if altitude, climbrate or thrust in D-direction are valid -> trigger failsafe if none
				// of these setpoints are valid
				if (!PX4_ISFINITE(setpoint.z) && !PX4_ISFINITE(setpoint.vz) && !PX4_ISFINITE(setpoint.thrust[2])) {
					failsafe(setpoint, _states, true);
				}
			}

			update_avoidance_waypoint_desired(_states, setpoint);

			vehicle_constraints_s constraints = _flight_tasks.getConstraints();

			// check if all local states are valid and map accordingly
			set_vehicle_states(setpoint.vz);

			// we can only do a smooth takeoff if a valid velocity or position is available and are
			// armed long enough
			if (_arm_hysteresis.get_state() && PX4_ISFINITE(_states.position(2)) && PX4_ISFINITE(_states.velocity(2))) {
				check_for_smooth_takeoff(setpoint.z, setpoint.vz, constraints);
				update_smooth_takeoff(setpoint.z, setpoint.vz);
			}

			if (_in_smooth_takeoff) {
				// during smooth takeoff, constrain speed to takeoff speed
				constraints.speed_up = _takeoff_speed;
				// disable yaw command
				setpoint.yaw = setpoint.yawspeed = NAN;
				// don't control position in xy
				setpoint.x = setpoint.y = NAN;
				setpoint.vx = setpoint.vy = 0.0f;
			}

			if (_vehicle_land_detected.landed && !_in_smooth_takeoff && !PX4_ISFINITE(setpoint.thrust[2])) {
				// Keep throttle low when landed and NOT in smooth takeoff
				setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = 0.0f;
				setpoint.x = setpoint.y = setpoint.z = NAN;
				setpoint.vx = setpoint.vy = setpoint.vz = NAN;
				setpoint.yawspeed = NAN;
				setpoint.yaw = _states.yaw;
				constraints.landing_gear = vehicle_constraints_s::GEAR_KEEP;
			}

			// limit altitude only if local position is valid
			if (PX4_ISFINITE(_states.position(2))) {
				limit_altitude(setpoint);
			}

			// Update states, setpoints and constraints.
			_control.updateConstraints(constraints);
			_control.updateState(_states);

			if (!use_obstacle_avoidance()) {
				_control.updateSetpoint(setpoint);

			} else {
				execute_avoidance_waypoint();
			}

			// Generate desired thrust and yaw.
			_control.generateThrustYawSetpoint(_dt);

			matrix::Vector3f thr_sp = _control.getThrustSetpoint();

			// Adjust thrust setpoint based on landdetector only if the
			// vehicle is NOT in pure Manual mode and NOT in smooth takeoff
			if (!_in_smooth_takeoff && !PX4_ISFINITE(setpoint.thrust[2])) {
				limit_thrust_during_landing(thr_sp);
			}

			// Fill local position, velocity and thrust setpoint.
			vehicle_local_position_setpoint_s local_sm_sp{};
			local_sm_sp.timestamp = hrt_absolute_time();
			local_sm_sp.x = _control.getPosSp()(0);
			local_sm_sp.y = _control.getPosSp()(1);
			local_sm_sp.z = _control.getPosSp()(2);
			local_sm_sp.yaw = _control.getYawSetpoint();
			local_sm_sp.yawspeed = _control.getYawspeedSetpoint();

			local_sm_sp.vx = _control.getVelSp()(0);
			local_sm_sp.vy = _control.getVelSp()(1);
			local_sm_sp.vz = _control.getVelSp()(2);
			thr_sp.copyTo(local_sm_sp.thrust);


			//printf("Local position: %f %f %f %f %f\n", (double)local_sm_sp.x, (double)local_sm_sp.y, (double)local_sm_sp.z, (double)local_sm_sp.yaw, (double)local_sm_sp.yawspeed);
			//printf("Local setpoint: %f %f %f\n", (double)setpoint.x, (double)setpoint.y, (double)setpoint.z );

			// Publish local position setpoint (for logging only) and attitude setpoint (for attitude controller).
			publish_local_sm_sp(local_sm_sp);


			// Fill attitude setpoint. Attitude is computed from yaw and thrust setpoint.
			_att_sp = ControlMath::thrustToAttitude(thr_sp, _control.getYawSetpoint());
			_att_sp.yaw_sp_move_rate = _control.getYawspeedSetpoint();
			_att_sp.fw_control_yaw = false;
			_att_sp.disable_mc_yaw_control = false;
			_att_sp.apply_flaps = false;

			if (!constraints.landing_gear) {
				if (constraints.landing_gear == vehicle_constraints_s::GEAR_UP) {
					_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_UP;
				}

				if (constraints.landing_gear == vehicle_constraints_s::GEAR_DOWN) {
					_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN;
				}
			}

			// publish attitude setpoint
			// Note: this requires review. The reason for not sending
			// an attitude setpoint is because for non-flighttask modes
			// the attitude septoint should come from another source, otherwise
			// they might conflict with each other such as in offboard attitude control.
			publish_attitude();

		} else {
			// no flighttask is active: set attitude setpoint to idle
			_att_sp.roll_body = _att_sp.pitch_body = 0.0f;
			_att_sp.yaw_body = _local_pos.yaw;
			_att_sp.yaw_sp_move_rate = 0.0f;
			_att_sp.fw_control_yaw = false;
			_att_sp.disable_mc_yaw_control = false;
			_att_sp.apply_flaps = false;
			matrix::Quatf q_sp = matrix::Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);
			q_sp.copyTo(_att_sp.q_d);
			_att_sp.q_d_valid = true;
			_att_sp.thrust = 0.0f;
		}
			*/
	}
	/*
	_control_task = -1;
	*/
}

int
SmCtrlTest::start()
{
	// start the task
	_control_task = px4_task_spawn_cmd("sm_test_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_POSITION_CONTROL,
					   1900,
					   (px4_main_t)&SmCtrlTest::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


int sm_ctrl_test_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_sm_test {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (sm_test::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		sm_test::g_control = new SmCtrlTest;

		if (sm_test::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != sm_test::g_control->start()) {
			delete sm_test::g_control;
			sm_test::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (sm_test::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete sm_test::g_control;
		sm_test::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (sm_test::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
