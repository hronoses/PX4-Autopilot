/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "Skyto.hpp"

static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}


Skyto::Skyto() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

Skyto::~Skyto()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool Skyto::init()
{
	// execute Run() on every sensor_accel publication
	if (!_sensor_accel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void Skyto::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	_vehicle_status_sub.update(&_vehicle_status);
	if (_start_mission){
		if (!_armed) {
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
						static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
						21196.f);  // force arm
						// 0.f);  // arm
			PX4_INFO("arming ...");
			_armed = true;

		}
		else if (_armed && !_stab_mode) {
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, STAB_MODE);
			_stab_mode = true;
		}
		else if (_stab_mode) {
			_q_sp = Quatf(1, 0, 0, 0);
			_thrust_z = 0.9f;
			publish_attitude();

		}
	}

	perf_end(_loop_perf);
}

void Skyto::publish_attitude(){
	// thrust_z in range [0, 1]
	_q_sp.copyTo(_vehicle_attitude_setpoint.q_d);
	_euler_angles_sp = Eulerf(_q_sp);
	_vehicle_attitude_setpoint.roll_body = _euler_angles_sp.phi();
	_vehicle_attitude_setpoint.pitch_body = _euler_angles_sp.theta();
	_vehicle_attitude_setpoint.yaw_body = _euler_angles_sp.psi();
	_vehicle_attitude_setpoint.thrust_body[0] = _thrust_z;
	_vehicle_attitude_setpoint.thrust_body[2] = 0;
	_vehicle_attitude_setpoint.thrust_body[1] = 0;
	_vehicle_attitude_setpoint.timestamp = hrt_absolute_time();
	_attitude_sp_pub.publish(_vehicle_attitude_setpoint);


}

int Skyto::task_spawn(int argc, char *argv[])
{
	Skyto *instance = new Skyto();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Skyto::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int Skyto::custom_command(int argc, char *argv[])
{
	auto instance = Skyto::get_instance();
	if (!strcmp(argv[0], "fly")) {
		instance->_start_mission = true;
		PX4_INFO("Starting mission");
		return 0;
	}
	return print_usage("unknown command");
}

int Skyto::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("skyto example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int skyto_main(int argc, char *argv[])
{
	return Skyto::main(argc, argv);
}
