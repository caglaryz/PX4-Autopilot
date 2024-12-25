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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "WorkItemExample.hpp"

WorkItemExample::WorkItemExample() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1),
	_actuator_outputs_sub(this, ORB_ID(actuator_outputs)) // Initialize the subscription
{
}

WorkItemExample::~WorkItemExample()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemExample::init()
{
	ScheduleOnInterval(100000); // 2000 us interval, 200 Hz rate

	return true;
}

void WorkItemExample::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	actuator_outputs_s actuator_outputs;
	if (_actuator_outputs_sub.copy(&actuator_outputs)) {
		float actuator5 = (_liquid_volume > 0.0f) ? 2 * actuator_outputs.output[4] : 0.0f;
		float actuator6 = (_liquid_volume > 0.0f) ? 2 * actuator_outputs.output[5] : 0.0f;

		float flow_rate = calculate_flow_rate(actuator5) + calculate_flow_rate(actuator6);
		float dispensed_volume = flow_rate * _timestep;
		_liquid_volume = fmaxf(_liquid_volume - dispensed_volume, 0.0f);

		debug_key_value_s debug_data{};
		debug_data.timestamp = hrt_absolute_time();
		if (_state_machine == 0) {
			debug_data.value = _liquid_volume;
			memcpy(debug_data.key, "WH", 2);
			_state_machine = 1;
		} else {
			debug_data.value = flow_rate;
			memcpy(debug_data.key, "FM", 2);
			_state_machine = 0;
		}
		_debug_key_value_pub.publish(debug_data);
	}

	perf_end(_loop_perf);
}

float WorkItemExample::calculate_flow_rate(float actuator_value)
{
	float flow_rate = 0.0f;
	if (actuator_value > 1000.0f && actuator_value <= 1200.0f) {
		flow_rate = 30.0f * ((actuator_value - 1000.0f) / 200.0f);
	} else if (actuator_value > 1200.0f && actuator_value <= 1400.0f) {
		flow_rate = 51.5f + (30.0f - 51.5f) * ((1400.0f - actuator_value) / 200.0f);
	} else if (actuator_value > 1400.0f && actuator_value <= 1850.0f) {
		float h = -2.78e-6f;
		float i = 0.0917f;
		float j = -86.41f;
		flow_rate = h * actuator_value * actuator_value + i * actuator_value + j;
	} else if (actuator_value > 1850.0f && actuator_value <= 2000.0f) {
		flow_rate = 67.212f + (80.0f - 67.212f) * ((actuator_value - 1850.0f) / 150.0f);
	}

	return flow_rate;
}

int WorkItemExample::task_spawn(int argc, char *argv[])
{
	WorkItemExample *instance = new WorkItemExample();

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

int WorkItemExample::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemExample::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemExample::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module for simulating liquid spraying and flow rate calculations.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int work_item_example_main(int argc, char *argv[])
{
	return WorkItemExample::main(argc, argv);
}
