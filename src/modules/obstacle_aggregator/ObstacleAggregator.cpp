/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file ObstacleAggregator.cpp
 * Obstacle Aggregator.
 *
 * @author Morten
 *
 */

#include "ObstacleAggregator.hpp"

ObstacleAggregator::ObstacleAggregator() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {
		_obstacles_pub.advertise();
	}

ObstacleAggregator::~ObstacleAggregator()
{
	perf_free(_loop_perf);
}

bool ObstacleAggregator::init()
{
	if (!_tof_obstacles_chunk_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

// void ObstacleAggregator::parameters_updated()
// {
// 	// TODO: do we need params?
// }

void ObstacleAggregator::Run(){
	if (should_exit()) {
		_tof_obstacles_chunk_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// // Check if parameters have changed
	// if (_parameter_update_sub.updated()) {
	// 	// clear update
	// 	parameter_update_s param_update;
	// 	_parameter_update_sub.copy(&param_update);

	// 	updateParams();
	// 	parameters_updated();
	// }

	// run on new chunks available
	tof_obstacles_chunk_s obs_chunk;
	if (_tof_obstacles_chunk_sub.update(&obs_chunk))
	{
		if (obs_chunk.chunk_id == 0)
		{
			_num_points_read = 0;
		}

		// TODO: read points
		for (size_t i = 0; i < obs_chunk.num_points_chunk; ++i)
		{
			const size_t j = i + _num_points_read;
			_obstacles.x[j] = obs_chunk.points_x[i];
			_obstacles.y[j] = obs_chunk.points_y[i];
			_obstacles.z[j] = obs_chunk.points_z[i];
		}
		_num_points_read += obs_chunk.num_points_chunk;

		// check if done
		if (_num_points_read == static_cast<uint16_t>(obs_chunk.num_points_total))
		{
			// finished reading
			_obstacles.timestamp = hrt_absolute_time();
			_obstacles.num_points = _num_points_read;
			_obstacles_pub.publish(_obstacles);
		}

		_prev_chunk_id = obs_chunk.chunk_id;
	}

	perf_end(_loop_perf);
}

int ObstacleAggregator::task_spawn(int argc, char *argv[])
{
	ObstacleAggregator *instance = new ObstacleAggregator();

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

int ObstacleAggregator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ObstacleAggregator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the obstacle aggregator. It takes obstacle chunks as inputs and
outputs an obstacle message.
)DESCR_STR");

	// TODO check if print usage name correct
	PRINT_MODULE_USAGE_NAME("obstacle_aggregator", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int obstacle_aggregator_main(int argc, char *argv[])
{
	return ObstacleAggregator::main(argc, argv);
}
