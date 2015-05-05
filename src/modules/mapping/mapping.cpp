/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file mapping.cpp
 *
 * Mapping module
 *
 * @author Hugo SERRAT <hugo.serrat@gmail.com>
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <mavlink/mavlink_log.h>


extern "C" __EXPORT int mapping_main(int argc, char *argv[]);

class Mapping
{
public:
	/**
	 * Constructor
	 */
	Mapping();

	/**
	 * Destructor
	 */
	~Mapping();

	/**
	 * Start the task.
	 *
	 * @return 	OK on success.
	 */
	int start();

private:
	bool 		_task_should_exit;
	int 		_main_task;
	int		_mavlink_fd;

	int		_command_sub;
	struct vehicle_command_s	_command;


	void task_main();

	void handle_command(struct vehicle_command_s *cmd);

	void answer_command(struct vehicle_command_s *cmd, enum VEHICLE_CMD_RESULT result);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void task_main_trampoline(int argc, char *argv[]);
};

namespace mapping
{
	Mapping *g_mapping;
}

Mapping::Mapping() :
	_task_should_exit(false),
	_main_task(-1),
	_mavlink_fd(-1),
	_command_sub(-1)
{	
}

Mapping::~Mapping()
{
	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	mapping::g_mapping = nullptr;
}

int Mapping::start()
{
	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = task_spawn_cmd("mapping",
				    SCHED_DEFAULT,
				    SCHED_PRIORITY_DEFAULT + 15,
				    1500,
				    (main_t)&Mapping::task_main_trampoline,
				    nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void Mapping::task_main()
{
	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(_mavlink_fd, "[mapping] started");

	warnx("started.");

	_command_sub = orb_subscribe(ORB_ID(vehicle_command));

	// wakeup source(s)
	struct pollfd fds[1];

	// Setup of loop
	fds[0].fd = _command_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		warnx("boucle");

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* vehicle commands updated */
		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
			handle_command(&_command);
		}


		//orb_check(vehicle_global_position_sub, &updated);
		//if (updated) {
		//	/* copy global position */
		//	orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub, &_global_pos);
		//}

		//if (_global_pos.timestamp == 0) {
		//	continue;
		//}

		const unsigned sleeptime_us = 9500;

		// run at roughly 100 Hz
		usleep(sleeptime_us);
	}


	warnx("exiting.");

	_main_task = -1;
	_exit(0);
}

void Mapping::handle_command(struct vehicle_command_s *cmd)
{
	switch (cmd->command) {
		default:
			warnx("default command break");
			break;
	}
}

void Mapping::answer_command(struct vehicle_command_s *cmd, enum VEHICLE_CMD_RESULT result)
{
	switch (result) {
		default:
			break;
	}
}

void Mapping::task_main_trampoline(int argc, char *argv[])
{
	mapping::g_mapping->task_main();
}

int mapping_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "usage: mapping {start|stop|status}");
	}

	
	if (!strcmp(argv[1], "start")) {

		if (mapping::g_mapping != nullptr) {
			errx(1, "already running");
		}

		mapping::g_mapping = new Mapping;

		if (mapping::g_mapping == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != mapping::g_mapping->start()) {
			delete mapping::g_mapping;
			mapping::g_mapping = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (mapping::g_mapping == nullptr) {
			errx(1, "not running");
		}

		delete mapping::g_mapping;
		mapping::g_mapping = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (mapping::g_mapping) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
