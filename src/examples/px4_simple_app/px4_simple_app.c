/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Spraying Simulator Debug App
 *
 * @author Y. Caglar <yilmaz.caglar@tubitak.gov.tr>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <signal.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/debug_key_value.h>

static volatile bool should_exit = false;

void signal_handler(int signum)
{
    should_exit = true;
}

__EXPORT int px4_simple_app_main(int b, char *c[]);

static float d(float e, float f);

int px4_simple_app_main(int b, char *c[])
{
    PX4_INFO("Init...");

    // Register signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int g = orb_subscribe_multi(ORB_ID(actuator_outputs), 1);
    orb_set_interval(g, 100);

    struct debug_key_value_s h;
    memset(&h, 0, sizeof(h));
    orb_advert_t i = orb_advertise(ORB_ID(debug_key_value), &h);

    float j = 10000.0f;
    float k = 0.1f;
    int l = 0;

    while (j > 0.0f && !should_exit) {
        usleep(100000);

        struct actuator_outputs_s m;
        if (orb_copy(ORB_ID(actuator_outputs), g, &m) == PX4_OK) {
		if (orb_copy(ORB_ID(actuator_outputs), g, &m) == PX4_OK) {
    			// PX4_INFO("Data received.");
		} else {
    			// PX4_WARN("No new data available.");
		}

            float n = 2*m.output[4];
            float o = 2*m.output[5];

            float p = (n >= 0.0f) ? d(n, k) : 0.0f;
            float q = (o >= 0.0f) ? d(o, k) : 0.0f;

            float r = p + q;
            float s = r * k;

            j = fmaxf(j - s, 0.0f);

            if (l == 0) {
                h.value = j;
                memcpy(&h.key, "WH", 2);
            } else {
                h.value = r;
                memcpy(&h.key, "FM", 2);
            }

            // PX4_INFO("Publishing key: %s, value: %f", h.key, (double)h.value);
	    orb_publish(ORB_ID(debug_key_value), i, &h);
	    // PX4_INFO("Published successfully.");
            l = 1 - l;
        }
    }
    PX4_INFO("Volume reached 0. Exiting.");

    // Cleanup
    orb_unsubscribe(g);
    orb_unadvertise(i);

    PX4_INFO("Exit.");
    return 0;
}

static float d(float e, float f)
{
    float g = 0.0f;

    if (e <= 1000.0f) {
        g = 0.0f;
    } else if (e > 1000.0f && e <= 1200.0f) {
        g = 30.0f * ((e - 1000.0f) / 200.0f);
    } else if (e > 1200.0f && e <= 1400.0f) {
        g = 51.5f + (30.0f - 51.5f) * ((1400.0f - e) / 200.0f);
    } else if (e > 1400.0f && e <= 1850.0f) {
        float h = -2.78e-6f;
        float i = 0.0917f;
        float j = -86.41f;
        g = h * e * e + i * e + j;
    } else if (e > 1850.0f && e <= 2000.0f) {
        g = 67.212f + (80.0f - 67.212f) * ((e - 1850.0f) / 150.0f);
    }

    return g;
}
