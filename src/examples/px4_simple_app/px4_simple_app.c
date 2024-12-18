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

#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/debug_key_value.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

static float s(float a, float b);

int px4_simple_app_main(int argc, char *argv[])
{
    PX4_INFO("Init...");

    int x = orb_subscribe(ORB_ID(actuator_outputs));
    orb_set_interval(x, 100);

    struct debug_key_value_s y;
    memset(&y, 0, sizeof(y));
    orb_advert_t z = orb_advertise(ORB_ID(debug_key_value), &y);

    float a = 10000.0f;
    float b = 0.1f;
    int c = 0;

    while (true) {
        usleep(100000);

        struct actuator_outputs_s d;
        if (orb_copy(ORB_ID(actuator_outputs), x, &d) == PX4_OK) {
            float e = d.output[4];
            float f = d.output[5];

            float g = (e >= 0.0f) ? s(e, b) : 0.0f;
            float h = (f >= 0.0f) ? s(f, b) : 0.0f;

            float i = g + h;
            float j = i * b;

            a = fmaxf(a - j, 0.0f);

            if (c == 0) {
                y.value = a;
                memcpy(&y.key, "WH", 2);
            } else {
                y.value = i;
                memcpy(&y.key, "FM", 2);
            }

            orb_publish(ORB_ID(debug_key_value), z, &y);
            c = 1 - c;
        }
    }

    PX4_INFO("Exit.");
    return 0;
}

static float s(float a, float b)
{
    float c = 0.0f;

    if (a < 0.4f) {
        c = 51.5f * (a / 0.4f);
    } else if (a > 0.85f) {
        c = 67.212f + (12.788f * ((a - 0.85f) / 0.15f));
    } else {
        c = -0.56295f * a * a + 37.7622f * a + 36.5212f;
    }

    return c;
}
