/*
 Copyright (c) 2017 Mathieu Laurendeau <mat.lau@laposte.net>
 License: GPLv3
 */

#include <gimxinput/include/ginput.h>
#include <gimxpoll/include/gpoll.h>
#include <gimxtimer/include/gtimer.h>
#include <gimxusb/include/gusb.h>
#include "gimx.h"
#include "calibration.h"
#include "macros.h"
#include <stdio.h>
#include <controller.h>
#include <connectors/usb_con.h>
#include <report2event/report2event.h>

#include <time.h>

static struct timespec TTT;

static int getUSEC(struct timespec T1, struct timespec T2) {
	return 1000 * 1000 * (T2.tv_sec - T1.tv_sec) + (T2.tv_nsec - T1.tv_nsec) / 1000;
}

/**
 * In microseconds.
 */
unsigned int getPoolTime() {
	struct timespec T1;
	clock_gettime(CLOCK_REALTIME, &T1);
	return getUSEC(TTT,T1);
}


static volatile int done = 0;

void set_done()
{
  done = 1;
}

int get_done()
{
  return done;
}

static int timer_read(void * user __attribute__((unused)))
{
  return 1;
}

static int timer_close(void * user __attribute__((unused)))
{
  set_done();
  return 1;
}

e_gimx_status mainloop()
{
  e_gimx_status status = E_GIMX_STATUS_SUCCESS;
  GE_Event events[EVENT_BUFFER_SIZE];
  int num_evt;
  GE_Event* event;
  unsigned int running_macros;
  struct gtimer * timer = NULL;

  if(adapter_get(0)->atype != E_ADAPTER_TYPE_BLUETOOTH || adapter_get(0)->ctype == C_TYPE_DS4)
  {
    GTIMER_CALLBACKS callbacks = {
            .fp_read = timer_read,
            .fp_close = timer_close,
            .fp_register = REGISTER_FUNCTION,
            .fp_remove = REMOVE_FUNCTION,
    };
    timer = gtimer_start(NULL, (unsigned int)gimx_params.refresh_period, &callbacks);
    if (timer == NULL)
    {
      done = 1;
    }
  }

  report2event_set_callback(process_event);

  while(!done)
  {
    /*
     * gpoll should always be executed as it drives the period.
     */
    gpoll();

    ginput_periodic_task();

    cfg_process_motion();

    cfg_profile_activation();

    if(adapter_send() < 0)
    {
      done = 1;
    }
    clock_gettime(CLOCK_REALTIME, &TTT);

    cfg_process_rumble();
    
    if (usb_poll_interrupts() < 0)
    {
      done = 1;
      status = E_GIMX_STATUS_AUTH_CONTROLLER_ERROR;
    }

    /*
     * These two functions generate events.
     */
    calibration_test();

    running_macros = macro_process();

    /*
     * This part of the loop processes events generated
     * by macros and calibration tests, and the --keygen argument.
     */

    num_evt = ginput_queue_pop(events, sizeof(events) / sizeof(events[0]));

    if (num_evt == EVENT_BUFFER_SIZE)
    {
      gwarn("buffer too small!!!\n");
    }
    
    for (event = events; event < events + num_evt; ++event)
    {
      process_event(event);
    }

    /*
     * The --keygen argument is used
     * and there are no more event or macro to process => exit.
     */
    if(!gimx_params.network_input && gimx_params.keygen && !running_macros && !num_evt)
    {
      done = 1;
    }
  }

  if (timer != NULL)
  {
    gtimer_close(timer);
  }

  return status;
}
