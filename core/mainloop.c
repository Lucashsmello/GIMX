/*
 Copyright (c) 2016 Mathieu Laurendeau <mat.lau@laposte.net>
 License: GPLv3
 */

#include <ginput.h>
#include <gpoll.h>
#include <gtimer.h>
#include <gusb.h>
#include "gimx.h"
#include "calibration.h"
#include "macros.h"
#include <stdio.h>
#include <adapter.h>
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

static int timer_read(int user __attribute__((unused)))
{
  return 1;
}

static int timer_close(int user __attribute__((unused)))
{
  set_done();
  return 1;
}

void mainloop()
{
  GE_Event events[EVENT_BUFFER_SIZE];
  int num_evt;
  GE_Event* event;
  unsigned int running_macros;
  int timer = -1;

  if(!adapter_get(0)->bdaddr_dst || adapter_get(0)->ctype == C_TYPE_DS4)
  {
    GTIMER_CALLBACKS callbacks = {
            .fp_read = timer_read,
            .fp_close = timer_close,
            .fp_register = REGISTER_FUNCTION,
            .fp_remove = REMOVE_FUNCTION,
    };
    timer = gtimer_start(0, (unsigned int)gimx_params.refresh_period, &callbacks);
    if (timer < 0)
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

    cfg_config_activation();

    if(adapter_send() < 0)
    {
      done = 1;
    }
    clock_gettime(CLOCK_REALTIME, &TTT);

    cfg_process_rumble();
    
    usb_poll_interrupts();

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
      printf("buffer too small!!!\n");
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

  if (timer >= 0)
  {
    gtimer_close(timer);
  }
}
