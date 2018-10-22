/*
 * mainloop.h
 *
 *  Created on: 4 déc. 2012
 *      Author: matlo
 */

#ifndef MAINLOOP_H_
#define MAINLOOP_H_

#include <gimx.h>

void set_done();
int get_done();
e_gimx_status mainloop();

/**
 * In microseconds.
 */
unsigned int getPoolTime();

#endif /* MAINLOOP_H_ */
