/*
 * adv-mouse2axis.c
 *
 *  Created on: 9 de fev de 2018
 *      Author: lmello
 */

#include <adv-mouse2axis.h>
#include <ginput.h>
#include "gimx.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
     

typedef struct {
	int x;
	int y;
	double mouse_x;
	double mouse_y;
} mouse2axisPoint;

static mouse2axisPoint mouse2axis_translation(double x_mouse, double y_mouse, int dead_zone,
		const mouse2axis_config* m2a_config) {
	mouse2axisPoint closest_point;
	double radius_ratio, xbar, ybar;
	double x_mouse_abs = fabs(x_mouse);
	double y_mouse_abs = fabs(y_mouse);
	double cur_distance, lowest_distance = (x_mouse_abs + y_mouse_abs) * (x_mouse_abs + y_mouse_abs) + 1;

	for (int i = 0; i <= 127; i++) {
		for (int j = 0; j <= 127; j++) {
			radius_ratio = 1 - dead_zone / sqrt(i * i + j * j);
			if (radius_ratio <= 0) {
				continue;
			}
			xbar = radius_ratio * i;
			ybar = radius_ratio * j;
			cur_distance = (xbar - x_mouse_abs) * (xbar - x_mouse_abs)
					+ (ybar - y_mouse_abs) * (ybar - y_mouse_abs);
			if (cur_distance < lowest_distance) {
				closest_point.x = i;
				closest_point.y = j;
				closest_point.mouse_x = xbar;
				closest_point.mouse_y = ybar;
				lowest_distance = cur_distance;
			}
		}
	}
	if (x_mouse < 0) {
		closest_point.x *= -1;
		closest_point.mouse_x *= -1;
		if (m2a_config->zero_axis_is_positive == true) {
			closest_point.x--;
		}
	}
	if (y_mouse < 0) {
		closest_point.y *= -1;
		closest_point.mouse_y *= -1;
		if (m2a_config->zero_axis_is_positive == true) {
			closest_point.y--;
		}
	}

	return closest_point;
}

boolean MOUSE2AXIS_DEBUG=false;

double adv_mouse2axis(s_adapter* controller, int which, double x, double y, s_axis_props* axis_props,
		double multiplier, int dead_zone, const mouse2axis_config* m2a_config) {
	int axis = axis_props->axis;
	if ((which == AXIS_X && x == 0) || (which == AXIS_Y && y == 0)) {
		controller->axis[axis] = 0;
		return 0;
	}
	double motion_residue = 0;
	double multiplier_x = multiplier * controller_get_axis_scale(controller->ctype, AXIS_X);
	double multiplier_y = multiplier * controller_get_axis_scale(controller->ctype, AXIS_Y);
	mouse2axisPoint p;
	x *= multiplier_x;
	y *= multiplier_y;
	
     
	if(MOUSE2AXIS_DEBUG==true){
		clock_t start, end;
		double cpu_time_used;
		start = clock();
		p = mouse2axis_translation(x, y, dead_zone, m2a_config);
		end = clock();
		cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC * 1000;
		printf("mouse2axis_translation time: %dms\n",(int) cpu_time_used);
	}else{
		p = mouse2axis_translation(x, y, dead_zone, m2a_config);
	}
	switch (which) {
	case AXIS_X:
		controller->axis[axis] = p.x;
		motion_residue = (x - p.mouse_x) / multiplier_x;
		break;
	case AXIS_Y:
		controller->axis[axis] = p.y;
		motion_residue = (y - p.mouse_y) / multiplier_y;
		break;
	}

	motion_residue = clamp(-128, motion_residue, 127);

	return motion_residue;
}
