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

static mouse2axisPoint mouse2axis_translation_internal(double x_mouse, double y_mouse, int dead_zone,
		const mouse2axis_config* m2a_config) {
	mouse2axisPoint closest_point;
	const double x_mouse_abs = fabs(x_mouse);
	const double y_mouse_abs = fabs(y_mouse);

	double radius_ratio, xbar, ybar, diff;
	double cur_distance, lowest_distance = (x_mouse_abs + y_mouse_abs) * (x_mouse_abs + y_mouse_abs) + 1;
	int i, j;
	int begin, end;
	if (m2a_config->zero_axis_is_positive == true) {
		begin = 1;
		end = 128;
		dead_zone++;
	} else {
		//TODO: set values correctly in case of negative values of movements (x_mouse<0 or y_mouse<0).
		begin = 0;
		end = 127;
//		begin_i = 0;
//		begin_j = 0;
//		end_i = 127;
//		end_j = 127;
		closest_point.x = 0;
		closest_point.y = 0;
		closest_point.mouse_x = 0;
		closest_point.mouse_y = 0;
	}
	for (i = begin; i <= end; i++) {
		if (i == 0) {
			j = dead_zone + 1;
		} else {
			j = i;
		}
		for (; j <= end; j++) {
			radius_ratio = 1 - dead_zone / sqrt(i * i + j * j);
			if (radius_ratio <= 0) {
				continue;
			}
			xbar = radius_ratio * i;
			ybar = radius_ratio * j;
			diff = xbar - x_mouse_abs;
			cur_distance = diff * diff;
			diff = ybar - y_mouse_abs;
			cur_distance += diff * diff;
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
		if (m2a_config->zero_axis_is_positive == false && closest_point.x < -64) {
			closest_point.x--;
		}
	} else {
		if (m2a_config->zero_axis_is_positive == true) {
			closest_point.x--;
		}
	}
	if (y_mouse < 0) {
		closest_point.y *= -1;
		closest_point.mouse_y *= -1;
		if (m2a_config->zero_axis_is_positive == false && closest_point.y < -64) {
			closest_point.y--;
		}
	} else {
		if (m2a_config->zero_axis_is_positive == true) {
			closest_point.y--;
		}
	}

	return closest_point;
}

static mouse2axisPoint mouse2axis_translation(double x_mouse, double y_mouse, int dead_zone,
		const mouse2axis_config* m2a_config) {
	int tmp1;
	double tmp2;
	mouse2axisPoint closest_point;
	boolean swap = false;

	if (fabs(x_mouse) > fabs(y_mouse)) {
		//		closest_point = mouse2axis_translation_internal(y_mouse, x_mouse, dead_zone, m2a_config);
		//		return closest_point;
		swap = true;
		tmp2 = x_mouse;
		x_mouse = y_mouse;
		y_mouse = tmp2;
	}
	closest_point = mouse2axis_translation_internal(x_mouse, y_mouse, dead_zone, m2a_config);
	if (swap == true) {
		tmp1 = closest_point.x;
		tmp2 = closest_point.mouse_x;
		closest_point.mouse_x = closest_point.mouse_y;
		closest_point.mouse_y = tmp2;
		closest_point.x = closest_point.y;
		closest_point.y = tmp1;
	}
	return closest_point;
}

boolean MOUSE2AXIS_DEBUG = false;

double adv_mouse2axis(s_adapter* controller, int which, double x, double y, s_axis_props* axis_props, double multiplier,
		int dead_zone, const mouse2axis_config* m2a_config) {
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

	if (MOUSE2AXIS_DEBUG == true) {
		clock_t start, end;
		double cpu_time_used;
		start = clock();
		p = mouse2axis_translation(x, y, dead_zone, m2a_config);
		end = clock();
		cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC * 1000;
		printf("mouse2axis_translation time: %.1fms\n", cpu_time_used);
	} else {
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
