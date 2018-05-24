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

static double norm(double x, double y) {
	return sqrt(x * x + y * y);
}

static double unsqrtNorm(double x, double y) {
	return x * x + y * y;
}

static double dclamp(double min, double x, double max) {
	if (x < min)
		return min;
	if (x > max)
		return max;
	return x;
}

static void fixRoundingError(mouse2axisPoint* p, double x_mouse, double y_mouse, int dead_zone, double exp,
		int max_pos_axis) {
	double best_dist = x_mouse * x_mouse + y_mouse * y_mouse;
	double cur_dist;
	double r_bar, fr_bar, x_bar, y_bar;
	int best_i = 0, best_j = 0;
	int end_i = 2, end_j = 2;
	int i, j;

	if (p->x >= max_pos_axis) {
		end_i = 0;
		end_j = max_pos_axis - p->y;
	}
	if (p->y >= max_pos_axis) {
		end_i = max_pos_axis - p->x;
		end_j = 0;
	}

	p->mouse_x = x_mouse;
	p->mouse_y = y_mouse;

	for (i = 0; i <= end_i; i++) {
		for (j = 0; j <= end_j; j++) {
			r_bar = norm(p->x + i, p->y + j);
			fr_bar = pow(r_bar - dead_zone, exp);
			x_bar = (p->x + i) * fr_bar / r_bar;
			y_bar = (p->y + j) * fr_bar / r_bar;
			cur_dist = unsqrtNorm(x_mouse - x_bar, y_mouse - y_bar);

			if (cur_dist < best_dist) {
				best_dist = cur_dist;
				best_i = i;
				best_j = j;
				p->mouse_x = x_bar;
				p->mouse_y = y_bar;
			}
		}
	}
	p->x += best_i;
	p->y += best_j;
}

static mouse2axisPoint mouse2axis_translation(double x_mouse, double y_mouse, int dead_zone, double exp,
		const mouse2axis_config* m2a_config) {
	mouse2axisPoint ret;
	const int max_pos_axis = m2a_config->zero_axis_is_positive == true ? 128 : 127;

	if (x_mouse == 0 && y_mouse == 0) {
		ret.x = 0;
		ret.y = 0;
		ret.mouse_x = 0;
		ret.mouse_y = 0;
		return ret;
	}

	const double x_mouse_abs = fabs(x_mouse);
	const double y_mouse_abs = fabs(y_mouse);
	if (m2a_config->zero_axis_is_positive == true) {
		dead_zone++;
	}

	double fr = norm(x_mouse, y_mouse);
	double r = pow(fr, 1 / exp) + dead_zone;
	double r_ret;
	double fr_ret;

	ret.x = x_mouse_abs * r / fr;
	ret.y = y_mouse_abs * r / fr;
	ret.x = clamp(-128, ret.x, max_pos_axis);
	ret.y = clamp(-128, ret.y, max_pos_axis);
	fixRoundingError(&ret, x_mouse_abs, y_mouse_abs, dead_zone, exp, max_pos_axis);
	ret.x = clamp(-128, ret.x, max_pos_axis);
	ret.y = clamp(-128, ret.y, max_pos_axis);

	r_ret = norm(ret.x, ret.y);
	fr_ret = pow(r_ret - dead_zone, exp);
	ret.mouse_x = ret.x * fr_ret / r_ret;
	ret.mouse_y = ret.y * fr_ret / r_ret;

	if (x_mouse <= 0) {
		ret.mouse_x *= -1;
		ret.x *= -1;
		if (m2a_config->zero_axis_is_positive == false && ret.x < -64) {
			ret.x--;
		}
	} else {
		if (m2a_config->zero_axis_is_positive == true) {
			ret.x--;
		}
	}
	if (y_mouse <= 0) {
		ret.mouse_y *= -1;
		ret.y *= -1;
		if (m2a_config->zero_axis_is_positive == false && ret.y < -64) {
			ret.y--;
		}
	} else {
		if (m2a_config->zero_axis_is_positive == true) {
			ret.y--;
		}
	}

	return ret;
}

//static double MAX_RADIUS = sqrt(2) * 128;

static double clampMotionResidue(double motion_residue, int x) {
	if (motion_residue > 0) {
		if (x >= 127)
			return 0;
		return motion_residue;
	}
	if (x == -128) {
		return 0;
	}
	return motion_residue;
}

double adv_mouse2axis(s_adapter* controller, int which, double x, double y, s_axis_props* axis_props, double exp,
		double multiplier, int dead_zone, const mouse2axis_config* m2a_config) {
	int axis = axis_props->axis;
	if ((which == AXIS_X && x == 0) || (which == AXIS_Y && y == 0)) {
		controller->axis[axis] = 0;
		return 0;
	}
	double motion_residue = 0;
	double multiplier_x = multiplier * controller_get_axis_scale(controller->ctype, AXIS_X);
	double multiplier_y = multiplier * controller_get_axis_scale(controller->ctype, AXIS_Y);

	multiplier_x = pow(multiplier_x, exp);
	multiplier_y = pow(multiplier_y, exp);

	mouse2axisPoint p;
	x *= multiplier_x;
	y *= multiplier_y;

	p = mouse2axis_translation(x, y, dead_zone, exp, m2a_config);

	switch (which) {
	case AXIS_X:
		controller->axis[axis] = p.x;
		motion_residue = (x - p.mouse_x) / multiplier_x;
		if (m2a_config->motion_residue_extrapolation == false) {
			motion_residue = clampMotionResidue(motion_residue, p.x);
		}

		break;
	case AXIS_Y:
		controller->axis[axis] = p.y;
		motion_residue = (y - p.mouse_y) / multiplier_y;
		if (m2a_config->motion_residue_extrapolation == false) {
			motion_residue = clampMotionResidue(motion_residue, p.y);
		}
		break;
	}

	if (m2a_config->motion_residue_extrapolation == true) {
		double max_motion_residue = 256 / multiplier;
		motion_residue = dclamp(-max_motion_residue, motion_residue, max_motion_residue);
	}

	return motion_residue;
}
