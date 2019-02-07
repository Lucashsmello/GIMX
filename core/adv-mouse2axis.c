/*
 * adv-mouse2axis.c
 *
 *  Created on: 9 de fev de 2018
 *      Author: lmello
 */

#include <adv-mouse2axis.h>
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

static double mouse2axis_table_out[127]; //lookuptable
static double mouse2axis_table_in[127];
static int mouse2axis_tablesize = 0;

static void postProcessTranslation(mouse2axisPoint* p, double x_mouse, double y_mouse,
		const mouse2axis_config* m2a_config) {
	if (x_mouse <= 0) {
		p->mouse_x *= -1;
		p->x *= -1;
//		if (m2a_config->zero_axis_is_positive == false && p->x < -64) {
//			p->x--;
//		}
	} else {
		if (m2a_config->zero_axis_is_positive == true) {
			p->x--;
		}
	}
	if (y_mouse <= 0) {
		p->mouse_y *= -1;
		p->y *= -1;
//		if (m2a_config->zero_axis_is_positive == false && p->y < -64) {
//			p->y--;
//		}
	} else {
		if (m2a_config->zero_axis_is_positive == true) {
			p->y--;
		}
	}
}

/**
 * Lookup Table
 */
void loadMouse2axisTable(const char* filename) {
	FILE* f = fopen(filename, "r");
	int k, n;
	mouse2axis_tablesize = 0;
	if (f == NULL) {
		printf("loadMouse2axisTable() has failed: Could not open file\n");
		return;
	}

	n = fscanf(f, "%lf;%lf", mouse2axis_table_in, mouse2axis_table_out);
//	printf("read pair values: (%lf,%lf)\n", *mouse2axis_table_in, *mouse2axis_table_out);
	for (k = 1; n == 2; k++) {
		n = fscanf(f, "%lf;%lf", mouse2axis_table_in + k, mouse2axis_table_out + k);
		if (n == 2) {
//			printf("read pair values: (%lf,%lf)\n", mouse2axis_table_in[k], mouse2axis_table_out[k]);
			if (mouse2axis_table_in[k] < mouse2axis_table_in[k - 1]) {
				printf("loadMouse2axisTable() has failed: Values are not in ascending order!\n");
				fclose(f);
				return;
			}
		}
	}
	printf("Mouse2axisTable has %d values\n", k - 1);
	mouse2axis_tablesize = k - 1;
	fclose(f);
}

static int binarySearch(const double* vet, int n, double x) {
	int a = 0, b = n - 1, i = (n - 1) / 2;
	while (a < b - 1) {
		i = (a + b) / 2;
		if (x > vet[i]) {
			a = i + 1;
		} else {
			if (x < vet[i]) {
				b = i - 1;
			} else {
				return i;
			}
		}
	}
	if (a > 0) {
		a--;
	}
	if (b < n - 1) {
		b++;
	}
	int closest_idx = b;
	double closest = vet[b] - x, c;
	for (i = a; i < b; i++) {
		c = fabs(vet[i] - x);
		if (c < closest) {
			closest_idx = i;
			closest = c;
		}
	}
	return closest_idx;
}

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

static mouse2axisPoint lookUpTable(double x_mouse, double y_mouse, const mouse2axis_config* m2a_config) {
	mouse2axisPoint p;
	const int max_pos_axis = m2a_config->zero_axis_is_positive == true ? 128 : 127;
	double fr = norm(x_mouse, y_mouse);
	int idx = binarySearch(mouse2axis_table_out, mouse2axis_tablesize, fr);
	double r = mouse2axis_table_in[idx];
	const double x_mouse_abs = fabs(x_mouse);
	const double y_mouse_abs = fabs(y_mouse);
	p.x = round(x_mouse_abs * r / fr);
	p.y = round(y_mouse_abs * r / fr);
	p.x = clamp(-128, p.x, max_pos_axis);
	p.y = clamp(-128, p.y, max_pos_axis);

	/*calculates how much will actually move:*/
	r = norm(p.x, p.y);
	idx = binarySearch(mouse2axis_table_in, mouse2axis_tablesize, r);
	fr = mouse2axis_table_out[idx];
	p.mouse_x = p.x * fr / r;
	p.mouse_y = p.y * fr / r;
	/********************/

	postProcessTranslation(&p, x_mouse, y_mouse, m2a_config);

	return p;
}

static void fixRoundingError(mouse2axisPoint* p, double x_mouse, double y_mouse, double dead_zone, double exp,
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

static mouse2axisPoint mouse2axis_translation(double x_mouse, double y_mouse, double dead_zone, double exp,
		const mouse2axis_config* m2a_config) {
	mouse2axisPoint ret;
//	const int max_pos_axis = m2a_config->zero_axis_is_positive == true ? 128 : 127;
	const int max_pos_axis = 128;

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

	postProcessTranslation(&ret, x_mouse, y_mouse, m2a_config);

	return ret;
}

//static double MAX_RADIUS = sqrt(2) * 128;

static double clampMotionResidue(double motion_residue, int x) {
	if (motion_residue > 0) {
		if (x >= 127)
			return 0;
		return motion_residue;
	}
	if (x <= -128) {
		return 0;
	}
	return motion_residue;
}

static double calculateMotionResidue(int axis_out, double v_out, double v_in, double mult,
		const mouse2axis_config* m2a_config) {
	double residue;
	residue = (v_in - v_out) / mult;
	if (m2a_config->motion_residue_extrapolation == false) {
		residue = clampMotionResidue(residue, axis_out);
	} else {
		double max_motion_residue = 256 / mult;
		residue = dclamp(-max_motion_residue, residue, max_motion_residue);
	}
	return residue;
}

void adv_mouse2axis(s_adapter* controller, const s_mapper * mapper_x, s_vector * input, s_mouse_control * mc,
		const mouse2axis_config* m2a_config) {
	s_mapper * mapper_y = mapper_x->other;
	double dz_x = mapper_x->dead_zone;
	if (input->x == 0 && input->y == 0) {
		controller->axis[mapper_y->axis_props.axis] = 0;
		if (dz_x < 0) {
			controller->axis[mapper_x->axis_props.axis] = dz_x;
		} else {
			controller->axis[mapper_x->axis_props.axis] = 0;
		}
		mc->residue.x = mc->residue.y = 0;
		return;
	}
	double axis_scale = controller_get_axis_scale(controller->ctype, mapper_x->axis);
	double multiplier_x = mapper_x->multiplier * axis_scale;
	double multiplier_y = mapper_y->multiplier * axis_scale;
	double x = input->x * multiplier_x;
	double y = input->y * multiplier_y;
	mouse2axisPoint p;
	if (mouse2axis_tablesize <= 0) {
		double dz;
		if (dz_x >= 0) { //GAMB
			dz = mapper_y->dead_zone;
			dz = dz / 100 + dz_x;
		} else {
			dz = 0;
		}
		double exponent = mapper_x->exponent;
		multiplier_x = pow(multiplier_x, 1 / exponent);
		multiplier_y = pow(multiplier_y, 1 / exponent);
		p = mouse2axis_translation(x, y, dz, 1 / exponent, m2a_config);
	} else {
		p = lookUpTable(x, y, m2a_config);
	}

	mc->residue.x = calculateMotionResidue(p.x, p.mouse_x, x, multiplier_x, m2a_config);
	mc->residue.y = calculateMotionResidue(p.y, p.mouse_y, y, multiplier_y, m2a_config);

	if (dz_x < 0) { //GAMB
		p.x--;
	}

	controller->axis[mapper_x->axis_props.axis] = clamp(-128, p.x, 127);
	controller->axis[mapper_y->axis_props.axis] = clamp(-128, p.y, 127);

//	printf("in: (%lf,%lf) move:(%d,%d) residue(%lf,%lf)\n", x, y, p.x, p.y, mc->residue.x, mc->residue.y);
//	mc->residue.x = 0;
//	mc->residue.y = 0;
}
