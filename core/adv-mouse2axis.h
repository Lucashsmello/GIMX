/*
 * adv-mouse2axis.h
 *
 *  Created on: 9 de fev de 2018
 *      Author: lmello
 */

#ifndef ADV_MOUSE2AXIS_H_
#define ADV_MOUSE2AXIS_H_

#include <controller.h>

typedef enum {
	false, true
} boolean;

typedef struct {
	boolean preserve_angle;
	boolean motion_residue_extrapolation; // if true, accepts very low or very high motion_residues.
	boolean zero_axis_is_positive;
} mouse2axis_config;

void adv_mouse2axis(s_adapter* controller, const s_mapper * mapper_x, s_vector * input, s_mouse_control * mc,
		const mouse2axis_config* m2a_config);
void loadMouse2axisTable(const char* filename);

#endif /* ADV_MOUSE2AXIS_H_ */
