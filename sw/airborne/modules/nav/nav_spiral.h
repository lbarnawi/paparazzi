/*
 * Copyright (C) 2011-2013  The Paparazzi Team
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/nav/nav_spiral.h
 *
 * Fixedwing navigation in a spiral/helix.
 *
 */

#ifndef NAV_SPIRAL_H
#define NAV_SPIRAL_H

#include "std.h"
#include "math/pprz_algebra_float.h"

enum SpiralStatus { SpiralOutside, SpiralStartCircle, SpiralCircle, SpiralInc };

struct NavSpiral {
  struct FloatVect3 trans_current;
  struct FloatVect2 fly_from;
  struct FloatVect2 last_circle;
  struct FloatVect3 center;
  struct FloatVect3 pos_incr;
  float dist_from_center;
  float alpha_limit;
  float segments;
  float radius;
  float radius_min;
  float radius_start;
  float radius_increment;
  enum SpiralStatus status;
};

extern struct NavSpiral nav_spiral;

extern void nav_spiral_init(void);

/** Run spiral navigation
 */
extern bool nav_spiral_run(void);

/** Initialize spiral based on:
 *    - waypoints (center, edge)
 *    - start radius
 *    - increments
 */
extern void nav_spiral_setup(uint8_t center_wp, uint8_t edge_wp, float radius_start,
                             float radius_inc, float segments);

/** Initialize spiral based on:
 *    - position X, Y
 *    - start and stop altitude
 *    - start and stop radius
 *    - speeds (horizontal and vertical
 */
extern void nav_spiral_setup2(float center_x, float center_y,
                              float alt_start, float alt_stop,
                              float radius_start, float radius_stop,
                              float vx, float vy, float vz);

#endif // NAV_SPIRAL_H

