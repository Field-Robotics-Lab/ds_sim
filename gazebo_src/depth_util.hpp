/*
 * Fofonoff depth equations
 * Now including an Inverse!
 *
 * 2017 Nov 22
 *
 * Ian Vaughn
 * ivaughn@whoi.edu
 *
 */

#pragma once

#include <math.h>

extern const double ALVIN_DEPTH_LAT;
extern double fofonoff_depth(double pressure_dbar, double latitude_deg);
extern double fofonoff_pressure(double depth_m, double latitude_deg);
