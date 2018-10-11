/**
* Copyright 2018 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
/*
 *
 * 2017 Nov 22
 *
 * Ian Vaughn
 * ivaughn@whoi.edu
 *
 */


#include "depth_util.hpp"

#include <math.h>


/// \brief Standard latitude used for Alvin depth
const double ALVIN_DEPTH_LAT = 30.0;


/// From depth.m in dslpp
///
/// Notes:  (ETP3, MBARI)
///         This algorithm was originally compiled by RP @ WHOI.
///         It was copied from the UNESCO technical report.
///         The algorithm was endorsed by SCOR Working Group 51.
///         The equations were originally developed by Saunders
///             and Fofonoff (1976).  DSR 23: 109-111.
///         The parameters were re-fit for the 1980 equation of
///             state for seawater (EOS80).
///
/// CHECKVALUE: D = 9712.653 M FOR P=10000 DECIBARS, LAT=30 DEG
///
/// CALCULATON ASSUMES STD OCEAN: T = 0 DEG C; S = 35 (IPSS-78)
double fofonoff_depth(double pressure_dbar, double latitude_deg) {
    // Compute the fofonoff gravitational anomaly
    double sinL2 = sin(latitude_deg / 57.29578);
    sinL2 *= sinL2;
    double Gr = 9.780318*(1.0+(5.2788e-3+2.36e-5*sinL2)*sinL2);


    double depth = (((-1.82E-15*pressure_dbar+2.279E-10)
                        *pressure_dbar-2.2512E-5)*pressure_dbar+9.72659)*pressure_dbar;

    return depth / (Gr + 1.092E-6*pressure_dbar);
}

// Use an empirical curve fit to invert pressure
// Good to 4.7664e-4 decibar (0.477 Pa) from 0-11.6km depth
double fofonoff_pressure(double depth_m, double latitude_deg) {

    // Compute the fofonoff gravitational anomaly
    double sinL2 = sin(latitude_deg / 57.29578);
    sinL2 *= sinL2;
    double Gr = 9.780318*(1.0+(5.2788e-3+2.36e-5*sinL2)*sinL2);

    // apply custom scaling
    double x = Gr / 10.0;
    double y = depth_m / 12000.0;

    // our GIANT polynomial fit

    // these coefficients come from a matlab fit object
    // using the code snippet:
    //
    // cnames = coeffnames(sf);
    // for i=1:length(cnames)
    //    fprintf('double %s = %.16g;\n', cnames{i}, sf.(cnames{i}));
    // end
    double p00 = 7.60574178224851e-05;
    double p10 = -0.0002408499992480632;
    double p01 = -0.0008374326178820384;
    double p20 = 0.0002530612703805393;
    double p11 = 1.030858432652839;
    double p02 = 0.001813427317861215;
    double p30 = -8.825692248043685e-05;
    double p21 = -0.002966529127746074;
    double p12 = -0.005114563983411041;
    double p03 = -0.0007631823220180533;
    double p31 = 0.001054693123038537;
    double p22 = 0.03675844695608228;
    double p13 = 0.004000155179706756;
    double p04 = -0.0004513342121006175;
    double p32 = -0.002712434485673277;
    double p23 = -0.005112823606704678;
    double p14 = 0.0004048861966281985;
    double p05 = 5.018581815489212e-05;

    // this polynomial is 5-th degree in depth and 3rd degree in gravitational
    // anomaly.
    double p = p00 + x*(p10 + x*(p20 + x*p30))
           + y*(p01 + x*(p11 + x * (p21 + p31 * x))
               + y*(p02 + x*(p12 + x*(p22 + p32*x))
                    + y*(p03 + x*(p13 + p23*x)
                        + y*(p14*x + p04 + p05*y))));

    // undo the pressure scaling
    return p * 12000.0;

}



