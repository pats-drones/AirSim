// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_RotorParams_hpp
#define msr_airlib_RotorParams_hpp

#include "common/Common.hpp"

namespace msr {
    namespace airlib {


        //In NED system, +ve torque would generate clockwise rotation
        enum class RotorTurningDirection :int {
            RotorTurningDirectionCCW = -1,
            RotorTurningDirectionCW = 1
        };

        struct RotorParams {
            /*
            Ref: http://physics.stackexchange.com/a/32013/14061
            force in Newton = C_T * \rho * n^2 * D^4
            torque in N.m = C_P * \rho * n^2 * D^5 / (2*pi)
            where,
            \rho = air density (1.225 kg/m^3)
            n = revolutions per sec
            D = propeller diameter in meters
            C_T, C_P = dimensionless constants available at
            propeller performance database http://m-selig.ae.illinois.edu/props/propDB.html
            Results for the standard AirSim drone
            * force in Newton = 4.17944
            * torque in Nm    = 0.05556
            force in Newton = C_T * \rho * n^2 * D^4
            4.17944 = C_T * 1.225 * (50000/60)^2 * 0.04^4
            C_T = 1.91913
            torque in N.m = C_T * \rho * n^2 * D^5 / (2*pi)
            0.05556 = C_P * 1.225 * (50000/60)^2 * 0.04^5 / (2*pi)
            C_P = 4.00745
            take the weight factor into account (1kg -> 50g)
            C_T = 1.91913 * 0.05 = 0.0959565
            C_P = 4.00745 * 0.05 = 0.2003725
            */

            real_T C_T = 0.0959565f * 1.3; // the thrust co-efficient 
            real_T C_P = 0.2003725f; // the torque co-efficient
            real_T air_density = 1.225f; //  kg/m^3
            real_T max_rpm = 50000.0f; // revolutions per minute
            real_T propeller_diameter = 0.04;   //diameter in meters
            real_T propeller_height = 1 / 100.0f;   //height of cylindrical area when propeller rotates
            real_T control_signal_filter_tc = 0.005f;    //time constant for low pass filter

            real_T revolutions_per_second;
            real_T max_speed; // in radians per second
            real_T max_speed_square;
            real_T max_thrust = 0; //computed from above formula for the given constants
            real_T max_torque = 0; //computed from above formula

            // call this method to recalculate thrust if you want to use different numbers for C_T, C_P, max_rpm, etc.
            void calculateMaxThrust() {
                revolutions_per_second = max_rpm / 60;
                max_speed = revolutions_per_second * 2 * M_PIf;  // radians / sec
                max_speed_square = pow(max_speed, 2.0f);

                real_T nsquared = revolutions_per_second * revolutions_per_second;
                max_thrust = C_T * air_density * nsquared * static_cast<real_T>(pow(propeller_diameter, 4));
                max_torque = C_P * air_density * nsquared * static_cast<real_T>(pow(propeller_diameter, 5)) / (2 * M_PIf);
            }

        };


    }
} //namespace
#endif
