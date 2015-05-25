/*
 * PID.hpp
 *
 *  Created on: May 23, 2015
 *      Author: Noe
 *
 *  This code was heavily based on :
 *  http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 */

#ifndef PID_HPP_
#define PID_HPP_

#include <stdio.h>

struct sPID
{
    double kp;
    double ki;
    double kd;
    uint8_t contr_dir;
    double max;
    double min;
    bool in_auto;
};

class PID
{
    public:

        #define AUTOMATIC   1
        #define MANUAL      0
        #define DIRECT      0
        #define REVERSE     1

        PID(double * input, double * output, double * setpoint,
                sPID pid, uint16_t sample_time);

        void set_mode(sPID pid);

        bool compute();

        void set_output_limits(sPID pid);

        void set_tunnings(sPID pid);

        void set_contr_dir(sPID pid);

        void set_sample_time(uint16_t);

    private:
        void init();

        double kp;
        double ki;
        double kd;

        uint8_t contr_dir;

        double * my_input;
        double * my_output;
        double * my_setpoint;

        float dt;
        double min;
        double max;
        bool in_auto;

        double iterm;
        double last_input;
};


#endif /* PID_HPP_ */
