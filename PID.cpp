/*
 * PID.cpp
 *
 *  Created on: May 23, 2015
 *      Author: Noe
 *
 *  This code was heavily based on :
 *  http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 */
#include <stdint.h>
#include "PID.hpp"

PID::PID(double * input, double * output, double * setpoint,
            sPID pid, uint16_t sample_time)
{
    printf("pid_settings: \n kp: %f\n ki: %f\n kd: %f\n Dir: %d\n max: %f\n",
            pid.kp,pid.ki,pid.kd, pid.contr_dir,pid.max);

    printf(" min: %f\n auto: %d",pid.min, pid.in_auto);

    my_input = input;
    my_output = output;
    my_setpoint = setpoint;
    in_auto = false;

    PID::set_output_limits(pid);

    dt = ((double)sample_time)/1000;

    PID::set_contr_dir(pid);
    PID::set_tunnings(pid);
}

bool PID::compute()
{
    if(!in_auto)
        return false;

    // Compute errors
    double error = *my_setpoint - *my_input;
//    printf("error: %f\n", error);
    iterm+=(ki * error);
    if(iterm > max)
        iterm = max;
    else if (iterm < min)
        iterm = min;
//    printf("iterm: %f\n", iterm);
    double dinput = *my_input - last_input;

    // Compute PID output
    double output = kp * error + iterm - kd * dinput;
    //printf("output: %f\n", output);
    if(output > max)
        output = max;
    else if(output < min)
        output = min;

    *my_output = output;

    // Store for future use
    last_input = *my_input;
    return true;
}

void PID::set_tunnings(sPID pid)
{
    if(pid.kp > 0 || pid.ki > 0 || pid.kd > 0)
    {
        kp = pid.kp;
        ki = pid.ki * dt;
        kd = pid.kd / dt;

        if(contr_dir == REVERSE)
        {
            kp = 0 - kp;
            ki = 0 - ki;
            kd = 0 - kd;
        }
    }
}

void PID::set_sample_time(uint16_t new_sample_time)
{
   if (new_sample_time > 0)
   {
      double ratio  = ((double)new_sample_time)/1000
                      / dt;
      ki *= ratio;
      kd /= ratio;
      dt = ((double)new_sample_time) /1000;
   }
}

void PID::set_output_limits(sPID pid)
{
    if(pid.min < pid.max)
    {
        min = pid.min;
        max = pid.max;
        if(in_auto)
        {
            if(*my_output > max)
                *my_output = max;
            else if(*my_output< min)
                *my_output = min;

            if(iterm > max)
                iterm = max;
            else if(iterm < min)
                iterm = min;
        }
    }
}

void PID::set_mode(sPID pid)
{
    bool new_auto = (pid.in_auto == AUTOMATIC);

    // Manual to auto
    if(new_auto == !in_auto)
    {
        PID::init();
    }
    in_auto = new_auto;
}

void PID::init()
{
    iterm = *my_output;
    last_input = *my_input;
    if(iterm > max)
        iterm = max;
    else if (iterm < min)
        iterm = min;
}

void PID::set_contr_dir(sPID pid)
{
    if(in_auto && pid.contr_dir != contr_dir)
    {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
    contr_dir = pid.contr_dir;
}



