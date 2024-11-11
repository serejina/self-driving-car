/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

namespace
{
   template <typename T>
   T clamp(T value, T min_value, T max_value)
   {
    return (value < min_value) ? min_value : (value > max_value) ? max_value : value;
   }
};

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   this->p_error = 0.0;
   this->d_error = 0.0;
   this->i_error = 0.0;

   this->p_k = Kpi;
   this->d_k = Kdi;
   this->i_k = Kii;

   this->output_lim_max = output_lim_maxi;
   this->output_lim_min = output_lim_mini;

   this->delta_time = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   this->p_error = cte;

   this->d_error = this->delta_time > 0 ? (cte - this->p_error)/this->delta_time : 0.0;
   
   this->i_error += cte * this->delta_time; 
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = this->p_k * this->p_error + this->d_k * this->d_error + this->i_k * this->i_error;
    return clamp(control, this->output_lim_min, this->output_lim_max);
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   this->delta_time = new_delta_time;
   return this->delta_time;
}