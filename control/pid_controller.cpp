/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

   /**
   * TODO: Create the PID class
   **/

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);

private:
    /*
    * Errors
    */
    double p_error;
    double d_error;
    double i_error;

    /*
    * Coefficients
    */
    double p_k;
    double d_k;
    double i_k;

    /*
    * Output limits
    */
    double output_lim_max;
    double output_lim_min;
  
    /*
    * Delta time
    */
    double delta_time;
};

#endif //PID_CONTROLLER_H
