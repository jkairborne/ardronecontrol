#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv, double dt );
        void set_gains(double new_Kp, double new_Kd, double new_Ki);
        void reset_integral();
            
    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};

PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}

double PID::calculate(double setpoint, double pv , double dt)
{
    return pimpl->calculate(setpoint,pv,dt);
    cout << "In the calculate block";
}
PID::~PID() 
{
    delete pimpl;
}

void PID::mod_params(double new_Kp, double new_Kd, double new_Ki)
{
    pimpl->set_gains(new_Kp, new_Kd, new_Ki);
   // cout << "in the set gains block - new gains are: " << new_Kp << " " << new_Kd << " " << new_Ki << '\n';
}

void PID::rst_integral()
{
    pimpl->reset_integral();
}



void PIDImpl::set_gains(double new_Kp, double new_Kd, double new_Ki)
{
    _Kp = new_Kp;
    _Kd = new_Kd;
    _Ki = new_Ki;

    // This is to "reset" the integral gain - or else it can have significant inertia from a previous setting. In fact should probably implement some min/max on the integral gains...
    _integral = 0;
}

void PIDImpl::reset_integral()
{
    _integral = 0;
}

/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

double PIDImpl::calculate(double setpoint, double pv, double dt)
{
    if(dt == 0){dt = _dt;}
    else if(dt < 0) {cout << "dt supplied to PID is less than 0\n";}

    //cout << "Kp, Kd, Ki: " << _Kp << " " << _Kd << " " << _Ki << "\n";
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
