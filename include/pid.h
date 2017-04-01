#ifndef _PID_H_
#define _PID_H_

class PIDImpl;
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double dt=0.01, double max=1, double min=-1, double Kp=1, double Kd=0, double Ki=0 );
//        PID() = default;
        void mod_params(double new_Kp, double new_Kd, double new_Ki);
        void set_dt(double dt);

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv, double dt );
        double calculate( double setpoint, double pv);
        ~PID();

    private:
        PIDImpl *pimpl;
};

#endif
