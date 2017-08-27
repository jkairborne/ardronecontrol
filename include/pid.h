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
        PID( double dt, double max, double min, double Kp, double Kd, double Ki );
//        PID() = default;
        void mod_params(double new_Kp, double new_Kd, double new_Ki);

        void rst_integral();

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv, double dt = 0 );
        ~PID();

    private:
        PIDImpl *pimpl;
};

#endif
