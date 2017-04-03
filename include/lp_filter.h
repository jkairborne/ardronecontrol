#ifndef _LP_FILTER_H_
#define _LP_FILTER_H_


class LP_FILTER
{
    public:
        // dt -  time interval
        // RC -  time constant

        LP_FILTER(double RC);
//        LP_FILTER() = default;
        void mod_RC(double new_RC);

        // Returns the manipulated variable given a setpoint and current process value
        double do_sample( double dt, double sample);
        ~LP_FILTER();

    private:
    	double _RC;
	double _last_val;
	double _alpha;
};

LP_FILTER::LP_FILTER (double RC)
{
	_RC = RC;

}

double LP_FILTER::do_sample(double dt, double sample)
{
	double curr_val;	
	_alpha = dt/(dt+_RC);
	curr_val = _alpha*sample + (1-_alpha) * _last_val;
	_last_val = curr_val;

	return curr_val;
}

void LP_FILTER::mod_RC(double new_RC){_RC = new_RC;}

LP_FILTER::~LP_FILTER()
{
}


#endif

