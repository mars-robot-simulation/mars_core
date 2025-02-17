#include "PID.hpp"

#include <algorithm>

namespace mars
{
    namespace core
    {
        PID::PID() :
            p(0), i(0), d(0), last_value(0), last_error(0), last_i(0),
            current_value(0), target_value(0), output_value(0),
            min_out(0), max_out(0), max_i(0), filter_value(0), limit_pos_error(false)
        {
        }

        PID::~PID()
        {
        }

        void PID::step()
        {
            // apply filter on current (sensed) value
            current_value = last_value * filter_value + current_value * (1-filter_value);
            last_value = current_value;
            interfaces::sReal error = target_value-current_value;
            if(limit_pos_error)
            {
                if(error > M_PI)
                    error = -2*M_PI+error;
                else if(error < -M_PI)
                    error = 2*M_PI+error;
            }
            interfaces::sReal p_part = error*p;
            interfaces::sReal d_part = (error-last_error)*d;
            interfaces::sReal i_part = last_i*i;
            last_error = error;
            last_i += error;
            last_i = std::max(-max_i, std::min(last_i, max_i));
            output_value = p_part + d_part + i_part;
            output_value = std::max(min_out, std::min(output_value, max_out));
        }
    }
}
