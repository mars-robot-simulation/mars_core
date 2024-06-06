#pragma once

#include <mars_interfaces/MARSDefs.h>

namespace mars
{
    namespace core
    {
        class PID
        {
        public:
            PID();
            ~PID();

            void step();

            interfaces::sReal p, i, d;
            interfaces::sReal last_value;
            interfaces::sReal last_error;
            interfaces::sReal last_i;
            interfaces::sReal current_value, target_value;
            interfaces::sReal filter_value;
            interfaces::sReal min_out, max_out, max_i;
            interfaces::sReal output_value;
        };
    }
}
