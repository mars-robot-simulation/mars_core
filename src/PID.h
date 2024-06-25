#pragma once

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

            sReal p, i, d;
            sReal last_value;
            sReal last_error;
            sReal last_i;
            sReal current_value, target_value;
            sReal filter_value;
        };
    }
}
