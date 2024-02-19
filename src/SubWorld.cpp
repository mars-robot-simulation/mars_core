/**
 * \file SubWorld.cpp
 * \author Malte Langosz
 *
 */

#include "SubWorld.hpp"
#include <mars_utils/misc.h>
#include <getopt.h>

namespace mars
{
    namespace core
    {
        void SubWorld::run()
        {
            calcStep = false;
            stopThread = false;
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 1000L;

            while(!stopThread)
            {
                if(calcStep)
                {
                    control->physics->stepTheWorld();
                    calcStep = false;
                }
                else
                {
                    //fprintf(stderr, "sleep.");
                    utils::msleep(1);

                    //select(0, 0, 0, 0, &tv);
                }
            }
        }

    } // end of namespace core
} // end of namespace mars
