/**
 * \file SubWorld.cpp
 * \author Malte Langosz
 *
 */

#include "SubWorld.hpp"
#include <mars_utils/misc.h>
#include <getopt.h>
#include <time.h>

namespace mars
{
    namespace core
    {
        void SubWorld::run()
        {
            calcStep = false;
            stopThread = false;
            static struct timespec tv;
            //struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_nsec = 10000L;

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
                    //utils::msleep(1);

#ifdef __APPLE__
                    nanosleep(&tv, NULL);
#else
                    clock_nanosleep(CLOCK_MONOTONIC, 0, &tv, 0);
#endif
                    //select(0, 0, 0, 0, &tv);
                }
            }
        }

    } // end of namespace core
} // end of namespace mars
