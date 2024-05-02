/**
 * \file SubWorld.hpp
 * \author Malte Langosz
 * \brief
 *
 */

#pragma once

#include <mars_utils/Thread.h>
#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_interfaces/sim/PhysicsInterface.h>

namespace mars
{
    namespace core
    {
        class SubWorld : public utils::Thread
        {
        public:
            std::shared_ptr<interfaces::SubControlCenter> control;
            bool calcStep, stopThread;

        protected:
            void run() override;
        };
    } // end of namespace core
} // end of namespace mars
