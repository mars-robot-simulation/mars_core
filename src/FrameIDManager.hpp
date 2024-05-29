#pragma once

#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_interfaces/sim/IDManager.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/graph/EnvireGraph.hpp>

namespace mars
{
    namespace core
    {
        class FrameIDManager :  public interfaces::IDManager,
                                public envire::core::GraphEventDispatcher
        {
        public:
            FrameIDManager()
            {
                GraphEventDispatcher::subscribe(interfaces::ControlCenter::envireGraph.get());
            }

            virtual void frameAdded(const envire::core::FrameAddedEvent& e) override
            {
                addIfUnknown(e.frame);
            }

            virtual void frameRemoved(const envire::core::FrameRemovedEvent& e) override
            {
                removeEntry(e.frame);
            }
        };
    }
}