/**
 * \file AbsolutePoseExtender.cpp
 * \author Malte Langosz
 */

#include "AbsolutePoseExtender.hpp"
#include <mars_interfaces/sim/AbsolutePose.hpp>

namespace mars
{
    namespace core
    {
        AbsolutePoseExtender::AbsolutePoseExtender(std::shared_ptr<envire::core::EnvireGraph> envireGraph) : envireGraph(envireGraph)
        {
            GraphEventDispatcher::subscribe(envireGraph.get());
        }

        AbsolutePoseExtender::~AbsolutePoseExtender()
        {

        }

        void AbsolutePoseExtender::frameAdded(const envire::core::FrameAddedEvent& e)
        {
            interfaces::AbsolutePose absolutePose;
            absolutePose.frameId = e.frame;
            envire::core::Item<interfaces::AbsolutePose>::Ptr absolutePoseItemPtr(new envire::core::Item<interfaces::AbsolutePose>(absolutePose));
            envireGraph->addItemToFrame(e.frame, absolutePoseItemPtr);

        };
    } // end of namespace sim
} // end of namespace mars
