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
            GraphEventDispatcher::unsubscribe();
        }

        void AbsolutePoseExtender::frameAdded(const envire::core::FrameAddedEvent& e)
        {
            interfaces::AbsolutePose absolutePose;
            absolutePose.setFrameId(e.frame);
            envire::core::Item<interfaces::AbsolutePose>::Ptr absolutePoseItemPtr(new envire::core::Item<interfaces::AbsolutePose>(absolutePose));
            envireGraph->addItemToFrame(e.frame, absolutePoseItemPtr);

        };
    } // end of namespace core
} // end of namespace mars
