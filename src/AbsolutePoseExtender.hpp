/**
 * \file AbsolutePoseExtender.hpp
 * \author Malte Langosz
 * \brief The class implements a frameAdded event callback and adds absolute pose items to the frames
 *
 */

#pragma once

#include <envire_core/items/Item.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/graph/EnvireGraph.hpp>

namespace mars
{
    namespace core
    {
        class AbsolutePoseExtender : public envire::core::GraphEventDispatcher
        {
        public:
            AbsolutePoseExtender(std::shared_ptr<envire::core::EnvireGraph> envireGraph);
            ~AbsolutePoseExtender();
            virtual void frameAdded(const envire::core::FrameAddedEvent& e) override;

        private:
            std::shared_ptr<envire::core::EnvireGraph> envireGraph;
        };
    } // end of namespace sim
} // end of namespace mars
