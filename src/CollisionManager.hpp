/**
 * \file CollisionMangaer.hpp
 * \author Malte Langosz
 * \brief
 *
 */

#pragma once

#include <memory>
#include <vector>

#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_interfaces/sim/CollisionInterface.hpp>
#include <mars_interfaces/sim/CollisionHandler.hpp>
#include <mars_interfaces/sim/ContactPluginInterface.hpp>

#include <envire_core/items/Item.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>

namespace mars
{
    namespace core
    {
        class CollisionManager :    public envire::core::GraphItemEventDispatcher<envire::core::Item<interfaces::ContactPluginInterfaceItem>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<interfaces::CollisionInterfaceItem>>
        {
        public:
            CollisionManager(const std::shared_ptr<interfaces::ControlCenter>& controlCenter);
            virtual ~CollisionManager();

            void addCollisionHandler(const std::string &name1, const std::string &name2,
                                     std::shared_ptr<interfaces::CollisionHandler> collisionHandler);
            void handleContacts();
            std::vector<interfaces::ContactData>& getContactVector();
            void updateTransforms();
            void clearAllPlugins();

        protected:
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<interfaces::ContactPluginInterfaceItem>>& event) override;
            virtual void itemRemoved(const envire::core::TypedItemRemovedEvent<envire::core::Item<interfaces::ContactPluginInterfaceItem>>& event) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<interfaces::CollisionInterfaceItem>>& event) override;
            virtual void itemRemoved(const envire::core::TypedItemRemovedEvent<envire::core::Item<interfaces::CollisionInterfaceItem>>& event) override;

        private:
            void setupContactVector();
            void applyContactPlugins();

            interfaces::ControlCenter* const controlCenter_;
            std::map<std::pair<std::string, std::string>, std::shared_ptr<interfaces::CollisionHandler>> collisionHandlers;
            std::vector<interfaces::ContactData> contactVector;
            std::vector<interfaces::CollisionInterfaceItem> collisionItems;
            std::vector<interfaces::ContactPluginInterface*> contactPlugins;
        };
    } // end of namespace core
} // end of namespace mars
