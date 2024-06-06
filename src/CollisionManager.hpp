/**
 * \file CollisionMangaer.hpp
 * \author Malte Langosz
 * \brief
 *
 */

#pragma once

#include <mars_interfaces/sim/CollisionInterface.hpp>
#include <mars_interfaces/sim/CollisionHandler.hpp>
#include <mars_interfaces/sim/ContactPluginInterface.hpp>

namespace mars
{
    namespace core
    {
        class CollisionManager
        {
        public:
            void addCollisionHandler(const std::string &name1, const std::string &name2,
                                     std::shared_ptr<interfaces::CollisionHandler> collisionHandler);
            void handleContacts();
            void addCollisionInterfaceItem(const interfaces::CollisionInterfaceItem &item);
            std::vector<interfaces::ContactData>& getContactVector();
            void updateTransforms();

        private:
            void setupContactVector();
            void applyContactPlugins();

            std::map<std::pair<std::string, std::string>, std::shared_ptr<interfaces::CollisionHandler>> collisionHandlers;
            std::vector<interfaces::ContactData> contactVector;
            std::vector<interfaces::CollisionInterfaceItem> collisionItems;
            std::vector<interfaces::ContactPluginInterfaceItem> contactPluginItems;
        };
    } // end of namespace core
} // end of namespace mars
