/**
 * \file CollisionManager.cpp
 * \author Malte Langosz
 *
 */

#include "CollisionManager.hpp"

#include <mars_interfaces/sim/ControlCenter.h>

namespace mars
{
    namespace core
    {
        void CollisionManager::addCollisionHandler(const std::string &name1, const std::string &name2,
                                                   std::shared_ptr<interfaces::CollisionHandler> collisionHandler)
        {
            if (collisionHandlers.count(std::make_pair(name2, name1)) > 0)
            {
                const auto msg = std::string{"CollisionManager::addCollisionManager: Adding collision handler for (\""} + name1 + "\", \"" + name2 + "\") but there is already one for the inverse tuple.";
                LOG_WARN(msg.c_str());
            }
            collisionHandlers[std::make_pair(name1, name2)] =  collisionHandler;
        }

        void CollisionManager::addCollisionInterfaceItem(const interfaces::CollisionInterfaceItem &item)
        {
            collisionItems.push_back(item);
        }

        std::vector<interfaces::ContactData>& CollisionManager::getContactVector()
        {
            return contactVector;
        }

        void CollisionManager::handleContacts()
        {
            // TODO: find a good mechanism to flixible get contacts between different collision spaces
            contactVector.clear();
            updateTransforms();

            // for future:
            // - where to load spaces? from envire_graph?
            // - how to deal with sub-collision-spaces?
            // - first check bounding boxes // collect boxes of sub-spaces
            decltype(collisionHandlers)::iterator handlerIt;
            for(size_t l=0; l<collisionItems.size(); ++l)
            {
                for(size_t k=l+1; k<collisionItems.size(); ++k)
                {
                    const auto& ci1 = collisionItems[l].collisionInterface;
                    const auto& ci2 = collisionItems[k].collisionInterface;
                    const auto& ci1PluginName = collisionItems[l].pluginName;
                    const auto& ci2PluginName = collisionItems[k].pluginName;
                    handlerIt = collisionHandlers.find(std::make_pair(ci1PluginName,
                                                                      ci2PluginName));
                    if(handlerIt != collisionHandlers.end())
                    {
                        // extends contacts
                        handlerIt->second->getContacts(ci1, ci2, contactVector);
                        continue;
                    }

                    handlerIt = collisionHandlers.find(std::make_pair(ci2PluginName,
                                                                      ci1PluginName));
                    if(handlerIt != collisionHandlers.end())
                    {
                        // extends contacts
                        handlerIt->second->getContacts(ci2, ci1, contactVector);
                        continue;
                    }
                }
            }
        }

        void CollisionManager::updateTransforms()
        {
            for(auto &it: collisionItems)
            {
                it.collisionInterface->updateTransforms();
            }
        }

    } // end of namespace core
} // end of namespace mars
