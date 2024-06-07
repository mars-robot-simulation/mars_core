/**
 * \file CollisionManager.cpp
 * \author Malte Langosz
 *
 */

#include "CollisionManager.hpp"

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/events/GraphEventPublisher.hpp>
#include <mars_interfaces/sim/ControlCenter.h>

namespace mars
{
    namespace core
    {
        CollisionManager::CollisionManager(const std::shared_ptr<interfaces::ControlCenter>& controlCenter)
        {
            envire::core::GraphItemEventDispatcher<envire::core::Item<interfaces::ContactPluginInterfaceItem>>::subscribe(controlCenter->envireGraph_.get());
            envire::core::GraphItemEventDispatcher<envire::core::Item<interfaces::CollisionInterfaceItem>>::subscribe(controlCenter->envireGraph_.get());
        }

        CollisionManager::~CollisionManager()
        {
            envire::core::GraphItemEventDispatcher<envire::core::Item<interfaces::ContactPluginInterfaceItem>>::unsubscribe();
            envire::core::GraphItemEventDispatcher<envire::core::Item<interfaces::CollisionInterfaceItem>>::unsubscribe();
        }

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

        std::vector<interfaces::ContactData>& CollisionManager::getContactVector()
        {
            return contactVector;
        }

        void CollisionManager::handleContacts()
        {
            updateTransforms();

            setupContactVector();
            applyContactPlugins();
        }

        void CollisionManager::updateTransforms()
        {
            for(auto &it: collisionItems)
            {
                it.collisionInterface->updateTransforms();
            }
        }

        void CollisionManager::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<interfaces::ContactPluginInterfaceItem>>& event)
        {
            auto& item = event.item->getData();
            item.contactPluginInterface->setFrameID(event.frame);
            contactPluginItems.push_back(item);
        }

        void CollisionManager::itemRemoved(const envire::core::TypedItemRemovedEvent<envire::core::Item<interfaces::ContactPluginInterfaceItem>>& event)
        {
            auto& item = event.item->getData();
            auto positionOfItem = std::find(std::begin(contactPluginItems), std::end(contactPluginItems), item);
            contactPluginItems.erase(positionOfItem);
        }

        void CollisionManager::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<interfaces::CollisionInterfaceItem>>& event)
        {
            auto& item = event.item->getData();
            collisionItems.push_back(item);
        }

        void CollisionManager::itemRemoved(const envire::core::TypedItemRemovedEvent<envire::core::Item<interfaces::CollisionInterfaceItem>>& event)
        {
            auto& item = event.item->getData();
            auto positionOfItem = std::find(std::begin(collisionItems), std::end(collisionItems), item);
            collisionItems.erase(positionOfItem);
        }

        void CollisionManager::setupContactVector()
        {
            // TODO: find a good mechanism to flixible get contacts between different collision spaces
            contactVector.clear();

            // TODO: for future:
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

        void CollisionManager::applyContactPlugins()
        {
            for (auto& contact : contactVector)
            {
                for (const auto& contactPluginItem : contactPluginItems)
                {
                    const auto& contactPlugin = contactPluginItem.contactPluginInterface;
                    if (!contactPlugin->affects(contact))
                    {
                        continue;
                    }

                    contactPlugin->updateContact(contact);
                    break; // TODO: Apply affecting contact plugin with highest priority.
                }
            }
        }

    } // end of namespace core
} // end of namespace mars
