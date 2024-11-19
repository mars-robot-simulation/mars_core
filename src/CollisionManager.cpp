/**
 * \file CollisionManager.cpp
 * \author Malte Langosz
 *
 */

#include "CollisionManager.hpp"

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/events/GraphEventPublisher.hpp>
#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_interfaces/sim/DynamicObject.hpp>
#include <envire_types/Link.hpp>
#include "JointManager.hpp"
#include "Simulator.hpp"


namespace mars
{
    namespace core
    {
        CollisionManager::CollisionManager(const std::shared_ptr<interfaces::ControlCenter>& controlCenter) : controlCenter_{controlCenter.get()}
        {
            envire::core::GraphItemEventDispatcher<envire::core::Item<interfaces::ItemPluginItem>>::subscribe(controlCenter_->envireGraph_.get());
            envire::core::GraphItemEventDispatcher<envire::core::Item<interfaces::CollisionInterfaceItem>>::subscribe(controlCenter_->envireGraph_.get());
        }

        CollisionManager::~CollisionManager()
        {
            envire::core::GraphItemEventDispatcher<envire::core::Item<interfaces::ItemPluginItem>>::unsubscribe();
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


        void CollisionManager::clear()
        {
            using VertexDesc = envire::core::GraphTraits::vertex_descriptor;
            auto collisionObjectRemover = [this](VertexDesc node, VertexDesc parent)
            {
                itemRemover<std::shared_ptr<mars::ode_collision::Object>>(this->controlCenter_->envireGraph_.get(), node);
            };
            const auto& rootVertex = controlCenter_->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            controlCenter_->graphTreeView_->visitDfs(rootVertex, collisionObjectRemover);

            for (auto& collisionItem : collisionItems)
            {
                collisionItem.collisionInterface->freeSpace();
            }

            contactVector.clear();
        }

        void CollisionManager::reset()
        {
            updateTransforms();
            for (auto& collisionItem : collisionItems)
            {
                collisionItem.collisionInterface->initSpace();
            }
            for (auto& contactPlugin : contactPlugins)
            {
                contactPlugin->reset();
            }
        }

        void CollisionManager::clearPlugins()
        {
            auto graph = controlCenter_->envireGraph_.get();
            using VertexType = envire::core::GraphTraits::vertex_descriptor;
            auto removeFunctor = [&graph](VertexType node, VertexType parent)
            {
                itemRemover<interfaces::ContactPluginInterfaceItem>(graph, node);
            };

            const auto& rootVertex = controlCenter_->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            controlCenter_->graphTreeView_->visitBfs(rootVertex, removeFunctor);
            assert(contactPlugins.empty());
        }

        void CollisionManager::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<interfaces::ItemPluginItem>>& event)
        {
            auto& item = event.item->getData();

            envire::core::FrameId linkFrame;
            try
            {
                using LinkItem = envire::core::Item<envire::types::Link>;
                LinkItem &linkItem = Simulator::searchForTopItem<envire::types::Link>(controlCenter_->envireGraph_,
                                                                                      controlCenter_->graphTreeView_,
                                                                                      event.frame, &linkFrame);

                using PluginPtr = std::shared_ptr<interfaces::ContactPluginInterface>;
                PluginPtr contactPlugin = std::dynamic_pointer_cast<interfaces::ContactPluginInterface>(item.itemPlugin);
                contactPlugin->setFrameID(linkFrame);
                contactPlugins.push_back(contactPlugin);
            }
            catch (...)
            {
                LOG_ERROR("CollisionManager: Can't add contact plugin to frame %s.", event.frame.c_str());
            }
        }

        void CollisionManager::itemRemoved(const envire::core::TypedItemRemovedEvent<envire::core::Item<interfaces::ItemPluginItem>>& event)
        {
            auto item = event.item->getData();
            using PluginPtr = std::shared_ptr<interfaces::ContactPluginInterface>;
            PluginPtr contactPlugin = std::dynamic_pointer_cast<interfaces::ContactPluginInterface>(item.itemPlugin);
            auto positionOfItem = std::find_if(std::begin(contactPlugins), std::end(contactPlugins),
                [&contactPlugin](std::shared_ptr<interfaces::ContactPluginInterface> x)
                {
                    return x == contactPlugin;
                });
            assert(positionOfItem != std::end(contactPlugins));
            contactPlugins.erase(positionOfItem);
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
                for (const auto& contactPlugin : contactPlugins)
                {
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
