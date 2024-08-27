/**
 * \file JointManager.cpp
 * \author  Malte Langosz, Julian Liersch
 * \brief "JointManager" implements the JointManagerInterface.
 * It is manages all joints and all joint
 * operations that are used for the communication between the simulation
 * modules.
 */

#include "JointManager.hpp"
#include "SimJoint.hpp"
#include "Simulator.hpp"
#include "NodeManager.hpp"

#ifndef JOINT_NAMESPACE
// TODO: This should be done differently!
#define JOINT_NAMESPACE "envire::types::joints::"
#endif

#include <stdexcept>

#include <envire_types/registration/TypeCreatorFactory.hpp>
#include <envire_types/joints/Fixed.hpp>
#include <envire_types/joints/Revolute.hpp>
#include <envire_types/joints/Continuous.hpp>
#include <envire_types/joints/Prismatic.hpp>

#include <data_broker/DataBrokerInterface.h>

#include <mars_interfaces/utils.h>
#include <mars_interfaces/MARSDefs.h>
#include <mars_interfaces/sim/JointInterface.h>
#include <mars_interfaces/sim/MotorManagerInterface.h>
#include <mars_interfaces/sim/SimulatorInterface.h>

#include <mars_utils/misc.h>
#include <mars_utils/mathUtils.h>
#include <mars_utils/MutexLocker.h>
#include <mars_interfaces/Logging.hpp>

// TODO: Clean up mutexes!
//    * Graph should be locked globally when it is being worked on
// TODO: Tests!

namespace mars
{
    namespace core
    {
        using namespace utils;
        using namespace interfaces;
        using namespace std;

        using Iterator = envire::core::EnvireGraph::ItemIterator<envire::core::Item<interfaces::JointInterfaceItem>>;

        /**
         *\brief Initialization of a new JointManager
         *
         * pre:
         *     - a pointer to a ControlCenter is needed
         * post:
         *     - next_node_id should be initialized to one
         */
        JointManager::JointManager(ControlCenter *c)
            : control(c),
            idManager_{new JointIDManager{c->envireGraph_}}
        {}

        unsigned long JointManager::addJoint(JointData *jointS, bool reload)
        {
            auto jointMap = constructEnvireJointConfigMap(*jointS);

            // TODO: This should be handled at a central location
            const auto className = std::string{JOINT_NAMESPACE} + jointMap["type"].toString();
            envire::core::ItemBase::Ptr item = envire::types::TypeCreatorFactory::createItem(className, jointMap);

            const envire::core::FrameId parentFrame = dynamic_cast<NodeManager*>(control->nodes)->getLinkName(jointS->nodeIndex1);
            const envire::core::FrameId childFrame = dynamic_cast<NodeManager*>(control->nodes)->getLinkName(jointS->nodeIndex2);

            if(!control->envireGraph_->containsFrame(parentFrame))
            {
                throw std::logic_error((std::string{"JointManager::addJoint: Tried adding joint linked to non-existant link "} + std::string{parentFrame}).c_str());
            }
            if(!control->envireGraph_->containsFrame(childFrame))
            {
                throw std::logic_error((std::string{"JointManager::addJoint: Tried adding joint linked to non-existant link "} + std::string{childFrame}).c_str());
            }

            const envire::core::FrameId jointFrameName = constructFrameIdFromJointData(*jointS);
            if(!isFixedJoint(*jointS))
            {
                // Insert new frame for non-fixed joints
                if(control->envireGraph_->containsFrame(jointFrameName))
                {
                    throw std::logic_error((std::string{"JointManager::addJoint: There is already a joint frame with the name "} + std::string{jointFrameName}).c_str());
                }

                const auto parentToChildTransform = control->envireGraph_->getTransform(parentFrame, childFrame);
                control->envireGraph_->removeTransform(parentFrame, childFrame);
                control->envireGraph_->addFrame(jointFrameName);
                control->envireGraph_->addTransform(parentFrame, jointFrameName, parentToChildTransform);
                envire::core::Transform identityTransform;
                identityTransform.setIdentity();
                control->envireGraph_->addTransform(jointFrameName, childFrame, identityTransform);
            }

            control->envireGraph_->addItemToFrame(jointFrameName, item);

            constexpr bool sceneWasReseted = false;
            control->sim->sceneHasChanged(sceneWasReseted);

            // id == 0 is invalid indicating getID that no specific id is desired
            const unsigned long desiredId = jointS->config.hasKey("desired_id") ? jointS->config["desired_id"] : 0;
            // TODO: Enable resquesting desired id for joint
            return idManager_->getID(jointS->name);
        }

        int JointManager::getJointCount()
        {
            return idManager_->size();
        }

        void JointManager::editJoint(JointData *jointS)
        {
            if(const auto joint = getJointInterface(jointS->index).lock())
            {
                joint->setAnchor(jointS->anchor);
                joint->setAxis(jointS->axis1);
                joint->setAxis2(jointS->axis2);
                joint->setLowStop(jointS->lowStopAxis1);
                joint->setLowStop2(jointS->lowStopAxis2);
                joint->setHighStop(jointS->highStopAxis1);
                joint->setHighStop2(jointS->highStopAxis2);
            }
        }

        void JointManager::getListJoints(std::vector<core_objects_exchange>* jointList)
        {
            core_objects_exchange obj;
            jointList->clear();
            for(const auto simJoint : getSimJoints())
            {
                simJoint->getCoreExchange(&obj);
                jointList->push_back(obj);
            }
        }

        void JointManager::getJointExchange(unsigned long id, core_objects_exchange* obj)
        {
            const auto simJoint = getSimJoint(id);
            if(simJoint)
            {
                simJoint->getCoreExchange(obj);
            } else
            {
                obj = nullptr;
            }
        }

        const JointData JointManager::getFullJoint(unsigned long index)
        {
            if(const auto joint = getJointInterface(index).lock())
            {
                return constructJointData(joint);
            }
            throw std::runtime_error((std::string{"Could not find joint with index "} + std::to_string(index)).c_str());
        }

        void JointManager::removeJoint(unsigned long index)
        {
            if(const auto jointInterfaceItemPtr = getItemBasePtr(index))
            {
                configmaps::ConfigMap configMap = boost::dynamic_pointer_cast<envire::core::Item<JointInterfaceItem>>(jointInterfaceItemPtr)->getData().jointInterface->getConfigMap();
                const bool b_isFixedJoint = configMap["type"].toString() == "fixed";
                const envire::core::FrameId jointFrameId = constructFrameIdFromJointName(configMap["name"].toString(), b_isFixedJoint);
                if(b_isFixedJoint)
                {
                    // Remove jointinterfaceitem from graph
                    control->envireGraph_->removeItemFromFrame(jointInterfaceItemPtr);

                    // Also remove corresponding envire fixed joint item
                    const auto& range = control->envireGraph_->getItems<envire::core::Item<envire::types::joints::Fixed>>(jointFrameId);
                    for(auto iter = range.first; iter != range.second; ++iter)
                    {
                        if(iter->getData().getName() == configMap["name"].toString())
                        {
                            control->envireGraph_->removeItemFromFrame(jointFrameId, iter);
                        }
                    }
                } else
                {
                    // Remove joint frame and reconnect graph
                    const auto& parentFrameId = configMap["parent_link_name"].toString();
                    const auto& childFrameId = configMap["child_link_name"].toString();
                    const auto& parentToJoint = control->envireGraph_->getTransform(parentFrameId, jointFrameId);
                    const auto& jointToChild = control->envireGraph_->getTransform(jointFrameId, childFrameId);

                    control->envireGraph_->disconnectFrame(jointFrameId);
                    control->envireGraph_->removeFrame(jointFrameId);
                    control->envireGraph_->addTransform(parentFrameId, childFrameId, parentToJoint * jointToChild);
                }
            } else
            {
                throw std::logic_error((std::string{"JointManager::removeJoint: Could not remove joint with index "} + std::to_string(index)).c_str());
            }
            idManager_->removeEntry(index);

            constexpr bool sceneWasReseted = false;
            control->sim->sceneHasChanged(sceneWasReseted);
        }

        void JointManager::removeJointByIDs(unsigned long id1, unsigned long id2)
        {
            const auto id = getIDByNodeIDs(id1, id2);
            if(id != 0) 
            {
                removeJoint(id);
                return;
            }
        }

        std::shared_ptr<mars::core::SimJoint> JointManager::getSimJoint(unsigned long id)
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                const auto simJoint = std::make_shared<core::SimJoint>(control, constructJointData(joint));
                // TODO: 
                // simJoint->setAttachedNodes(node1, node2);
                simJoint->setPhysicalJoint(joint);
                return simJoint;
            }
            return nullptr;
        }

        std::vector<std::shared_ptr<mars::core::SimJoint>> JointManager::getSimJoints(void)
        {
            vector<std::shared_ptr<mars::core::SimJoint>> simJoints;
            simJoints.reserve(idManager_->size());
            for(const auto potentialJoint : getJoints())
            {
                if(const auto& joint = potentialJoint.lock())
                {
                    const auto simJoint = std::make_shared<core::SimJoint>(control, constructJointData(joint));
                    // TODO: 
                    // simJoint->setAttachedNodes(node1, node2);
                    simJoint->setPhysicalJoint(joint);
                    simJoints.emplace_back(std::move(simJoint));
                }
            }
            return simJoints;
        }

        void JointManager::reattacheJoints(unsigned long node_id)
        {
            const auto& nodeName = dynamic_cast<NodeManager*>(control->nodes)->getLinkName(node_id);
            if (!control->envireGraph_->containsFrame(nodeName))
            {
                LOG_WARN(std::string{"JointManager::reattacheJoints: Node named \"" + nodeName + "\" does not represent a frame."}.c_str());
                return;
            }
            std::shared_ptr<envire::core::EnvireGraph> graph = control->envireGraph_;
            auto jointReattachor = [nodeName, graph] (envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
                {
                    const size_t numJoints = graph->getItemCount<envire::core::Item<JointInterfaceItem>>(node);
                    if(numJoints == 0)
                    {
                        return;
                    }

                    const auto typeIndex = std::type_index{typeid(envire::core::Item<mars::interfaces::JointInterfaceItem>)};
                    const auto& items = graph->getItems(node, typeIndex);
                    for(const auto& item : items)
                    {
                        const auto jointItemPtr = boost::dynamic_pointer_cast<envire::core::Item<interfaces::JointInterfaceItem>>(item);
                        auto configMap = jointItemPtr->getData().jointInterface->getConfigMap();
                        const auto parentFrameId = configMap["parent_link_name"].toString();
                        const auto childFrameId = configMap["child_link_name"].toString();
                        if(parentFrameId == nodeName || childFrameId == nodeName)
                        {
                            jointItemPtr->getData().jointInterface->reattacheJoint();
                        }
                    }
                };

            const auto rootVertex = control->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            control->graphTreeView_->visitBfs(rootVertex, jointReattachor);
        }

        void JointManager::reloadJoints(void)
        {
            envire::core::EnvireGraph* const graph = control->envireGraph_.get();
            auto readdFunctor = [&graph](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
            {
                itemReadder<envire::types::joints::Fixed>(graph, node);
                itemReadder<envire::types::joints::Continuous>(graph, node);
                itemReadder<envire::types::joints::Prismatic>(graph, node);
                itemReadder<envire::types::joints::Revolute>(graph, node);
            };

            const auto& rootVertex = control->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            control->graphTreeView_->visitBfs(rootVertex, readdFunctor);
        }

        void JointManager::updateJoints(sReal calc_ms)
        {
            // TODO: Discuss: Can this be dropped or should the interface still be provided?
            throw std::logic_error("updateJoints is not implemented.");
        }

        void JointManager::clearAllJoints(bool clear_all)
        {
            
            envire::core::EnvireGraph* const graph = control->envireGraph_.get();
            auto removeFunctor = [clear_all, &graph](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
            {
                itemRemover<interfaces::JointInterfaceItem>(graph, node);

                // TODO: Discuss how to handle clear_all: Remove frames or only additionally remove envire joint items? Is there even still use for clear_all?
                if (clear_all)
                {
                    // TODO: For non-fixed joints remove frame!
                    itemRemover<envire::types::joints::Fixed>(graph, node);
                    itemRemover<envire::types::joints::Continuous>(graph, node);
                    itemRemover<envire::types::joints::Prismatic>(graph, node);
                    itemRemover<envire::types::joints::Revolute>(graph, node);
                }
            };
            const auto& rootVertex = control->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            control->graphTreeView_->visitBfs(rootVertex, removeFunctor);

            constexpr bool sceneWasReseted = false;
            control->sim->sceneHasChanged(sceneWasReseted);

            idManager_->clear();
        }

        std::list<JointData>::iterator JointManager::getReloadJoint(unsigned long id)
        {
            throw std::logic_error("getReloadJoint not implemented yet");
        }

        void JointManager::setReloadJointOffset(unsigned long id, sReal offset)
        {
            throw std::logic_error("JointManager::setReloadJointOffset is obsolete");
        }

        void JointManager::setReloadJointAxis(unsigned long id, const Vector &axis)
        {
            throw std::logic_error("JointManager::setReloadJointAxis is obsolete");
        }


        void JointManager::scaleReloadJoints(sReal x_factor, sReal y_factor, sReal z_factor)
        {
            // TODO: What to do here
            // list<JointData>::iterator iter;
            // MutexLocker locker(&iMutex);
            // for(iter = simJointsReload.begin(); iter != simJointsReload.end(); iter++) {
            //     iter->anchor.x() *= x_factor;
            //     iter->anchor.y() *= y_factor;
            //     iter->anchor.z() *= z_factor;
            // }
        }


        void JointManager::setJointTorque(unsigned long id, sReal torque)
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                joint->setTorque(torque);
            }
        }


        void JointManager::changeStepSize(void) 
        {
            // This works because the constructed SimJoints contain a reference to their corresponding JointInterface.
            for(const auto simJoint : getSimJoints())
            {
                simJoint->updateStepSize();
            }
        }

        void JointManager::setReloadAnchor(unsigned long id, const Vector &anchor)
        {
            throw std::logic_error("JointManager::setReloadAnchor is obsolete");
        }


        void JointManager::setSDParams(unsigned long id, JointData *sJoint)
        {
            // This works because the constructed SimJoint contains a reference to the corresponding JointInterface.
            const auto simJoint = getSimJoint(id);
            simJoint->setSDParams(sJoint);
        }


        void JointManager::setVelocity(unsigned long id, sReal velocity)
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                joint->setVelocity(velocity);
            }
        }


        void JointManager::setVelocity2(unsigned long id, sReal velocity)
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                joint->setVelocity2(velocity);
            }
        }


        void JointManager::setForceLimit(unsigned long id, sReal max_force, bool first_axis)
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                if(first_axis)
                {
                    joint->setForceLimit(max_force);
                } else
                {
                    joint->setForceLimit2(max_force);
                }
            }
        }


        unsigned long JointManager::getID(const std::string& joint_name) const
        {
            const auto& jointID = idManager_->getID(joint_name);
            if (jointID == INVALID_ID)
            {
                const auto msg = std::string{"JointManager::getID: Can't find joint with the name \""} + joint_name + "\".";
                LOG_ERROR(msg.c_str());
            }
            return jointID;
        }

        std::vector<unsigned long> JointManager::getIDsByNodeID(unsigned long node_id)
        {
            const auto& num_joints = idManager_->size();
            auto jointIds = std::vector<unsigned long>(num_joints);
            const auto& frameId = dynamic_cast<NodeManager*>(control->nodes)->getLinkName(node_id);
            interfaces::ControlCenter *c = control;
            std::shared_ptr<envire::core::EnvireGraph> graph = control->envireGraph_;
            auto jointIdCollector = [&jointIds, frameId, c] (envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
            {
                const auto& numJoints = c->envireGraph_->getItemCount<envire::core::Item<JointInterfaceItem>>(node);
                if(numJoints == 0)
                {
                    return;
                }

                const auto typeIndex = std::type_index{typeid(envire::core::Item<mars::interfaces::JointInterfaceItem>)};
                const auto& items = c->envireGraph_->getItems(node, typeIndex);
                for(const auto item : items)
                {
                    const auto jointItemPtr = boost::dynamic_pointer_cast<envire::core::Item<interfaces::JointInterfaceItem>>(item);
                    auto configMap = jointItemPtr->getData().jointInterface->getConfigMap();
                    const auto parentFrameId = configMap["parent_link_name"].toString();
                    const auto childFrameId = configMap["child_link_name"].toString();
                    if(parentFrameId == frameId || childFrameId == frameId)
                    {
                        std::string jointName;
                        jointItemPtr->getData().jointInterface->getName(&jointName);
                        jointIds.push_back(c->joints->getID(jointName));
                    }
                }
            };

            const auto rootVertex = control->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            control->graphTreeView_->visitBfs(rootVertex, jointIdCollector);

            return jointIds;
        }

        unsigned long JointManager::getIDByNodeIDs(unsigned long id1, unsigned long id2)
        {
            const auto& frameId1 = dynamic_cast<NodeManager*>(control->nodes)->getLinkName(id1);
            const auto& frameId2 = dynamic_cast<NodeManager*>(control->nodes)->getLinkName(id2);
      
            if(const auto joint = getJointInterface(frameId1, frameId2).lock())
            {
                std::string jointName;
                joint->getName(&jointName);
                return idManager_->getID(jointName);
            }
            return 0;
        }

        bool JointManager::getDataBrokerNames(unsigned long id, std::string *groupName, std::string *dataName) const
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                joint->getDataBrokerNames(*groupName, *dataName);
                return true;
            }
            return false;
        }

        void JointManager::setOfflineValue(unsigned long id, sReal value)
        {
            // TODO: If value is too large, warn to avoid potential precision issues.

            if(const auto joint = getJointInterface(id).lock())
            {
                const auto jointType = joint->getType();
                if(jointType == JointType::JOINT_TYPE_HINGE)
                {
                    constexpr bool b_isFixedJoint = false;
                    const double& absoluteRotationRad = value;
                    const double currentRotationRad = static_cast<double>(joint->getPosition()); // in (-pi, pi)
                    const double relativeRotationRad = absoluteRotationRad - currentRotationRad;

                    std::string jointName;
                    joint->getName(&jointName);
                    envire::core::FrameId jointFrameName = constructFrameIdFromJointName(jointName, b_isFixedJoint);
                    const auto jointFrameVertex = control->envireGraph_->getVertex(jointFrameName);

                    Simulator* const simulator = dynamic_cast<Simulator*>(control->sim);
                    simulator->rotateHingeJoint(jointFrameVertex, relativeRotationRad, control->envireGraph_, control->graphTreeView_);
                } else
                {
                    throw std::logic_error((std::string{"JointManager::setOfflineValue can't handle JointType "} + std::to_string(static_cast<int>(jointType))).c_str());
                }
            }
        }

        sReal JointManager::getLowStop(unsigned long id) const
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                return joint->getLowStop();
            }
            return 0.0;
        }

        sReal JointManager::getHighStop(unsigned long id) const
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                return joint->getHighStop();
            }
            return 0.0;
        }

        sReal JointManager::getLowStop2(unsigned long id) const
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                return joint->getLowStop2();
            }
            return 0.0;
        }

        sReal JointManager::getHighStop2(unsigned long id) const 
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                return joint->getHighStop2();
            }
            return 0.0;
        }

        void JointManager::setLowStop(unsigned long id, sReal lowStop)
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                joint->setLowStop(lowStop);
            }
        }

        void JointManager::setHighStop(unsigned long id, sReal highStop)
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                joint->setHighStop(highStop);
            }
        }

        void JointManager::setLowStop2(unsigned long id, sReal lowStop2)
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                joint->setLowStop2(lowStop2);
            }
        }

        void JointManager::setHighStop2(unsigned long id, sReal highStop2)
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                joint->setHighStop2(highStop2);
            }
        }

        void JointManager::edit(interfaces::JointId id, const std::string &key, const std::string &value)
        {
            if(const auto joint = getJointInterface(id).lock())
            {
                joint->edit(key, value);
            }
        }

        configmaps::ConfigMap JointManager::constructEnvireJointConfigMap(const interfaces::JointData& jointData)
        {
            configmaps::ConfigMap result;
            result["name"] = jointData.name;
            switch (jointData.type)
            {
            case JointType::JOINT_TYPE_FIXED:
                result["type"] = "Fixed";
                break;
            case JointType::JOINT_TYPE_HINGE2:
            case JointType::JOINT_TYPE_HINGE:
                result["axis"] = utils::vectorToConfigItem(jointData.axis1);
                result["type"] = "Revolute";
                result["minPosition"] = jointData.lowStopAxis1;
                result["maxPosition"] = jointData.highStopAxis1;
                result["maxEffort"] = 0.0; // TODO!
                result["maxVelocity"] = 0.0; // TODO!
                break;
            case JointType::JOINT_TYPE_SLIDER:
            default:
                throw std::logic_error((std::string{"JointManager::constructEnvireJointConfigMap: Not implemented for joint type "} + interfaces::getJointTypeString(jointData.type)).c_str());
                break;
            }

            return result;
        }

        const interfaces::JointData JointManager::constructJointData(const std::shared_ptr<interfaces::JointInterface> joint)
        {
            std::string jointName;
            joint->getName(&jointName);
            const auto& jointId = idManager_->getID(jointName);

            auto configMap = joint->getConfigMap();
            const auto parentNodeName = configMap["parent_link_name"].toString();
            const auto childNodeName = configMap["child_link_name"].toString();
            const auto& parentNodeId = control->nodes->getID(parentNodeName);
            const auto& childNodeId = control->nodes->getID(childNodeName);

            return JointData::fromJointInterface(joint, jointId, parentNodeId, childNodeId);
        }

        // TODO: This should be handled at a central location
        envire::core::FrameId JointManager::constructFrameIdFromJointName(const std::string& jointName, bool isFixedJoint)
        {
            return envire::core::FrameId{isFixedJoint ? jointName : jointName + "_joint"};
        }

        // TODO: This should be handled at a central location
        envire::core::FrameId JointManager::constructFrameIdFromJointData(const interfaces::JointData& jointData)
        {
            return constructFrameIdFromJointName(jointData.name, isFixedJoint(jointData));
        }

        bool JointManager::isFixedJoint(const interfaces::JointData& jointData)
        {
            return jointData.type == JointType::JOINT_TYPE_FIXED;
        }

        bool JointManager::isFixedJoint(const unsigned int jointId)
        {
            if(const auto joint = getJointInterface(jointId).lock())
            {
                return joint->getType() == JointType::JOINT_TYPE_FIXED;
            }

            return false;
        }

        envire::core::ItemBase::Ptr JointManager::getItemBasePtr(unsigned long jointId)
        {
            const auto jointName = std::string{idManager_->getName(jointId)};
            return getItemBasePtr(jointName);
        }

        envire::core::ItemBase::Ptr JointManager::getItemBasePtr(const std::string& jointName)
        {
            envire::core::ItemBase::Ptr foundItem = nullptr;
            interfaces::ControlCenter *c = control;
            auto jointInterfaceSearchFunctor = [&foundItem, jointName, c](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent) 
            {
                // a item with the given name is already found
                if(foundItem)
                {
                    return;
                }

                // the current frame has no joint items
                const size_t numItems = c->envireGraph_->getItemCount<envire::core::Item<JointInterfaceItem>>(node);
                if(numItems == 0)
                {
                    return;
                }

                const auto typeIndex = std::type_index{typeid(envire::core::Item<mars::interfaces::JointInterfaceItem>)};
                const auto& items = c->envireGraph_->getItems(node, typeIndex);
                for(const auto item : items)
                {
                    std::string currentJointName;
                    auto jointItemPtr = boost::dynamic_pointer_cast<envire::core::Item<interfaces::JointInterfaceItem>>(item);
                    jointItemPtr->getData().jointInterface->getName(&currentJointName);
                    if(jointName == currentJointName)
                    {
                        foundItem = item;
                        return;
                    }
                }
            };

            const auto rootVertex = c->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            c->graphTreeView_->visitBfs(rootVertex, jointInterfaceSearchFunctor);
            return foundItem;
        }

        std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(unsigned long jointId)
        {
            const auto jointName = idManager_->getName(jointId);
            return getJointInterface(jointName);
        }

        const std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(unsigned long jointId) const
        {
            const auto jointName = idManager_->getName(jointId);
            return getJointInterface(jointName);
        }

        std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(const std::string& jointName)
        {
            auto foundJoint = std::shared_ptr<interfaces::JointInterface>{nullptr};
            std::shared_ptr<interfaces::JointInterface> *foundJointPtr = &foundJoint;
            interfaces::ControlCenter *c = control;
            auto jointInterfaceSearchFunctor = [foundJointPtr, jointName, c](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent) 
            {
                // a joint with the given name is already found
                if(*foundJointPtr)
                {
                    return;
                }

                Iterator begin, end;
                boost::tie(begin, end) = c->envireGraph_->getItems<envire::core::Item<interfaces::JointInterfaceItem>>(node);
                for(auto item = begin; item != end; item++)
                {
                    std::string currentJointName;
                    if(!item->getData().jointInterface) {
                        LOG_ERROR("we have an joint interface item but jointInterface is not set...");
                        continue;
                    }
                    item->getData().jointInterface->getName(&currentJointName);
                    if(jointName == currentJointName)
                    {
                        *foundJointPtr = item->getData().jointInterface;
                        return;
                    }
                }
            };

            const auto rootVertex = control->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            control->graphTreeView_->visitBfs(rootVertex, jointInterfaceSearchFunctor);
            return foundJoint;
        }

        std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(const envire::core::FrameId& linkedFrame0, const envire::core::FrameId& linkedFrame1) const
        {
            // Ensure that frames are directly connected.
            const bool parent0 = control->envireGraph_->containsEdge(linkedFrame0, linkedFrame1);
            const bool parent1 = control->envireGraph_->containsEdge(linkedFrame1, linkedFrame0);
            if(!parent0 && !parent1)
            {
                throw std::logic_error((std::string{"Tried getting Joint between unconnected frames \""} + linkedFrame0 + "\" and \"" + linkedFrame1 + "\".").c_str());
            }

            const auto& parentFrame = parent0 ? linkedFrame0 : linkedFrame1;
            const auto& childFrame = parent0 ? linkedFrame1 : linkedFrame0;

            const auto& range = control->envireGraph_->getItems<envire::core::Item<interfaces::JointInterfaceItem>>(childFrame);
            for(auto item = range.first; item != range.second; item++)
            {
                auto configMap = item->getData().jointInterface->getConfigMap();
                if(configMap["parent_link_name"] == std::string{parentFrame})
                {
                    // This assumes there is maximally one joint between each pair of frames.
                    return item->getData().jointInterface;
                }
            }

            throw std::logic_error((std::string{"There is no Joint between the frames \""} + linkedFrame0 + "\" and \"" + linkedFrame1 + "\".").c_str());
        }

        const std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(const std::string& jointName) const
        {
            auto foundJoint = std::shared_ptr<interfaces::JointInterface>{nullptr};
            std::shared_ptr<interfaces::JointInterface> *foundJointPtr = &foundJoint;
            const interfaces::ControlCenter *c = control;
            auto jointInterfaceSearchFunctor = [foundJointPtr, jointName, c](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent) 
            {
                // a joint with the given name is already found
                if(*foundJointPtr)
                {
                    return;
                }

                Iterator begin, end;
                boost::tie(begin, end) = c->envireGraph_->getItems<envire::core::Item<interfaces::JointInterfaceItem>>(node);
                for(auto item = begin; item != end; item++)
                {
                    std::string currentJointName;
                    if(!item->getData().jointInterface) {
                        LOG_ERROR("we have an joint interface item but jointInterface is not set...");
                        continue;
                    }
                    item->getData().jointInterface->getName(&currentJointName);
                    if(jointName == currentJointName)
                    {
                        *foundJointPtr = item->getData().jointInterface;
                        return;
                    }
                }
            };

            const auto rootVertex = control->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            control->graphTreeView_->visitBfs(rootVertex, jointInterfaceSearchFunctor);
            return foundJoint;
        }

        std::list<std::weak_ptr<interfaces::JointInterface>> JointManager::getJoints()
        {
            std::list<std::weak_ptr<interfaces::JointInterface>> joints;
            interfaces::ControlCenter *c = control;
            auto jointCollector = [&joints, c](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
            {
                const auto& numItems = c->envireGraph_->getItemCount<envire::core::Item<JointInterfaceItem>>(node);
                if(numItems == 0)
                {
                    return;
                }

                const auto typeIndex = std::type_index{typeid(envire::core::Item<mars::interfaces::JointInterfaceItem>)};
                const auto& items = c->envireGraph_->getItems(node, typeIndex);
                for(const auto item : items)
                {
                    const auto jointItemPtr = boost::dynamic_pointer_cast<envire::core::Item<interfaces::JointInterfaceItem>>(item);
                    joints.emplace_back(jointItemPtr->getData().jointInterface);
                }
            };

            const auto rootVertex = control->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            control->graphTreeView_->visitBfs(rootVertex, jointCollector);
            return joints;
        }
    } // end of namespace core
} // end of namespace mars
