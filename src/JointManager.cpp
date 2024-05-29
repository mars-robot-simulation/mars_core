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

        /**
         *\brief Initialization of a new JointManager
         *
         * pre:
         *     - a pointer to a ControlCenter is needed
         * post:
         *     - next_node_id should be initialized to one
         */
        JointManager::JointManager(ControlCenter *c) : control(c) {}

        unsigned long JointManager::addJoint(JointData *jointS, bool reload)
        {
            auto jointMap = constructEnvireJointConfigMap(*jointS);

            // TODO: This should be handled at a central location
            const auto className = std::string{JOINT_NAMESPACE} + jointMap["type"].toString();
            envire::core::ItemBase::Ptr item = envire::types::TypeCreatorFactory::createItem(className, jointMap);

            const envire::core::FrameId parentFrame = ControlCenter::linkIDManager->getName(jointS->nodeIndex1);
            const envire::core::FrameId childFrame = ControlCenter::linkIDManager->getName(jointS->nodeIndex2);

            if(!ControlCenter::envireGraph->containsFrame(parentFrame))
            {
                throw std::logic_error((std::string{"JointManager::addJoint: Tried adding joint linked to non-existant link "} + std::string{parentFrame}).c_str());
            }
            if(!ControlCenter::envireGraph->containsFrame(childFrame))
            {
                throw std::logic_error((std::string{"JointManager::addJoint: Tried adding joint linked to non-existant link "} + std::string{childFrame}).c_str());
            }

            const envire::core::FrameId jointFrameName = constructFrameIdFromJointData(*jointS);
            if(!isFixedJoint(*jointS))
            {
                // Insert new frame for non-fixed joints
                if(ControlCenter::envireGraph->containsFrame(jointFrameName))
                {
                    throw std::logic_error((std::string{"JointManager::addJoint: There is already a joint frame with the name "} + std::string{jointFrameName}).c_str());
                }

                const auto parentToChildTransform = ControlCenter::envireGraph->getTransform(parentFrame, childFrame);
                ControlCenter::envireGraph->removeTransform(parentFrame, childFrame);
                ControlCenter::envireGraph->addFrame(jointFrameName);
                ControlCenter::envireGraph->addTransform(parentFrame, jointFrameName, parentToChildTransform);
                envire::core::Transform identityTransform;
                identityTransform.setIdentity();
                ControlCenter::envireGraph->addTransform(jointFrameName, childFrame, identityTransform);
            }

            ControlCenter::envireGraph->addItemToFrame(jointFrameName, item);

            constexpr bool sceneWasReseted = false;
            control->sim->sceneHasChanged(sceneWasReseted);

            // id == 0 is invalid indicating getID that no specific id is desired
            const unsigned long desiredId = jointS->config.hasKey("desired_id") ? jointS->config["desired_id"] : 0;

            return ControlCenter::jointIDManager->addIfUnknown(jointS->frameID, desiredId);
        }

        int JointManager::getJointCount()
        {
            return ControlCenter::jointIDManager->size();
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
                    ControlCenter::envireGraph->removeItemFromFrame(jointInterfaceItemPtr);

                    // Also remove corresponding envire fixed joint item
                    const auto& range = ControlCenter::envireGraph->getItems<envire::core::Item<envire::types::joints::Fixed>>(jointFrameId);
                    for(auto iter = range.first; iter != range.second; ++iter)
                    {
                        if(iter->getData().name == configMap["name"].toString())
                        {
                            ControlCenter::envireGraph->removeItemFromFrame(jointFrameId, iter);
                        }
                    }
                } else
                {
                    // Remove joint frame and reconnect graph
                    const auto& parentFrameId = configMap["parent_link_name"].toString();
                    const auto& childFrameId = configMap["child_link_name"].toString();
                    const auto& parentToJoint = ControlCenter::envireGraph->getTransform(parentFrameId, jointFrameId);
                    const auto& jointToChild = ControlCenter::envireGraph->getTransform(jointFrameId, childFrameId);

                    ControlCenter::envireGraph->disconnectFrame(jointFrameId);
                    ControlCenter::envireGraph->removeFrame(jointFrameId);
                    ControlCenter::envireGraph->addTransform(parentFrameId, childFrameId, parentToJoint * jointToChild);
                }
            } else
            {
                throw std::logic_error((std::string{"JointManager::removeJoint: Could not remove joint with index "} + std::to_string(index)).c_str());
            }
            ControlCenter::jointIDManager->removeEntry(index);

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
            simJoints.reserve(ControlCenter::jointIDManager->size());
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
            const auto& nodeName = ControlCenter::linkIDManager->getName(node_id);
            if (!ControlCenter::envireGraph->containsFrame(nodeName))
            {
                LOG_WARN(std::string{"JointManager::reattacheJoints: Node named \"" + nodeName + "\" does not represent a frame."}.c_str());
                return;
            }
            auto jointReattachor = [nodeName] (envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
                {
                    const size_t numJoints = ControlCenter::envireGraph->getItemCount<envire::core::Item<JointInterfaceItem>>(node);
                    if(numJoints == 0)
                    {
                        return;
                    }

                    const auto typeIndex = std::type_index{typeid(envire::core::Item<mars::interfaces::JointInterfaceItem>)};
                    const auto& items = ControlCenter::envireGraph->getItems(node, typeIndex);
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

            const auto rootVertex = ControlCenter::envireGraph->getVertex(SIM_CENTER_FRAME_NAME);
            ControlCenter::graphTreeView->visitBfs(rootVertex, jointReattachor);
        }

        void JointManager::reloadJoints(void)
        {
            auto readdFunctor = [](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
            {
                itemReadder<envire::types::joints::Fixed>(node);
                itemReadder<envire::types::joints::Continuous>(node);
                itemReadder<envire::types::joints::Prismatic>(node);
                itemReadder<envire::types::joints::Revolute>(node);
            };

            const auto& rootVertex = ControlCenter::envireGraph->getVertex(SIM_CENTER_FRAME_NAME);
            ControlCenter::graphTreeView->visitBfs(rootVertex, readdFunctor);
        }

        void JointManager::updateJoints(sReal calc_ms)
        {
            // TODO: Discuss: Can this be dropped or should the interface still be provided?
            throw std::logic_error("updateJoints is not implemented.");
        }

        void JointManager::clearAllJoints(bool clear_all)
        {
            auto removeFunctor = [clear_all](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
            {
                itemRemover<interfaces::JointInterfaceItem>(node);

                // TODO: Discuss how to handle clear_all: Remove frames or only additionally remove envire joint items? Is there even still use for clear_all?
                if (clear_all)
                {
                    // TODO: For non-fixed joints remove frame!
                    itemRemover<envire::types::joints::Fixed>(node);
                    itemRemover<envire::types::joints::Continuous>(node);
                    itemRemover<envire::types::joints::Prismatic>(node);
                    itemRemover<envire::types::joints::Revolute>(node);
                }
            };
            const auto& rootVertex = ControlCenter::envireGraph->getVertex(SIM_CENTER_FRAME_NAME);
            ControlCenter::graphTreeView->visitBfs(rootVertex, removeFunctor);

            constexpr bool sceneWasReseted = false;
            control->sim->sceneHasChanged(sceneWasReseted);

            ControlCenter::jointIDManager->clear();
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
            return ControlCenter::jointIDManager->getID(joint_name);
        }

        std::vector<unsigned long> JointManager::getIDsByNodeID(unsigned long node_id)
        {
            const auto& num_joints = ControlCenter::jointIDManager->size();
            auto jointIds = std::vector<unsigned long>(num_joints);
            const auto& frameId = ControlCenter::linkIDManager->getName(node_id);
            auto jointIdCollector = [&jointIds, frameId] (envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
            {
                const auto& numJoints = ControlCenter::envireGraph->getItemCount<envire::core::Item<JointInterfaceItem>>(node);
                if(numJoints == 0)
                {
                    return;
                }

                const auto typeIndex = std::type_index{typeid(envire::core::Item<mars::interfaces::JointInterfaceItem>)};
                const auto& items = ControlCenter::envireGraph->getItems(node, typeIndex);
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
                        jointIds.push_back(ControlCenter::jointIDManager->getID(jointName));
                    }
                }
            };

            const auto rootVertex = ControlCenter::envireGraph->getVertex(SIM_CENTER_FRAME_NAME);
            ControlCenter::graphTreeView->visitBfs(rootVertex, jointIdCollector);

            return jointIds;
        }

        unsigned long JointManager::getIDByNodeIDs(unsigned long id1, unsigned long id2)
        {
            const auto& frameId1 = ControlCenter::linkIDManager->getName(id1);
            const auto& frameId2 = ControlCenter::linkIDManager->getName(id2);
      
            if(const auto joint = getJointInterface(frameId1, frameId2).lock())
            {
                std::string jointName;
                joint->getName(&jointName);
                return ControlCenter::jointIDManager->getID(jointName);
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
                    const auto jointFrameVertex = ControlCenter::envireGraph->getVertex(jointFrameName);

                    Simulator* const simulator = dynamic_cast<Simulator*>(control->sim);
                    simulator->rotateHingeJoint(jointFrameVertex, relativeRotationRad, ControlCenter::envireGraph, ControlCenter::graphTreeView);
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
            const auto& jointId = ControlCenter::jointIDManager->getID(jointName);

            auto configMap = joint->getConfigMap();
            const auto parentNodeName = configMap["parent_link_name"].toString();
            const auto childNodeName = configMap["child_link_name"].toString();
            const auto& parentNodeId = ControlCenter::linkIDManager->getID(parentNodeName);
            const auto& childNodeId = ControlCenter::linkIDManager->getID(childNodeName);

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
            const auto jointName = std::string{ControlCenter::jointIDManager->getName(jointId)};
            return getItemBasePtr(jointName);
        }

        envire::core::ItemBase::Ptr JointManager::getItemBasePtr(const std::string& jointName)
        {
            envire::core::ItemBase::Ptr foundItem = nullptr;
            auto jointInterfaceSearchFunctor = [&foundItem, jointName](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent) 
            {
                // a item with the given name is already found
                if(foundItem)
                {
                    return;
                }

                // the current frame has no joint items
                const size_t numItems = ControlCenter::envireGraph->getItemCount<envire::core::Item<JointInterfaceItem>>(node);
                if(numItems == 0)
                {
                    return;
                }

                const auto typeIndex = std::type_index{typeid(envire::core::Item<mars::interfaces::JointInterfaceItem>)};
                const auto& items = ControlCenter::envireGraph->getItems(node, typeIndex);
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

            const auto rootVertex = ControlCenter::envireGraph->getVertex(SIM_CENTER_FRAME_NAME);
            ControlCenter::graphTreeView->visitBfs(rootVertex, jointInterfaceSearchFunctor);
            return foundItem;
        }

        std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(unsigned long jointId)
        {
            const auto jointName = ControlCenter::jointIDManager->getName(jointId);
            return getJointInterface(jointName);
        }

        std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(const std::string& jointName)
        {
            auto foundJoint = std::shared_ptr<interfaces::JointInterface>{nullptr};
            auto jointInterfaceSearchFunctor = [&foundJoint, jointName](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent) 
            {
                // a joint with the given name is already found
                if(foundJoint)
                {
                    return;
                }

                const auto& range = ControlCenter::envireGraph->getItems<envire::core::Item<interfaces::JointInterfaceItem>>(node);
                for(auto item = range.first; item != range.second; item++)
                {
                    std::string currentJointName;
                    item->getData().jointInterface->getName(&currentJointName);
                    if(jointName == currentJointName)
                    {
                        foundJoint = item->getData().jointInterface;
                        return;
                    }
                }
            };

            const auto rootVertex = ControlCenter::envireGraph->getVertex(SIM_CENTER_FRAME_NAME);
            ControlCenter::graphTreeView->visitBfs(rootVertex, jointInterfaceSearchFunctor);
            return foundJoint;
        }

        std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(const envire::core::FrameId& linkedFrame0, const envire::core::FrameId& linkedFrame1)
        {
            // Ensure that frames are directly connected.
            const bool parent0 = ControlCenter::envireGraph->containsEdge(linkedFrame0, linkedFrame1);
            const bool parent1 = ControlCenter::envireGraph->containsEdge(linkedFrame1, linkedFrame0);
            if(!parent0 && !parent1)
            {
                throw std::logic_error((std::string{"Tried getting Joint between unconnected frames \""} + linkedFrame0 + "\" and \"" + linkedFrame1 + "\".").c_str());
            }

            const auto& parentFrame = parent0 ? linkedFrame0 : linkedFrame1;
            const auto& childFrame = parent0 ? linkedFrame1 : linkedFrame0;

            const auto& range = ControlCenter::envireGraph->getItems<envire::core::Item<interfaces::JointInterfaceItem>>(childFrame);
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

        std::list<std::weak_ptr<interfaces::JointInterface>> JointManager::getJoints()
        {
            std::list<std::weak_ptr<interfaces::JointInterface>> joints;
            auto jointCollector = [&joints](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
            {
                const auto& numItems = ControlCenter::envireGraph->getItemCount<envire::core::Item<JointInterfaceItem>>(node);
                if(numItems == 0)
                {
                    return;
                }

                const auto typeIndex = std::type_index{typeid(envire::core::Item<mars::interfaces::JointInterfaceItem>)};
                const auto& items = ControlCenter::envireGraph->getItems(node, typeIndex);
                for(const auto item : items)
                {
                    const auto jointItemPtr = boost::dynamic_pointer_cast<envire::core::Item<interfaces::JointInterfaceItem>>(item);
                    joints.emplace_back(jointItemPtr->getData().jointInterface);
                }
            };

            const auto rootVertex = ControlCenter::envireGraph->getVertex(SIM_CENTER_FRAME_NAME);
            ControlCenter::graphTreeView->visitBfs(rootVertex, jointCollector);
            return joints;
        }
    } // end of namespace core
} // end of namespace mars
