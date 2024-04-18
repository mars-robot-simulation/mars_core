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

#ifndef SIM_CENTER_FRAME_NAME
#define SIM_CENTER_FRAME_NAME "world"
#endif

#include <stdexcept>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>
#include <envire_core/graph/GraphTypes.hpp>

#include <data_broker/DataBrokerInterface.h>
#include <mars_interfaces/sim/JointInterface.h>
#include <mars_interfaces/sim/MotorManagerInterface.h>

#include <mars_utils/misc.h>
#include <mars_utils/mathUtils.h>
#include <mars_utils/MutexLocker.h>
#include <mars_interfaces/Logging.hpp>

// TODO: Clean up mutexes!
//    * Graph should be locked globally when it is being worked on
//    * IDManager analogously

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
      const MutexLocker locker{&iMutex};
      throw std::logic_error("addJoint is not implemented yet");
      // TODO Add suited envire joint item in graph; envire_mars_ode_phyics will do the rest

      if (jointS->config.hasKey("desired_id"))
      {
        // TODO: Enable setting desired id
        //return ControlCenter::jointIDManager->getID(jointS->name, jointS->config["desired_id"]);
      }
      return ControlCenter::jointIDManager->getID(jointS->name);
    }

    int JointManager::getJointCount()
    {
      const MutexLocker locker{&iMutex};
      return ControlCenter::jointIDManager->size();
    }

    void JointManager::editJoint(JointData *jointS)
    {
      if (const auto joint = getJointInterface(jointS->index).lock())
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
      for (const auto simJoint : getSimJoints())
      {
        simJoint->getCoreExchange(&obj);
        jointList->push_back(obj);
      }
    }

    void JointManager::getJointExchange(unsigned long id, core_objects_exchange* obj)
    {
      const auto simJoint = getSimJoint(id);
      if (simJoint)
      {
        simJoint->getCoreExchange(obj);
      }
      else
      {
        obj = nullptr;
      }
    }

    const JointData JointManager::getFullJoint(unsigned long index)
    {
      if (const auto joint = getJointInterface(index).lock())
      {
        return constructJointData(joint);
      }
      throw std::runtime_error((std::string{"Could not find joint with index "} + std::to_string(index)).c_str());
    }

    void JointManager::removeJoint(unsigned long index)
    {
      const MutexLocker locker{&iMutex};
      const auto jointInterfaceItemPtr = getItemBasePtr(index);
      ControlCenter::jointIDManager->removeEntry(index);
      ControlCenter::envireGraph->removeItemFromFrame(jointInterfaceItemPtr);

      // TODO: Remove envire joint item?
      // TODO: Scene changed?
      //  control->sim->sceneHasChanged(false);
    }

    void JointManager::removeJointByIDs(unsigned long id1, unsigned long id2)
    {
      unsigned long id = getIDByNodeIDs(id1, id2);
      if (id != 0) 
      {
        removeJoint(id);
        return;
      }
    }

    std::shared_ptr<mars::sim::SimJoint> JointManager::getSimJoint(unsigned long id)
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        const auto simJoint = std::make_shared<sim::SimJoint>(control, constructJointData(joint));
        // simJoint->setAttachedNodes(node1, node2);
        simJoint->setPhysicalJoint(joint);
        return simJoint;
      }
      return nullptr;
    }

    std::vector<std::shared_ptr<mars::sim::SimJoint>> JointManager::getSimJoints(void)
    {
      const MutexLocker locker{&iMutex};
      vector<std::shared_ptr<mars::sim::SimJoint>> simJoints;
      simJoints.reserve(ControlCenter::jointIDManager->size());
      for (const auto potentialJoint : getJoints())
      {
        if (const auto joint = potentialJoint.lock())
        {
          const auto simJoint = std::make_shared<sim::SimJoint>(control, constructJointData(joint));
          // simJoint->setAttachedNodes(node1, node2);
          simJoint->setPhysicalJoint(joint);
          simJoints.emplace_back(std::move(simJoint));
        }
      }
      return simJoints;
    }

    void JointManager::reattacheJoints(unsigned long node_id)
    {
      throw std::logic_error("reattacheJoints not implemented yet");
      // map<unsigned long, std::shared_ptr<SimJoint>>::iterator iter;
      // MutexLocker locker(&iMutex);
      // for (iter = simJoints.begin(); iter != simJoints.end(); iter++) {
      //   if (iter->second->getSJoint().nodeIndex1 == node_id ||
      //       iter->second->getSJoint().nodeIndex2 == node_id) {
      //     iter->second->reattachJoint();
      //   }
      // }
    }

    void JointManager::reloadJoints(void)
    {
      list<JointData>::iterator iter;
      throw std::logic_error("reloadJoints not implemented yet");
      // for(iter = simJointsReload.begin(); iter != simJointsReload.end(); iter++)
      //   addJoint(&(*iter), true);
    }

    void JointManager::updateJoints(sReal calc_ms)
    {
      // TODO: Discuss: Can this be dropped or should the interface still be provided?
      throw std::logic_error("updateJoints is not implemented.");
    }

    void JointManager::clearAllJoints(bool clear_all)
    {
      throw std::logic_error("clearAllJoints not implemented yet");
      // map<unsigned long, std::shared_ptr<SimJoint>>::iterator iter;
      // MutexLocker locker(&iMutex);
      // if(clear_all) simJointsReload.clear();

      // while(!simJoints.empty()) {
      //   control->motors->removeJointFromMotors(simJoints.begin()->first);
      //   simJoints.begin()->second.reset();
      //   simJoints.erase(simJoints.begin());
      // }
      // control->sim->sceneHasChanged(false);

      // next_joint_id = 1;
    }

    std::list<JointData>::iterator JointManager::getReloadJoint(unsigned long id)
    {

      throw std::logic_error("getReloadJoint not implemented yet");
      // std::list<JointData>::iterator iter = simJointsReload.begin();
      // for(;iter!=simJointsReload.end(); ++iter) {
      //   if(iter->index == id) break;
      // }
      // return iter;
    }

    void JointManager::setReloadJointOffset(unsigned long id, sReal offset)
    {
      MutexLocker locker(&iMutex);
      list<JointData>::iterator iter = getReloadJoint(id);
      if (iter != simJointsReload.end())
        iter->angle1_offset = offset;
    }

    void JointManager::setReloadJointAxis(unsigned long id, const Vector &axis)
    {
      MutexLocker locker(&iMutex);
      list<JointData>::iterator iter = getReloadJoint(id);
      if (iter != simJointsReload.end())
        iter->axis1 = axis;
    }


    void JointManager::scaleReloadJoints(sReal x_factor, sReal y_factor, sReal z_factor)
    {
      list<JointData>::iterator iter;
      MutexLocker locker(&iMutex);
      for(iter = simJointsReload.begin(); iter != simJointsReload.end(); iter++) {
        iter->anchor.x() *= x_factor;
        iter->anchor.y() *= y_factor;
        iter->anchor.z() *= z_factor;
      }
    }


    void JointManager::setJointTorque(unsigned long id, sReal torque)
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        joint->setTorque(torque);
      }
    }


    void JointManager::changeStepSize(void) 
    {
      // This works because the constructed SimJoints contain a reference to their corresponding JointInterface.
      for (const auto simJoint : getSimJoints())
      {
        simJoint->updateStepSize();
      }
    }

    void JointManager::setReloadAnchor(unsigned long id, const Vector &anchor)
    {
      MutexLocker locker(&iMutex);
      list<JointData>::iterator iter = getReloadJoint(id);
      if (iter != simJointsReload.end())
        iter->anchor = anchor;
    }


    void JointManager::setSDParams(unsigned long id, JointData *sJoint)
    {
      // This works because the constructed SimJoint contains a reference to the corresponding JointInterface.
      const auto simJoint = getSimJoint(id);
      simJoint->setSDParams(sJoint);
    }


    void JointManager::setVelocity(unsigned long id, sReal velocity)
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        joint->setVelocity(velocity);
      }
    }


    void JointManager::setVelocity2(unsigned long id, sReal velocity)
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        joint->setVelocity2(velocity);
      }
    }


    void JointManager::setForceLimit(unsigned long id, sReal max_force, bool first_axis)
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        if (first_axis)
        {
          joint->setForceLimit(max_force);
        }
        else
        {
          joint->setForceLimit2(max_force);
        }
      }
    }


    unsigned long JointManager::getID(const std::string& joint_name) const
    {
      const MutexLocker locker{&iMutex};
      return ControlCenter::jointIDManager->getID(joint_name);
    }

    std::vector<unsigned long> JointManager::getIDsByNodeID(unsigned long node_id)
    {
      throw std::logic_error("getIDsByNodeID not implemented yet");
      // map<unsigned long, std::shared_ptr<SimJoint>>::iterator iter;
      // MutexLocker locker(&iMutex);
      std::vector<unsigned long> out;
      // for (iter = simJoints.begin(); iter != simJoints.end(); iter++)
      //   if((iter->second->getNodeId() == node_id ||
      //       iter->second->getNodeId(2) == node_id) ||
      //      (iter->second->getNodeId() == node_id ||
      //       iter->second->getNodeId(2) == node_id)) {
      //     out.push_back(iter->first);
      //   }
      return out;
    }

    unsigned long JointManager::getIDByNodeIDs(unsigned long id1, unsigned long id2)
    {
      const envire::core::FrameId frameId1 = ControlCenter::nodeIDManager->getName(id1);
      const envire::core::FrameId frameId2 = ControlCenter::nodeIDManager->getName(id2);
      
      const MutexLocker locker{&iMutex};
      if (const auto joint = getJointInterface(frameId1, frameId2).lock())
      {
        std::string jointName;
        joint->getName(&jointName);
        return ControlCenter::jointIDManager->getID(jointName);
      }
      return 0;
    }

    bool JointManager::getDataBrokerNames(unsigned long id, std::string *groupName, std::string *dataName) const
    {
      const MutexLocker locker{&iMutex};
      if (const auto joint = getJointInterface(id).lock())
      {
        std::string jointName;
        joint->getName(&jointName);
        *groupName = "mars_sim";
        *dataName = constructDataBrokerName(id, jointName);
        return true;
      }
      return false;
    }

    void JointManager::setOfflineValue(unsigned long id, sReal value)
    {
      const MutexLocker locker{&iMutex};
      if (const auto joint = getJointInterface(id).lock())
      {
        joint->setOfflinePosition(value);
      }
    }

    sReal JointManager::getLowStop(unsigned long id) const
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        return joint->getLowStop();
      }
      return 0.0;
    }

    sReal JointManager::getHighStop(unsigned long id) const
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        return joint->getHighStop();
      }
      return 0.0;
    }

    sReal JointManager::getLowStop2(unsigned long id) const
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        return joint->getLowStop2();
      }
      return 0.0;
    }

    sReal JointManager::getHighStop2(unsigned long id) const 
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        return joint->getHighStop2();
      }
      return 0.0;
    }

    void JointManager::setLowStop(unsigned long id, sReal lowStop)
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        joint->setLowStop(lowStop);
      }
    }

    void JointManager::setHighStop(unsigned long id, sReal highStop)
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        joint->setHighStop(highStop);
      }
    }

    void JointManager::setLowStop2(unsigned long id, sReal lowStop2)
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        joint->setLowStop2(lowStop2);
      }
    }

    void JointManager::setHighStop2(unsigned long id, sReal highStop2)
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        joint->setHighStop2(highStop2);
      }
    }

    void JointManager::edit(interfaces::JointId id, const std::string &key, const std::string &value)
    {
      if (const auto joint = getJointInterface(id).lock())
      {
        joint->edit(key, value);
      }
    }

    // TODO: Discuss: Is the format still up to date?
    // TODO: Refactor to make more readable.
    std::string JointManager::constructDataBrokerName(const unsigned int jointId, const std::string& jointName)
    {
      char format[] = "Joints/%05lu_%s";
      int size = snprintf(0, 0, format, jointId, jointName.c_str());
      char buffer[size+1];
      sprintf(buffer, format, jointId, jointName.c_str());
      return buffer;
    }

    const interfaces::JointData JointManager::constructJointData(const std::shared_ptr<interfaces::JointInterface> joint)
    {
      std::string jointName;
      joint->getName(&jointName);
      const unsigned int jointId = ControlCenter::jointIDManager->getID(jointName);

      auto configMap = joint->getConfigMap();
      const std::string parentNodeName = configMap["parent_link_name"];
      const std::string childNodeName = configMap["child_link_name"];
      const unsigned int parentNodeId = ControlCenter::nodeIDManager->getID(parentNodeName);
      const unsigned int childNodeId = ControlCenter::nodeIDManager->getID(childNodeName);

      return JointData::fromJointInterface(joint, jointId, parentNodeId, childNodeId);
    }

    envire::core::ItemBase::Ptr JointManager::getItemBasePtr(unsigned long jointId) const
    {
      const std::string jointName{ControlCenter::jointIDManager->getName(jointId)};
      return getItemBasePtr(jointName);
    }

    envire::core::ItemBase::Ptr JointManager::getItemBasePtr(const std::string& jointName) const
    {
      envire::core::ItemBase::Ptr foundItem = nullptr;
      auto jointInterfaceSearchFunctor = [&foundItem, jointName](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent) 
      {
        // a item with the given name is already found
        if (foundItem)
        {
          return;
        }

        // the current frame has no joint items
        const size_t numItems = ControlCenter::envireGraph->getItemCount<envire::core::Item<JointInterfaceItem>>(node);
        if (numItems == 0)
        {
          return;
        }

        const std::type_index typeIndex{typeid(envire::core::Item<mars::interfaces::JointInterfaceItem>)};
        const auto& items = ControlCenter::envireGraph->getItems(node, typeIndex);
        for (const auto item : items)
        {
          std::string currentJointName;
          auto jointItemPtr = boost::dynamic_pointer_cast<envire::core::Item<interfaces::JointInterfaceItem>>(item);
          jointItemPtr->getData().jointInterface->getName(&currentJointName);
          if (jointName == currentJointName)
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

    std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(unsigned long jointId) const
    {
      const std::string jointName{ControlCenter::jointIDManager->getName(jointId)};
      return getJointInterface(jointName);
    }

    std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(const std::string& jointName) const
    {
      std::shared_ptr<interfaces::JointInterface> foundJoint;
      auto jointInterfaceSearchFunctor = [&foundJoint, jointName](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent) 
      {
        // a joint with the given name is already found
        if (foundJoint)
        {
          return;
        }

        const auto& range = ControlCenter::envireGraph->getItems<envire::core::Item<interfaces::JointInterfaceItem>>(node);
        for (auto item = range.first; item != range.second; item++)
        {
          std::string currentJointName;
          item->getData().jointInterface->getName(&currentJointName);
          if (jointName == currentJointName)
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

    std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(const envire::core::FrameId& linkedFrame0, const envire::core::FrameId& linkedFrame1) const
    {
      // Ensure that frames are directly connected.
      const bool parent0 = ControlCenter::envireGraph->containsEdge(linkedFrame0, linkedFrame1);
      const bool parent1 = ControlCenter::envireGraph->containsEdge(linkedFrame1, linkedFrame0);
      if (!parent0 && !parent1)
      {
        throw std::logic_error((std::string{"Tried getting Joint between unconnected frames \""} + linkedFrame0 + "\" and \"" + linkedFrame1 + "\".").c_str());
      }

      const envire::core::FrameId& parentFrame = parent0 ? linkedFrame0 : linkedFrame1;
      const envire::core::FrameId& childFrame = parent0 ? linkedFrame1 : linkedFrame0;

      const auto& range = ControlCenter::envireGraph->getItems<envire::core::Item<interfaces::JointInterfaceItem>>(childFrame);
      for (auto item = range.first; item != range.second; item++)
      {
        auto configMap = item->getData().jointInterface->getConfigMap();
        if (configMap["parent_link_name"] == std::string{parentFrame})
        {
          // This assumes there is maximally one joint between each pair of frames.
          return item->getData().jointInterface;
        }
      }

      throw std::logic_error((std::string{"There is no Joint between the frames \""} + linkedFrame0 + "\" and \"" + linkedFrame1 + "\".").c_str());
    }

    std::list<std::weak_ptr<interfaces::JointInterface>> JointManager::getJoints() const
    {
      std::list<std::weak_ptr<interfaces::JointInterface>> joints;
      auto jointCollector = [&joints](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
      {
        const size_t numItems = ControlCenter::envireGraph->getItemCount<envire::core::Item<JointInterfaceItem>>(node);
        if (numItems == 0)
        {
          return;
        }

        const std::type_index typeIndex{typeid(envire::core::Item<mars::interfaces::JointInterfaceItem>)};
        const auto& items = ControlCenter::envireGraph->getItems(node, typeIndex);
        for (const auto item : items)
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