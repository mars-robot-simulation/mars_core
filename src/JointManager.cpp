/**
 * \file JointManager.cpp
 * \author  Malte Langosz, Julian Liersch
 * \brief "JointManager" implements the JointManagerInterface.
 * It is manages all joints and all joint
 * operations that are used for the communication between the simulation
 * modules.
 */

#include "JointManager.hpp"

#include <stdexcept>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>
#include <envire_core/graph/GraphTypes.hpp>

#include <data_broker/DataBrokerInterface.h>
#include <mars_interfaces/sim/JointInterface.h>

#include <mars_utils/misc.h>
#include <mars_utils/mathUtils.h>
#include <mars_utils/MutexLocker.h>
#include <mars_interfaces/Logging.hpp>

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
      // TODO Add suited envire joint item in graph; envire_mars_ode_phyics will do the rest
      return ControlCenter::jointIDManager->getID(jointS->name);
    }

    int JointManager::getJointCount()
    {
      return ControlCenter::jointIDManager->size();
    }

    void JointManager::editJoint(JointData *jointS)
    {
      MutexLocker locker(&iMutex);
      throw std::logic_error("editJoint not implemented yet");
      // TODO: Find by name in graph; edit if found
      // std::map<unsigned long, std::shared_ptr<SimJoint>>::iterator iter = simJoints.find(jointS->index);
      // if (iter != simJoints.end()) {
      //   iter->second->setAnchor(jointS->anchor);
      //   iter->second->setAxis(jointS->axis1);
      //   iter->second->setAxis(jointS->axis2, 2);
      //   iter->second->setLowerLimit(jointS->lowStopAxis1);
      //   iter->second->setUpperLimit(jointS->highStopAxis1);
      //   iter->second->setLowerLimit(jointS->lowStopAxis2, 2);
      //   iter->second->setUpperLimit(jointS->highStopAxis2, 2);
      // }
    }

    void JointManager::getListJoints(std::vector<core_objects_exchange>* jointList)
    {
      core_objects_exchange obj;

      throw std::logic_error("getListJoints not implemented yet");
      // std::map<unsigned long, std::shared_ptr<SimJoint>>::iterator iter;
      // MutexLocker locker(&iMutex);
      // jointList->clear();
      // for (iter = simJoints.begin(); iter != simJoints.end(); iter++) {
      //   iter->second->getCoreExchange(&obj);
      //   jointList->push_back(obj);
      // }
    }

    void JointManager::getJointExchange(unsigned long id, core_objects_exchange* obj)
    {
      MutexLocker locker(&iMutex);

      throw std::logic_error("getJointExchange not implemented yet");
      // std::map<unsigned long, std::shared_ptr<SimJoint>>::iterator iter = simJoints.find(id);
      // if (iter != simJoints.end())
      //   iter->second->getCoreExchange(obj);
      // else
      //   obj = NULL;
    }


    const JointData JointManager::getFullJoint(unsigned long index)
    {
      MutexLocker locker(&iMutex);

      throw std::logic_error("getFullJoint not implemented yet");
      // map<unsigned long, std::shared_ptr<SimJoint>>::iterator iter = simJoints.find(index);
      // if (iter != simJoints.end())
      //   return iter->second->getSJoint();
      // else {
      //   char msg[128];
      //   sprintf(msg, "could not find joint with index: %lu", index);
      //   throw std::runtime_error(msg);
      // }
    }

    void JointManager::removeJoint(unsigned long index)
    {

      throw std::logic_error("removeJoint not implemented yet");
      // std::shared_ptr<SimJoint> tmpJoint = 0;
      // MutexLocker locker(&iMutex);
      // map<unsigned long, std::shared_ptr<SimJoint>>::iterator iter = simJoints.find(index);

      // if (iter != simJoints.end()) {
      //   tmpJoint = iter->second;
      //   simJoints.erase(iter);
      // }

      // control->motors->removeJointFromMotors(index);

      // if (tmpJoint)
      //   tmpJoint.reset();
      // control->sim->sceneHasChanged(false);
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
      MutexLocker locker(&iMutex);
      throw std::logic_error("getSimJoint not implemented yet");
      // map<unsigned long, std::shared_ptr<mars::sim::SimJoint>>::iterator iter = simJoints.find(id);
      // if (iter != simJoints.end())
      //   return iter->second;
      // else
      //   return NULL;
    }


    std::vector<std::shared_ptr<mars::sim::SimJoint>> JointManager::getSimJoints(void)
    {
      vector<std::shared_ptr<mars::sim::SimJoint>> v_simJoints;
      map<unsigned long, std::shared_ptr<mars::sim::SimJoint>>::iterator iter;
      MutexLocker locker(&iMutex);
      throw std::logic_error("getSimJoints not implemented yet");
      // for (iter = simJoints.begin(); iter != simJoints.end(); iter++)
      //   v_simJoints.push_back(iter->second);
      // return v_simJoints;
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
      MutexLocker locker(&iMutex);
      throw std::logic_error("updateJoints not implemented yet");
      // map<unsigned long, std::shared_ptr<SimJoint>>::iterator iter;
      // for(iter = simJoints.begin(); iter != simJoints.end(); iter++) {
      //   iter->second->update(calc_ms);
      // }
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
      if (auto jointInterface = getJointInterface(id).lock())
      {
        jointInterface->setTorque(torque);
      }
    }


    void JointManager::changeStepSize(void) 
    {
      throw std::logic_error("changeStepSize not implemented yet");
      // map<unsigned long, std::shared_ptr<SimJoint>>::iterator iter;
      // MutexLocker locker(&iMutex);
      // for (iter = simJoints.begin(); iter != simJoints.end(); iter++) {
      //   iter->second->updateStepSize();
      // }
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
      MutexLocker locker(&iMutex);
      throw std::logic_error("setSDParams not implemented yet");
      // map<unsigned long, std::shared_ptr<SimJoint>>::iterator iter = simJoints.find(id);
      // if (iter != simJoints.end())
      //   iter->second->setSDParams(sJoint);
    }


    void JointManager::setVelocity(unsigned long id, sReal velocity)
    {
      if (auto jointInterface = getJointInterface(id).lock())
      {
        jointInterface->setVelocity(velocity);
      }
    }


    void JointManager::setVelocity2(unsigned long id, sReal velocity)
    {
      if (auto jointInterface = getJointInterface(id).lock())
      {
        jointInterface->setVelocity2(velocity);
      }
    }


    void JointManager::setForceLimit(unsigned long id, sReal max_force, bool first_axis)
    {
      if (auto jointInterface = getJointInterface(id).lock())
      {
        if (first_axis)
        {
          jointInterface->setForceLimit(max_force);
        }
        else
        {
          jointInterface->setForceLimit2(max_force);
        }
      }
    }


    unsigned long JointManager::getID(const std::string& joint_name) const
    {
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
      throw std::logic_error("getIDByNodeIDs not implemented yet");
      // map<unsigned long, std::shared_ptr<SimJoint>>::iterator iter;
      // MutexLocker locker(&iMutex);

      // for (iter = simJoints.begin(); iter != simJoints.end(); iter++)
      //   if((iter->second->getNodeId() == id1 &&
      //       iter->second->getNodeId(2) == id2) ||
      //      (iter->second->getNodeId() == id2 &&
      //       iter->second->getNodeId(2) == id1)) {
      //     return iter->first;
      //   }
      return 0;
    }

    bool JointManager::getDataBrokerNames(unsigned long id, std::string *groupName, std::string *dataName) const
    {
      throw std::logic_error("getDataBrokerNames not implemented yet");
      // map<unsigned long, std::shared_ptr<SimJoint>>::const_iterator iter;
      // iter = simJoints.find(id);
      // if(iter == simJoints.end())
      //   return false;
      // iter->second->getDataBrokerNames(groupName, dataName);
      return true;
    }

    void JointManager::setOfflineValue(unsigned long id, sReal value)
    {
      throw std::logic_error("setOfflineValue not implemented yet");
      // map<unsigned long, std::shared_ptr<SimJoint>>::const_iterator iter;
      // iter = simJoints.find(id);
      // if(iter == simJoints.end())
      //   return;
      // iter->second->setOfflinePosition(value);
    }

    sReal JointManager::getLowStop(unsigned long id) const
    {
      if (const auto jointInterface = getJointInterface(id).lock())
      {
        return jointInterface->getLowStop();
      }
      return 0.0;
    }

    sReal JointManager::getHighStop(unsigned long id) const
    {
      if (const auto jointInterface = getJointInterface(id).lock())
      {
        return jointInterface->getHighStop();
      }
      return 0.0;
    }

    sReal JointManager::getLowStop2(unsigned long id) const
    {
      if (const auto jointInterface = getJointInterface(id).lock())
      {
        return jointInterface->getLowStop2();
      }
      return 0.0;
    }

    sReal JointManager::getHighStop2(unsigned long id) const 
    {
      if (const auto jointInterface = getJointInterface(id).lock())
      {
        return jointInterface->getHighStop2();
      }
      return 0.0;
    }

    void JointManager::setLowStop(unsigned long id, sReal lowStop)
    {
      if (auto jointInterface = getJointInterface(id).lock())
      {
        jointInterface->setLowStop(lowStop);
      }
    }

    void JointManager::setHighStop(unsigned long id, sReal highStop)
    {
      if (auto jointInterface = getJointInterface(id).lock())
      {
        jointInterface->setHighStop(highStop);
      }
    }

    void JointManager::setLowStop2(unsigned long id, sReal lowStop2)
    {
      if (auto jointInterface = getJointInterface(id).lock())
      {
        jointInterface->setLowStop2(lowStop2);
      }
    }

    void JointManager::setHighStop2(unsigned long id, sReal highStop2)
    {
      if (auto jointInterface = getJointInterface(id).lock())
      {
        jointInterface->setHighStop2(highStop2);
      }
    }

    // todo: do we need to edit angle offsets
    void JointManager::edit(interfaces::JointId id, const std::string &key, const std::string &value)
      {
      MutexLocker locker(&iMutex);
      
      //   if(matchPattern("*/type", key)) {
      //   }
      //   else if(matchPattern("*/axis1/*", key)) {
      //     double v = atof(value.c_str());
      //     Vector axis = iter->second->getAxis();
      //     if(key[key.size()-1] == 'x') axis.x() = v;
      //     else if(key[key.size()-1] == 'y') axis.y() = v;
      //     else if(key[key.size()-1] == 'z') axis.z() = v;
      //     iter->second->setAxis(axis);
      //   }
      //   else if(matchPattern("*/lowStopAxis1", key)) {
      //     iter->second->setLowerLimit(atof(value.c_str()));
      //   }
      //   else if(matchPattern("*/highStopAxis1", key)) {
      //     iter->second->setUpperLimit(atof(value.c_str()));
      //   }
      //   else if(matchPattern("*/damping_const_constraint_axis1", key)) {
      //     JointData jd = iter->second->getSJoint();
      //     jd.damping_const_constraint_axis1 = atof(value.c_str());
      //     iter->second->setSDParams(&jd);
      //   }
      //   else if(matchPattern("*/spring_const_constraint_axis1", key)) {
      //     JointData jd = iter->second->getSJoint();
      //     jd.spring_const_constraint_axis1 = atof(value.c_str());
      //     iter->second->setSDParams(&jd);
      //   }
      //   else if(matchPattern("*/axis2/*", key)) {
      //     double v = atof(value.c_str());
      //     Vector axis = iter->second->getAxis(2);
      //     if(key[key.size()-1] == 'x') axis.x() = v;
      //     else if(key[key.size()-1] == 'y') axis.y() = v;
      //     else if(key[key.size()-1] == 'z') axis.z() = v;
      //     iter->second->setAxis(axis, 2);
      //   }
      //   else if(matchPattern("*/lowStopAxis2", key)) {
      //     iter->second->setLowerLimit(atof(value.c_str()), 2);
      //   }
      //   else if(matchPattern("*/highStopAxis2", key)) {
      //     iter->second->setUpperLimit(atof(value.c_str()), 2);
      //   }
      //   else if(matchPattern("*/damping_const_constraint_axis2", key)) {
      //     JointData jd = iter->second->getSJoint();
      //     jd.damping_const_constraint_axis2 = atof(value.c_str());
      //     iter->second->setSDParams(&jd);
      //   }
      //   else if(matchPattern("*/spring_const_constraint_axis2", key)) {
      //     JointData jd = iter->second->getSJoint();
      //     jd.spring_const_constraint_axis2 = atof(value.c_str());
      //     iter->second->setSDParams(&jd);
      //   }
      //   else if(matchPattern("*/anchorpos", key)) {
      //     NodeId id1 = iter->second->getNodeId();
      //     NodeId id2 = iter->second->getNodeId(2);
      //     if(value == "node1") {
      //       iter->second->setAnchor(control->nodes->getPosition(id1));
      //     }
      //     else if(value == "node2") {
      //       iter->second->setAnchor(control->nodes->getPosition(id2));
      //     }
      //     else if(value == "center") {
      //       Vector pos1 = control->nodes->getPosition(id1);
      //       Vector pos2 = control->nodes->getPosition(id2);
      //       iter->second->setAnchor((pos1 + pos2) / 2.);
      //     }
      //   }
      //   else if(matchPattern("*/anchor/*", key)) {
      //     double v = atof(value.c_str());
      //     Vector anchor = iter->second->getAnchor();
      //     if(key[key.size()-1] == 'x') anchor.x() = v;
      //     else if(key[key.size()-1] == 'y') anchor.y() = v;
      //     else if(key[key.size()-1] == 'z') anchor.z() = v;
      //     iter->second->setAnchor(anchor);

      //   }
      //   else if(matchPattern("*/invertAxis", key)) {
      //     ConfigItem b;
      //     b = key;
      //     iter->second->setInvertAxis(b);
      //   }
      // }
    }

    std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(unsigned long jointId) const
    {
      const std::string jointName{ControlCenter::jointIDManager->getName(jointId)};
      return getJointInterface(jointName);
    }

    std::weak_ptr<interfaces::JointInterface> JointManager::getJointInterface(const std::string& jointName) const
    {
      // TODO "world" as global define
      const auto& rootVertex = ControlCenter::envireGraph->getVertex("world");
      std::shared_ptr<interfaces::JointInterface> foundJoint;
      auto jointInterfaceSearchFunctor = [&foundJoint, jointName](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent) 
      {
        // a joint with the given name is already found
        if (foundJoint)
        {
          return;
        }

        const size_t itemCount = ControlCenter::envireGraph->getItemCount<envire::core::Item<interfaces::JointInterfaceItem>>(node);
        for (size_t i = 0; i < itemCount; ++i)
        {
          auto item = ControlCenter::envireGraph->getItem<envire::core::Item<interfaces::JointInterfaceItem>>(node);
          std::string currentJointName;
          item->getData().jointInterface->getName(&currentJointName);
          if (jointName == currentJointName)
          {
            foundJoint = item->getData().jointInterface;
            return;
          }
        }
      };

      ControlCenter::graphTreeView->visitBfs(rootVertex, jointInterfaceSearchFunctor);

      return foundJoint;
    }
  } // end of namespace core
} // end of namespace mars
