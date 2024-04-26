/**
 * \file JointManager.hpp
 * \author Malte Langosz, Julian Liersch
 * \brief "JointManager" implements the JointManagerInterface.
 * It is manages all joints and all joint
 * operations that are used for the communication between the simulation
 * modules.
 */

#pragma once

#include <envire_core/items/Item.hpp>
#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_interfaces/sim/JointInterface.h>
#include <mars_interfaces/sim/JointManagerInterface.h>
#include <mars_interfaces/JointData.h>
#include <mars_utils/Mutex.h>

namespace mars
{
  namespace core 
  {
    class JointManager : public interfaces::JointManagerInterface 
    {
    public:
      JointManager(interfaces::ControlCenter *c);
      virtual ~JointManager(){}
      virtual unsigned long addJoint(interfaces::JointData *jointS, bool reload = false);
      virtual int getJointCount();
      virtual void editJoint(interfaces::JointData *jointS);
      virtual void getListJoints(std::vector<interfaces::core_objects_exchange> *jointList);
      virtual void getJointExchange(unsigned long id, interfaces::core_objects_exchange *obj);
      virtual const interfaces::JointData getFullJoint(unsigned long index);
      virtual void removeJoint(unsigned long index);
      virtual void removeJointByIDs(unsigned long id1, unsigned long id2);
      virtual std::shared_ptr<mars::sim::SimJoint> getSimJoint(unsigned long id);
      virtual std::vector<std::shared_ptr<mars::sim::SimJoint>> getSimJoints(void);
      virtual void reattacheJoints(unsigned long node_id);
      virtual void reloadJoints(void);
      virtual void updateJoints(interfaces::sReal calc_ms);
      virtual void clearAllJoints(bool clear_all=false);
      virtual void setReloadJointOffset(unsigned long id, interfaces::sReal offset);
      virtual void setReloadJointAxis(unsigned long id, const utils::Vector &axis);
      virtual void scaleReloadJoints(interfaces::sReal x, interfaces::sReal y, interfaces::sReal z);
      virtual void setJointTorque(unsigned long id, interfaces::sReal torque);
      virtual void changeStepSize(void);
      virtual void setReloadAnchor(unsigned long id, const utils::Vector &anchor);
      virtual void setSDParams(unsigned long id, interfaces::JointData *sJoint);

      virtual void setVelocity(unsigned long id, interfaces::sReal velocity);
      virtual void setVelocity2(unsigned long id, interfaces::sReal velocity);
      virtual void setForceLimit(unsigned long id, interfaces::sReal max_force,
                                 bool first_axis = 1);

      virtual unsigned long getID(const std::string &joint_name) const;
      virtual std::vector<unsigned long> getIDsByNodeID(unsigned long node_id);
      virtual unsigned long getIDByNodeIDs(unsigned long id1, unsigned long id2);
      virtual bool getDataBrokerNames(unsigned long id, std::string *groupName,
                                      std::string *dataName) const;
      virtual void setOfflineValue(unsigned long id, interfaces::sReal value);

      virtual interfaces::sReal getLowStop(unsigned long id) const;
      virtual interfaces::sReal getHighStop(unsigned long id) const;
      virtual interfaces::sReal getLowStop2(unsigned long id) const;
      virtual interfaces::sReal getHighStop2(unsigned long id) const;
      virtual void setLowStop(unsigned long id, interfaces::sReal lowStop);
      virtual void setHighStop(unsigned long id, interfaces::sReal highStop);
      virtual void setLowStop2(unsigned long id, interfaces::sReal lowStop2);
      virtual void setHighStop2(unsigned long id, interfaces::sReal highStop2);
      virtual void edit(interfaces::JointId id, const std::string &key,
                        const std::string &value);

      // TODO: Discuss: Expose shared pointer counter for jointInterface? This may break internal working if user stores shared pointers to JointInterface.
      static std::weak_ptr<interfaces::JointInterface> getJointInterface(unsigned long jointId);
      static std::weak_ptr<interfaces::JointInterface> getJointInterface(const std::string& jointName);
    private:
      static configmaps::ConfigMap constructEnvireJointConfigMap(const interfaces::JointData& jointData);
      static std::string constructDataBrokerName(const unsigned long jointId, const std::string& jointName);
      static const interfaces::JointData constructJointData(const std::shared_ptr<interfaces::JointInterface> joint);
      static envire::core::FrameId constructFrameIdFromJointName(const std::string& jointName, bool isFixedJoint);
      static envire::core::FrameId constructFrameIdFromJointData(const interfaces::JointData& jointData);
      static bool isFixedJoint(const interfaces::JointData& jointData);
      static bool isFixedJoint(const unsigned int jointId);

      static envire::core::ItemBase::Ptr getItemBasePtr(unsigned long jointId);
      static envire::core::ItemBase::Ptr getItemBasePtr(const std::string& jointName);
      static std::weak_ptr<interfaces::JointInterface> getJointInterface(const envire::core::FrameId& linkedFrame0, const envire::core::FrameId& linkedFrame1);
      static std::list<std::weak_ptr<interfaces::JointInterface>> getJoints();

      std::list<interfaces::JointData> simJointsReload;
      interfaces::ControlCenter *control;
      mutable utils::Mutex iMutex;

      std::list<interfaces::JointData>::iterator getReloadJoint(unsigned long id);
    };
  } // end of namespace core
} // end of namespace mars
