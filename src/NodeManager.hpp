/**
 * \file NodeManager.h
 * \author Malte Langosz, Julian Liersch
 * \brief "NodeManager" is the class that manage all nodes and their
 * operations and communication between the different modules of the simulation.
 */
#pragma once

#ifdef _PRINT_HEADER_
  #warning "NodeManager.h"
#endif

#include <mars_utils/Mutex.h>
#include <mars_interfaces/graphics/GraphicsUpdateInterface.h>
#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_interfaces/sim/NodeManagerInterface.h>

namespace mars
{
    namespace interfaces
    {
        class CollisionInterface;
    }


    namespace core
    {
        class SimJoint;
        class SimNode;

        typedef std::map<interfaces::NodeId, std::shared_ptr<SimNode>> NodeMap;

        /**
         * The declaration of the NodeManager class.
         *
         */
        class NodeManager : public interfaces::NodeManagerInterface,
                            public interfaces::GraphicsUpdateInterface 
        {
        public:
            NodeManager(interfaces::ControlCenter *c, lib_manager::LibManager *theManager);
            virtual ~NodeManager(){}

            virtual interfaces::NodeId createPrimitiveNode(const std::string &name,
                                                          interfaces::NodeType type,
                                                          bool movable=false,
                                                          const utils::Vector &pos=utils::Vector::Zero(),
                                                          const utils::Vector &extension=utils::Vector::Identity(),
                                                          double mass=0,
                                                          const utils::Quaternion &orientation=utils::Quaternion::Identity(),
                                                          bool disablePhysics=false);

            virtual interfaces::NodeId addNode(interfaces::NodeData *nodeS,
                                              bool reload = false,
                                              bool loadGraphics = true);
            virtual interfaces::NodeId addTerrain(interfaces::terrainStruct *terrainS);
            virtual std::vector<interfaces::NodeId> addNode(std::vector<interfaces::NodeData> v_NodeData);
            virtual interfaces::NodeId addPrimitive(interfaces::NodeData *snode);
            virtual bool exists(interfaces::NodeId id) const;
            virtual int getNodeCount() const;
            virtual interfaces::NodeId getNextNodeID() const;
            virtual void editNode(interfaces::NodeData *nodeS, int changes);
            virtual void changeGroup(interfaces::NodeId id, int group);
            virtual void getListNodes(std::vector<interfaces::core_objects_exchange> *nodeList) const;
            virtual void getNodeExchange(interfaces::NodeId id,
                                        interfaces::core_objects_exchange *obj) const;
            virtual const interfaces::NodeData getFullNode(interfaces::NodeId id) const;
            virtual void removeNode(interfaces::NodeId id, bool clearGraphics=true);
            virtual void setNodeState(interfaces::NodeId id, const interfaces::nodeState &state);
            virtual void getNodeState(interfaces::NodeId id, interfaces::nodeState *state) const;
            virtual const utils::Vector getCenterOfMass(const std::vector<interfaces::NodeId> &ids) const;
            virtual void setPosition(interfaces::NodeId id, const utils::Vector &pos);
            virtual const utils::Vector getPosition(interfaces::NodeId id) const;
            virtual void setRotation(interfaces::NodeId id, const utils::Quaternion &rot);
            virtual const utils::Quaternion getRotation(interfaces::NodeId id) const;
            virtual const utils::Vector getLinearVelocity(interfaces::NodeId id) const;
            virtual const utils::Vector getAngularVelocity(interfaces::NodeId id) const;
            virtual const utils::Vector getLinearAcceleration(interfaces::NodeId id) const;
            virtual const utils::Vector getAngularAcceleration(interfaces::NodeId id) const;
            virtual void applyForce(interfaces::NodeId id, const utils::Vector &force,
                                    const utils::Vector &pos);
            virtual void applyForce(interfaces::NodeId id, const utils::Vector &force);
            virtual void applyTorque(interfaces::NodeId id, const utils::Vector &torque);
            virtual void setContactParamMotion1(interfaces::NodeId id, interfaces::sReal motion);
            virtual void addNodeSensor(interfaces::BaseNodeSensor *sensor);
            virtual void reloadNodeSensor(interfaces::BaseNodeSensor *sensor);
            virtual std::shared_ptr<mars::core::SimNode> getSimNode(interfaces::NodeId id);
            virtual const std::shared_ptr<mars::core::SimNode> getSimNode(interfaces::NodeId id) const;
            virtual void reloadNodes(bool reloadGraphics);
            virtual const utils::Vector setReloadExtent(interfaces::NodeId id, const utils::Vector &ext);
            virtual void setReloadPosition(interfaces::NodeId id, const utils::Vector &pos);
            virtual void setReloadFriction(interfaces::NodeId id, interfaces::sReal friction1,
                                          interfaces::sReal friction2);
            virtual void updateDynamicNodes(interfaces::sReal calc_ms, bool physics_thread = true);
            virtual void clearAllNodes(bool clear_all=false, bool clearGraphics=true);
            virtual void setReloadAngle(interfaces::NodeId id, const utils::sRotation &angle);
            virtual void setContactParams(interfaces::NodeId id, const interfaces::contact_params &cp);
            virtual const interfaces::contact_params getContactParams(interfaces::NodeId id) const;
            virtual void setVelocity(interfaces::NodeId id, const utils::Vector& vel);
            virtual void setAngularVelocity(interfaces::NodeId id, const utils::Vector &vel);
            virtual void scaleReloadNodes(interfaces::sReal x, interfaces::sReal y, interfaces::sReal z);
            virtual void getNodeMass(interfaces::NodeId id, interfaces::sReal *mass, interfaces::sReal *inertia = 0) const;
            virtual void setAngularDamping(interfaces::NodeId id, interfaces::sReal damping);
            virtual void setLinearDamping(interfaces::NodeId id, interfaces::sReal damping);
            virtual void addRotation(interfaces::NodeId id, const utils::Quaternion &q);
            virtual void setReloadQuaternion(interfaces::NodeId id, const utils::Quaternion &q);
            virtual void preGraphicsUpdate(void);
            virtual void exportGraphicNodesByID(const std::string &folder) const;
            virtual void getContactPoints(std::vector<interfaces::NodeId> *ids,
                                          std::vector<utils::Vector> *contact_points) const;
            virtual void getContactIDs(const interfaces::NodeId &id,
                                      std::list<interfaces::NodeId> *ids) const;
            virtual void updateRay(interfaces::NodeId id);
            virtual interfaces::NodeId getDrawID(interfaces::NodeId id) const;
            virtual interfaces::NodeId getDrawID2(interfaces::NodeId id) const;
            virtual void setVisualRep(interfaces::NodeId id, int val);
            virtual const utils::Vector getContactForce(interfaces::NodeId id) const;
            virtual void setVisualQOffset(interfaces::NodeId id, const utils::Quaternion &q);

            virtual void updatePR(interfaces::NodeId id, const utils::Vector &pos,
                                  const utils::Quaternion &rot,
                                  const utils::Vector &visOffsetPos,
                                  const utils::Quaternion &visOffsetRot,
                                  bool doLock = true);
            /**
             * Retrieve the id of a node by name
             * \param node_name Name of the node to get the id for
             * \return Id of the node if it exists, otherwise 0
             */
            virtual interfaces::NodeId getID(const std::string& node_name) const;
            virtual std::vector<interfaces::NodeId> getNodeIDs(const std::string& str_in_name) const;
            virtual double getCollisionDepth(interfaces::NodeId id) const;
            virtual bool getDataBrokerNames(interfaces::NodeId id, std::string *groupName,
                                            std::string *dataName) const;

            virtual std::vector<interfaces::NodeId> getConnectedNodes(interfaces::NodeId id);

            virtual bool getIsMovable(interfaces::NodeId id) const;
            virtual void setIsMovable(interfaces::NodeId id, bool isMovable);
            virtual void lock() {}
            virtual void unlock() {}
            virtual void rotateNode(interfaces::NodeId id, utils::Vector pivot,
                                    utils::Quaternion q,
                                    unsigned long excludeJointId, bool includeConnected = true);
            virtual void positionNode(interfaces::NodeId id, utils::Vector pos,
                                      unsigned long excludeJointId);
            virtual void setSingleNodePose(interfaces::NodeId id, utils::Vector pos, utils::Quaternion q);
            virtual unsigned long getMaxGroupID() { return maxGroupID; }
            virtual void edit(interfaces::NodeId id, const std::string &key,
                              const std::string &value);

        private:
            static interfaces::DynamicObject* getDynamicObject(const interfaces::NodeId& node_id);
            static interfaces::AbsolutePose& getAbsolutePose(const interfaces::NodeId& node_id);
            static void moveDynamicObjects(const interfaces::NodeId& node_id, const utils::Vector& translation, const bool move_all);

            interfaces::CollisionInterface* getGlobalCollisionInterface();

            bool addGlobalCollisionObject(interfaces::NodeData& nodeData);

            std::weak_ptr<interfaces::CollisionInterface> globalCollisionInterface_;

            bool update_all_nodes;
            int visual_rep;
            NodeMap vizNodes;
            unsigned long maxGroupID;
            lib_manager::LibManager *libManager;
            interfaces::ControlCenter *control;

            void removeNode(interfaces::NodeId id, bool lock,
                            bool clearGraphics=true);
            void pushToUpdate(std::shared_ptr<SimNode>  node);

            void printNodeMasses(bool onlysum);
            void changeNode(std::shared_ptr<SimNode> editedNode, interfaces::NodeData *nodeS);
        };

    } // end of namespace core
} // end of namespace mars
