/**
 * \file NodeManager.cpp
 * \author Malte Langosz, Julian Liersch
 * \brief "NodeManager" is the class that manage all nodes and their
 * operations and communication between the different modules of the simulation.
 */

#include "SimNode.hpp"
#include "SimJoint.hpp"
#include "NodeManager.hpp"
#include "JointManager.hpp"
//#include "PhysicsMapper.hpp"

#include <mars_interfaces/sim/LoadCenter.h>
#include <mars_interfaces/sim/SimulatorInterface.h>
#include <mars_interfaces/sim/CollisionInterface.hpp>
#include <mars_interfaces/sim/AbsolutePose.hpp>
#include <mars_interfaces/sim/DynamicObject.hpp>
#include <mars_ode_collision/objects/Object.hpp>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/terrainStruct.h>
#include <mars_interfaces/Logging.hpp>

#include <lib_manager/LibManager.hpp>

#include <mars_interfaces/MARSDefs.h>
#include <mars_interfaces/utils.h>
#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>

#include <stdexcept>

#include <mars_utils/MutexLocker.h>

namespace mars
{
    namespace core
    {
        using namespace std;
        using namespace utils;
        using namespace interfaces;

        /**
         *\brief Initialization of a new NodeManager
          *
          * pre:
          *     - a pointer to a ControlCenter is needed
          * post:
          *     - next_node_id should be initialized to one
          */
        NodeManager::NodeManager(ControlCenter *c, lib_manager::LibManager *theManager) :
            update_all_nodes{false},
            visual_rep{1},
            maxGroupID{0},
            control{c},
            idManager_{new FrameIDManager{}},
            libManager{theManager}
        {
            idManager_->add(SIM_CENTER_FRAME_NAME);

            if(control->graphics)
            {
                GraphicsUpdateInterface *gui = static_cast<GraphicsUpdateInterface*>(this);
                control->graphics->addGraphicsUpdateInterface(gui);
            }
        }

        NodeManager::~NodeManager()
        {
            if(control->graphics)
            {
                GraphicsUpdateInterface *gui = static_cast<GraphicsUpdateInterface*>(this);
                control->graphics->removeGraphicsUpdateInterface(gui);
            }
        }

        NodeId NodeManager::createPrimitiveNode(const std::string &name,
                                                NodeType type,
                                                bool moveable,
                                                const Vector &pos,
                                                const Vector &extension,
                                                double mass,
                                                const Quaternion &orientation,
                                                bool disablePhysics)
        {
            throw std::logic_error("NodeManager::createPrimitiveNode not implemented yet");
            // NodeData s;
            // s.initPrimitive(type, extension, mass);
            // s.name = name;
            // s.pos = pos;
            // s.rot = orientation;
            // s.movable = moveable;
            // s.noPhysical = disablePhysics;
            // return addNode(&s);
        }

        /**
         *\brief Add a node to the node pool of the simulation
          *
          * It is very important to assure the serialization between the threads to
          * have the desired results.
          *
          * pre:
          *     - nodeS->groupID >= 0
          */
        NodeId NodeManager::addNode(NodeData *nodeS, bool reload,
                                    bool loadGraphics)
        {
            if (!nodeS->noPhysical)
            {
                // TODO:
                //  * Add or find world frame with collision object
                //  * Add suited child frames
                //  * Create suited envire items
                //  * Add envire items to child frames
            }

            // TODO: Implement handling of nodes
            // return nodeS->index;

            throw std::logic_error("NodeManager::addNode not implemented yet");
        }

        /**
         *\brief This function maps a terrainStruct to a node struct and adds
          * that node to the simulation.
          *
          */
        NodeId NodeManager::addTerrain(terrainStruct* terrain)
        {
            throw std::logic_error("NodeManager::addTerrain not implemented yet");
            NodeData newNode;
            // terrainStruct *newTerrain = new terrainStruct(*terrain);
            // sRotation trot = {0, 0, 0};

            // newNode.name = terrain->name;
            // newNode.filename = terrain->srcname;
            // newNode.terrain = newTerrain;
            // newNode.movable = false;
            // newNode.groupID = 0;
            // newNode.relative_id = 0;
            // newNode.physicMode = NODE_TYPE_TERRAIN;
            // newNode.pos = Vector(0.0, 0.0, 0.0);
            // newNode.rot = eulerToQuaternion(trot);
            // newNode.density = 1;
            // newNode.mass = 0;
            // newNode.ext = Vector(terrain->targetWidth, terrain->targetHeight,
            //                       terrain->scale);
            // newNode.material = terrain->material;
            // control->sim->sceneHasChanged(false);
            return addNode(&newNode, true);
        }

        /**
         *\brief This function adds an vector of nodes to the factory.
          * The functionality is implemented in the GUI, but should
          * move to the node factory soon.
          *
          */
        vector<NodeId> NodeManager::addNode(vector<NodeData> v_NodeData)
        {
            throw std::logic_error("NodeManager::addNode not implemented yet");
            vector<NodeId> tmp;
            // vector<NodeData>::iterator iter;

            // control->sim->sceneHasChanged(false);
            // for(iter=v_NodeData.begin(); iter!=v_NodeData.end(); iter++)
            // {
            //     tmp.push_back(addNode(&(*iter)));
            // }
            return tmp;
        }

        /**
         *\brief This function adds an primitive to the simulation.
          * The functionality is implemented in the GUI, but should
          * move to the node factory soon.
          *
          */
        NodeId NodeManager::addPrimitive(NodeData *snode)
        {
            constexpr bool scene_was_reseted = false;
            control->sim->sceneHasChanged(scene_was_reseted);
            return addNode(snode);
        }

        /**
         *\brief returns true if the node with the given id exists
          *
          */
        bool NodeManager::exists(NodeId id) const
        {
            return idManager_->isKnown(id);
        }

        /**
         *\brief returns the number of nodes added to the simulation
          *
          */
        int NodeManager::getNodeCount() const
        {
            return static_cast<int>(idManager_->size());
        }

        NodeId NodeManager::getNextNodeID() const
        {
            throw std::logic_error("NodeManager::getNextNodeID not implemented yet");
            // TODO: Enable IDManager to return the next id.
            // return control->linkIDManger->getNextID();
            return 0;
        }

        /**
         * \brief Change a node. The simulation must be updated in here.
         * doc has to be written
         */
        void NodeManager::editNode(NodeData *nodeS, int changes)
        {
            if (nodeS->groupID != 0)
            {
                throw std::logic_error{"NodeManager::editNode: Handling of groupID != 0 is not supported yet."};
            }
            const auto& node_id = nodeS->index;
            if (isRootFrame(node_id))
            {
                LOG_WARN("NodeManager::editNode: Can't edit root node.");
                return;
            }
            if (!getDynamicObject(node_id))
            {
                LOG_WARN("NodeManager::editNode currently only supports nodes with dynamic objects.");
                return;
            }

            if (changes & EDIT_NODE_POS)
            {
                const auto& absolutePose = getAbsolutePose(node_id);
                const auto& currentPosition = absolutePose.getPosition();
                const auto& translation = nodeS->pos - currentPosition;
                const bool move_all = changes & EDIT_NODE_MOVE_ALL;
                moveDynamicObjects(node_id, translation, move_all);

                if (!move_all)
                {
                    updateTransformations(node_id, translation, utils::Quaternion::Identity());
                    ControlCenter::joints->reattacheJoints(node_id);
                }

                // TODO: What does this do? -> It's related to preGraphicsUpdate
                // update_all_nodes = true;
            }

            if(changes & EDIT_NODE_ROT)
            {
                const auto& absolutePose = getAbsolutePose(node_id);
                const auto& currentRotation = absolutePose.getRotation();
                const auto rotationChange = nodeS->rot * currentRotation.inverse();
                const bool move_all = changes & EDIT_NODE_MOVE_ALL;
                rotateDynamicObjects(node_id, rotationChange, move_all);

                if (!move_all)
                {
                    updateTransformations(node_id, utils::Vector::Zero(), rotationChange);
                    ControlCenter::joints->reattacheJoints(node_id);
                }

                // TODO: What does this do? -> It's related to preGraphicsUpdate
                // update_all_nodes = true;
            }

            if ((changes & EDIT_NODE_SIZE) || (changes & EDIT_NODE_TYPE) || (changes & EDIT_NODE_CONTACT) ||
                (changes & EDIT_NODE_MASS) || (changes & EDIT_NODE_NAME) ||
                (changes & EDIT_NODE_GROUP) || (changes & EDIT_NODE_PHYSICS))
            {
                throw std::logic_error("NodeManager::editNode does not yet support the requested edit.");
                // TODO: What to do here?
                // changeNode(editedNode, nodeS);
            }
            if ((changes & EDIT_NODE_MATERIAL))
            {
                throw std::logic_error("NodeManager::editNode does not yet support the requested edit.");
                // TODO: Change material
                // editedNode->setMaterial(nodeS->material);
                // if(control->graphics)
                // {
                //     control->graphics->setDrawObjectMaterial(editedNode->getGraphicsID(), nodeS->material);
                // }
            }

            constexpr interfaces::sReal time_step_ms = 0;
            constexpr bool called_from_physics_thread = false;
            // TODO: Is this needed?
            // updateDynamicNodes(time_step_ms, called_from_physics_thread);
        }

        void NodeManager::changeGroup(NodeId id, int group)
        {
            throw std::logic_error("NodeManager::changeGroup not implemented yet");
            CPP_UNUSED(id);
            CPP_UNUSED(group);
        }

        /**
         * \brief Fills a list of core_object_exchange objects with node
         * iformations.
         */
        void NodeManager::getListNodes(vector<core_objects_exchange>* nodeList) const
        {
            nodeList->clear();
            for(const auto id : idManager_->getAllIDs())
            {
                const auto& linkName = idManager_->getName(id);
                core_objects_exchange obj;

                obj.index = id;

                constexpr size_t max_link_name_length = 1000;
                if(linkName.length() > max_link_name_length)
                {
                    // TODO: Is this needed?
                    fprintf(stderr, "to long name: %d\n", (int)linkName.length());
                    obj.name = "";
                }
                else
                {
                    obj.name = linkName;
                }

                using AbsolutePoseEnvireItem = envire::core::Item<interfaces::AbsolutePose>;
                if (control->envireGraph_->containsFrame(linkName) && control->envireGraph_->containsItems<AbsolutePoseEnvireItem>(linkName))
                {
                    const auto& absolutePose = control->envireGraph_->getItem<AbsolutePoseEnvireItem>(linkName)->getData();
                    obj.pos = absolutePose.getPosition();
                    obj.rot = absolutePose.getRotation();
                }
                else
                {
                    // TODO: Handle non-link nodes
                }

                // TODO: set these as well?
                // obj->groupID = sNode.groupID;
                // obj->visOffsetPos = sNode.visual_offset_pos;
                // obj->visOffsetRot = sNode.visual_offset_rot;

                nodeList->push_back(obj);
            }
        }

        /** \brief
         * Fills one core_object_exchange object with node information
         * of the node with the given id.
         */
        void NodeManager::getNodeExchange(NodeId id, core_objects_exchange* obj) const
        {
            throw std::logic_error("NodeManager::getNodeExchange not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->getCoreExchange(obj);
            // }
            // else
            // {
            //    obj = NULL;
            // }
        }

        /**
         * \brief get the full struct of a Node for editing purposes
         * \throw std::runtime_error if the node cannot be found
         */
        const NodeData NodeManager::getFullNode(NodeId id) const
        {
            throw std::logic_error("NodeManager::getFullNode not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     return iter->second->getSNode();
            // }
            // else
            // {
            //     char msg[128];
            //     sprintf(msg, "could not find node with id: %lu", id);
            //     throw std::runtime_error(msg);
            // }
        }


        /**
         * \brief removes the node with the corresponding id.
         *
         * Ok, first we have to check if the node is an element of a composite object.
         * If so, we have to cases:
         *
         * first case:
         *       - other nodes exist, which are element of the composite object.
         *       -> we can delete the node and keep the group
         *
         * second case:
         *       - this node is the only (last) one of the composite object
         *       -> we have to delete both, the node and the group
         *
         * At this moment we only the physical implementation handle the groups,
         * so the tow cases can be handled here in the same way:
         * -> tell the physics to destroy the object and remove the node from
         *    the core.
         *
         * What about joints?
         */
        void NodeManager::removeNode(NodeId id, bool clearGraphics)
        {
            throw std::logic_error("NodeManager::removeNode not implemented yet");
            // removeNode(id, true, clearGraphics);
        }

        void NodeManager::removeNode(NodeId id, bool lock, bool clearGraphics)
        {
            throw std::logic_error("NodeManager::removeNode not implemented yet");
            // NodeMap::iterator iter; //NodeMap is a map containing an id and a SimNode
            // std::shared_ptr<SimNode> tmpNode = 0;

            // if(lock) iMutex.lock();

            // iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     tmpNode = iter->second; //iter->second is a pointer to the SimNode associated with the map
            //     simNodes.erase(iter);
            // }

            // iter = vizNodes.find(id);
            // if (iter != vizNodes.end())
            // {
            //     // todo: handle remove child in graphics
            //     tmpNode = iter->second; //iter->second is a pointer to the SimNode associated with the map
            //     vizNodes.erase(iter);
            // }

            // iter = nodesToUpdate.find(id);
            // if (iter != nodesToUpdate.end())
            // {
            //     nodesToUpdate.erase(iter);
            // }

            // if (tmpNode && tmpNode->isMovable())
            // {
            //     iter = simNodesDyn.find(id);
            //     if (iter != simNodesDyn.end())
            //     {
            //         simNodesDyn.erase(iter);
            //     }
            // }

            // iMutex.unlock();
            // if(!lock)
            // {
            //     iMutex.lock();
            // }
            // if (tmpNode)
            // {
            //     clearRelativePosition(id, lock);
            //     if(control->graphics && clearGraphics)
            //     {
            //         control->graphics->removeDrawObject(tmpNode->getGraphicsID());
            //         control->graphics->removeDrawObject(tmpNode->getGraphicsID2());
            //     }
            //     tmpNode.reset();
            // }
            // control->sim->sceneHasChanged(false);
        }

        /**
         *\brief Set physical dynamic values for the node with the given id.
          */
        void NodeManager::setNodeState(NodeId id, const nodeState &state)
        {
            throw std::logic_error("NodeManager::setNodeState not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->setPhysicalState(state);
            // }
        }

        /**
         *\brief Get physical dynamic values for the node with the given id.
          */
        void NodeManager::getNodeState(NodeId id, nodeState *state) const
        {
            throw std::logic_error("NodeManager::getNodeState not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->getPhysicalState(state);
            // }
        }

        /**
         *\brief Return the center of mass for the nodes corresponding to
          * the id's from the given vector.
          */
        const Vector NodeManager::getCenterOfMass(const std::vector<NodeId> &ids) const
        {
            throw std::logic_error("NodeManager::getCenterOfMass not implemented yet");
            // std::vector<NodeId>::const_iterator iter;
            // NodeMap::const_iterator nter;
            // std::vector<std::shared_ptr<NodeInterface>> pNodes;

            // MutexLocker locker(&iMutex);

            // for (iter = ids.begin(); iter != ids.end(); iter++)
            // {
            //     nter = simNodes.find(*iter);
            //     if (nter != simNodes.end())
            //     {
            //         pNodes.push_back(nter->second->getInterface());
            //     }
            // }

            // return control->sim->getPhysics()->getCenterOfMass(pNodes);
        }

        /**
         *\brief Sets the position of the node with the given id.
          */
        void NodeManager::setPosition(NodeId id, const Vector &pos)
        {
            if (auto* const dynamicObject = getDynamicObject(id))
            {
                dynamicObject->setPosition(pos);
                dynamicObject->setLinearVelocity(utils::Vector::Zero());
            }

            // TODO: Handle frames without dynamic objects
        }


        const Vector NodeManager::getPosition(NodeId id) const
        {
            if (!isRootFrame(id))
            {
                return getAbsolutePose(id).getPosition();
            }
            return utils::Vector::Zero();
        }


        const Quaternion NodeManager::getRotation(NodeId id) const
        {
            if (!isRootFrame(id))
            {
                return getAbsolutePose(id).getRotation();
            }
            return Quaternion::Identity();
        }

        const Vector NodeManager::getLinearVelocity(NodeId id) const
        {
            if (const auto* const dynamicObject = getDynamicObject(id))
            {
                Vector velocity;
                dynamicObject->getLinearVelocity(&velocity);
                return velocity;
            }
            return utils::Vector::Zero();
            // TODO: Handle frames without dynamic objects?
        }

        const Vector NodeManager::getAngularVelocity(NodeId id) const
        {
            if (const auto* const dynamicObject = getDynamicObject(id))
            {
                Vector velocity;
                dynamicObject->getAngularVelocity(&velocity);
                return velocity;
            }
            return utils::Vector::Zero();
            // TODO: Handle frames without dynamic objects?
        }

        const Vector NodeManager::getLinearAcceleration(NodeId id) const
        {
            throw std::logic_error("NodeManager::getLinearAcceleration not implemented yet");
            // Vector acc(0.0,0.0,0.0);
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     acc = iter->second->getLinearAcceleration();
            // }
            // return acc;
        }

        const Vector NodeManager::getAngularAcceleration(NodeId id) const
        {
            throw std::logic_error("NodeManager::getAngularAcceleration not implemented yet");
            // Vector aacc(0.0,0.0,0.0);
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     aacc = iter->second->getAngularAcceleration();
            // }
            // return aacc;
        }

        /**
         *\brief Sets the rotation of the node with the given id.
          */
        void NodeManager::setRotation(NodeId id, const Quaternion &rot)
        {
            if (auto* const dynamicObject = getDynamicObject(id))
            {
                dynamicObject->setRotation(rot);
                dynamicObject->setAngularVelocity(utils::Vector::Zero());
            }
            // TODO: Handle frames without dynamic objects
        }

        /**
         *\brief Adds a off-center Force to the node with the given id.
          */
        void NodeManager::applyForce(NodeId id, const Vector &force, const Vector &pos)
        {
            if (auto* const dynamicObject = getDynamicObject(id))
            {
                dynamicObject->addForce(force, pos);
            }
            else
            {
                LOG_WARN((std::string{"Can't apply force to node with id "} + std::to_string(id) + std::string{" as it does not contain a dynamic object."}).c_str());
            }
        }
        /**
         *\brief Adds a Force to the node with the given id.
          */
        void NodeManager::applyForce(NodeId id, const Vector &force)
        {
            if (auto* const dynamicObject = getDynamicObject(id))
            {
                dynamicObject->addForce(force);
            }
            else
            {
                LOG_WARN((std::string{"Can't apply force to node with id "} + std::to_string(id) + std::string{" as it does not contain a dynamic object."}).c_str());
            }
        }

        /**
         *\brief Adds a Torque to the node with the given id.
          */
        void NodeManager::applyTorque(NodeId id, const Vector &torque)
        {
            throw std::logic_error("NodeManager::applyTorque not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->applyTorque(torque);
            // }
        }


        /**
         *\brief Sets the contact parameter motion1 for the node with the given id.
          */
        void NodeManager::setContactParamMotion1(NodeId id, sReal motion)
        {
            throw std::logic_error("NodeManager::setContactParamMotion1 not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->setContactMotion1(motion);
            // }
        }


        /**
         *\brief Adds a physical sensor to the node with the given id.
          */
        void NodeManager::addNodeSensor(BaseNodeSensor *sensor)
        {
            throw std::logic_error("NodeManager::addNodeSensor not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = simNodes.find(sensor->getAttachedNode());
            // if (iter != simNodes.end())
            // {
            //     iter->second->addSensor(sensor);
            //     NodeMap::iterator kter = simNodesDyn.find(sensor->getAttachedNode());
            //     if (kter == simNodesDyn.end())
            //     {
            //         simNodesDyn[iter->first] = iter->second;
            //     }
            // }
            // else
            // {
            //     std::cerr << "Could not find node id " << sensor->getAttachedNode() << " in simNodes and did not call addSensors on the node." << std::endl;
            // }
        }

        void NodeManager::reloadNodeSensor(BaseNodeSensor* sensor)
        {
            throw std::logic_error("NodeManager::reloadNodeSensor not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = simNodes.find(sensor->getAttachedNode());
            // if (iter != simNodes.end())
            // {
            //     iter->second->reloadSensor(sensor);
            // }
        }

        /**
         *\brief Returns a pointer to the SimNode Object.
          */
        std::shared_ptr<mars::core::SimNode> NodeManager::getSimNode(mars::interfaces::NodeId id)
        {
            throw std::logic_error("NodeManager::getSimNode not implemented yet");
            // return const_pointer_cast<mars::core::SimNode>(static_cast<const NodeManager*>(this)->getSimNode(id));
        }

        const std::shared_ptr<mars::core::SimNode> NodeManager::getSimNode(NodeId id) const
        {
            throw std::logic_error("NodeManager::getSimNode not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     return iter->second;
            // }
            // else
            // {
            //     return nullptr;
            // }
        }

        void NodeManager::rotateNode(NodeId id, Vector pivot, Quaternion q,
                                      unsigned long excludeJointId, bool includeConnected)
        {
            throw std::logic_error("NodeManager::rotateNode not implemented yet");
            // std::vector<int> gids;
            // NodeMap::iterator iter = simNodes.find(id);
            // if(iter == simNodes.end())
            // {
            //     iMutex.unlock();
            //     LOG_ERROR("NodeManager::rotateNode: node id not found!");
            //     return;
            // }

            // std::shared_ptr<SimNode> editedNode = iter->second;
            // editedNode->rotateAtPoint(pivot, q, true);

            // if (includeConnected)
            // {
            //     std::vector<std::shared_ptr<SimJoint> > joints = control->joints->getSimJoints();
            //     std::vector<std::shared_ptr<SimJoint> >::iterator jter;
            //     for(jter=joints.begin(); jter!=joints.end(); ++jter)
            //     {
            //         if((*jter)->getIndex() == excludeJointId)
            //         {
            //             joints.erase(jter);
            //             break;
            //         }
            //     }

            //     if(editedNode->getGroupID())
            //     {
            //         gids.push_back(editedNode->getGroupID());
            //     }

            //     NodeMap nodes = simNodes;
            //     nodes.erase(nodes.find(editedNode->getID()));

            //     rotateNodeRecursive(id, pivot, q, &joints,
            //                         &gids, &nodes);
            // }
            // update_all_nodes = true;
            // updateDynamicNodes(0, false);
        }

        void NodeManager::positionNode(NodeId id, Vector pos,
                                        unsigned long excludeJointId)
        {
            throw std::logic_error("NodeManager::positionNode not implemented yet");
            // std::vector<int> gids;
            // NodeMap::iterator iter = simNodes.find(id);
            // if(iter == simNodes.end())
            // {
            //     iMutex.unlock();
            //     LOG_ERROR("NodeManager::rotateNode: node id not found!");
            //     return;
            // }

            // std::shared_ptr<SimNode> editedNode = iter->second;
            // Vector offset = pos - editedNode->getPosition();
            // editedNode->setPosition(pos, true);

            // std::vector<std::shared_ptr<SimJoint> > joints = control->joints->getSimJoints();
            // std::vector<std::shared_ptr<SimJoint> >::iterator jter;
            // for(jter=joints.begin(); jter!=joints.end(); ++jter)
            // {
            //     if((*jter)->getIndex() == excludeJointId)
            //     {
            //         joints.erase(jter);
            //         break;
            //     }
            // }

            // if(editedNode->getGroupID())
            // {
            //     gids.push_back(editedNode->getGroupID());
            // }

            // NodeMap nodes = simNodes;
            // nodes.erase(nodes.find(editedNode->getID()));

            // moveNodeRecursive(id, offset, &joints, &gids, &nodes);

            // update_all_nodes = true;
            // updateDynamicNodes(0, false);
        }

        void NodeManager::setSingleNodePose(NodeId id, utils::Vector pos, utils::Quaternion q)
        {
            throw std::logic_error("NodeManager::setSingleNodePose not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = simNodes.find(id);
            // if(iter == simNodes.end())
            // {
            //     iMutex.unlock();
            //     LOG_ERROR("NodeManager::setSingleNodePose: node id not found!");
            //     return;
            // }
            // std::shared_ptr<SimNode> editedNode = iter->second;
            // editedNode->setPosition(pos, false);
            // editedNode->setRotation(q, false);
            // nodesToUpdate[id] = iter->second;
        }

        /**
         *\brief Reloads all nodes in the simulation.
          */
        void NodeManager::reloadNodes(bool reloadGrahpics)
        {
            throw std::logic_error("NodeManager::reloadNodes not implemented yet");
            // std::list<NodeData>::iterator iter;
            // NodeData tmp;
            // Vector* friction;

            // iMutex.lock();
            // for(iter = simNodesReload.begin(); iter != simNodesReload.end(); iter++)
            // {
            //     tmp = *iter;
            //     if(tmp.c_params.friction_direction1)
            //     {
            //         friction = new Vector(0.0, 0.0, 0.0);
            //         *friction = *(tmp.c_params.friction_direction1);
            //         tmp.c_params.friction_direction1 = friction;
            //     }
            //     if(tmp.terrain)
            //     {
            //         tmp.terrain = new(terrainStruct);
            //         *(tmp.terrain) = *(iter->terrain);
            //         tmp.terrain->pixelData = (double*)calloc((tmp.terrain->width*
            //                                                     tmp.terrain->height),
            //                                                   sizeof(double));
            //         memcpy(tmp.terrain->pixelData, iter->terrain->pixelData,
            //                 (tmp.terrain->width*tmp.terrain->height)*sizeof(double));
            //     }
            //     iMutex.unlock();
            //     addNode(&tmp, true, reloadGrahpics);
            //     iMutex.lock();
            // }
            // iMutex.unlock();
            // updateDynamicNodes(0);
        }

        interfaces::DynamicObject* NodeManager::getDynamicObject(const NodeId& node_id) const
        {
            const auto& nodeName = idManager_->getName(node_id);
            if (!control->envireGraph_->containsFrame(nodeName))
            {
                LOG_WARN(std::string{"NodeManager::getDynamicObject: Node named \"" + nodeName + "\" does not represent a frame."}.c_str());
                return nullptr;
            }

            const auto& vertex = control->envireGraph_->getVertex(nodeName);
            using DynamicObjectEnvireItem = envire::core::Item<interfaces::DynamicObjectItem>;
            if (!control->envireGraph_->containsItems<DynamicObjectEnvireItem>(vertex))
            {
                LOG_WARN(std::string{"NodeManager::getDynamicObject: Frame \"" + nodeName + "\" does not contain a DynamicObjectItem."}.c_str());
                return nullptr;
            }

            return control->envireGraph_->getItem<DynamicObjectEnvireItem>(vertex)->getData().dynamicObject.get();
        }


        interfaces::AbsolutePose& NodeManager::getAbsolutePose(const interfaces::NodeId& node_id) const
        {
            // Method shall only be called for nodes representing frames
            const auto& nodeName = idManager_->getName(node_id);
            assert(control->envireGraph_->containsFrame(nodeName));

            // Method shall only be called for nodes containing AbsolutePose instances.
            const auto& vertex = control->envireGraph_->getVertex(nodeName);
            using AbsolutePoseEnvireItem = envire::core::Item<AbsolutePose>;
            assert(control->envireGraph_->containsItems<AbsolutePoseEnvireItem>(vertex));

            return control->envireGraph_->getItem<AbsolutePoseEnvireItem>(vertex)->getData();
        }

        void NodeManager::moveDynamicObjects(const interfaces::NodeId& node_id, const utils::Vector& translation, const bool move_all)
        {
            // Set up processing pool
            std::vector<mars::interfaces::DynamicObject*> processingPool;
            processingPool.push_back(getDynamicObject(node_id));

            // set up set of already processed objects
            std::set<const mars::interfaces::DynamicObject*> processedObjects;

            // Process linked dynamic objects
            const auto zero = mars::utils::Vector{0.0, 0.0, 0.0};
            mars::utils::Vector position;
            while(!processingPool.empty())
            {
                // Get next object
                auto& currentObject = processingPool.back();
                processingPool.pop_back();

                // Set new transformation
                currentObject->getPosition(&position);
                currentObject->setPosition(position + translation);

                // Reset velocities
                currentObject->setLinearVelocity(zero);
                currentObject->setAngularVelocity(zero);

                if (!move_all)
                {
                    return;
                }

                // Mark processed
                processedObjects.insert(currentObject);

                // Extend processing pool
                for (const auto& linkedObject: currentObject->getLinkedFrames())
                {
                    auto* const linkedObjectRaw = linkedObject.get();
                    if (processedObjects.find(linkedObjectRaw) != processedObjects.end())
                    {
                        // already processed
                        continue;
                    }

                    const auto it = std::find(processingPool.begin(), processingPool.end(), linkedObjectRaw);
                    if (it != processingPool.end())
                    {
                        // already planned for processing
                        continue;
                    }

                    processingPool.push_back(linkedObjectRaw);
                }
            }
        }

        void NodeManager::rotateDynamicObjects(const interfaces::NodeId& node_id, const utils::Quaternion& rotationChange, const bool move_all)
        {
            assert(!isRootFrame(node_id));
            const auto& globalPose = getAbsolutePose(node_id);

            // Set up processing pool
            std::vector<mars::interfaces::DynamicObject*> processingPool;
            processingPool.push_back(getDynamicObject(node_id));

            // set up set of already processed objects
            std::set<const mars::interfaces::DynamicObject*> processedObjects;

            // Process linked dynamic objects
            const auto& zero = mars::utils::Vector::Zero();
            mars::utils::Vector position;
            while(!processingPool.empty())
            {
                // Get next object
                auto& currentObject = processingPool.back();
                processingPool.pop_back();

                currentObject->rotateAtPoint(globalPose.getPosition(), rotationChange, false);

                // Reset velocities
                currentObject->setLinearVelocity(zero);
                currentObject->setAngularVelocity(zero);

                if (!move_all)
                {
                    return;
                }

                // Mark processed
                processedObjects.insert(currentObject);

                // Extend processing pool
                for (const auto& linkedObject: currentObject->getLinkedFrames())
                {
                    auto* const linkedObjectRaw = linkedObject.get();
                    if (processedObjects.find(linkedObjectRaw) != processedObjects.end())
                    {
                        // already processed
                        continue;
                    }

                    const auto it = std::find(processingPool.begin(), processingPool.end(), linkedObjectRaw);
                    if (it != processingPool.end())
                    {
                        // already planned for processing
                        continue;
                    }

                    processingPool.push_back(linkedObjectRaw);
                }
            }
        }

        void NodeManager::updateTransformations(const interfaces::NodeId& node_id, const utils::Vector& translation, const utils::Quaternion& rotation)
        {
            const auto& nodeName = idManager_->getName(node_id);
            if (!control->envireGraph_->containsFrame(nodeName))
            {
                LOG_WARN(std::string{"NodeManager::updateTransformation: Node named \"" + nodeName + "\" does not represent a frame."}.c_str());
                return;
            }

            throw std::logic_error{"NodeManager::updateTransformations not implemented yet."};
        }

        /**
         *\brief set the size for the node with the given id.
          */
        const Vector NodeManager::setReloadExtent(NodeId id, const Vector &ext)
        {
            throw std::logic_error("NodeManager::setReloadExtent not implemented yet");
            // Vector x(0.0,0.0,0.0);
            // MutexLocker locker(&iMutex);
            // std::list<NodeData>::iterator iter = getReloadNode(id);
            // if (iter != simNodesReload.end())
            // {
            //     if(iter->filename != "PRIMITIVE")
            //     {
            //         x.x() = ext.x() / iter->ext.x();
            //         x.y() = ext.y() / iter->ext.y();
            //         x.z() = ext.z() / iter->ext.z();
            //     }
            //     iter->ext = ext;
            // }
            // return x;
        }

        void NodeManager::setReloadFriction(NodeId id, sReal friction1,
                                            sReal friction2)
        {
            throw std::logic_error("NodeManager::setReloadFriction not implemented yet");
            // MutexLocker locker(&iMutex);

            // std::list<NodeData>::iterator iter = getReloadNode(id);
            // if (iter != simNodesReload.end())
            // {
            //     iter->c_params.friction1 = friction1;
            //     iter->c_params.friction2 = friction2;
            // }
        }


        /**
         *\brief set the position for the node with the given id.
          */
        void NodeManager::setReloadPosition(NodeId id, const Vector &pos)
        {
            throw std::logic_error("NodeManager::setReloadPosition not implemented yet");
            // MutexLocker locker(&iMutex);
            // std::list<NodeData>::iterator iter = getReloadNode(id);
            // if (iter != simNodesReload.end())
            // {
            //     iter->pos = pos;
            // }
        }


        /**
         *\brief Updates the Node values of dynamical nodes from the physics.
          */
        void NodeManager::updateDynamicNodes(sReal calc_ms, bool physics_thread)
        {
            throw std::logic_error("NodeManager::updateDynamicNodes not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter;
            // for(iter = simNodesDyn.begin(); iter != simNodesDyn.end(); iter++)
            // {
            //     iter->second->update(calc_ms, physics_thread);
            // }
        }

        void NodeManager::preGraphicsUpdate()
        {
            return;
            throw std::logic_error("NodeManager::preGraphicsUpdate not implemented yet");
            // NodeMap::iterator iter;
            // if(!control->graphics)
            //   return;

            // iMutex.lock();
            // if(update_all_nodes)
            // {
            //     update_all_nodes = false;
            //     for(iter = simNodes.begin(); iter != simNodes.end(); iter++)
            //     {
            //         control->graphics->setDrawObjectPos(iter->second->getGraphicsID(),
            //                                             iter->second->getVisualPosition());
            //         control->graphics->setDrawObjectRot(iter->second->getGraphicsID(),
            //                                             iter->second->getVisualRotation());
            //         control->graphics->setDrawObjectPos(iter->second->getGraphicsID2(),
            //                                             iter->second->getPosition());
            //         control->graphics->setDrawObjectRot(iter->second->getGraphicsID2(),
            //                                             iter->second->getRotation());
            //     }
            // }
            // else
            // {
            //     for(iter = simNodesDyn.begin(); iter != simNodesDyn.end(); iter++)
            //     {
            //         control->graphics->setDrawObjectPos(iter->second->getGraphicsID(),
            //                                             iter->second->getVisualPosition());
            //         control->graphics->setDrawObjectRot(iter->second->getGraphicsID(),
            //                                             iter->second->getVisualRotation());
            //         control->graphics->setDrawObjectPos(iter->second->getGraphicsID2(),
            //                                             iter->second->getPosition());
            //         control->graphics->setDrawObjectRot(iter->second->getGraphicsID2(),
            //                                             iter->second->getRotation());
            //     }
            //     for(iter = nodesToUpdate.begin(); iter != nodesToUpdate.end(); iter++)
            //     {
            //         control->graphics->setDrawObjectPos(iter->second->getGraphicsID(),
            //                                             iter->second->getVisualPosition());
            //         control->graphics->setDrawObjectRot(iter->second->getGraphicsID(),
            //                                             iter->second->getVisualRotation());
            //         control->graphics->setDrawObjectPos(iter->second->getGraphicsID2(),
            //                                             iter->second->getPosition());
            //         control->graphics->setDrawObjectRot(iter->second->getGraphicsID2(),
            //                                             iter->second->getRotation());
            //     }
            //     nodesToUpdate.clear();
            // }
            // iMutex.unlock();
        }

        /**
         *\brief Removes all nodes from the simulation to clear the world.
          */
        void NodeManager::clearAllNodes(bool clear_all, bool clearGraphics)
        {
            throw std::logic_error("NodeManager::clearAllNodes not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter;
            // while (!simNodes.empty())
            // {
            //     removeNode(simNodes.begin()->first, false, clearGraphics);
            // }
            // while (!vizNodes.empty())
            // {
            //     removeNode(vizNodes.begin()->first, false, clearGraphics);
            // }
            // simNodes.clear();
            // vizNodes.clear();
            // simNodesDyn.clear();
            // if(clear_all) simNodesReload.clear();
            // next_node_id = 1;
            // iMutex.unlock();
        }

        /**
         *\brief Set the reload orientation of a node.
          */
        void NodeManager::setReloadAngle(NodeId id, const sRotation &angle)
        {
            throw std::logic_error("NodeManager::setReloadAngle not implemented yet");
            // setReloadQuaternion(id, eulerToQuaternion(angle));
        }


        /**
         *\brief Set the reload orientation of a node by using a quaternion.
          */
        void NodeManager::setReloadQuaternion(NodeId id, const Quaternion &q)
        {
            throw std::logic_error("NodeManager::setReloadQuaternion not implemented yet");
            // MutexLocker locker(&iMutex);
            // std::list<NodeData>::iterator iter = getReloadNode(id);
            // if (iter != simNodesReload.end())
            // {
            //     iter->rot = q;
            // }
        }

        /**
         *\brief Set the contact parameter of a node.
          */
        void NodeManager::setContactParams(NodeId id, const contact_params &cp)
        {
            throw std::logic_error("NodeManager::setContactParams not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->setContactParams(cp);
            // }
        }


        void NodeManager::setVelocity(NodeId id, const Vector& vel)
        {
            if (auto* const dynamicObject = getDynamicObject(id))
            {
                dynamicObject->setLinearVelocity(vel);
            }
        }

        void NodeManager::setAngularVelocity(NodeId id, const Vector& vel)
        {
            if (auto* const dynamicObject = getDynamicObject(id))
            {
                dynamicObject->setAngularVelocity(vel);
            }
        }


        /**
         *\brief Scales the nodes to reload.
          */
        void NodeManager::scaleReloadNodes(sReal factor_x, sReal factor_y,
                                            sReal factor_z)
        {
            throw std::logic_error("NodeManager::scaleReloadNodes not implemented yet");
            // std::list<NodeData>::iterator iter;

            // iMutex.lock();
            // for(iter = simNodesReload.begin(); iter != simNodesReload.end(); iter++)
            // {
            //     iter->pos.x() *= factor_x;
            //     iter->pos.y() *= factor_y;
            //     iter->pos.z() *= factor_z;
            //     iter->ext.x() *= factor_x;
            //     iter->ext.y() *= factor_y;
            //     iter->ext.z() *= factor_z;
            //     iter->visual_offset_pos.x() *= factor_x;
            //     iter->visual_offset_pos.y() *= factor_y;
            //     iter->visual_offset_pos.z() *= factor_z;
            //     iter->visual_size.x() *= factor_x;
            //     iter->visual_size.y() *= factor_y;
            //     iter->visual_size.z() *= factor_z;
            // }
            // iMutex.unlock();
        }

        void NodeManager::getNodeMass(NodeId id, sReal *mass,
                                      sReal* inertia) const
        {
            throw std::logic_error("NodeManager::getNodeMass not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->getMass(mass, inertia);
            // }
        }


        void NodeManager::setAngularDamping(NodeId id, sReal damping)
        {
            throw std::logic_error("NodeManager::setAngularDamping not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->setAngularDamping(damping);
            // }
        }

        void NodeManager::setLinearDamping(NodeId id, sReal damping)
        {
            throw std::logic_error("NodeManager::setLinearDamping not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->setLinearDamping(damping);
            // }
        }

        void NodeManager::addRotation(NodeId id, const Quaternion &q)
        {
            throw std::logic_error("NodeManager::addRotation not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->addRotation(q);
            // }
        }

        const contact_params NodeManager::getContactParams(NodeId id) const
        {
            throw std::logic_error("NodeManager::getContactParams not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     return iter->second->getContactParams();
            // }
            // contact_params a;
            // return a;
        }


        void NodeManager::exportGraphicNodesByID(const std::string &folder) const
        {
            throw std::logic_error("NodeManager::exportGraphicNodesByID not implemented yet");
            // if(control->graphics)
            // {
            //     char text[255];
            //     std::string filename;

            //     NodeMap::const_iterator iter;
            //     iMutex.lock();
            //     for(iter=simNodes.begin(); iter!=simNodes.end(); ++iter)
            //     {
            //         sprintf(text, "/%lu.stl", iter->first);
            //         filename = folder+std::string(text);
            //         control->graphics->exportDrawObject(iter->second->getGraphicsID(), filename);
            //         sprintf(text, "/%lu.obj", iter->first);
            //         filename = folder+std::string(text);
            //         control->graphics->exportDrawObject(iter->second->getGraphicsID(), filename);
            //     }
            //     iMutex.unlock();
            // }
        }

        void NodeManager::getContactPoints(std::vector<NodeId> *ids,
                                            std::vector<Vector> *contact_points) const
        {
            throw std::logic_error("NodeManager::getContactPoints not implemented yet");
            // NodeMap::const_iterator iter;
            // std::vector<Vector>::const_iterator lter;
            // std::vector<Vector> points;

            // iMutex.lock();
            // for(iter=simNodes.begin(); iter!=simNodes.end(); ++iter)
            // {
            //     iter->second->getContactPoints(&points);
            //     for(lter=points.begin(); lter!=points.end(); ++lter)
            //     {
            //         ids->push_back(iter->first);
            //         contact_points->push_back((*lter));
            //     }
            // }
            // iMutex.unlock();
        }

        void NodeManager::getContactIDs(const interfaces::NodeId &id,
                                        std::list<interfaces::NodeId> *ids) const
        {
            throw std::logic_error("NodeManager::getContactIDs not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->getContactIDs(ids);
            // }
        }

        void NodeManager::updateRay(NodeId id)
        {
            throw std::logic_error("NodeManager::updateRay not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->updateRay();
            // }
        }


        NodeId NodeManager::getDrawID(NodeId id) const
        {
            throw std::logic_error("NodeManager::getDrawID not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     return iter->second->getGraphicsID();
            // }
            // else
            // {
            //     return INVALID_ID;
            // }
        }

        NodeId NodeManager::getDrawID2(NodeId id) const
        {
            throw std::logic_error("NodeManager::getDrawID2 not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     return iter->second->getGraphicsID2();
            // }
            // else
            // {
            //     return INVALID_ID;
            // }
        }


        const Vector NodeManager::getContactForce(NodeId id) const
        {
            throw std::logic_error("NodeManager::getContactForce not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     return iter->second->getContactForce();
            // }
            // else
            // {
            //     return Vector(0.0, 0.0, 0.0);
            // }
        }


        double NodeManager::getCollisionDepth(NodeId id) const
        {
            throw std::logic_error("NodeManager::getCollisionDepth not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     return iter->second->getCollisionDepth();
            // }
            // else
            // {
            //     return 0.0;
            // }
        }


        void NodeManager::setVisualRep(NodeId id, int val)
        {
            throw std::logic_error("NodeManager::setVisualRep not implemented yet");
            // if(!(control->graphics))
            // {
            //     return;
            // }
            // visual_rep = val;
            // NodeMap::iterator iter;
            // int current;

            // iMutex.lock();
            // for(iter = simNodes.begin(); iter != simNodes.end(); iter++)
            // {
            //     if(id == 0 || iter->first == id)
            //     {
            //         current = iter->second->getVisualRep();
            //         if(val & 1 && !(current & 1))
            //         {
            //             control->graphics->setDrawObjectShow(iter->second->getGraphicsID(), true);
            //         }
            //         else if(!(val & 1) && current & 1)
            //         {
            //             control->graphics->setDrawObjectShow(iter->second->getGraphicsID(), false);
            //         }
            //         if(val & 2 && !(current & 2))
            //         {
            //             control->graphics->setDrawObjectShow(iter->second->getGraphicsID2(), true);
            //         }
            //         else if(!(val & 2) && current & 2)
            //         {
            //             control->graphics->setDrawObjectShow(iter->second->getGraphicsID2(), false);
            //         }

            //         iter->second->setVisualRep(val);
            //         if(id != 0) 
            //         {
            //             break;
            //         }
            //     }
            // }
            // iMutex.unlock();
        }

        NodeId NodeManager::getID(const std::string& node_name) const
        {
            const auto& node_id = idManager_->getID(node_name);
            if (node_id == INVALID_ID)
            {
                const auto msg = std::string{"NodeManager::getID: Can't find node with the name \""} + node_name + "\".";
                LOG_ERROR(msg.c_str());
            }
            return node_id;
        }


        const std::string& NodeManager::getLinkName(const unsigned int linkID) const
        {
            return idManager_->getName(linkID);
        }

        std::vector<interfaces::NodeId> NodeManager::getNodeIDs(const std::string& str_in_name) const
        {
            throw std::logic_error("NodeManager::getNodeIDs not implemented yet");
            // TODO: Enable IDManager to find all entries which contain str_in_name in their name.
            // idManager_->getIDsContaining(str_in_name);
            //
            // iMutex.lock();
            // NodeMap::const_iterator iter;
            // std::vector<interfaces::NodeId> out;
            // for(iter = simNodes.begin(); iter != simNodes.end(); iter++)
            // {
            //     if (iter->second->getName().find(str_in_name) != std::string::npos)
            //     {
            //         out.push_back(iter->first);
            //     }
            // }
            // iMutex.unlock();
            // return out;
        }

        void NodeManager::pushToUpdate(std::shared_ptr<SimNode>  node)
        {
            throw std::logic_error("NodeManager::pushToUpdate not implemented yet");
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = nodesToUpdate.find(node->getID());
            // if (iter == nodesToUpdate.end())
            // {
            //     nodesToUpdate[node->getID()] = node;
            // }
        }

        std::vector<NodeId> NodeManager::getConnectedNodes(NodeId id)
        {
            throw std::logic_error("NodeManager::getConnectedNodes not implemented yet");
            // std::vector<NodeId> connected;
            // MutexLocker locker(&iMutex);
            // NodeMap::iterator iter = simNodes.find(id);
            // if (iter == simNodes.end())
            // {
            //     return connected;
            // }

            // std::shared_ptr<SimNode> current = iter->second;
            // std::vector<std::shared_ptr<SimJoint> > simJoints = control->joints->getSimJoints();

            // if (current->getGroupID() != 0)
            // {
            //     for (iter = simNodes.begin(); iter != simNodes.end(); iter++)
            //     {
            //         if (iter->second->getGroupID() == current->getGroupID())
            //         {
            //             connected.push_back(iter->first);
            //         }
            //     }
            // }

            // for (size_t i = 0; i < simJoints.size(); i++)
            // {
            //     if (simJoints[i]->getAttachedNode() &&
            //         simJoints[i]->getAttachedNode()->getID() == id &&
            //         simJoints[i]->getAttachedNode(2))
            //     {
            //         connected.push_back(simJoints[i]->getAttachedNode(2)->getID());
            //         /*    current = simNodes.find(connected.back())->second;
            //               if (current->getGroupID() != 0)
            //               for (iter = simNodes.begin(); iter != simNodes.end(); iter++)
            //               if (iter->second->getGroupID() == current->getGroupID())
            //               connected.push_back(iter->first);*/
            //     }

            //     if (simJoints[i]->getAttachedNode(2) &&
            //         simJoints[i]->getAttachedNode(2)->getID() == id &&
            //         simJoints[i]->getAttachedNode())
            //     {
            //         connected.push_back(simJoints[i]->getAttachedNode()->getID());
            //         /*      current = simNodes.find(connected.back())->second;
            //                 if (current->getGroupID() != 0)
            //                 for (iter = simNodes.begin(); iter != simNodes.end(); iter++)
            //                 if (iter->second->getGroupID() == current->getGroupID())
            //                 connected.push_back(iter->first);*/
            //     }
            // }

            // return connected;
        }

        bool NodeManager::getDataBrokerNames(NodeId id, std::string *groupName,
                                              std::string *dataName) const
        {
            if (!idManager_->isKnown(id))
            {
                return false;
            }

            *groupName = "mars_sim";
            *dataName = std::string{"Frames/"} + idManager_->getName(id);

            return true;
        }

        void NodeManager::setVisualQOffset(NodeId id, const Quaternion &q)
        {
            throw std::logic_error("NodeManager::setVisualQOffset not implemented yet");
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if (iter != simNodes.end())
            // {
            //     iter->second->setVisQOffset(q);
            // }
        }

        void NodeManager::updatePR(NodeId id, const Vector &pos,
                                    const Quaternion &rot,
                                    const Vector &visOffsetPos,
                                    const Quaternion &visOffsetRot,
                                    bool doLock)
        {
            throw std::logic_error("NodeManager::updatePR not implemented yet");
            // NodeMap::const_iterator iter = simNodes.find(id);

            // if (iter != simNodes.end())
            // {
            //     iter->second->updatePR(pos, rot, visOffsetPos, visOffsetRot);
            //     if(doLock)
            //     {
            //         MutexLocker locker(&iMutex);
            //     }
            //     nodesToUpdate[id] = iter->second;
            // }
        }

        bool NodeManager::getIsMovable(NodeId id) const
        {
            throw std::logic_error("NodeManager::getIsMovable not implemented yet");
            // NodeMap::const_iterator iter = simNodes.find(id);
            // if(iter != simNodes.end())
            // {
            //     return iter->second->isMovable();
            // }
            // return false;
        }

        void NodeManager::setIsMovable(NodeId id, bool isMovable)
        {
            throw std::logic_error("NodeManager::setIsMovable not implemented yet");
            // NodeMap::iterator iter = simNodes.find(id);
            // if(iter != simNodes.end())
            // {
            //     iter->second->setMovable(isMovable);
            // }
        }

        void NodeManager::printNodeMasses(bool onlysum)
        {
            throw std::logic_error("NodeManager::printNodeMasses not implemented yet");
            // NodeMap::iterator it;
            // double masssum = 0;
            // for(it=simNodes.begin(); it!=simNodes.end(); ++it)
            // {
            //     if (!onlysum)
            //     {
            //         fprintf(stderr, "%s: %f\n", it->second->getName().c_str(), it->second->getMass());
            //     }
            //     masssum+=it->second->getMass();
            // }
            // fprintf(stderr, "Sum of masses of imported model: %f\n", masssum);
        }

        void NodeManager::edit(NodeId id, const std::string &key,
                                const std::string &value)
        {
            // TODO: Implement
            throw std::logic_error("NodeManager::edit not implemented yet");
            // iMutex.lock();
            // NodeMap::iterator iter;

            // // todo: cfdir1 is a vector
            // iter = simNodes.find(id);
            // if(iter == simNodes.end())
            // {
            //     // hack to attache camera to node:
            //     iMutex.unlock();
            //     if(control->graphics)
            //     {
            //         if(matchPattern("*/node", key))
            //         {
            //             id = getID(value);
            //             unsigned long drawId = 0;
            //             if(id)
            //             {
            //                 drawId = getDrawID(id);
            //             }
            //             control->graphics->attacheCamToNode(1, drawId);
            //         }
            //     }
            //     return;
            // }
            // NodeData nd = iter->second->getSNode();
            // //// fprintf(stderr, "change: %s %s\n", key.c_str(), value.c_str());
            // if(matchPattern("*/attach_camera", key))
            // {
            //     iMutex.unlock();
            //     if(control->graphics)
            //     {
            //         int cameraId = atoi(value.c_str());
            //         unsigned long drawId = getDrawID(id);
            //         control->graphics->attacheCamToNode(cameraId, drawId);
            //     }
            //     return;
            // }
            // else if(matchPattern("*/attach_node", key))
            // {
            //     iMutex.unlock();
            //     interfaces::JointData jointdata;
            //     jointdata.nodeIndex1 = id;
            //     jointdata.nodeIndex2 = getID(value);
            //     jointdata.type = interfaces::JOINT_TYPE_FIXED;
            //     jointdata.name = "connector_"+nd.name+"_"+value;
            //     unsigned long jointid = control->joints->addJoint(&jointdata, true);
            //     return;
            // }
            // else if(matchPattern("*/dettach_node", key))
            // {
            //     iMutex.unlock();
            //     string name =  "connector_"+nd.name+"_"+value;
            //     unsigned long jointid = control->joints->getID(name);
            //     if(jointid)
            //     {
            //         control->joints->removeJoint(jointid);
            //     }
            //     return;
            // }
            // else if(matchPattern("*/position", key))
            // {
            //     //// fprintf(stderr, "position\n");
            //     double v = atof(value.c_str());
            //     if(key[key.size()-1] == 'x') nd.pos.x() = v;
            //     else if(key[key.size()-1] == 'y') nd.pos.y() = v;
            //     else if(key[key.size()-1] == 'z') nd.pos.z() = v;
            //     iMutex.unlock();
            //     control->nodes->editNode(&nd, (EDIT_NODE_POS | EDIT_NODE_MOVE_ALL));
            // }
            // else if(matchPattern("*/linear_velocity", key))
            // {
            //     double v = atof(value.c_str());
            //     Vector v1 = iter->second->getLinearVelocity();
            //     if(key[key.size()-1] == 'x') v1.x() = v;
            //     else if(key[key.size()-1] == 'y') v1.y() = v;
            //     else if(key[key.size()-1] == 'z') v1.z() = v;
            //     iMutex.unlock();
            //     setVelocity(id, v1);
            // }
            // else if(matchPattern("*/angular_velocity", key))
            // {
            //     double v = atof(value.c_str());
            //     Vector v1 = iter->second->getAngularVelocity();
            //     if(key[key.size()-1] == 'x') v1.x() = v;
            //     else if(key[key.size()-1] == 'y') v1.y() = v;
            //     else if(key[key.size()-1] == 'z') v1.z() = v;
            //     iMutex.unlock();
            //     setAngularVelocity(id, v1);
            // }
            // else if(matchPattern("*/rotation", key))
            // {
            //     //// fprintf(stderr, "rotation\n");
            //     double v = atof(value.c_str());
            //     sRotation r = quaternionTosRotation(nd.rot);
            //     bool setEuler = false;
            //     if(key.find("alpha") != string::npos) r.alpha = v, setEuler = true;
            //     else if(key.find("beta") != string::npos) r.beta = v, setEuler = true;
            //     else if(key.find("gamma") != string::npos) r.gamma = v, setEuler = true;
            //     else if(key[key.size()-1] == 'x') nd.rot.x() = v;
            //     else if(key[key.size()-1] == 'y') nd.rot.y() = v;
            //     else if(key[key.size()-1] == 'z') nd.rot.z() = v;
            //     else if(key[key.size()-1] == 'w') nd.rot.w() = v;
            //     if(setEuler) nd.rot = eulerToQuaternion(r);
            //     iMutex.unlock();
            //     control->nodes->editNode(&nd, (EDIT_NODE_ROT | EDIT_NODE_MOVE_ALL));
            // }
            // else if(matchPattern("*/extend/*", key))
            // {
            //     //// fprintf(stderr, "extend\n");
            //     double v = atof(value.c_str());
            //     if(key[key.size()-1] == 'x') nd.ext.x() = v;
            //     else if(key[key.size()-1] == 'y') nd.ext.y() = v;
            //     else if(key[key.size()-1] == 'z') nd.ext.z() = v;
            //     changeNode(iter->second, &nd);
            //     iMutex.unlock();
            // }
            // else if(matchPattern("*/material", key))
            // {
            //     //// fprintf(stderr, "material\n");
            //     iMutex.unlock();
            //     if(control->graphics)
            //     {
            //         std::vector<interfaces::MaterialData> mList;
            //         std::vector<interfaces::MaterialData>::iterator it;
            //         mList = control->graphics->getMaterialList();
            //         for(it=mList.begin(); it!=mList.end(); ++it)
            //         {
            //             if(it->name == value)
            //             {
            //                 unsigned long drawID = getDrawID(id);
            //                 control->graphics->setDrawObjectMaterial(drawID, *it);
            //                 iter->second->setMaterialName(it->name);
            //                 break;
            //             }
            //         }
            //     }
            // }
            // else if(matchPattern("*/cullMask", key))
            // {
            //     //// fprintf(stderr, "cullMask\n");
            //     int v = atoi(value.c_str());
            //     iter->second->setCullMask(v);
            //     iMutex.unlock();
            // }
            // else if(matchPattern("*/c*", key))
            // {
            //     //// fprintf(stderr, "contact\n");
            //     contact_params c = iter->second->getContactParams();
            //     if(matchPattern("*/cmax_num_contacts", key))
            //     {
            //         c.max_num_contacts = atoi(value.c_str());;
            //     }
            //     else if(matchPattern("*/cerp", key)) c.erp = atof(value.c_str());
            //     else if(matchPattern("*/ccfm", key)) c.cfm = atof(value.c_str());
            //     else if(matchPattern("*/cfriction1", key)) c.friction1 = atof(value.c_str());
            //     else if(matchPattern("*/cfriction2", key)) c.friction2 = atof(value.c_str());
            //     else if(matchPattern("*/cmotion1", key)) c.motion1 = atof(value.c_str());
            //     else if(matchPattern("*/cmotion2", key)) c.motion2 = atof(value.c_str());
            //     else if(matchPattern("*/cfds1", key)) c.fds1 = atof(value.c_str());
            //     else if(matchPattern("*/cfds2", key)) c.fds2 = atof(value.c_str());
            //     else if(matchPattern("*/cbounce", key)) c.bounce = atof(value.c_str());
            //     else if(matchPattern("*/cbounce_vel", key)) c.bounce_vel = atof(value.c_str());
            //     else if(matchPattern("*/capprox", key))
            //     {
            //         if(value == "true" || value == "True") c.approx_pyramid = true;
            //         else c.approx_pyramid = false;
            //     }
            //     else if(matchPattern("*/coll_bitmask", key)) c.coll_bitmask = atoi(value.c_str());
            //     else if(matchPattern("*/cfdir1*", key))
            //     {
            //         double v = atof(value.c_str());
            //         if(!c.friction_direction1) c.friction_direction1 = new Vector(0,0,0);
            //         if(key[key.size()-1] == 'x') c.friction_direction1->x() = v;
            //         else if(key[key.size()-1] == 'y') c.friction_direction1->y() = v;
            //         else if(key[key.size()-1] == 'z') c.friction_direction1->z() = v;
            //         if(c.friction_direction1->norm() < 0.00000001)
            //         {
            //             delete c.friction_direction1;
            //             c.friction_direction1 = 0;
            //         }
            //     }
            //     else if(matchPattern("*/rolling_friction2", key)) c.rolling_friction2 = atof(value.c_str());
            //     else if(matchPattern("*/rolling_friction", key)) c.rolling_friction = atof(value.c_str());
            //     else if(matchPattern("*/spinning_friction", key)) c.spinning_friction = atof(value.c_str());
            //     iter->second->setContactParams(c);
            //     iMutex.unlock();
            // }
            // else if(matchPattern("*/brightness", key))
            // {
            //     //// fprintf(stderr, "brightness\n");
            //     double v = atof(value.c_str());
            //     iter->second->setBrightness(v);
            //     iMutex.unlock();
            // }
            // else if(matchPattern("*/name", key))
            // {
            //     //// fprintf(stderr, "name\n");
            //     nd.name = value;
            //     changeNode(iter->second, &nd);
            //     iMutex.unlock();
            // }
            // else if(matchPattern("*/mass", key))
            // {
            //     //// fprintf(stderr, "mass\n");
            //     nd.mass = atof(value.c_str());
            //     changeNode(iter->second, &nd);
            //     iMutex.unlock();
            // }
            // else if(matchPattern("*/density", key))
            // {
            //     //// fprintf(stderr, "density\n");
            //     nd.density = atof(value.c_str());
            //     changeNode(iter->second, &nd);
            //     iMutex.unlock();
            // }
            // else if(matchPattern("*/movable", key))
            // {
            //     //// fprintf(stderr, "movable\n");
            //     ConfigMap b;
            //     b["bool"] = value;
            //     nd.movable = b["bool"];
            //     changeNode(iter->second, &nd);
            //     iMutex.unlock();
            // }
            // else if(matchPattern("*/groupid", key))
            // {
            //     //// fprintf(stderr, "groupid\n");
            //     nd.groupID = atoi(value.c_str());
            //     changeNode(iter->second, &nd);
            //     iMutex.unlock();
            // }
            // else if(matchPattern("*/relativeid", key))
            // {
            //     //// fprintf(stderr, "relativeid\n");
            //     nd.relative_id = atoi(value.c_str());
            //     iter->second->setRelativeID(nd.relative_id);
            //     iMutex.unlock();
            // }
            // else
            // {
            //     fprintf(stderr, "pattern not found: %s %s\n", key.c_str(),
            //             value.c_str());
            //     iMutex.unlock();
            // }
        }

        void NodeManager::changeNode(std::shared_ptr<SimNode> editedNode, NodeData *nodeS)
        {
            throw std::logic_error("NodeManager::changeNode not implemented yet");
            // NodeData sNode = editedNode->getSNode();
            // if(control->graphics)
            // {
            //     Vector scale;
            //     if(sNode.filename == "PRIMITIVE")
            //     {
            //         scale = nodeS->ext;
            //         if(sNode.physicMode == NODE_TYPE_SPHERE)
            //         {
            //             scale.x() *= 2;
            //             scale.y() = scale.z() = scale.x();
            //         }
            //         // todo: set scale for cylinder and capsule
            //     }
            //     else
            //     {
            //         scale = sNode.visual_size-sNode.ext;
            //         scale += nodeS->ext;
            //         nodeS->visual_size = scale;
            //     }
            //     control->graphics->setDrawObjectScale(editedNode->getGraphicsID(), scale);
            //     control->graphics->setDrawObjectScale(editedNode->getGraphicsID2(), nodeS->ext);
            // }
            // editedNode->changeNode(nodeS);
            // if(sNode.groupID != 0 || nodeS->groupID != 0)
            // {
            //     for(auto it: simNodes)
            //     {
            //         if(it.second->getGroupID() == sNode.groupID ||
            //             it.second->getGroupID() == nodeS->groupID) 
            //         {
            //             control->joints->reattacheJoints(it.second->getID());
            //         }
            //     }
            // }
            // control->joints->reattacheJoints(nodeS->index);

            // if(nodeS->groupID > maxGroupID)
            // {
            //     maxGroupID = nodeS->groupID;
            // }
            // nodesToUpdate[sNode.index] = editedNode;
        }

        bool NodeManager::isRootFrame(const interfaces::NodeId& node_id) const
        {
            if (!idManager_->isKnown(node_id))
            {
                return false;
            }

            const auto& nodeName = idManager_->getName(node_id);
            return nodeName == SIM_CENTER_FRAME_NAME;
        }
    } // end of namespace core
} // end of namespace mars
