/**
 * \file MotorManager.cpp
 * \author  Malte Langosz
 * \brief "MotorManager" implements MotorManagerInterface.
 * It is manages all motors and all motor
 * operations that are used for the communication between the simulation
 * modules.
 */

#include "SimMotor.hpp"
#include "MotorManager.hpp"
#include "JointManager.hpp"

#include <stdexcept>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_types/motors/DC.hpp>
#include <envire_types/motors/PID.hpp>
#include <envire_types/motors/DirectEffort.hpp>

#include <mars_interfaces/sim/SimulatorInterface.h>
#include <mars_utils/MutexLocker.h>
#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>


namespace mars
{
    namespace core
    {
        using namespace utils;
        using namespace interfaces;

        /**
         * \brief Constructor.
         *
         * \param c The pointer to the ControlCenter of the simulation.
         */
        MotorManager::MotorManager(ControlCenter *c) : control{c}
        {}

        /**
         * \brief Add a motor to the simulation.
         *
         * \param motorS A pointer to the MotorData that defines the new motor.
         *
         * \param reload Used internally by the simulation. The
         * default value is \c false. If this param is set to \c true the new motor
         * will not be reloaded after a reset of the simulation.
         *
         * \return The unique id of the newly added motor.
         */
        unsigned long MotorManager::addMotor(MotorData *motorS, bool reload)
        {
            motorS->index = ControlCenter::motorIDManager->addIfUnknown(motorS->name);

            // TODO: Create envire motor from MotorData
            //  - SimMotor will be added from envire_mars_motors

            return motorS->index;
        }


        /**
         *\brief Returns the number of motors that are currently present in the simulation.
         *
         *\return The number of all motors.
         */
        int MotorManager::getMotorCount() const
        {
            const MutexLocker locker{&simMotorsMutex};
            return simMotors.size();
        }


        /**
         * \brief Change motor properties.
         *
         * \details The old struct is replaced
         * by the new one completely, so prior to calling this function, one must
         * ensure that all properties of this parameter are valid and as desired.
         *
         * \param motorS The id of the MotorData referred by this pointer must be the
         * same as the id of the motor that is to be edited.
         */
        void MotorManager::editMotor(const MotorData &motorS)
        {
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(motorS.index);
            if(iter != simMotors.end())
                iter->second->setSMotor(motorS);
        }


        /**
         * \brief Gives information about core exchange data for motors.
         *
         * \param motorList A pointer to a vector that is filled with a
         * core_objects_exchange struct for every motor and its index. The vector is cleared
         * in the beginning of this function.
         */
        void MotorManager::getListMotors(std::vector<core_objects_exchange> *motorList)const
        {
            core_objects_exchange obj;
            decltype(simMotors)::const_iterator iter;
            motorList->clear();
            simMotorsMutex.lock();
            for(iter = simMotors.begin(); iter != simMotors.end(); iter++)
            {
                iter->second->getCoreExchange(&obj);
                motorList->push_back(obj);
            }
            simMotorsMutex.unlock();
        }


        /**
         * \brief Gives all information of a certain motor.
         *
         * \param index The unique id of the motor to get information for.
         *
         * \return A pointer to the MotorData of the motor with the given id.
         * \throw std::runtime_error if a motor with the given index does not exist.
         */
        const MotorData MotorManager::getFullMotor(unsigned long index) const
        {
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(index);
            if(iter != simMotors.end())
            {
                return iter->second->getSMotor();
            }
            else
            {
                const auto errorMessage = std::string{"could not find motor with index: " + std::to_string(index)};
                throw std::runtime_error(errorMessage);
            }
        }


        /**
         * \brief Removes a motor from the simulation.
         *
         * \param index The unique id of the motor to remove form the simulation.
         */
        void MotorManager::removeMotor(unsigned long index)
        {
            simMotorsMutex.lock();
            const auto& iter = simMotors.find(index);
            if(iter != simMotors.end())
            {
                simMotors.erase(iter);
            }
            simMotorsMutex.unlock();

            if (ControlCenter::motorIDManager->isKnown(index))
            {
                ControlCenter::motorIDManager->removeEntry(index);
            }

            if(control)
            {
                constexpr bool sceneWasReseted = false;
                control->sim->sceneHasChanged(sceneWasReseted);
            }
        }


        /**
         * \brief This function returns the SimMotor object for a given id.
         *
         * \warning This method is only internal used by the
         * MotorManager. Generally no other modules know the SimMotor class and
         * shouldn't use this method. All motor operations from outside the core
         * should be done over the MotorManager.
         *
         * \param index The id of the motor to get the core node object.
         *
         * \returns Returns a pointer to the corresponding motor object.
         */
        SimMotor* MotorManager::getSimMotor(unsigned long id) const
        {
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(id);
            if(iter == simMotors.end())
            {
                return nullptr;
            }

            return iter->second;
        }



        /**
         * \brief This function returns the SimMotor object for a given name.
         *
         * \warning This method is only internal used by the
         * MotorManager. Generally no other modules know the SimMotor class and
         * shouldn't use this method. All motor operations from outside the core
         * should be done over the MotorManager.
         *
         * \param name The name of the motor to get the core node object.
         *
         * \returns Returns a pointer to the corresponding motor object.
         */
        SimMotor* MotorManager::getSimMotorByName(const std::string &name) const
        {
            const auto& motorID = getID(name);
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(motorID);
            if (iter==simMotors.end())
            {
                return nullptr;
            }

            return iter->second;
        }


        /**
         * \brief Sets the value of the motor with the given id to the given value.
         *
         * Essentially this function triggers the motor and moves the joint that is
         * attached to it.
         * Equivalent to \c moveMotor
         *
         * \param id The id of the motor whose value is to be changed.
         *
         * \param value The new value.
         */
        void MotorManager::setMotorValue(unsigned long id, sReal value)
        {
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(id);
            if(iter != simMotors.end())
            {
                iter->second->setControlValue(value);
            }
        }


        void MotorManager::setMotorValueDesiredVelocity(unsigned long id, sReal velocity)
        {
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(id);
            if(iter != simMotors.end())
            {
                iter->second->setVelocity(velocity);
            }
        }



        /**
         * \brief Sets the proportional term of the motor with the given id to the given value.
         *
         * \details Only has effect on a PID motor. If the type of the motor with
         * the given id is different from PID, no effect is observed, although the
         * P value of the motor object is still changed.
         *
         * \param id The id of the motor whose P value is to be changed.
         *
         * \param value The new P value.
         */
        void MotorManager::setMotorP(unsigned long id, sReal value)
        {
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(id);
            if(iter != simMotors.end())
                iter->second->setP(value);
        }


        /**
         * \brief Sets the integral term of the motor with the given id to the given value.
         *
         * \details Only has effect on a PID motor. If the type of the motor with
         * the given id is different from PID, no effect is observed, although the
         * I value of the motor object is still changed.
         *
         * \param id The id of the motor whose I value is to be changed.
         *
         * \param value The new I value.
         */
        void MotorManager::setMotorI(unsigned long id, sReal value)
        {
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(id);
            if(iter != simMotors.end())
                iter->second->setI(value);
        }


        /**
         * \brief Sets the derivative term of the motor with the given id to the given value.
         *
         * \details Only has effect on a PID motor. If the type of the motor with
         * the given id is different from PID, no effect is observed, although the
         * D value of the motor object is still changed.
         *
         * \param id The id of the motor whose D value is to be changed.
         *
         * \param value The new D value.
         */
        void MotorManager::setMotorD(unsigned long id, sReal value)
        {
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(id);
            if(iter != simMotors.end())
                iter->second->setD(value);
        }


        /**
         * \brief Deactivates the motor with the given id.
         *
         * \param id The id of the motor that is to be deactivated.
         */
        void MotorManager::deactivateMotor(unsigned long id)
        {
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(id);
            if(iter != simMotors.end())
                iter->second->deactivate();
        }


        /**
         * \brief Retrieves the id of a motor by name
         *
         * \param motor_name Name of the motor to get the id for
         *
         * \return Id of the motor if it exists, otherwise 0
         */
        unsigned long MotorManager::getID(const std::string& name) const
        {
            const auto& id = ControlCenter::motorIDManager->getID(name);
            if (id == INVALID_ID)
            {
                const auto msg = std::string{"MotorManager::getID: Can't find motor with the name \""} + name + "\".";
                LOG_ERROR(msg.c_str());
            }
            return id;
        }


        /**
         * \brief Sets the value of the motor with the given id to the given value.
         *
         * Essentially this function triggers the motor and moves the joint that is
         * attached to it.
         * Equivalent to \c setMotorValue
         *
         * \param id The id of the motor whose value is to be changed.
         *
         * \param value The new value.
         */
        void MotorManager::moveMotor(unsigned long index, double value)
        {
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(index);
            if(iter != simMotors.end())
                iter->second->setControlValue(value);
        }


        /**
         * \brief Destroys all motors in the simulation.
         *
         * \details The \c clear_all flag indicates if the reload motors should
         * be destroyed as well. If set to \c false they are left intact.
         *
         * \param clear_all Indicates if the reload motors should
         * be destroyed as well. If set to \c false they are left intact.
         */
        void MotorManager::clearAllMotors(bool clear_all)
        {
            {
                const MutexLocker locker{&simMotorsMutex};
                simMotors.clear();
            }

            auto removeFunctor = [clear_all](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
            {
                itemRemover<std::shared_ptr<SimMotor>>(node);

                // TODO: Is this still needed?
                if (clear_all)
                {
                    itemRemover<envire::types::motors::DC>(node);
                    itemRemover<envire::types::motors::PID>(node);
                    itemRemover<envire::types::motors::DirectEffort>(node);
                }
            };

            const auto& rootVertex = ControlCenter::envireGraph->getVertex(SIM_CENTER_FRAME_NAME);
            ControlCenter::graphTreeView->visitBfs(rootVertex, removeFunctor);

            ControlCenter::motorIDManager->clear();
        }


        /**
         * \brief This function reloads all motors from a temporary motor pool.
         *
         * \details All motors that have been added with \c reload value as \c true
         * are added back to the simulation again with a \c reload value of \c true.
         */
        void MotorManager::reloadMotors(void)
        {
            auto readdFunctor = [](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
            {
                itemReadder<envire::types::motors::DC>(node);
                itemReadder<envire::types::motors::PID>(node);
                itemReadder<envire::types::motors::DirectEffort>(node);
            };
            const auto& rootVertex = ControlCenter::envireGraph->getVertex(SIM_CENTER_FRAME_NAME);
            ControlCenter::graphTreeView->visitBfs(rootVertex, readdFunctor);
        }

        /**
         * \brief This function updates all motors with timing value \c calc_ms in miliseconds.
         *
         * \warning This function is only used internally and should not be called
         * outside the core.
         *
         * \param calc_ms The timing value in miliseconds.
         */
        void MotorManager::updateMotors(double calc_ms)
        {
            decltype(simMotors)::iterator iter;
            const MutexLocker locker{&simMotorsMutex};
            for(iter=simMotors.begin(); iter!=simMotors.end(); iter++)
            {
                iter->second->update(calc_ms);
            }
        }


        sReal MotorManager::getActualPosition(unsigned long motorId) const
        {
            const MutexLocker locker{&simMotorsMutex};
            decltype(simMotors)::const_iterator iter;
            iter = simMotors.find(motorId);
            if(iter != simMotors.end())
            {
                return iter->second->getPosition();
            }
            return 0.;
        }

        sReal MotorManager::getTorque(unsigned long motorId) const
        {
            const MutexLocker locker{&simMotorsMutex};
            decltype(simMotors)::const_iterator iter;
            iter = simMotors.find(motorId);
            if(iter != simMotors.end())
            {
                return iter->second->getEffort();
            }
            return 0.;
        }

        void MotorManager::setMaxTorque(unsigned long id, sReal maxTorque)
        {
            const MutexLocker locker{&simMotorsMutex};
            decltype(simMotors)::const_iterator iter;
            iter = simMotors.find(id);
            if(iter != simMotors.end())
            {
                iter->second->setMaxEffort(maxTorque);
            }
        }

        void MotorManager::setMaxSpeed(unsigned long id, sReal maxSpeed)
        {
            const MutexLocker locker{&simMotorsMutex};
            decltype(simMotors)::const_iterator iter;
            iter = simMotors.find(id);
            if(iter != simMotors.end())
            {
                iter->second->setMaxSpeed(maxSpeed);
            }
        }


        /**
         * \brief Detaches the joint with the given index from all motors that act on
         * it, if any. Used when a joint is destroyed.
         *
         * \warning The detached motors are not destroyed and are still present in the
         * simulation, although they do not have any effect on it. A call to
         * \c removeMotor must be made to remove the motor completely.
         *
         * \param joint_index The id of the joint that is to be detached.
         */
        void MotorManager::removeJointFromMotors(unsigned long joint_index)
        {
            // This is handled indirectly by the weak_ptr to the joint being expired
        }

        void MotorManager::getDataBrokerNames(unsigned long jointId,
                                              std::string *groupName,
                                              std::string *dataName) const
        {
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(jointId);
            if(iter == simMotors.end())
            {
                return;
            }

            iter->second->getDataBrokerNames(groupName, dataName);
        }

        void MotorManager::connectMimics()
        {
            throw std::logic_error("MotorManager::connectMimics is not implemented.");
            // std::map<unsigned long, std::string>::iterator it;
            // for(it=mimicmotors.begin(); it!=mimicmotors.end(); ++it)
            // {
            //     SimMotor* parentmotor = getSimMotorByName(it->second);
            //     if(parentmotor != NULL)
            //         parentmotor->addMimic(simMotors[it->first]);
            // }
        }

        void MotorManager::setOfflinePosition(interfaces::MotorId id,
                                              sReal pos)
        {
            const MutexLocker locker{&simMotorsMutex};
            const auto& iter = simMotors.find(id);
            if(iter == simMotors.end())
            {
                return;
            }

            iter->second->setOfflinePosition(pos);
        }

        void MotorManager::edit(interfaces::MotorId id, const std::string &key,
                                const std::string &value)
        {
            const MutexLocker locker{&simMotorsMutex};
            if(simMotors.count(id) == 0)
            {
                return;
            }

            simMotors.at(id)->edit(key, value);
        }


        void MotorManager::addSimMotor(std::shared_ptr<SimMotor> newMotor)
        {
            const auto& motorID = ControlCenter::motorIDManager->getID(newMotor->getName());
            if (motorID == INVALID_ID)
            {
                throw std::runtime_error{(std::string{"Tried adding unknown motor \""} + newMotor->getName() + "\".").c_str()};
            }
            simMotors[motorID] = newMotor.get();
        }

    } // end of namespace core
} // end of namespace mars
