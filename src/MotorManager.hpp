/**
 * \file MotorManager.hpp
 * \author  Malte Langosz
 * \brief "MotorManager" implements the MotorManagerInterface.
 * It is manages all motors and all motor
 * operations that are used for the communication between the simulation
 * modules.
 *
 */

#pragma once

#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_interfaces/sim/MotorManagerInterface.h>
#include <mars_interfaces/sim/IDManager.hpp>
#include <mars_utils/Mutex.h>
namespace mars
{
    namespace core
    {
        class SimMotor;

        /**
         * \brief "MotorManager" imlements the interfaces for all motor
         * operations that are used for the communication between the simulation
         * modules. Inherits from MotorManagerInterface.
         *
         * \warning It is very important to assure the serialization between the threads to
         * have the desired results. Currently the verified use of the functions
         * is only guaranteed by calling it within the main thread (update
         * callback from \c gui_thread).
         */
        class MotorManager : public interfaces::MotorManagerInterface 
        {
        public:

            /**
             * \brief Constructor.
             *
             * \param c The pointer to the ControlCenter of the simulation.
             */
            MotorManager(interfaces::ControlCenter *c);

            /**
             * \brief Destructor.
             */
            virtual ~MotorManager() override {}

            MotorManager(const MotorManager&) = delete;
            MotorManager operator=(const MotorManager&) = delete;
            MotorManager(MotorManager&&) = delete;
            MotorManager operator=(MotorManager&&) = delete;

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
            virtual unsigned long addMotor(interfaces::MotorData *motorS, bool reload =  false) override;

            /**
             *\brief Returns the number of motors that are currently present in the simulation.
             *
             *\return The number of all motors.
             */
            virtual int getMotorCount() const override;

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
            virtual void editMotor(const interfaces::MotorData &motorS) override;

            /**
             * \brief Gives information about core exchange data for motors.
             *
             * \param motorList A pointer to a vector that is filled with a
             * core_objects_exchange struct for every motor and its index. The vector is cleared
             * in the beginning of this function.
             */
            virtual void getListMotors(std::vector<interfaces::core_objects_exchange> *motorList) const override;


            /**
             * \brief Gives all information of a certain motor.
             *
             * \param index The unique id of the motor to get information for.
             *
             * \return A pointer to the MotorData of the motor with the given id.
             */
            virtual const interfaces::MotorData getFullMotor(unsigned long index) const override;

            /**
             * \brief Removes a motor from the simulation.
             *
             * \param index The unique id of the motor to remove form the simulation.
             */
            virtual void removeMotor(unsigned long index) override;

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
            virtual SimMotor* getSimMotor(unsigned long id) const override;

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
            virtual SimMotor* getSimMotorByName(const std::string &name) const override;

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
            virtual void setMotorValue(unsigned long id, interfaces::sReal value) override;
            virtual void setMotorFFTorque(unsigned long id, interfaces::sReal value) override;

            /**
             * \brief Sets the maximum torque of the motor with the given id to the given value.
             *
             * \param id The id of the motor whose value is to be changed.
             *
             * \param maxTorque The new maximum torque for the motor.
             */
            virtual void setMaxTorque(unsigned long id, interfaces::sReal maxTorque) override;
            virtual void setMaxSpeed(unsigned long id, interfaces::sReal maxSpeed) override;

            /**
             * \brief Sets the desired speed of a motor.
             * \param id The id of the motor whose value is to be changed.
             * \param value The new value in rad/s.
             */
            virtual void setMotorValueDesiredVelocity(unsigned long id, interfaces::sReal velocity) override;

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
            virtual void setMotorP(unsigned long id, interfaces::sReal value) override;

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
            virtual void setMotorI(unsigned long id, interfaces::sReal value) override;

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
            virtual void setMotorD(unsigned long id, interfaces::sReal value) override;

            /**
             * \brief Deactivates the motor with the given id.
             *
             * \param id The id of the motor that is to be deactivated.
             */
            virtual void deactivateMotor(unsigned long id) override;

            /**
             * \brief Destroys all motors in the simulation.
             *
             * \details The \c clear_all flag indicates if the reload motors should
             * be destroyed as well. If set to \c false they are left intact.
             *
             * \param clear_all Indicates if the reload motors should
             * be destroyed as well. If set to \c false they are left intact.
             */
            virtual void clearAllMotors(bool clear_all = false) override;

            /**
             * \brief This function reloads all motors from a temporary motor pool.
             *
             * \details All motors that have been added with \c reload value as \c true
             * are added back to the simulation again with a \c reload value of \c true.
             */
            virtual void reloadMotors(void) override;

            /**
             * \brief This function updates all motors with timing value \c calc_ms in miliseconds.
             *
             * \warning This function is only used internally and should not be called
             * outside the core.
             *
             * \param calc_ms The timing value in miliseconds.
             */
            virtual void updateMotors(interfaces::sReal calc_ms) override;

            /**
             * \returns the actual position of the motor with the given Id.
             *          returns 0 if a motor with the given Id doesn't exist.
             */
            virtual interfaces::sReal getActualPosition(unsigned long motorId) const override;

            /**
             * \returns the torque excerted by the motor with the given Id.
             *          returns 0 if a motor with the given Id doesn't exist.
             */
            virtual interfaces::sReal getTorque(unsigned long motorId) const override;

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
            virtual void moveMotor(unsigned long index, interfaces::sReal value) override;

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
            virtual void removeJointFromMotors(unsigned long joint_index) override;

            /**
             * \brief Retrieves the id of a motor by name
             *
             * \param motor_name Name of the motor to get the id for
             *
             * \return Id of the motor if it exists, otherwise 0
             */
            virtual unsigned long getID(const std::string& motor_name) const override;
            unsigned long registerMotorID(const std::string& motor_name) const;

            virtual void getDataBrokerNames(unsigned long jointId,
                                            std::string *groupName,
                                            std::string *dataName) const override;

            virtual void connectMimics() override;
            virtual void setOfflinePosition(interfaces::MotorId id,
                                            interfaces::sReal pos);
            virtual void edit(interfaces::MotorId id, const std::string &key,
                              const std::string &value) override;

            void addSimMotor(std::shared_ptr<SimMotor> newMotor);
        private:
            //! a container for all motors currently present in the simulation
            std::map<unsigned long, SimMotor*> simMotors;
            //! a mutex for the motor containter
            mutable utils::Mutex simMotorsMutex;
            std::unique_ptr<interfaces::IDManager> idManager_;

            //! a pointer to the control center
            interfaces::ControlCenter *control;

        }; // class MotorManager
    } // end of namespace core
} // end of namespace mars
