#pragma once

//#include "SimJoint.h"
#include "PID.hpp"

#include <data_broker/ProducerInterface.h>
#include <data_broker/ReceiverInterface.h>
#include <data_broker/DataPackage.h>
#include <mars_interfaces/MotorData.h>
#include <mars_interfaces/core_objects_exchange.h>
#include <mars_interfaces/ConfigMapInterface.hpp>
#include <mars_utils/mathUtils.h>

#include <iostream>

namespace mars
{

    namespace interfaces
    {
        class ControlCenter;
        class JointInterface;
    }

    namespace core
    {

        double SpaceClimberCurrent(double* torque, double* velocity, std::vector<interfaces::sReal>* c);

        /**
         * Each SimMotor object publishes its state on the dataBroker.
         * The name under which the data is published can be obtained from the
         * motorId via MotorManager::getDataBrokerNames.
         * The data_broker::DataPackage will contain the following items:
         *  - "id" (long)
         *  - "value" (double)
         *  - "position" (double)
         *  - "current" (double)
         *  - "torque" (double)
         */
        class SimMotor : public data_broker::ProducerInterface ,
                         public data_broker::ReceiverInterface ,
                         public mars::interfaces::ConfigMapInterface
        {

        public:
            SimMotor(interfaces::ControlCenter *control,
                     const interfaces::MotorData &sMotor,
                     std::weak_ptr<interfaces::JointInterface> joint);
            ~SimMotor(void);

            void init(const std::string& name = "", interfaces::MotorType type = interfaces::MOTOR_TYPE_UNDEFINED);

            // function methods

            void update(interfaces::sReal time_ms);
            void updateController();
            void activate(void);
            void deactivate(void);
            //void attachJoint(std::shared_ptr<SimJoint> joint);
            //void attachPlayJoint(std::shared_ptr<SimJoint> joint);
            void estimateCurrent();
            void estimateTemperature(interfaces::sReal time_ms);
            // the following two functions might be simple getters or carry out calculations
            interfaces::sReal getMomentaryMaxEffort();
            interfaces::sReal getMomentaryMaxSpeed();
            void refreshPosition();
            void refreshPositions();
            void setOfflinePosition(interfaces::sReal value);
            void runPositionController(interfaces::sReal time_ms);
            void runVelocityController(interfaces::sReal time_ms);
            void runEffortController(interfaces::sReal time_ms);
            void runEffortPipe(interfaces::sReal time);
            void runFFEffortPipe(interfaces::sReal time);
            void addMimic(SimMotor* mimic);
            void removeMimic(std::string mimicname);
            void clearMimics();
            void setMaxEffortApproximation(utils::ApproximationFunction type,
                                           std::vector<double>* coefficients);
            void setMaxSpeedApproximation(utils::ApproximationFunction type,
                                          std::vector<double>* coefficients);
            void setCurrentApproximation(utils::ApproximationFunction2D type,
                                         std::vector<double>* coefficients);

            // getters
            int getAxis() const;
            interfaces::sReal getAxisPosition(void) const;
            void getCoreExchange(interfaces::core_objects_exchange* obj) const;
            interfaces::sReal getCurrent(void) const;
            interfaces::sReal getEffort(void) const;
            unsigned long getIndex(void) const;
            bool isServo() const;
            //std::shared_ptr<SimJoint>  getJoint() const;
            unsigned long getJointIndex(void) const;
            std::string getJointName(void) const;
            const std::string getName() const;
            interfaces::sReal getMaxEffort() const;
            interfaces::sReal getMaxSpeed() const;
            //std::shared_ptr<SimJoint>  getPlayJoint() const;
            interfaces::sReal getPosition() const;
            const interfaces::MotorData getSMotor(void) const;
            interfaces::sReal getVelocity() const;
            interfaces::sReal getControlParameter(void) const;
            interfaces::sReal getControlValue(void) const;
            interfaces::sReal getP() const;
            interfaces::sReal getI() const;
            interfaces::sReal getD() const;

            void getDataBrokerNames(std::string *groupName, std::string *dataName) const;

            // setters
            void setPosition(interfaces::sReal angle);
            void setMaxEffort(interfaces::sReal effort);
            void setMaxSpeed(interfaces::sReal value);
            void setName(const std::string &newname);
            void setSMotor(const interfaces::MotorData &sMotor);
            void setType(interfaces::MotorType mtype);
            void setP(interfaces::sReal p);
            void setI(interfaces::sReal i);
            void setD(interfaces::sReal d);
            void setPID(interfaces::sReal mP, interfaces::sReal mI, interfaces::sReal mD);
            void setMinValue(interfaces::sReal d);
            void setMaxValue(interfaces::sReal d);
            void setVelocity(interfaces::sReal v);
            void setControlValue(interfaces::sReal value);
            void setFeedForwardTorque(interfaces::sReal value);
            void setMimic(interfaces::sReal multiplier, interfaces::sReal offset);


            // methods inherited from data broker interfaces
            virtual void produceData(const data_broker::DataInfo &info,
                                     data_broker::DataPackage *package,
                                     int callbackParam) override;
            virtual void receiveData(const data_broker::DataInfo &info,
                                     const data_broker::DataPackage &package,
                                     int callbackParam) override;

            // methods inherited from mars::interfaces::ConfigMapInterface
            virtual configmaps::ConfigMap getConfigMap() const override;
            virtual std::vector<std::string> getEditPattern(const std::string& basePath) const override;
            virtual void edit(const std::string& configPath, const std::string& value) override;

            // methods to be deprecated in future MARS versions
            interfaces::sReal getMotorMaxForce() const __attribute__ ((deprecated("use getMaxEffort")));
            interfaces::sReal getMaximumVelocity() const __attribute__ ((deprecated("use getMaxSpeed")));
            interfaces::sReal getTorque(void) const __attribute__ ((deprecated("use getEffort")));
            interfaces::sReal getValue(void) const __attribute__ ((deprecated("use getControlValue")));
            interfaces::sReal getActualPosition(void) const __attribute__ ((deprecated("use getPosition")));
            interfaces::sReal getDesiredMotorAngle() const __attribute__ ((deprecated("use getControlValue")));

            void setActualAngle(interfaces::sReal angle) __attribute__ ((deprecated("use setCurrentPosition")));
            void setDesiredMotorAngle(interfaces::sReal angle) __attribute__ ((deprecated("use setPosition / setControlValue")));
            void setMaximumVelocity(interfaces::sReal value) __attribute__ ((deprecated("use setMaxSpeed")));
            void setMotorMaxForce(interfaces::sReal force) __attribute__ ((deprecated("use setMaxEffort")));
            void setValue(interfaces::sReal value) __attribute__ ((deprecated("use setControlValue")));
            void setDesiredMotorVelocity(interfaces::sReal value) __attribute__ ((deprecated("if use dc motor setControlValue otherwise you could use setMaxVelocity?")));
            void setValueDesiredVelocity(interfaces::sReal value) __attribute__ ((deprecated("if use dc motor setControlValue otherwise you could use setMaxVelocity?")));
            void refreshAngle() __attribute__ ((deprecated("use refreshPosition(s)")));

        private:
            void initPIDs();

            // typedefs for function pointers
            typedef  void (interfaces::JointInterface::*JointControlFunction)(interfaces::sReal);
            typedef void (SimMotor::*MotorControlFunction)(interfaces::sReal);
            typedef double (*ApproximationFunction)(double*, std::vector<double>*);
            typedef double (*ApproximationFunction2D)(double*, double*, std::vector<double>*);

            // motor
            unsigned char axis;
            //std::shared_ptr<SimJoint>  myJoint, myPlayJoint;
            interfaces::ControlCenter *control;
            interfaces::MotorData sMotor;
            std::weak_ptr<interfaces::JointInterface> joint;
            interfaces::sReal time;
            interfaces::sReal lastVelocity, velocity, position1, velocity1, position2, effort;
            interfaces::sReal sensedEffort;
            interfaces::sReal tmpmaxeffort, tmpmaxspeed;
            interfaces::sReal current, temperature, filterValue;
            interfaces::sReal *position; // we use this pointer to access whatever axis-position is used
            interfaces::sReal feedForwardEffort, feedForwardEffortIntern;
            bool active;
            std::map<std::string, SimMotor*> mimics;
            bool mimic;
            interfaces::sReal mimic_multiplier;
            interfaces::sReal mimic_offset;

            // controller part
            interfaces::sReal controlValue;
            interfaces::sReal* controlParameter;
            interfaces::sReal* controlLimit;
            JointControlFunction setJointControlParameter;
            MotorControlFunction runController;
            interfaces::sReal last_error;
            interfaces::sReal integ_error;
            interfaces::sReal joint_velocity;
            interfaces::sReal error;

            // function approximation
            double * maxspeed_x;
            double * maxeffort_x;
            // TODO: These should not be pointers.
            std::vector<interfaces::sReal>* maxeffort_coefficients;
            std::vector<interfaces::sReal>* maxspeed_coefficients;
            std::vector<interfaces::sReal>* current_coefficients;
            ApproximationFunction maxEffortApproximation;
            ApproximationFunction maxSpeedApproximation;
            ApproximationFunction2D currentApproximation;

            // current estimation
            void initCurrentEstimation();
            interfaces::sReal kXY, kX, kY, k;

            // temperature estimation
            //FIXME: add voltage & ambientTemperature to sMotor and read from ConfigMap
            void initTemperatureEstimation();
            interfaces::sReal voltage;
            interfaces::sReal ambientTemperature; // not all motors are exposed to a general outside temperature
            interfaces::sReal heatlossCoefficient;
            interfaces::sReal heatCapacity;
            interfaces::sReal heatTransferCoefficient;
            interfaces::sReal calcHeatDissipation(interfaces::sReal time_ms) const;
            interfaces::sReal calcHeatProduction(interfaces::sReal time_ms) const;

            // for dataBroker communication
            data_broker::DataPackage dbPackage, cmdPackage;
            unsigned long dbPushId, dbCmdId;
            long dbIdIndex, dbControlParameterIndex, dbPositionIndex, dbCurrentIndex, dbEffortIndex, dbMaxEffortIndex, dbVelocityIndex, dbTargetVelocityIndex, dbTargetTorqueIndex, dbVelErrorIndex, dbVelIIndex;
            bool effortMotor;
            int pushToDataBroker;
            PID posPID, velPID;
            interfaces::sReal ffEffortFilter, ffEffortGain;
        };

    } // end of namespace core
} // end of namespace mars
