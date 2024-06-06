#include "SimMotor.hpp"
#include <mars_utils/misc.h> // matchPattern
#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_interfaces/sim/SimulatorInterface.h>
#include <mars_interfaces/sim/JointInterface.h>
#include <data_broker/DataBrokerInterface.h>

#include <cstdio>
#include <cmath>
#include <algorithm>

namespace mars
{
    namespace core
    {

        using namespace utils;
        using namespace interfaces;

        double SpaceClimberCurrent(double* torque, double* velocity,
                                   std::vector<interfaces::sReal>* c)
        {
            return fabs((*c)[0]*fabs((*torque)*(*velocity)) +
                        (*c)[1]*fabs((*torque)) +
                        (*c)[2]*fabs((*velocity)) + (*c)[3]);
        }

        SimMotor::SimMotor(ControlCenter *c, const MotorData &sMotor_, std::weak_ptr<JointInterface> joint)
            : control(c), joint(joint)
        {

            sMotor.index = sMotor_.index;
            sMotor.type = sMotor_.type;
            sMotor.value = sMotor_.value;
            sMotor.value = 0;
            sMotor.name = sMotor_.name.c_str();
            sMotor.maxSpeed = sMotor_.maxSpeed;
            sMotor.maxEffort = sMotor_.maxEffort;
            sMotor.maxAcceleration = sMotor_.maxAcceleration;
            sMotor.maxValue = sMotor_.maxValue;
            sMotor.minValue = sMotor_.minValue;
            sMotor.config = sMotor_.config;
            axis = 0;
            position1 = 0;
            position2 = 0;
            position = &position1;
            lastVelocity = velocity=0;
            joint_velocity = 0;
            time = 10;
            current = 0;
            sensedEffort = effort = 0;
            tmpmaxeffort = 0;
            tmpmaxspeed = 0;
            //myJoint = 0;
            mimic = false;
            mimic_multiplier=1.0;
            mimic_offset=0;
            maxEffortApproximation = &utils::pipe;
            maxSpeedApproximation = &utils::pipe;
            currentApproximation = &SpaceClimberCurrent;
            maxeffort_coefficients = nullptr;
            maxspeed_coefficients = nullptr;
            current_coefficients = nullptr;
            maxspeed_x = &sMotor.maxSpeed;
            maxeffort_x = &sMotor.maxEffort;
            effortMotor = 0;
            feedForwardEffort = 0;
            initPIDs();

            //myPlayJoint = 0;
            active = true;

            // controller
            last_error = 0;
            integ_error = 0;
            controlValue = 0;
            updateController();

            initTemperatureEstimation();
            initCurrentEstimation();

            pushToDataBroker = 2;
            configmaps::ConfigMap &map = sMotor.config;
            if(map.hasKey("noDataPackage") && (bool)map["noDataPackage"] == true)
            {
                pushToDataBroker = 0;
            }
            if(pushToDataBroker > 0)
            {
                cmdPackage.add("value", 0.0);
                dbPackage.add("id", (long)sMotor.index);
                dbPackage.add("value", controlValue);
                dbPackage.add("position", getPosition());
                dbPackage.add("current", getCurrent());
                dbPackage.add("torque", getEffort());
                dbPackage.add("maxtorque", sMotor.maxEffort);

                dbIdIndex = dbPackage.getIndexByName("id");
                dbControlParameterIndex = dbPackage.getIndexByName("value");
                dbPositionIndex = dbPackage.getIndexByName("position");
                dbCurrentIndex = dbPackage.getIndexByName("current");
                dbEffortIndex = dbPackage.getIndexByName("torque");
                dbMaxEffortIndex = dbPackage.getIndexByName("maxtorque");

                std::string groupName, dataName;
                getDataBrokerNames(&groupName, &dataName);
                if(control && control->dataBroker)
                {
                    dbPushId = control->dataBroker->pushData(groupName, dataName,
                                                             dbPackage, nullptr,
                                                             data_broker::DATA_PACKAGE_READ_FLAG);
                    control->dataBroker->registerTimedProducer(this, groupName, dataName,
                                                               "mars_sim/simTimer", 0);
                    dbCmdId = control->dataBroker->pushData(groupName, "cmd/"+dataName,
                                                            cmdPackage, nullptr,
                                                            data_broker::DATA_PACKAGE_READ_WRITE_FLAG);
                    control->dataBroker->registerSyncReceiver(this, groupName, "cmd/"+dataName, 0);
                }
            }
        }

        SimMotor::~SimMotor(void)
        {
            std::string groupName, dataName;
            getDataBrokerNames(&groupName, &dataName);
            if(control && control->dataBroker)
            {
                if(pushToDataBroker)
                {
                    control->dataBroker->unregisterTimedProducer(this, groupName, dataName,
                                                                 "mars_sim/simTimer");
                }
                control->dataBroker->unregisterSyncReceiver(this, "*", "*");
            }
            // if we have to delete something we can do it here
            // if(myJoint) {
            //     myJoint->setEffortLimit(0, sMotor.axis);
            //     myJoint->detachMotor(sMotor.axis);
            // }

            // delete any coefficient vectors we might have created
            delete maxspeed_coefficients;
            delete maxeffort_coefficients;
            delete current_coefficients;
            mimics.clear();
        }

        void SimMotor::addMimic(SimMotor* mimic)
        {
            mimics[mimic->getName()] = mimic;
        }

        void SimMotor::removeMimic(std::string mimicname)
        {
            mimics.erase(mimicname);
        }

        void SimMotor::clearMimics()
        {
            mimics.clear();
        }

        void SimMotor::setMimic(sReal multiplier, sReal offset)
        {
            mimic = true;
            mimic_multiplier = multiplier;
            mimic_offset = offset;
        }

        void SimMotor::setMaxEffortApproximation(utils::ApproximationFunction type,
                                                 std::vector<double>* coefficients)
        {
            switch (type)
            {
            case FUNCTION_PIPE:
                maxEffortApproximation =&utils::pipe;
                maxeffort_x = &sMotor.maxEffort;
                break;
            case FUNCTION_POLYNOM2:
                maxEffortApproximation =&utils::polynom2;
                maxeffort_x = position;
                break;
            case FUNCTION_POLYNOM3:
                maxEffortApproximation =&utils::polynom3;
                maxeffort_x = position;
                break;
            case FUNCTION_POLYNOM4:
                maxEffortApproximation =&utils::polynom4;
                maxeffort_x = position;
                break;
            case FUNCTION_POLYNOM5:
                maxEffortApproximation =&utils::polynom5;
                maxeffort_x = position;
                break;
            case FUNCTION_GAUSSIAN:
            case FUNCTION_UNKNOWN:
                LOG_WARN("SimMotor: Approximation function not implemented or unknown.");
                break;
            }
            maxeffort_coefficients = coefficients;
        }

        void SimMotor::setMaxSpeedApproximation(utils::ApproximationFunction type,
                                                std::vector<double>* coefficients)
        {
            switch (type)
            {
            case FUNCTION_PIPE:
                maxSpeedApproximation = &utils::pipe;
                maxspeed_x = &sMotor.maxSpeed;
                break;
            case FUNCTION_POLYNOM2:
                maxSpeedApproximation = &utils::polynom2;
                maxspeed_x = position;
                break;
            case FUNCTION_POLYNOM3:
                maxSpeedApproximation = &utils::polynom3;
                maxspeed_x = position;
                break;
            case FUNCTION_POLYNOM4:
                maxSpeedApproximation = &utils::polynom4;
                maxspeed_x = position;
                break;
            case FUNCTION_POLYNOM5:
                maxSpeedApproximation = &utils::polynom5;
                maxspeed_x = position;
                break;
            case FUNCTION_GAUSSIAN:
            case FUNCTION_UNKNOWN:
                LOG_WARN("SimMotor: Approximation function not implemented or unknown.");
                break;
            }
            maxspeed_coefficients = coefficients;
        }

        void SimMotor::setCurrentApproximation(utils::ApproximationFunction2D type,
                                               std::vector<double>* coefficients)
        {
            switch (type)
            {
            case FUNCTION_UNKNOWN2D:
                LOG_WARN("SimMotor: Approximation function not implemented or unknown.");
                break;
            case FUNCTION_POLYNOM2D2:
                currentApproximation = &utils::polynom2D2;
                break;
            case FUNCTION_POLYNOM2D1:
                currentApproximation = &utils::polynom2D1;
                break;
            }
            current_coefficients = coefficients;
        }

        void SimMotor::updateController()
        {
            axis = (unsigned char) sMotor.axis;
            switch (sMotor.type)
            {
            case MOTOR_TYPE_POSITION:
            case MOTOR_TYPE_PID: // deprecated
                controlParameter = &velocity;
                controlValue = sMotor.value;
                controlLimit = &(sMotor.maxSpeed);
                if(axis == 1)
                {
                    setJointControlParameter = &JointInterface::setVelocity;
                } else
                {
                    setJointControlParameter = &JointInterface::setVelocity2;
                }
                runController = &SimMotor::runPositionController;
                break;
            case MOTOR_TYPE_VELOCITY:
            case MOTOR_TYPE_DC: //deprecated
                controlParameter = &velocity;
                controlValue = sMotor.value;
                controlLimit = &(sMotor.maxAcceleration); // this is a stand-in for acceleration
                if(axis == 1)
                {
                    setJointControlParameter = &JointInterface::setVelocity;
                } else
                {
                    setJointControlParameter = &JointInterface::setVelocity2;
                }
                runController = &SimMotor::runVelocityController;
                break;
            case MOTOR_TYPE_PID_FORCE: // deprecated
            case MOTOR_TYPE_EFFORT:
                controlParameter = &effort;
                controlValue = sMotor.value;
                controlLimit = &(sMotor.maxEffort);
                //setJointControlParameter = &SimJoint::setEffort;
                runController = &SimMotor::runEffortController;
                break;
            case MOTOR_TYPE_DIRECT_EFFORT:
                controlValue = sMotor.value;
                controlLimit = &(sMotor.maxEffort);
                if(sMotor.config.hasKey("maxEffortControl") and
                   (bool)(sMotor.config["maxEffortControl"]) == true)
                {
                    controlParameter = &velocity;
                    setJointControlParameter = &JointInterface::setVelocity;
                } else
                {
                    controlParameter = &effort;
                    // TODO: handle axis
                    setJointControlParameter = &JointInterface::setTorque;
                }
                runController = &SimMotor::runEffortPipe;
                break;
            case MOTOR_TYPE_FF_EFFORT:
                controlValue = sMotor.value;
                controlLimit = &(sMotor.maxEffort);
                if(sMotor.config.hasKey("maxEffortControl") and
                   (bool)(sMotor.config["maxEffortControl"]) == true)
                {
                    controlParameter = &velocity;
                    setJointControlParameter = &JointInterface::setVelocity;
                } else
                {
                    controlParameter = &effort;
                    // TODO: handle axis
                    setJointControlParameter = &JointInterface::setTorque;
                }
                posPID.p = sMotor.p;
                runController = &SimMotor::runFFEffortPipe;
                break;
            case MOTOR_TYPE_UNDEFINED:
                // TODO: output error
                controlParameter = &velocity; // default to position
                controlValue = sMotor.value;
                controlLimit = &(sMotor.maxSpeed);
                if(axis==1)
                {
                    setJointControlParameter = &JointInterface::setVelocity;
                } else
                {
                    setJointControlParameter = &JointInterface::setVelocity2;
                }
                runController = &SimMotor::runPositionController;
                break;
            }
            //TODO: update the remaining parameters
        }

        void SimMotor::runEffortController(sReal time)
        {
            // limit to range of motion
            controlValue = std::max(sMotor.minValue,
                                    std::min(controlValue, sMotor.maxValue));

            if(controlValue > 2*M_PI)
                controlValue = 0;
            else if(controlValue > M_PI)
                controlValue = -2*M_PI + controlValue;
            else if(controlValue < -2*M_PI)
                controlValue = 0;
            else if(controlValue < -M_PI)
                controlValue = 2*M_PI + controlValue;

            error = controlValue - *position;
            if(error > M_PI) error = -2*M_PI + error;
            else if(error < -M_PI) error = 2*M_PI + error;
            integ_error += error * time;
            // P part of the motor
            effort = error * sMotor.p;
            // I part of the motor
            effort += integ_error * sMotor.i;
            // D part of the motor
            effort += ((error - last_error)/time) * sMotor.d;
            last_error = error;
            effort = std::max(-sMotor.maxEffort, std::min(effort, sMotor.maxEffort));
        }

        void SimMotor::runEffortPipe(sReal time)
        {
            // limit to range of motion
            controlValue += feedForwardEffort;
            controlValue = std::max(-sMotor.maxEffort,
                                    std::min(controlValue, sMotor.maxEffort));

            if(sMotor.config.hasKey("maxEffortControl") &&
               (bool)sMotor.config["maxEffortControl"] == true)
            {
                if(controlValue >= 0)
                {
                    velocity = 10000;
                } else
                {
                    velocity = -10000;
                }
                if (auto validJoint = joint.lock())
                {
                    if(sMotor.axis == 1)
                    {
                        validJoint->setForceLimit(fabs(controlValue));
                    } else
                    {
                        validJoint->setForceLimit2(fabs(controlValue));
                    }
                }
            } else
            {
                effort = controlValue;
            }
        }

        void SimMotor::runFFEffortPipe(sReal time)
        {
            controlValue = mimic_multiplier * controlValue + mimic_offset;
            sReal origControlValue = controlValue;
            sReal effort = 0;
            controlValue = std::max(sMotor.minValue,
                                    std::min(controlValue, sMotor.maxValue));
            // set current state
            // todo: better get the velocity directly from the physics state

            velPID.current_value = (*position - posPID.current_value)*(1000.0/time);
            posPID.current_value = *position;

            posPID.target_value = controlValue;
            posPID.step();
            velPID.target_value = posPID.output_value;
            velPID.step();
            //fprintf(stderr, "%lu %g %g\n", sMotor.index, posPID.output_value, velPID.output_value);
            controlValue = velPID.output_value;
            runEffortPipe(time);
            controlValue = origControlValue;
        }

        void SimMotor::runVelocityController(sReal time)
        {
            *controlParameter = controlValue;
        }

        void SimMotor::runPositionController(sReal time)
        {
            // the following implements a simple PID controller using the value
            // pointed to by controlParameter

            controlValue = mimic_multiplier * controlValue + mimic_offset;

            // limit to range of motion
            controlValue = std::max(sMotor.minValue,
                                    std::min(controlValue, sMotor.maxValue));

            // calculate control values
            error = controlValue - *position;
            if (std::abs(error) < 0.000001)
                error = 0.0;

            // FIXME: not sure if this makes sense, because it forbids turning
            //        motors in the same direction for multiple revolutions
            // and can possibly lead to the motor turning in the wrong direction
            // beyond its limit, as limit checking was done BEFORE??
            //if(er > M_PI)
            //  er = -2*M_PI+er;
            //else
            //if(er < -M_PI)
            //  er = 2*M_PI+er;

            integ_error += error*time;

            //anti wind up, this code limits the integral error
            //part of the pid to the maximum velocity. This makes
            //the pid react way faster. This also eleminates the
            //overshooting errors seen before in the simulation
            double iPart = integ_error * sMotor.i;
            if(iPart > sMotor.maxSpeed)
            {
                iPart = sMotor.maxSpeed;
                integ_error = sMotor.maxSpeed / sMotor.i;
            }

            if(iPart < -sMotor.maxSpeed)
            {
                iPart = -sMotor.maxSpeed;
                integ_error = -sMotor.maxSpeed / sMotor.i;
            }

            // set desired velocity. @TODO add inertia
            velocity = 0; // by setting a different value we could specify a minimum
            // P part of the motor
            velocity += error * sMotor.p;
            // I part of the motor
            velocity += iPart;
            // D part of the motor
            velocity += ((error - last_error)/time) * sMotor.d;
            // apply filter
            velocity = lastVelocity*(filterValue) + velocity*(1-filterValue);
            lastVelocity = velocity;
            last_error = error;

            if(sMotor.config.hasKey("spring"))
            {
                if (auto validJoint = joint.lock())
                {
                    if(axis == 1)
                    {
                        validJoint->setForceLimit(sMotor.maxEffort*error*(double)sMotor.config["spring"]);
                    } else
                    {
                        validJoint->setForceLimit2(sMotor.maxEffort*error*(double)sMotor.config["spring"]);
                    }
                }
            }
        }

        void SimMotor::update(sReal time_ms)
        {
            time = time_ms;// / 1000;
            sReal play_position = 0.0;

            // if the attached joint does not exist (any more)
            //if (!myJoint) deactivate();

            if(active)
            {
                // set play offset to 0
                //if(myPlayJoint) play_position = myPlayJoint->getPosition();

                refreshPosition();
                *position += play_position;

                // sense effort value from motor
                //Vector v;
                //joint->getAxisTorque(&v);
                //sensedEffort = v.norm();
                if (auto validJoint = joint.lock())
                {
                    sensedEffort = validJoint->getMotorTorque();
                }
                // call control function for current motor type
                //fprintf(stderr, "run controller - control value: %g\n", controlValue);
                (this->*runController)(time_ms);

                // cap speed
                tmpmaxspeed = getMomentaryMaxSpeed();
                //if(sMotor.type != MOTOR_TYPE_DIRECT_EFFORT) {
                velocity = std::max(-tmpmaxspeed, std::min(velocity, tmpmaxspeed));
                //}

                // cap effort
                if(!effortMotor && sMotor.type != MOTOR_TYPE_DIRECT_EFFORT)
                {
                    tmpmaxeffort = getMomentaryMaxEffort();
                    effort = std::max(-tmpmaxeffort, std::min(effort, tmpmaxeffort));
                    //myJoint->setEffortLimit(tmpmaxeffort, sMotor.axis);
                }

                for(std::map<std::string, SimMotor*>::iterator it = mimics.begin();
                    it != mimics.end(); ++it)
                {
                    it->second->setControlValue(controlValue);
                    //it->second->setControlValue(*position);
                }


                // estimate motor parameters based on achieved status
                estimateCurrent();
                estimateTemperature(time_ms);

                // pass speed (position/speed control) or torque to the attached
                // joint's setSpeed1/2 or setTorque1/2 methods
                //joint->setVelocity(controlParameter);
                if (auto validJoint = joint.lock())
                {
                    ((*validJoint).*setJointControlParameter)(*controlParameter);
                }
                //for mimic in myJoint->mimics:
                //  mimic->*setJointControlParameter)(mimic_multiplier*controlParameter, axis);
            }
        }

        void SimMotor::estimateCurrent()
        {
            // calculate current
            //sReal joint_velocity = myJoint->getVelocity();
            //current = (*currentApproximation)(&sensedEffort, &joint_velocity, current_coefficients);
        }

        void SimMotor::estimateTemperature(sReal time_ms)
        {
            temperature = temperature - calcHeatDissipation(time_ms) + calcHeatProduction(time_ms);
        }

        /*
         * Calculates the heat energy dissipating in the update interval.
         * Normally, heat transfer would be calculated as
         * P = k*A*(Ti - Ta)/d
         * where k is the thermal conductivity, A the surface area,
         * d the distance (material thickness) and Ti the temperature inside
         * and Ta the ambient temperature. Since k, A and d are all constant
         * in our case, heatTransferCoefficient = k*A/d and thus:
         */
        sReal SimMotor::calcHeatDissipation(sReal time_ms) const
        {
            return (heatTransferCoefficient * (temperature - ambientTemperature))*time_ms/1000.0;
        }

        /*
         * Calculates the heat energy lost from motor activity in the update interval.
         * heatlossCoefficient would be the fraction of the energy the motor gets from the
         * power supply which gets transformed into heat energy.
         */
        sReal SimMotor::calcHeatProduction(sReal time_ms) const
        {
            return (current * voltage * heatlossCoefficient)*time_ms/1000.0;
        }

        void SimMotor::initTemperatureEstimation()
        {
            temperature = 0;
            voltage = 0;
            heatlossCoefficient = 0;
            heatTransferCoefficient = 0;
            heatCapacity = 0;
        }

        void SimMotor::initCurrentEstimation()
        {
            kXY = 100.0*((0.00002) / ((9.81*0.07)*(2*M_PI/60)));
            kX  = 0.00512 / (9.81*0.07);
            kY  = 100.0*(0.00006 / (2*M_PI/60));
            k   = 0.025;
            std::vector<sReal>* spaceclimber_coefficients = new std::vector<sReal>;
            spaceclimber_coefficients->push_back(kXY);
            spaceclimber_coefficients->push_back(kX);
            spaceclimber_coefficients->push_back(kY);
            spaceclimber_coefficients->push_back(k);
            current_coefficients = spaceclimber_coefficients;
            current = 0;
        }

        void SimMotor::refreshPosition()
        {
            if(auto validJoint = joint.lock())
            {
                if(sMotor.axis == 1)
                    position1 = validJoint->getPosition();
                else
                    position2 = validJoint->getPosition2();
            }
        }

        void SimMotor::refreshPositions()
        {
            if(auto validJoint = joint.lock())
            {
                position1 = validJoint->getPosition();
                position2 = validJoint->getPosition2();
            }
        }

        void SimMotor::refreshAngle()
        { // deprecated
            refreshPosition();
        }

        /*
         * This function can be overloaded in a child class in order to
         * implement a specifically variable effort.
         */
        sReal SimMotor::getMomentaryMaxEffort()
        {
            return (*maxEffortApproximation)(maxeffort_x, maxeffort_coefficients);
        }

        /*
         * This function can be overloaded in a child class in order to
         * implement a specifically speed.
         */
        sReal SimMotor::getMomentaryMaxSpeed()
        {
            return (*maxSpeedApproximation)(maxspeed_x, maxspeed_coefficients);
        }


// from here on only getters and setters

        // void SimMotor::attachJoint(std::shared_ptr<SimJoint> joint){
        //     myJoint = joint;
        // }

        // void SimMotor::attachPlayJoint(std::shared_ptr<SimJoint> joint){
        //     myPlayJoint = joint;
        // }

        // std::shared_ptr<SimJoint> SimMotor::getJoint() const {
        //     return myJoint;
        // }

        // std::shared_ptr<SimJoint> SimMotor::getPlayJoint() const {
        //     return myPlayJoint;
        // }

        int SimMotor::getAxis() const
        {
            return sMotor.axis;
        }

        const std::string SimMotor::getName() const
        {
            return sMotor.name;
        }

        void SimMotor::setName(const std::string &newname)
        {
            sMotor.name = newname;
        }

        void SimMotor::setDesiredMotorAngle(sReal angle)
        { // deprecated
            switch(sMotor.type)
            {
            case MOTOR_TYPE_PID:
            case MOTOR_TYPE_POSITION:
            case MOTOR_TYPE_PID_FORCE:
            case MOTOR_TYPE_EFFORT:
                controlValue = angle;
                break;
            case MOTOR_TYPE_VELOCITY:
            case MOTOR_TYPE_DC:
            case MOTOR_TYPE_UNDEFINED:
            case MOTOR_TYPE_DIRECT_EFFORT:
                break;
            }
        }

        void SimMotor::setDesiredMotorVelocity(sReal velocity)
        { // deprecated
            switch(sMotor.type)
            {
            case MOTOR_TYPE_DC:
            case MOTOR_TYPE_VELOCITY:
                controlValue = velocity;
                break;
            case MOTOR_TYPE_PID:
            case MOTOR_TYPE_POSITION:
            case MOTOR_TYPE_PID_FORCE:
            case MOTOR_TYPE_EFFORT:
            case MOTOR_TYPE_UNDEFINED:
            case MOTOR_TYPE_DIRECT_EFFORT:
                break;
            }
        }

        sReal SimMotor::getDesiredMotorAngle() const
        { // deprecated
            switch(sMotor.type)
            {
            case MOTOR_TYPE_PID:
            case MOTOR_TYPE_POSITION:
            case MOTOR_TYPE_PID_FORCE:
            case MOTOR_TYPE_EFFORT:
                return controlValue;
                break;
            case MOTOR_TYPE_DC:
            case MOTOR_TYPE_VELOCITY:
            case MOTOR_TYPE_UNDEFINED:
            case MOTOR_TYPE_DIRECT_EFFORT:
                break;
            }
            return 0.0; // return 0 if it doesn't apply
        }

        void SimMotor::setMaxEffort(sReal force)
        {
            sMotor.maxEffort = force;
            if (auto validJoint = joint.lock())
            {
                if(sMotor.axis == 1)
                {
                    validJoint->setForceLimit(sMotor.maxEffort);
                }
                else
                {
                    validJoint->setForceLimit2(sMotor.maxEffort);
                }
            }
        }

        void SimMotor::setMotorMaxForce(sReal force)
        { // deprecated
            setMaxEffort(force);
        }

        sReal SimMotor::getMaxEffort() const
        {
            return sMotor.maxEffort;
        }

        sReal SimMotor::getMotorMaxForce() const
        { // deprecated
            return getMaxEffort();
        }

        sReal SimMotor::getPosition() const
        {
            return position1;
        }

        sReal SimMotor::getActualPosition() const
        { // deprecated
            return getPosition();
        }

        void SimMotor::setPosition(sReal angle)
        {
            position1 = angle;
        }

        void SimMotor::setActualAngle(sReal angle)
        { // deprecated
            setPosition(angle);
        }

        void SimMotor::setMaxSpeed(sReal speed)
        {
            sMotor.maxSpeed = fabs(speed);
        }

        void SimMotor::setMaximumVelocity(sReal v)
        { // deprecated
            setMaxSpeed(v);
        }

        sReal SimMotor::getMaxSpeed() const
        {
            return sMotor.maxSpeed;
        }

        sReal SimMotor::getMaximumVelocity() const
        { // deprecated
            return getMaxSpeed();
        }

        bool SimMotor::isServo() const
        {
            return !(sMotor.type == MOTOR_TYPE_DC ||
                     sMotor.type == MOTOR_TYPE_VELOCITY ||
                     sMotor.type == MOTOR_TYPE_UNDEFINED);
        }

        void SimMotor::setType(interfaces::MotorType mtype)
        {
            sMotor.type = mtype;
            updateController();
        }

        void SimMotor::setVelocity(sReal v)
        {
            switch(sMotor.type)
            {
            case MOTOR_TYPE_DC:
            case MOTOR_TYPE_VELOCITY:
                setControlValue(v);
                break;
            case MOTOR_TYPE_PID:
            case MOTOR_TYPE_POSITION:
            case MOTOR_TYPE_PID_FORCE:
            case MOTOR_TYPE_EFFORT:
            case MOTOR_TYPE_UNDEFINED:
            case MOTOR_TYPE_DIRECT_EFFORT:
                break;
            }
        }

        sReal SimMotor::getControlParameter(void) const
        {
            return *controlParameter;
        }

        sReal SimMotor::getVelocity() const
        { // deprecated
            return velocity;
        }

        void SimMotor::setMinValue(interfaces::sReal d)
        {
            sMotor.minValue = d;
        }

        void SimMotor::setMaxValue(interfaces::sReal d)
        {
            sMotor.maxValue = d;
        }

        void SimMotor::setP(sReal p)
        {
            sMotor.p = p;
            posPID.p = p;
        }

        void SimMotor::setI(sReal i)
        {
            sMotor.i = i;
            posPID.i = i;
        }

        void SimMotor::setD(sReal d)
        {
            sMotor.d = d;
            posPID.d = d;
        }

        sReal SimMotor::getP() const
        {
            return sMotor.p;
        }

        sReal SimMotor::getI() const
        {
            return sMotor.i;
        }

        sReal SimMotor::getD() const
        {
            return sMotor.d;
        }

        void SimMotor::setSMotor(const MotorData &sMotor)
        {
            // TODO: handle name change correctly
            this->sMotor = sMotor;
            initPIDs();
            filterValue = 0.0;
            if(this->sMotor.config.hasKey("filterValue"))
            {
                filterValue = this->sMotor.config["filterValue"];
            }
            effortMotor = false;
            if(sMotor.type == MOTOR_TYPE_PID_FORCE ||
               sMotor.type == MOTOR_TYPE_EFFORT)
            {
                effortMotor = true;
            } else if(sMotor.type == MOTOR_TYPE_DIRECT_EFFORT)
            {
                effortMotor = true;
                ConfigMap map = sMotor.config;
                if(map.hasKey("maxEffortControl") and
                   (bool)map["maxEffortControl"])
                {
                    effortMotor = false;
                }
            }
            if(!effortMotor)
            {
                //myJoint->attachMotor(sMotor.axis);
                if (auto validJoint = joint.lock())
                {
                    if(sMotor.axis == 1)
                    {
                        validJoint->setForceLimit(sMotor.maxEffort);
                    } else
                    {
                        validJoint->setForceLimit2(sMotor.maxEffort);
                    }
                }
            }
        }

        const MotorData SimMotor::getSMotor(void) const
        {
            return sMotor;
        }

        void SimMotor::setValue(sReal value)
        {
            setControlValue(value);
        }

        void SimMotor::setOfflinePosition(interfaces::sReal value)
        {
            if(control && control->sim && control->sim->isSimRunning())
            {
                LOG_WARN("SimMotor: Setting the \"offline\" position if the simulation is running can produce bad simulation states!");
            }
            if(sMotor.type == MOTOR_TYPE_POSITION ||
               sMotor.type == MOTOR_TYPE_PID)
            {
                controlValue = value;
            }
            //myJoint->setOfflinePosition(value);
            refreshPositions();
        }

        void SimMotor::setControlValue(interfaces::sReal value)
        {
            controlValue = value;
            //fprintf(stderr, "control value: %g\n", controlValue);
            if(sMotor.type == MOTOR_TYPE_POSITION ||
               sMotor.type == MOTOR_TYPE_PID)
            {
                if(control && control->sim && !control->sim->isSimRunning())
                {
                    //myJoint->setOfflinePosition(value);
                    refreshPositions();
                }
            }
        }

        void SimMotor::setFeedForwardTorque(interfaces::sReal value)
        {
            feedForwardEffort = value;
        }

        /*switch (sMotor.type) {
          case MOTOR_TYPE_POSITION:
          case MOTOR_TYPE_PID:
          desired_position = value;
          break;
          case MOTOR_TYPE_SPEED:
          case MOTOR_TYPE_DC:
          desired_speed = value;
          break;
          case MOTOR_TYPE_PID_FORCE:
          desired_position = value;
          break;
          case MOTOR_TYPE_UNDEFINED:
          break;
          }*/

        sReal SimMotor::getValue(void) const
        {
            return getControlValue();
        }

        sReal SimMotor::getControlValue(void) const
        {
            return controlValue;
        }

        void SimMotor::setPID(sReal mP, sReal mI, sReal mD)
        {
            switch (sMotor.type)
            {
            case MOTOR_TYPE_PID: // deprecated
            case MOTOR_TYPE_POSITION:
            case MOTOR_TYPE_PID_FORCE: // deprecated
            case MOTOR_TYPE_EFFORT:
                sMotor.p = mP;
                sMotor.i = mI;
                sMotor.d = mD;
                break;
            case MOTOR_TYPE_DC: // deprecated
            case MOTOR_TYPE_VELOCITY:
            case MOTOR_TYPE_UNDEFINED:
            case MOTOR_TYPE_DIRECT_EFFORT:
                // information not relevant for these types
                break;
            }
        }

        unsigned long SimMotor::getIndex(void) const
        {
            return sMotor.index;
        }

        unsigned long SimMotor::getJointIndex(void) const
        {
            if (auto validJoint = joint.lock())
            {
                std::string jointName;
                validJoint->getName(&jointName);
                return control->jointIDManager->getID(jointName);
            }
            return 0;
        }

        std::string SimMotor::getJointName(void) const
        {
            return sMotor.jointName;
        }

        void SimMotor::getCoreExchange(core_objects_exchange* obj) const
        {
            obj->index = sMotor.index;
            obj->name = sMotor.name;
            obj->groupID = sMotor.type;
            obj->value = controlValue;
        }

        sReal SimMotor::getCurrent(void) const
        {
            return current;
        }

        sReal SimMotor::getEffort() const
        {
            return sensedEffort;
        }

        sReal SimMotor::getTorque(void) const
        { // deprecated
            return getEffort();
        }

        void SimMotor::deactivate(void)
        {
            active = false;
        }

        void SimMotor::activate(void)
        {
            active = true;
        }

        void SimMotor::getDataBrokerNames(std::string *groupName,
                                          std::string *dataName) const
        {
            char format[] = "Motors/%05lu_%s";
            int size = snprintf(0, 0, format, sMotor.index, sMotor.name.c_str());
            char buffer[size+1];
            sprintf(buffer, format, sMotor.index, sMotor.name.c_str());
            *groupName = "mars_sim";
            *dataName = buffer;
        }

        void SimMotor::produceData(const data_broker::DataInfo &info,
                                   data_broker::DataPackage *dbPackage,
                                   int callbackParam)
        {
            dbPackage->set(dbIdIndex, (long)sMotor.index);
            dbPackage->set(dbControlParameterIndex, controlValue);
            dbPackage->set(dbPositionIndex, getPosition());
            dbPackage->set(dbCurrentIndex, getCurrent());
            dbPackage->set(dbEffortIndex, getEffort());
            dbPackage->set(dbMaxEffortIndex, sMotor.maxEffort);
        }

        void SimMotor::receiveData(const data_broker::DataInfo& info,
                                   const data_broker::DataPackage& package,
                                   int id)
        {
            sReal value;
            package.get(0, &value);
            setControlValue(value);
        }

        // methods inherited from mars::interfaces::ConfigMapInterface
        configmaps::ConfigMap SimMotor::getConfigMap() const
        {
            configmaps::ConfigMap result;

            const mars::interfaces::MotorData& motorData{getSMotor()};
            result["name"] = getName();
            result["type"] = motorData.type;
            result["axis"] = getAxis();
            result["current"] = getCurrent();
            result["effort"] = getEffort();
            result["index"] = getIndex();
            result["is servo"] = isServo();
            result["joint name"] = getJointName();
            result["maxEffort"] = getMaxEffort();
            result["maxSpeed"] = getMaxSpeed();
            result["position"] = getPosition();
            result["velocity"] = getVelocity();
            result["control parameter"] = getControlParameter();
            result["control value"] = getControlValue();
            result["p"] = getP();
            result["i"] = getI();
            result["d"] = getD();
            result["minValue"] = motorData.minValue;
            result["maxValue"] = motorData.maxValue;
            return result;
        }

        std::vector<std::string> SimMotor::getEditPattern(const std::string& basePath) const
        {
            return std::vector<std::string>
            {
                basePath + "p",
                basePath + "i",
                basePath + "d",
                basePath + "maxSpeed",
                basePath + "maxEffort",
                basePath + "minValue",
                basePath + "maxValue",
                basePath + "type"
            };
        }

        void SimMotor::edit(const std::string& configPath, const std::string& value)
        {
                if(mars::utils::matchPattern("*/p", configPath))
                {
                    setP(atof(value.c_str()));
                } else if(mars::utils::matchPattern("*/i", configPath))
                {
                    setI(atof(value.c_str()));
                } else if(mars::utils::matchPattern("*/d", configPath))
                {
                    setI(atof(value.c_str()));
                } else if(mars::utils::matchPattern("*/maxSpeed", configPath))
                {
                    setMaxSpeed(atof(value.c_str()));
                } else if(mars::utils::matchPattern("*/maxEffort", configPath))
                {
                    setMaxEffort(atof(value.c_str()));
                } else if(mars::utils::matchPattern("*/minValue", configPath))
                {
                    setMinValue(atof(value.c_str()));
                } else if(mars::utils::matchPattern("*/maxValue", configPath))
                {
                    setMaxValue(atof(value.c_str()));
                } else if(mars::utils::matchPattern("*/type", configPath))
                {
                    if(value == "DC" || value == "2")
                    {
                        setType(MOTOR_TYPE_VELOCITY);
                    } else
                    {
                        setType(MOTOR_TYPE_POSITION);
                    }
                }
        }

        void SimMotor::setValueDesiredVelocity(interfaces::sReal value)
        {
            switch (sMotor.type)
            {
            case MOTOR_TYPE_VELOCITY:
            case MOTOR_TYPE_DC: //deprecated
                setControlValue(value);
                break;
            case MOTOR_TYPE_PID:
            case MOTOR_TYPE_POSITION:
            case MOTOR_TYPE_PID_FORCE:
            case MOTOR_TYPE_EFFORT:
            case MOTOR_TYPE_UNDEFINED:
            case MOTOR_TYPE_DIRECT_EFFORT:
                break;
            }
        };

        void SimMotor::initPIDs()
        {
            posPID.p = sMotor.p;
            posPID.i = sMotor.i;
            posPID.d = sMotor.d;
            posPID.max_out = sMotor.maxSpeed;
            posPID.min_out = -sMotor.maxSpeed;
            if(sMotor.config.hasKey("filetPos"))
            {
                posPID.filter_value = sMotor.config["filterPos"];
            }
            if(sMotor.config.hasKey("maxPosI"))
            {
                posPID.max_i = sMotor.config["maxPosI"];
            }
            posPID.last_error = 0;
            posPID.last_i = 0;

            if(sMotor.config.hasKey("velP"))
            {
                velPID.p = sMotor.config["velP"];
            }
            if(sMotor.config.hasKey("velI"))
            {
                velPID.i = sMotor.config["velI"];
            }
            if(sMotor.config.hasKey("velD"))
            {
                velPID.d = sMotor.config["velD"];
            }
            velPID.max_out = sMotor.maxEffort;
            velPID.min_out = -sMotor.maxEffort;
            if(sMotor.config.hasKey("filterVel"))
            {
                velPID.filter_value = sMotor.config["filterVel"];
            }
            if(sMotor.config.hasKey("maxVelI"))
            {
                velPID.max_i = sMotor.config["maxVelI"];
            }
            velPID.last_error = 0;
            velPID.last_i = 0;
        }

    } // end of namespace core
} // end of namespace mars
