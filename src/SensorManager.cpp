/**
 * \file SensorManager.cpp
 * \author  Malte Langosz
 * \brief SensorManager implements SensorManagerInterface and
 * manages all sensors and all sensor
 * operations that are used for the communication between the simulation
 * modules.
 */

#include "SensorManager.hpp"

// sensor includes
// #include "JointAVGTorqueSensor.h"
// #include "JointLoadSensor.h"
// #include "NodePositionSensor.h"
// #include "NodeRotationSensor.h"
// #include "NodeContactSensor.h"
// #include "NodeIMUSensor.h"
// #include "NodeContactForceSensor.h"
// #include "NodeCOMSensor.h"
// #include "JointPositionSensor.h"
// #include "JointVelocitySensor.h"
#include "sensors/CameraSensor.hpp"
// #include "NodeVelocitySensor.h"
#include "sensors/RaySensor.hpp"
// #include "RotatingRaySensor.h"
// #include "MultiLevelLaserRangeFinder.h"
// #include "RayGridSensor.h"
// #include "NodeAngularVelocitySensor.h"
// #include "MotorCurrentSensor.h"
// #include "HapticFieldSensor.h"
// #include "Joint6DOFSensor.h"
// #include "JointTorqueSensor.h"
// #include "ScanningSonar.h"

#include <mars_interfaces/Logging.hpp>
#include <mars_interfaces/sim/SimulatorInterface.h>
#include <mars_utils/MutexLocker.h>

#include <envire_types/sensors/CameraSensor.hpp>
#include <envire_types/sensors/RaySensor.hpp>

// TODO: itemRemover from JointManager.hpp needed. Remove include as soon as the function is moved to a central location.
#include "JointManager.hpp"

#include <cstdio>
#include <stdexcept>

namespace mars
{
    namespace core
    {

        using namespace std;
        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        /**
         * \brief Constructor.
         *
         * \param c The pointer to the ControlCenter of the simulation.
         */
        SensorManager::SensorManager(ControlCenter *c) : control{c}
        {
            addSensorTypes();
            addMarsParsers();
        }

        void SensorManager::addSensorTypes()
        {
            addSensorType("RaySensor",&RaySensor::instanciate);
            // addSensorType("RotatingRaySensor",&RotatingRaySensor::instanciate);
            // addSensorType("MultiLevelLaserRangeFinder",&MultiLevelLaserRangeFinder::instanciate);
            addSensorType("CameraSensor",&CameraSensor::instanciate);
            // addSensorType("ScanningSonar",&ScanningSonar::instanciate);
            // addSensorType("JointPosition",&JointPositionSensor::instanciate);
            // addSensorType("JointVelocity",&JointVelocitySensor::instanciate);
            // addSensorType("JointLoad",&JointLoadSensor::instanciate);
            // addSensorType("JointTorque",&JointTorqueSensor::instanciate);
            // addSensorType("JointAVGTorque",&JointAVGTorqueSensor::instanciate);
            // addSensorType("Joint6DOF",&Joint6DOFSensor::instanciate);
            // addSensorType("NodeContact",&NodeContactSensor::instanciate);
            // addSensorType("NodeIMU", &NodeIMUSensor::instanciate);
            // addSensorType("NodePosition",&NodePositionSensor::instanciate);
            // addSensorType("NodeRotation",&NodeRotationSensor::instanciate);
            // addSensorType("NodeContactForce",&NodeContactForceSensor::instanciate);
            // addSensorType("NodeCOM",&NodeCOMSensor::instanciate);
            // addSensorType("NodeVelocity",&NodeVelocitySensor::instanciate);
            // addSensorType("NodeAngularVelocity",&NodeAngularVelocitySensor::instanciate);
            // addSensorType("MotorCurrent",&MotorCurrentSensor::instanciate);
            // addSensorType("HapticField",&HapticFieldSensor::instanciate);

            // missing sensors:
            //   RayGridSensor
        }

        void SensorManager::addMarsParsers()
        {
            addMarsParser("RaySensor",&RaySensor::parseConfig);
            // addMarsParser("RotatingRaySensor",&RotatingRaySensor::parseConfig);
            // addMarsParser("MultiLevelLaserRangeFinder",&MultiLevelLaserRangeFinder::parseConfig);
            addMarsParser("CameraSensor",&CameraSensor::parseConfig);
            // addMarsParser("ScanningSonar",&ScanningSonar::parseConfig);
            // addMarsParser("JointPosition",&JointArraySensor::parseConfig);
            // addMarsParser("JointVelocity",&JointArraySensor::parseConfig);
            // addMarsParser("JointLoad",&JointArraySensor::parseConfig);
            // addMarsParser("JointTorque",&JointArraySensor::parseConfig);
            // addMarsParser("JointAVGTorque",&JointArraySensor::parseConfig);
            // addMarsParser("Joint6DOF",&Joint6DOFSensor::parseConfig);
            // addMarsParser("NodeContact",&NodeContactSensor::parseConfig);
            // addMarsParser("NodeIMU", &NodeIMUSensor::parseConfig);
            // addMarsParser("NodePosition",&NodeArraySensor::parseConfig);
            // addMarsParser("NodeRotation",&NodeArraySensor::parseConfig);
            // addMarsParser("NodeContactForce",&NodeArraySensor::parseConfig);
            // addMarsParser("NodeCOM",&NodeArraySensor::parseConfig);
            // addMarsParser("NodeVelocity",&NodeArraySensor::parseConfig);
            // addMarsParser("NodeAngularVelocity",&NodeArraySensor::parseConfig);
            // addMarsParser("MotorCurrent",&MotorCurrentSensor::parseConfig);
            // addMarsParser("HapticField",&HapticFieldSensor::parseConfig);

            // missing sensors:
            //   RayGridSensor
        }

        /**
         *\brief Returns true, if the sensor with the given id exists.
         *
         * \param id The id of the sensor to look for.
         * \return boolean, whether the node exists.
         */
        bool SensorManager::exists(unsigned long index) const
        {
            return control->sensorIDManager->isKnown(index);
        }

        /**
         * \brief Gives information about core exchange data for sensors.
         *
         * \param sensorList A pointer to a vector that is filled with a
         * core_objects_exchange struct for every sensor. The vector is cleared
         * in the beginning of this function.
         */
        void SensorManager::getListSensors(vector<core_objects_exchange> *sensorList) const
        {
            core_objects_exchange obj;
            sensorList->clear();
            simSensorsMutex.lock();
            for(auto iter = simSensors.begin(); iter != simSensors.end(); iter++)
            {
                iter->second->getCoreExchange(&obj);
                sensorList->push_back(obj);
            }
            simSensorsMutex.unlock();
        }

        /**
         * \brief Gives all information of a certain sensor.
         *
         * \param index The unique id of the sensor to get information for.
         *
         * \return A pointer to the BaseSensor of the sensor with the given id.
         * \throw std::runtime_error if a motor with the given index does not exist.
         */
        const BaseSensor* SensorManager::getFullSensor(unsigned long index) const
        {
            const MutexLocker locker{&simSensorsMutex};
            const auto iter = simSensors.find(index);
            if (iter != simSensors.end())
            {
                return iter->second;
            }
            else
            {
                const auto msg = std::string{"could not find sensor with index: "} + std::to_string(index);
                throw std::runtime_error(msg);
            }
        }

        unsigned long SensorManager::getSensorID(std::string name) const
        {
            const auto& id = control->sensorIDManager->getID(name);
            if (id == INVALID_ID)
            {
                const auto msg = std::string{"SensorManager::getSensorID: Can't find sensor with the name \""} + name + "\".";
                LOG_ERROR(msg.c_str());
            }

            return id;
        }

        /**
         * \brief Removes a sensor from the simulation.
         *
         * \param index The unique id of the sensor to remove form the simulation.
         */
        void SensorManager::removeSensor(unsigned long index)
        {
            // TODO: Remove envire sensor item and base sensor item from graph.
            BaseSensor* tmpSensor = nullptr;
            simSensorsMutex.lock();
            const auto iter = simSensors.find(index);
            if(iter != simSensors.end())
            {
                tmpSensor = iter->second;
                simSensors.erase(iter);
                if(tmpSensor)
                {
                    delete tmpSensor;
                }
            }
            simSensorsMutex.unlock();

            control->sensorIDManager->removeEntry(index);

            constexpr bool sceneWasReseted = false;
            control->sim->sceneHasChanged(sceneWasReseted);
        }


        /**
         * \brief This function returns the SimSensor object for a given index.
         *
         * \param name The index of the sensor to get the core sensor object.
         *
         * \returns Returns a pointer to the corresponding sensor object.
         */
        BaseSensor* SensorManager::getSimSensor(unsigned long index) const
        {
            const MutexLocker locker{&simSensorsMutex};
            const auto iter = simSensors.find(index);

            if(iter != simSensors.end())
            {
                return iter->second;
            }
            else
            {
                return nullptr;
            }
        }

        /**
         * \brief This function provides the sensor data for a given index.
         *
         * \param data The sensor data of the sensor.
         *
         * \param index The index of the sensor to get the data
         */
        int SensorManager::getSensorData(unsigned long id, sReal **data) const
        {
            const MutexLocker locker{&simSensorsMutex};

            const auto iter = simSensors.find(id);
            if(iter != simSensors.end())
            {
                return iter->second->getSensorData(data);
            }

            LOG_DEBUG((std::string{"Cannot Find Sensor with id: "} + std::to_string(id)).c_str());
            return 0;
        }


        /**
         *\brief Returns the number of sensors that are currently present in the simulation.
         *
         *\return The number of all sensors.
         */
        int SensorManager::getSensorCount() const
        {
            return control->sensorIDManager->size();
        }


        /**
         * \brief Destroys all sensors in the simulation.
         *
         * \details The \c clear_all flag indicates if the reload sensors should
         * be destroyed as well. If set to \c false they are left intact.
         *
         * \param clear_all Indicates if the reload sensors should
         * be destroyed as well. If set to \c false they are left intact.
         */
        void SensorManager::clearAllSensors(bool clear_all)
        {
            {
                const MutexLocker locker{&simSensorsMutex};
                simSensors.clear();
            }
            envire::core::EnvireGraph* const graph = control->envireGraph_.get();
            auto removeFunctor = [clear_all, &graph](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
            {
                itemRemover<std::shared_ptr<BaseSensor>>(graph, node);

                // TODO: Is this still needed?
                if (clear_all)
                {
                    itemRemover<envire::types::sensors::CameraSensor>(graph, node);
                    itemRemover<envire::types::sensors::RaySensor>(graph, node);
                }
            };

            const auto& rootVertex = control->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            control->graphTreeView_->visitBfs(rootVertex, removeFunctor);

            control->sensorIDManager->clear();
        }

        /**
         * \brief This function reloads all sensors from a temporary sensor pool.
         *
         * \details All sensors that have been added with \c reload value as \c true
         * are added back to the simulation again with a \c reload value of \c true.
         */
        void SensorManager::reloadSensors(void)
        {
            envire::core::EnvireGraph* const graph = control->envireGraph_.get();
            auto readdFunctor = [&graph](envire::core::GraphTraits::vertex_descriptor node, envire::core::GraphTraits::vertex_descriptor parent)
            {
                itemReadder<envire::types::sensors::CameraSensor>(graph, node);
                itemReadder<envire::types::sensors::RaySensor>(graph, node);
            };

            const auto& rootVertex = control->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            control->graphTreeView_->visitBfs(rootVertex, readdFunctor);
        }

        void SensorManager::addMarsParser(const std::string string, BaseConfig* (*func)(ControlCenter*, ConfigMap*))
        {
            marsParser.insert(std::pair<const std::string, BaseConfig* (*)(ControlCenter*, ConfigMap*)>(string,func));
        }

        void SensorManager::addSensorType(const std::string &name, BaseSensor* (*func)(ControlCenter*, BaseConfig*))
        {
            availableSensors.insert(std::pair<const std::string,BaseSensor* (*)(ControlCenter*,BaseConfig*)>(name,func));
        }

       unsigned long SensorManager::createAndAddSensor(const std::string &type_name, BaseConfig *config, bool reload)
       {
            // TODO: This should be moved to envire_mars_sensors and instead add an envire sensor item to the envire graph.
            assert(config);
            const auto it = availableSensors.find(type_name);
            if(it == availableSensors.end())
            {
                std::cerr << "Could not load unknown Sensor with name: \"" << type_name << "\"" << std::endl;
                return 0;
            }

            if(config->name.empty())
            {
                throw std::logic_error("SensorManager::createAndAddSensor: Empty sensor name is not supported yet.");
                // std::stringstream str;
                // str << "SENSOR-" << id;
                // config->name = str.str();
            }
            config->id = control->sensorIDManager->addIfUnknown(config->name);

            BaseSensor *sensor = ((*it).second)(this->control,config);
            simSensorsMutex.lock();
            simSensors[sensor->getID()] = sensor;
            simSensorsMutex.unlock();

            return sensor->getID();
        }

        unsigned long SensorManager::createAndAddSensor(ConfigMap *config, bool reload)
        {
            const std::string type = (*config)["type"][0].getString();
            const auto it = marsParser.find(type);
            if(it == marsParser.end())
            {
                std::cerr << "Could not find MarsParser for sensor with name: \"" << type.c_str()<< "\"" << std::endl;
                return 0;
            }
            //LOG_DEBUG("found sensor: %s", type.c_str());
            BaseConfig *cfg = ((*it).second)(control, config);
            cfg->name = (*config)["name"][0].getString();
            return createAndAddSensor(type, cfg);
        }

    } // end of namespace core
} // end of namespace mars
