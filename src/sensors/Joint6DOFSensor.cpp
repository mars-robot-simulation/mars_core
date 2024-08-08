#include <cstdlib>
#include <cstdio>
#include <cstring>

#include "Joint6DOFSensor.hpp"

#include <mars_interfaces/sim/SimulatorInterface.h>
#include <mars_interfaces/sim/NodeManagerInterface.h>
#include <mars_interfaces/sim/JointManagerInterface.h>
#include <mars_interfaces/sim/LoadCenter.h>

#include <data_broker/DataBrokerInterface.h>

namespace mars
{
    namespace core
    {

        using namespace configmaps;
        using namespace utils;
        using namespace interfaces;

        BaseSensor* Joint6DOFSensor::instanciate(ControlCenter *control,
                                                 BaseConfig *config )
        {
            Joint6DOFConfig *cfg = dynamic_cast<Joint6DOFConfig*>(config);
            assert(cfg);
            return new Joint6DOFSensor(control,*cfg);
        }


        enum { CALLBACK_NODE, CALLBACK_JOINT };

        Joint6DOFSensor::Joint6DOFSensor(ControlCenter *control,
                                         Joint6DOFConfig config) :
            SensorInterface(control),
            BaseSensor(config.id, config.name),
            config(config)
        {

            std::vector<unsigned long>::iterator iter;
            jointPackageId = nodePackageId = 0;

            std::string groupName, dataName;

            if(control->dataBroker)
            {
                control->dataBroker->registerTimedReceiver(this, config.nodeGroupName,
                                                           config.nodeDataName,
                                                           "mars_sim/simTimer",
                                                           config.updateRate,
                                                           CALLBACK_NODE);
                control->dataBroker->registerTimedReceiver(this, config.jointGroupName,
                                                           config.jointDataName,
                                                           "mars_sim/simTimer",
                                                           config.updateRate,
                                                           CALLBACK_JOINT);
                dbPackage.add("id", (long)config.id);
                dbPackage.add("fx", 0.0);
                dbPackage.add("fy", 0.0);
                dbPackage.add("fz", 0.0);
                dbPackage.add("tx", 0.0);
                dbPackage.add("ty", 0.0);
                dbPackage.add("tz", 0.0);

                char text[55];
                sprintf(text, "Sensors/%s", config.name.c_str());
                dbPushId = control->dataBroker->pushData("mars_sim", text,
                                                         dbPackage, NULL,
                                                         data_broker::DATA_PACKAGE_READ_FLAG);
                control->dataBroker->registerTimedProducer(this, "mars_sim", text,
                                                           "mars_sim/simTimer", 0);
            }

            // NodeData theNode = control->nodes->getFullNode(sensor_data.body_id);
            // JointData theJoint = control->joints->getFullJoint(sensor_data.joint_id);
            // sensor_data.anchor = theJoint.anchor;
            // sensor_data.anchor -= theNode.pos;
            // sensor_data.tmp = 1 / (sensor_data.anchor.x()*sensor_data.anchor.x() +
            //                        sensor_data.anchor.y()*sensor_data.anchor.y() +
            //                        sensor_data.anchor.z()*sensor_data.anchor.z());

            // sensor_data.length = sensor_data.anchor.norm();
            sensor_data.force = Vector(0, 0, 0);
            sensor_data.torque = Vector(0, 0, 0);
            sensor_data.body_q.x() = sensor_data.body_q.y() = sensor_data.body_q.z() = 0.0;
            sensor_data.body_q.w() = 1.0;
        }

        Joint6DOFSensor::~Joint6DOFSensor(void)
        {
            if (control->dataBroker)
            {
                control->dataBroker->unregisterTimedProducer(this, "*", "*",
                                                             "mars_sim/simTimer");
                control->dataBroker->unregisterTimedReceiver(this, "*", "*",
                                                             "mars_sim/simTimer");
            }
        }

        // this function should be overwritten by the special sensor to
        int Joint6DOFSensor::getAsciiData(char* data) const
        {
            char *p;
            Vector force, torque;
            Vector pro;
 
            force =  (sensor_data.body_q * sensor_data.force);
            torque = (sensor_data.body_q * sensor_data.torque);
            p = data;
            sprintf(p, " %10.6g %10.6g %10.6g %10.6g %10.6g %10.6g",
                    force.x(), force.y(), force.z(), torque.x(), torque.y(), torque.z());

            return strlen(p);
        }


        int Joint6DOFSensor::getSensorData(sReal** data) const
        {
            Vector tmp;
  
            *data = (sReal*)malloc(sizeof(sReal)*6);
            tmp = (sensor_data.body_q * sensor_data.force);
            (*data)[0] = tmp.x();
            (*data)[1] = tmp.y();
            (*data)[2] = tmp.z();
            tmp = (sensor_data.body_q * sensor_data.torque);
            (*data)[3] = tmp.x();
            (*data)[4] = tmp.y();
            (*data)[5] = tmp.z();
            return 6;
        }

        void Joint6DOFSensor::getForceData(utils::Vector *force)
        {
            *force = (sensor_data.body_q * sensor_data.force);
        }
        void Joint6DOFSensor::getTorqueData(utils::Vector *torque)
        {
            *torque = (sensor_data.body_q * sensor_data.torque);
        }
        void Joint6DOFSensor::getAnchor(utils::Vector *anchor)
        {
            //*anchor = sensor_data.anchor;
        }
        void Joint6DOFSensor::getBodyQ(utils::Quaternion* body_q)
        {
            *body_q = sensor_data.body_q;
        }


        void Joint6DOFSensor::produceData(const data_broker::DataInfo &info,
                                          data_broker::DataPackage *dbPackage,
                                          int callbackParam)
        {
            Vector tmp;
            dbPackage->set(0, (long)id);
            tmp = (sensor_data.body_q * sensor_data.force);
            dbPackage->set(1, tmp.x());
            dbPackage->set(2, tmp.y());
            dbPackage->set(3, tmp.z());

            tmp = (sensor_data.body_q * sensor_data.torque);
            dbPackage->set(4, tmp.x());
            dbPackage->set(5, tmp.y());
            dbPackage->set(6, tmp.z());
        }

        void Joint6DOFSensor::receiveData(const data_broker::DataInfo &info,
                                          const data_broker::DataPackage &package,
                                          int callbackParam)
        {
            if(nodePackageId == info.dataId)
            {
                package.get(nodeRotIndices[0], &sensor_data.body_q.x());
                package.get(nodeRotIndices[1], &sensor_data.body_q.y());
                package.get(nodeRotIndices[2], &sensor_data.body_q.z());
                package.get(nodeRotIndices[3], &sensor_data.body_q.w());
                sensor_data.body_q.x() = -sensor_data.body_q.x();
                sensor_data.body_q.y() = -sensor_data.body_q.y();
                sensor_data.body_q.z() = -sensor_data.body_q.z();
            }
            else if(jointPackageId == info.dataId)
            {
                for(int i = 0; i < 3; ++i)
                {
                    package.get(jointForceIndices[i], &sensor_data.force[i]);
                    package.get(jointTorqueIndices[i], &sensor_data.torque[i]);
                }
            }
            else
            {
                // we don't know the dataId yet, so assign the ids...
                switch(callbackParam)
                {
                case CALLBACK_NODE:
                    nodePackageId = info.dataId;
                    nodeRotIndices[0] = package.getIndexByName("rotation/x");
                    nodeRotIndices[1] = package.getIndexByName("rotation/y");
                    nodeRotIndices[2] = package.getIndexByName("rotation/z");
                    nodeRotIndices[3] = package.getIndexByName("rotation/w");
                    break;
                case CALLBACK_JOINT:
                    jointPackageId = info.dataId;
                    jointForceIndices[0] = package.getIndexByName("force1/x");
                    jointForceIndices[1] = package.getIndexByName("force1/y");
                    jointForceIndices[2] = package.getIndexByName("force1/z");
                    jointTorqueIndices[0] = package.getIndexByName("torque1/x");
                    jointTorqueIndices[1] = package.getIndexByName("torque1/y");
                    jointTorqueIndices[2] = package.getIndexByName("torque1/z");
                    break;
                default:
                    // Error: We are not interested in this package. Why did we get it?
                    return;
                }
                // ...and call this method again.
                receiveData(info, package, callbackParam);
            }
        }

        BaseConfig* Joint6DOFSensor::parseConfig(ControlCenter *control,
                                                 ConfigMap *config)
        {
            Joint6DOFConfig *cfg = new Joint6DOFConfig;
            cfg->nodeGroupName = (*config)["nodeGroupName"].getString();
            cfg->nodeDataName = (*config)["nodeDataName"].getString();
            cfg->jointGroupName = (*config)["jointGroupName"].getString();
            cfg->jointDataName = (*config)["jointDataName"].getString();
            cfg->updateRate = (*config)["rate"];
            return cfg;
        }

        ConfigMap Joint6DOFSensor::createConfig() const
        {
            ConfigMap cfg;
            cfg["name"] = config.name;
            cfg["index"] = config.id;
            cfg["type"] = std::string("Joint6DOF");
            cfg["rate"] = config.updateRate;
            cfg["nodeGroupName"] = config.nodeGroupName;
            cfg["nodeDataName"] = config.nodeDataName;
            cfg["jointGroupName"] = config.jointGroupName;
            cfg["jointDataName"] = config.jointDataName;
            return cfg;
        }

    } // end of namespace core
} // end of namespace mars
