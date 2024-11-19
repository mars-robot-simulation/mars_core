#pragma once

#include <data_broker/ProducerInterface.h>
#include <data_broker/ReceiverInterface.h>
#include <data_broker/DataPackage.h>
#include <mars_interfaces/sim/SensorInterface.h>

namespace mars
{
    namespace core
    {

        class Joint6DOFConfig : public interfaces::BaseConfig
        {
        public:
            Joint6DOFConfig()
                {
                    name = "Unknown 6DOFSensor";
                    //nodeID = 0;
                    //jointID = 0;
                }

            //unsigned long nodeID, jointID;
            std::string nodeGroupName, nodeDataName, jointGroupName, jointDataName;
        };

        typedef struct joint6DOFData
        {
            unsigned long body_id, joint_id;
            utils::Vector force, torque, anchor;
            utils::Quaternion body_q;
            interfaces::sReal tmp, length;
        }joint6DOFData;

        class Joint6DOFSensor : public interfaces::SensorInterface,
                                public interfaces::BaseSensor,
                                public data_broker::ProducerInterface,
                                public data_broker::ReceiverInterface
        {

        public:

            Joint6DOFSensor(interfaces::ControlCenter *control, Joint6DOFConfig config);
            ~Joint6DOFSensor(void);

            virtual int getAsciiData(char* data) const;
            virtual int getSensorData(interfaces::sReal **data) const;

            void getForceData(utils::Vector *force);
            void getTorqueData(utils::Vector *torque);
            void getAnchor(utils::Vector *anchor);
            void getBodyQ(utils::Quaternion* body_q);


            virtual void produceData(const data_broker::DataInfo &info,
                                     data_broker::DataPackage *package,
                                     int callbackParam);
            virtual void receiveData(const data_broker::DataInfo &info,
                                     const data_broker::DataPackage &package,
                                     int callbackParam);

            static interfaces::BaseSensor* instanciate(interfaces::ControlCenter *control,
                                                       interfaces::BaseConfig* config);
            static interfaces::BaseConfig* parseConfig(interfaces::ControlCenter *control,
                                                       configmaps::ConfigMap *config);
            virtual configmaps::ConfigMap createConfig() const;

            // TODO: Add to interface?
            void getDataBrokerNames(std::string& groupName, std::string& dataName) const;

        private:
            Joint6DOFConfig config;
            joint6DOFData sensor_data;
            unsigned long jointPackageId, nodePackageId;
            long nodeRotIndices[4];
            long jointForceIndices[3];
            long jointTorqueIndices[3];
            data_broker::DataPackage dbPackage;
            unsigned long dbPushId;
        };

    } // end of namespace core
} // end of namespace mars
