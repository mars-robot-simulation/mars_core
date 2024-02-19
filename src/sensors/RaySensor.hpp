/*
 *  RaySensor.hpp
 *
 *  Created by Malte Langosz
 *
 */

#pragma once

#include <mars_interfaces/sim/SensorInterface.h>
#include <data_broker/ReceiverInterface.h>
#include <mars_utils/Vector.h>
#include <mars_utils/Quaternion.h>
#include <mars_interfaces/graphics/draw_structs.h>

namespace mars
{
    namespace core
    {

        class RayConfig : public interfaces::BaseConfig
        {
        public:
            RayConfig()
                {
                    name = "Unknown RaySensor";
                    width=1;
                    height=1;
                    pos_offset.setZero();
                    ori_offset.setIdentity();
                    init_position.setZero();
                    init_orientation.setIdentity();
                    opening_width=0.5*M_PI;
                    opening_height=0.5*M_PI;
                    attached_node = 0;
                    maxDistance = 100.0;
                    draw_rays = true;
                }

            unsigned long attached_node;
            int width;
            int height;
            utils::Vector pos_offset, init_position;
            utils::Quaternion ori_offset, init_orientation;
            double opening_width;
            double opening_height;
            double maxDistance;
            bool draw_rays;
            std::string groupName, dataName;
        };

        class RaySensor :
            public interfaces::BasePolarIntersectionSensor ,
            public interfaces::SensorInterface,
            public data_broker::ReceiverInterface,
            public interfaces::DrawInterface
        {

        public:
            static interfaces::BaseSensor* instanciate(interfaces::ControlCenter *control,
                                                       interfaces::BaseConfig* config);
            RaySensor(interfaces::ControlCenter *control, RayConfig config);
            ~RaySensor(void);

            std::vector<double> getSensorData() const;
            int getSensorData(double**) const;
            virtual void receiveData(const data_broker::DataInfo &info,
                                     const data_broker::DataPackage &package,
                                     int callbackParam);
            virtual void update(std::vector<interfaces::draw_item>* drawItems);

            static interfaces::BaseConfig* parseConfig(interfaces::ControlCenter *control,
                                                       configmaps::ConfigMap *config);
            virtual configmaps::ConfigMap createConfig() const;

            const RayConfig& getConfig() const;

        private:
            RayConfig config;
            std::vector<utils::Vector> directions;
            int rad_steps;
            bool have_update;
            mutable bool deactivate;
            long positionIndices[3];
            long rotationIndices[4];
        };

    } // end of namespace core
} // end of namespace mars
