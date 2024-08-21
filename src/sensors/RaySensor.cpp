/*
 *  RaySensor.cpp
 *
 *  Created by Malte Langosz
 *
 */

#include "RaySensor.hpp"

#include <mars_utils/mathUtils.h>
#include <mars_interfaces/sim/SimulatorInterface.h>

#include <data_broker/DataBrokerInterface.h>
#include <mars_interfaces/sim/CollisionInterface.hpp>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace mars
{
    namespace core
    {

        using namespace utils;
        using namespace configmaps;
        using namespace interfaces;

        BaseSensor* RaySensor::instanciate(ControlCenter *control, BaseConfig *config )
        {
            RayConfig *cfg = dynamic_cast<RayConfig*>(config);
            assert(cfg);
            return new RaySensor(control,*cfg);
        }

        RaySensor::RaySensor(ControlCenter *control, RayConfig config):
            BasePolarIntersectionSensor(config.id, config.name, config.width,
                                        config.height, config.opening_width,
                                        config.opening_height),
            SensorInterface(control), config(config)
        {
            updateRate = config.updateRate;
            maxDistance = config.maxDistance;
            //this->attached_node = config.attached_node;

            drawStruct draw;
            draw_item item;
            int i;
            Vector tmp;
            for(int i = 0; i < 3; ++i)
                positionIndices[i] = -1;
            for(int i = 0; i < 4; ++i)
                rotationIndices[i] = -1;

            if(control->dataBroker->registerTimedReceiver(this, config.groupName, config.dataName,"mars_sim/simTimer",updateRate))
            {
            }

            fprintf(stderr, "RaySensor init pos: %g %g %g\n", config.init_position.x(), config.init_position.y(), config.init_position.z());
            position = config.init_position + config.init_orientation*config.pos_offset;
            orientation = config.init_orientation * config.ori_offset;

            // generate the directions only once:
            rad_steps = getCols(); //rad_angle/(sReal)(sensor.resolution-1);
            double rad_start = -((rad_steps-1)/2.0)*stepX; //Starting to Left, because 0 is in front and rock convention posive CCW //(M_PI-rad_angle)/2;
            if(rad_steps == 1)
            {
                rad_start = 0;
            }
            for(i=0; i<rad_steps; i++)
            {
                tmp = Vector(cos(rad_start+i*stepX),
                             sin(rad_start+i*stepX), 0);
                directions.push_back(tmp);
                data[i] = config.maxDistance;
            }

            //Drawing Stuff
            if(config.draw_rays)
            {
                draw.ptr_draw = (DrawInterface*)this;
                item.id = 0;
                item.type = DRAW_LINE;
                item.draw_state = DRAW_STATE_CREATE;
                item.point_size = 1;
                item.myColor.r = 1;
                item.myColor.g = 0;
                item.myColor.b = 0;
                item.myColor.a = 1;
                item.texture = "";
                item.t_width = item.t_height = 0;
                item.get_light = 0.0;

                //printf("Rad Start: %f, rad_steps: %f, stepX: %f\n",rad_start,rad_steps,stepX);

                for(i=0; i<rad_steps; i++)
                {
                    item.start = position;
                    item.end = position + orientation * directions[i];
                    item.end *= data[i];
                    draw.drawItems.push_back(item); // TEST
                }

                if(control->graphics)
                    control->graphics->addDrawItems(&draw);


                assert(rad_steps == data.size());
            }
            // todo: need more intelligent solution: maybe generate data only on demand?
            //       or move it to seperated thread depending on performance
            deactivate = true;
        }

        RaySensor::~RaySensor(void)
        {
            if(control->graphics)
                control->graphics->removeDrawItems((DrawInterface*)this);
            if (control->dataBroker)
                control->dataBroker->unregisterTimedReceiver(this, "*", "*",
                                                             "mars_sim/simTimer");
        }

        std::vector<double> RaySensor::getSensorData() const
        {
            std::vector<double> result;
            result.resize(data.size());
            if(deactivate)
            {
                deactivate = false;
                return result;
            }
            for(unsigned int i=0; i<data.size(); i++)
            {
                result[i] = data[i];
            }
            return result;
        }

        int RaySensor::getSensorData(double **data_) const
        {
            if(deactivate)
            {
                deactivate = false;
                return NULL;
            }
            *data_ = (double*)malloc(data.size()*sizeof(double));
            for(unsigned int i=0; i<data.size(); i++)
            {
                (*data_)[i] = data[i];
            }
            return data.size();
        }

        void RaySensor::receiveData(const data_broker::DataInfo &info,
                                    const data_broker::DataPackage &package,
                                    int callbackParam)
        {
            CPP_UNUSED(info);
            CPP_UNUSED(callbackParam);
            long id;
            package.get(0, &id);

            if(positionIndices[0] == -1)
            {
                positionIndices[0] = package.getIndexByName("position/x");
                positionIndices[1] = package.getIndexByName("position/y");
                positionIndices[2] = package.getIndexByName("position/z");
                rotationIndices[0] = package.getIndexByName("rotation/x");
                rotationIndices[1] = package.getIndexByName("rotation/y");
                rotationIndices[2] = package.getIndexByName("rotation/z");
                rotationIndices[3] = package.getIndexByName("rotation/w");
            }
            for(int i = 0; i < 3; ++i)
            {
                package.get(positionIndices[i], &position[i]);
            }

            package.get(rotationIndices[0], &orientation.x());
            package.get(rotationIndices[1], &orientation.y());
            package.get(rotationIndices[2], &orientation.z());
            package.get(rotationIndices[3], &orientation.w());

            position += orientation*config.pos_offset;
            orientation *= config.ori_offset;
            if(!deactivate)
            {
                Vector tmp;
                for(size_t i=0; i<directions.size(); i++)
                {
                    tmp = orientation * directions[i];
                    tmp *= config.maxDistance;
                    data[i] = control->sim->getVectorCollision(position, tmp);
                }
            }
        }

        void RaySensor::update(std::vector<draw_item>* drawItems)
        {
            if(config.draw_rays)
            {
                if(!(*drawItems)[0].draw_state)
                {
                    for(size_t i=0; i<data.size(); i++)
                    {
                        (*drawItems)[i].draw_state = DRAW_STATE_UPDATE;
                        (*drawItems)[i].start = position;
                        (*drawItems)[i].end = (orientation * directions[i]);
                        (*drawItems)[i].end *= data[i];

                        (*drawItems)[i].end += (*drawItems)[i].start;
                    }
                }
            }
        }

        BaseConfig* RaySensor::parseConfig(ControlCenter *control,
                                           ConfigMap *config)
        {
            RayConfig *cfg = new RayConfig;
            unsigned int mapIndex = (*config)["mapIndex"];
            unsigned long attachedNodeID = (*config)["attached_node"];

            ConfigMap::iterator it;
            if((it = config->find("width")) != config->end())
                cfg->width = it->second;
            if((it = config->find("opening_width")) != config->end())
                cfg->opening_width = it->second;
            if((it = config->find("max_distance")) != config->end())
                cfg->maxDistance = it->second;
            if((it = config->find("draw_rays")) != config->end())
                cfg->draw_rays = it->second;
            if((it = config->find("rate")) != config->end())
                cfg->updateRate = it->second;

            cfg->attached_node = attachedNodeID;
#warning Parse stepX stepY cols and rows
            cfg->groupName << (*config)["groupName"];
            cfg->dataName << (*config)["dataName"];
            if((it = config->find("pos_offset")) != config->end())
            {
                vectorFromConfigItem(it->second, &(cfg->pos_offset));
            }
            if((it = config->find("ori_offset")) != config->end())
            {
                quaternionFromConfigItem(it->second, &(cfg->ori_offset));
            }
            if((it = config->find("init_position")) != config->end())
            {
                vectorFromConfigItem(it->second, &(cfg->init_position));
            }
            if((it = config->find("init_orientation")) != config->end())
            {
                quaternionFromConfigItem(it->second, &(cfg->init_orientation));
            }

            /*
              ConfigMap::iterator it;

              if((it = config->find("stepX")) != config->end())
              cfg->stepX = it->second[0].getDouble();

              if((it = config->find("stepY")) != config->end())
              cfg->stepY = it->second[0].getDouble();

              if((it = config->find("cols")) != config->end())
              cfg->cols = it->second[0].getUInt();

              if((it = config->find("rows")) != config->end())
              cfg->rows = it->second[0].getUInt();
            */
            return cfg;
        }

        ConfigMap RaySensor::createConfig() const
        {
            ConfigMap cfg;
            cfg["name"] = config.name;
            cfg["id"] = config.id;
            cfg["type"] = "RaySensor";
            cfg["attached_node"] = config.attached_node;
            cfg["width"] = config.width;
            cfg["opening_width"] = config.opening_width;
            cfg["max_distance"] = config.maxDistance;
            cfg["draw_rays"] = config.draw_rays;
            cfg["rate"] = config.updateRate;
            /*
              cfg["stepX"] = config.stepX;
              cfg["stepY"] = config.stepY;
              cfg["cols"] = config.cols;
              cfg["rows"] = config.rows;
            */
            return cfg;
        }

        const RayConfig& RaySensor::getConfig() const
        {
            return config;
        }

    } // end of namespace core
} // end of namespace mars
