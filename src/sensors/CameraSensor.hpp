#pragma once

#include <data_broker/ReceiverInterface.h>

#include <mars_interfaces/sim/SensorInterface.h>
//#include <mars_interfaces/sim/EntityManagerInterface.h>
#include <mars_utils/Vector.h>
#include <mars_utils/Quaternion.h>
#include <mars_utils/Mutex.h>
#include <mars_interfaces/graphics/GraphicsWindowInterface.h>
#include <mars_interfaces/graphics/GraphicsUpdateInterface.h>
#include <mars_interfaces/graphics/GraphicsCameraInterface.h>
//#include "SimEntity.h"
#include <base/Eigen.hpp>

#include <inttypes.h>
typedef uint8_t  u_int8_t;


namespace mars
{

    namespace graphics
    {
        class GraphicsWindowInterface;
        class GraphicsCameraInterface;
    }

    namespace core
    {

        enum ViewMode
        {
            CENTER,
            VERTEX_OF_BBOX,
            EVERYTHING,
            NOTHING
        };

        class Pixel
        {
        public:
            u_int8_t r;
            u_int8_t g;
            u_int8_t b;
            u_int8_t a;
        } __attribute__ ((packed)) ;

        typedef float DistanceMeasurement;

        struct CameraConfigStruct: public interfaces::BaseConfig
        {
            CameraConfigStruct()
                {
                    name = "Unknown Camera";
                    width=640;
                    height=480;
                    enabled = true;
                    show_cam = false;
                    hud_pos=0;
                    pos_offset.setZero();
                    ori_offset.setIdentity();
                    opening_width=90;
                    opening_height=-1;
                    hud_width = 320;
                    hud_height = -1;
                    depthImage = false;
                    logicalImage = false;
                    frameOffset = 0;
                    frameSkip = 0;
                }

            unsigned long attached_node;
            int width;
            int height;
            bool show_cam;
            int hud_pos;
            int frameOffset, frameSkip;
            utils::Vector pos_offset;
            utils::Quaternion ori_offset;
            double opening_width; // deprecated: we should probably rename this to opening_angle
            double opening_height; // deprecated: we should probably rename this to opening_angle2
            int hud_width;
            int hud_height;
            bool depthImage;
            bool logicalImage;
            bool enabled;
            configmaps::ConfigMap map;
            static int globalFrameOffset, globalHudPos;
        };

        class CameraSensor : public interfaces::BaseNodeSensor,
                             public interfaces::SensorInterface,
                             public data_broker::ReceiverInterface,
                             public interfaces::GraphicsUpdateInterface
        {
        public:
            static interfaces::BaseSensor* instanciate(interfaces::ControlCenter *control,
                                                       interfaces::BaseConfig *config );
            CameraSensor(interfaces::ControlCenter *control, const CameraConfigStruct config);
            ~CameraSensor(void);

            virtual int getSensorData(interfaces::sReal** data) const;
            void getColoredPointcloud(std::vector<utils::Vector> *data,
                                      std::vector<base::Vector4d> *colors);

            void getImage(std::vector<Pixel> &buffer) const;
            void getDepthImage(std::vector<DistanceMeasurement> &buffer) const;
            //void getEntitiesInView(std::map<unsigned long, SimEntity*> &buffer, unsigned int visVert_threshold);

            virtual void receiveData(const data_broker::DataInfo &info,
                                     const data_broker::DataPackage &package,
                                     int callbackParam);

            virtual void preGraphicsUpdate(void);
            virtual void postGraphicsUpdate(void);
            void getCameraInfo( interfaces::cameraStruct *cs );

            static interfaces::BaseConfig* parseConfig(interfaces::ControlCenter *control,
                                                       configmaps::ConfigMap *config);
            virtual configmaps::ConfigMap createConfig() const;

            const CameraConfigStruct &getConfig() const
                {
                    return config;
                }

            void deactivateRendering();
            void activateRendering();
            unsigned long getWindowID() const {return cam_window_id;}
            bool haveNewData();
            void getPose(utils::Vector &p, utils::Quaternion &q);

        private:
            CameraConfigStruct config;
            interfaces::BaseCameraSensor<double> depthCamera;
            interfaces::BaseCameraSensor<char*> imageCamera;
            //interfaces::BaseCameraSensor<SimEntity*> logicalCamera;
            unsigned long cam_window_id;
            interfaces::GraphicsWindowInterface *gw;
            interfaces::GraphicsCameraInterface* gc;
            long dbPosIndices[3];
            long dbRotIndices[4];
            unsigned int cam_id;
            utils::Mutex mutex;
            int renderCam;
            unsigned long draw_id;
            double maxDistance;
            bool have_new_data;
        };

    } // end of namespace core
} // end of namespace mars

