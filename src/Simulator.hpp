/**
 * \file Simulator.hpp
 * \author Malte Langosz
 * \brief "Simulator" is the main class of the simulation
 *
 */

#pragma once

#include "SubWorld.hpp"
#include "CollisionManager.hpp"
#include "AbsolutePoseExtender.hpp"

#include <data_broker/DataPackage.h>
#include <data_broker/ReceiverInterface.h>
#include <cfg_manager/CFGManagerInterface.h>
#include <mars_utils/Thread.h>
#include <mars_utils/Mutex.h>
#include <mars_utils/WaitCondition.h>
#include <mars_utils/ReadWriteLock.h>
#include <mars_interfaces/sim/SimulatorInterface.h>
#include <mars_interfaces/sim/PhysicsInterface.h>
#include <mars_interfaces/sim/JointInterface.h>

#include <mars_interfaces/sim/PluginInterface.h>
#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_interfaces/graphics/GraphicsUpdateInterface.h>
#include <mars_utils/Vector.h>

// for graphical debugging
#include <osg_lines/Lines.hpp>
#include <osg_lines/LinesFactory.hpp>

#include <envire_core/items/Item.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>

#include <envire_core/graph/GraphTypes.hpp>
#include <base/TransformWithCovariance.hpp>

#include <envire_types/World.hpp>

#include <iostream>
#include <memory>

// TODO: remove smurf::Collidable, base types Collision object are move to envire_mars_ode_collision
// TODO: move motor and sensors to separate library
namespace mars
{
    namespace ode_physics
    {
        class WorldPhysicsLoader;
    }

    namespace ode_collision
    {
        class CollisionSpaceLoader;
    }

    namespace core
    {
        /**
         *\brief The Simulator class implements the main functions of the MARS simulation.
         *
         * Its constructor presents the core function in the simulation and directly takes the arguments
         * given by the user if the simulation is starting from a command line.
         * It inherits the mars::Thread \c class and the i_GuiToSim
         * A Simulator object presents a separate thread and shares data with all other threads within the process.(remarque)
         * To handle and access the data of the Simulator properly, the mutex variable \c coreMutex is used.
         *
         */
        class Simulator : public utils::Thread,
                          public interfaces::SimulatorInterface,
                          public interfaces::GraphicsUpdateInterface,
                          public lib_manager::LibInterface,
                          public cfg_manager::CFGClient,
                          public data_broker::ReceiverInterface,
                          public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::World>>
        {

        public:

            enum Status
            {
                UNKNOWN = -1,
                STOPPED = 0,
                RUNNING = 1,
                STOPPING = 2,
                STEPPING = 3
            };

            Simulator(lib_manager::LibManager *theManager); ///< Constructor of the \c class Simulator.
            virtual ~Simulator();
            static Simulator *activeSimulator;


            // --- LibInterface ---
            int getLibVersion() const override
            {
                return 1;
            }

            const std::string getLibName() const override
            {
                return std::string{"mars_core"};
            }

            void newLibLoaded(const std::string &libName) override;
            CREATE_MODULE_INFO();

            // --- SimulatorInterface ---

            // controlling the simulation
            void updateSim(); ///< Updates the graphical simulation.
            void myRealTime(void); ///< control the realtime calculation
            void runSimulation(bool startThread = true) override; ///< Initiates the simulation

            /**
             * Returns the tmp path for temprary files, on linux /tmp/mars on windows the current config_dir
             */
            std::string getTmpPath() const;

            virtual void StartSimulation() override;

            virtual void StopSimulation() override
            {
                stepping_mutex.lock();
                if(simulationStatus != STOPPED)
                {
                    simulationStatus = STOPPING;
                    //stepping_wc.wakeAll();
                }
                stepping_mutex.unlock();
            }

            virtual void resetSim(bool resetGraphics=true) override;
            virtual bool isSimRunning() const override;
            bool startStopTrigger() override; ///< Starts and pauses the simulation.
            virtual void singleStep(void) override;
            virtual void newWorld(bool clear_all = false) override;
            virtual void exitMars(void) override;
            virtual void readArguments(int argc, char **argv) override;
            virtual interfaces::ControlCenter* getControlCenter(void) const override;

            // simulation contents
            virtual void addLight(interfaces::LightData light) override;
            virtual void connectNodes(unsigned long id1, unsigned long id2) override;
            virtual void disconnectNodes(unsigned long id1, unsigned long id2) override;
            virtual void rescaleEnvironment(interfaces::sReal x, interfaces::sReal y, interfaces::sReal z) override;

            // scenes
            virtual int loadScene(const std::string &filename,
                                  const std::string &robotname,bool threadsave=false, bool blocking=false) override;
            virtual int loadScene(const std::string &filename,
                                  bool wasrunning=false,
                                  const std::string &robotname="",bool threadsave=false, bool blocking=false) override;
            virtual int loadScene(const std::string &filename,
                                  const std::string &robotname,
                                  utils::Vector pos,
                                  utils::Vector rot, bool threadsave=false, bool blocking=false, bool wasrunning=false) override;
            virtual int saveScene(const std::string &filename, bool wasrunning) override;
            virtual void exportScene() const override; ///< Exports the current scene as both *.obj and *.osg file.
            virtual bool sceneChanged() const override;
            virtual void sceneHasChanged(bool reset) override;

            //threads
            bool allConcurrencysHandled(); ///< Checks if external requests are open.
            void setSyncThreads(bool value) override; ///< Syncs the threads of GUI and simulation.
            virtual void physicsThreadLock(void) override;
            virtual void physicsThreadUnlock(void) override;


            //physics
            virtual std::shared_ptr<interfaces::PhysicsInterface> getPhysics(void) const override;
            virtual void handleError(interfaces::PhysicsError error) override;
            virtual void setGravity(const utils::Vector &gravity) override;
            virtual int checkCollisions(void) override;
            virtual bool hasSimFault() const override; ///< Checks if the physic simulation thread has been stopped caused by an ODE error.

            //graphics
            virtual void preGraphicsUpdate(void) override;
            virtual void postGraphicsUpdate(void) override;
            virtual void finishedDraw(void) override;
            void allowDraw(void) override; ///< Allows the osgWidget to draw a frame.

            virtual bool getAllowDraw(void) override
            {
                return allow_draw;
            }

            virtual bool getSyncGraphics(void) override
            {
                return sync_graphics;
            }

            // plugins
            virtual void addPlugin(const interfaces::pluginStruct& plugin) override;
            virtual void removePlugin(interfaces::PluginInterface *pl) override;
            virtual void switchPluginUpdateMode(int mode, interfaces::PluginInterface *pl) override;
            virtual void sendDataToPlugin(int plugin_index, void* data) override;

            //  virtual double initTimer(void);
            //  virtual double getTimer(double start) const;

            virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) override;

            const std::string getConfigDir() const
            {
                return config_dir;
            }

            //public slots:
            // TODO: currently this is disabled
            void noGUITimerUpdate(void);


            // --- ReceiverInterface ---

            virtual void receiveData(const data_broker::DataInfo &info,
                                     const data_broker::DataPackage &package,
                                     int callbackParam) override;

            virtual const utils::Vector& getGravity(void) override;

            virtual void step(bool setState = false) override;

            /*
             * returns the real startTimestamp plus the calculated simulation time
             */
            virtual unsigned long getTime() override;


            // envire callbacks
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::World>>& e) override;

            // access graph
            // TODO: probably it is temporary and will be moved in the extra class
            virtual std::shared_ptr<envire::core::EnvireGraph> getGraph() override;
            virtual std::string getRootFrame() override;

            virtual std::shared_ptr<interfaces::SubControlCenter> createSubWorld(const std::string &name) override;
            std::shared_ptr<interfaces::SubControlCenter> getSubControl(envire::core::FrameId frame);

            void saveGraph(const std::string &fileName) override;

            static void updateChildPositions(const envire::core::GraphTraits::vertex_descriptor vertex,
                                             const base::TransformWithCovariance& rootToFrame,
                                             std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                             std::shared_ptr<envire::core::TreeView> &graphTreeView);
            static void updateChildPositions(const envire::core::GraphTraits::vertex_descriptor vertex,
                                             std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                             std::shared_ptr<envire::core::TreeView> &graphTreeView);
            static void updatePositions( const envire::core::GraphTraits::vertex_descriptor origin,
                                         const envire::core::GraphTraits::vertex_descriptor target,
                                         const base::TransformWithCovariance& rootToOrigin,
                                         std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                         std::shared_ptr<envire::core::TreeView> &graphTreeView);
            static void applyChildPositions(const envire::core::GraphTraits::vertex_descriptor vertex,
                                            const base::TransformWithCovariance& rootToFrame,
                                            std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                            std::shared_ptr<envire::core::TreeView> &graphTreeView);
            static void applyChildPositions(const envire::core::GraphTraits::vertex_descriptor vertex,
                                            std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                            std::shared_ptr<envire::core::TreeView> &graphTreeView);
            static void applyPositions( const envire::core::GraphTraits::vertex_descriptor origin,
                                        const envire::core::GraphTraits::vertex_descriptor target,
                                        const base::TransformWithCovariance& rootToOrigin,
                                        std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                        std::shared_ptr<envire::core::TreeView> &graphTreeView);
            static void rotateRevolute( envire::core::FrameId origin,
                                        double angle,
                                        std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                        std::shared_ptr<envire::core::TreeView> &graphTreeView);
            static void rotateRevolute( const envire::core::GraphTraits::vertex_descriptor origin,
                                        double angle,
                                        std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                        std::shared_ptr<envire::core::TreeView> &graphTreeView);

            // @getStepSizeS: Returns step size of the physics simulations in seconds.
            interfaces::sReal getStepSizeS() const;
            virtual interfaces::sReal getVectorCollision(utils::Vector position, utils::Vector ray);

        private:

            struct LoadOptions {
                std::string filename;
                std::string robotname;
                bool wasRunning;
                bool zeroPose;
                utils::Vector pos;
                utils::Vector rot;
            };

            // simulation control
            void processRequests();
            void reloadWorld(void);

            // Setup
            void checkOptionalDependency(const std::string &libName);
            void setupDataBroker();
            void setupCFGManager();
            void setupMarsGraphics();
            void setupLogConsole();
            void setupManagers(lib_manager::LibManager* const libManager);
            void setupPhysics();
            void setupCollisions();
            void loadConfigurations();

            int arg_no_gui, arg_run, arg_grid, arg_ortho;
            bool reloadSim, reloadGraphics;
            short running;
            char was_running;
            bool kill_sim;
            ode_physics::WorldPhysicsLoader* physicsLoader; // plugin reference
            ode_collision::CollisionSpaceLoader* collisionSpaceLoader; // plugin reference
            std::map<std::string, std::unique_ptr<SubWorld>> subWorlds;
            std::unique_ptr<CollisionManager> collisionManager;
            std::shared_ptr<interfaces::ControlCenter> control; ///< Pointer to instance of ControlCenter (created in Simulator::Simulator(lib_manager::LibManager *theManager))
            std::vector<LoadOptions> filesToLoad;
            bool sim_fault;
            bool exit_sim;
            Status simulationStatus;
            interfaces::sReal sync_time;
            bool my_real_time;
            bool fast_step;

            // graphics
            bool allow_draw;
            bool sync_graphics;
            int cameraMenuCheckedIndex;

            // threads
            bool erased_active;
            utils::ReadWriteLock pluginLocker;
            int sync_count;
            utils::Mutex externalMutex;
            utils::Mutex coreMutex;
            utils::Mutex physicsMutex;
            utils::Mutex physicsCountMutex;
            utils::Mutex stepping_mutex; ///< Used for preventing active waiting for a single step or start event.
            utils::WaitCondition stepping_wc; ///< Used for preventing active waiting for a single step or start event.
            utils::Mutex getTimeMutex;
            int physics_mutex_count;
            double avg_log_time, avg_step_time;
            int count, avg_count_steps;
            interfaces::sReal calc_time;
            long simStartTime, simRealTime;

            // physics
            std::shared_ptr<interfaces::CollisionInterface> collisionSpace;

            // @calc_ms: step size of the simulation.
            // Is assumed to not change as ode advises against changeing the step size of the simulation (https://ode.org/wiki/index.php/Manual#Variable_Step_Size:_Don.27t.21)
            double calc_ms;
            int load_option;
            int std_port; ///< Controller port (default value: 1600)
            utils::Vector gravity;
            unsigned long dbPhysicsUpdateId;
            unsigned long dbSimTimeId, dbSimDebugId;
            unsigned long realStartTime;

            std::unique_ptr<AbsolutePoseExtender> absolutePoseExtender;

            // plugins
            std::vector<interfaces::pluginStruct> allPlugins;
            std::vector<interfaces::pluginStruct> newPlugins;
            std::vector<interfaces::pluginStruct> activePlugins;
            std::vector<interfaces::pluginStruct> guiPlugins;
            std::vector<interfaces::pluginStruct> physicsPlugins;

            // scenes
            int loadScene_internal(const std::string &filename, bool wasrunning, const std::string &robotname);
            int loadScene_internal(const std::string &filename, const std::string &robotname,
                                   utils::Vector pos, utils::Vector rot, bool wasrunning);

            std::string scenename;
            std::list<std::string> arg_v_scene_name;
            bool b_SceneChanged;
            bool haveNewPlugin;

            // for graphical debuggin
            std::unique_ptr<osg_lines::Lines> contactLines;
            utils::Mutex contactLinesDataMutex;
            std::list<osg_lines::Vector> contactLinesData;

            // configuration
            void initCfgParams(void);
            std::string config_dir;
            cfg_manager::cfgPropertyStruct cfgCalcMs, cfgFaststep;
            cfg_manager::cfgPropertyStruct cfgRealtime, cfgDebugTime;
            cfg_manager::cfgPropertyStruct cfgSyncGui, cfgDrawContact;
            cfg_manager::cfgPropertyStruct cfgGX, cfgGY, cfgGZ;
            cfg_manager::cfgPropertyStruct cfgWorldErp, cfgWorldCfm;
            cfg_manager::cfgPropertyStruct cfgSyncTime;
            cfg_manager::cfgPropertyStruct cfgVisRep;
            cfg_manager::cfgPropertyStruct configPath;
            cfg_manager::cfgPropertyStruct cfgUseNow;
            cfg_manager::cfgPropertyStruct cfgAvgCountSteps;

            // data
            data_broker::DataPackage dbPhysicsUpdatePackage;
            data_broker::DataPackage dbSimTimePackage;
            data_broker::DataPackage dbSimDebugPackage;

        protected:

            // simulation control
            void run() override; ///< The simulator main loop.

        private:
            void resetPoses();
            void reloadObjects();
        };
    } // end of namespace core
} // end of namespace mars
