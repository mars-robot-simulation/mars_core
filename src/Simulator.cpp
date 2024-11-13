/**
 * \file Simulator.cpp
 * \author Malte Langosz
 *
 */

/* Conventions:
 *   - the includes should be defined in the header file
 *   - atomic variables shoud use snake_case
 *   - instances of classes should use camel_case
 *   - method names are camel_case
 *   - always use braces
 *   - braces start in new line
 *   - indent with four tabs
 */

#include "Simulator.hpp"
#include "MotorManager.hpp"
#include "SensorManager.hpp"
#include "JointManager.hpp"
#include "NodeManager.hpp"

//#include "PhysicsMapper.h"

#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>
#include <mars_interfaces/SceneParseException.h>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/LoadCenter.h>
#include <mars_interfaces/sim/LoadSceneInterface.h>
#include <mars_interfaces/sim/DynamicObject.hpp>
#include <data_broker/DataBrokerInterface.h>
#include <lib_manager/LibInterface.hpp>

// TODO: extend interfaces to not require the ode_physics library on compile time
#include <mars_ode_physics/WorldPhysicsLoader.hpp>

#include <mars_ode_collision/CollisionSpaceLoader.hpp>
#include <mars_ode_collision/CollisionHandler.hpp>
#include <mars_ode_collision/objects/Object.hpp>

#include <mars_interfaces/Logging.hpp>
#include <mars_interfaces/MARSDefs.h>

#include <mars_interfaces/sim/AbsolutePose.hpp>
#include <mars_interfaces/sim/JointInterface.h>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/TreeView.hpp>

#include <envire_core/graph/GraphDrawing.hpp>
#include <envire_types/joints/Continuous.hpp>
#include <envire_types/joints/Revolute.hpp>
#include <envire_types/Link.hpp>
#include <envire_types/Inertial.hpp>
#include <envire_types/geometry/Box.hpp>
#include <envire_types/geometry/Capsule.hpp>
#include <envire_types/geometry/Cylinder.hpp>
#include <envire_types/geometry/Mesh.hpp>
#include <envire_types/geometry/Plane.hpp>
#include <envire_types/geometry/Sphere.hpp>
#include <envire_types/geometry/Heightfield.hpp>

// TODO: should be replace by MotorInterface later
#include "SimMotor.hpp"

#include <signal.h>
#include <getopt.h>
#include <stdexcept>
#include <algorithm>
#include <cctype> // for tolower()
#include <chrono>

#include <time.h>
#ifdef __linux__
#include <unistd.h> //for getpid()
#endif

#ifndef DEFAULT_CONFIG_DIR
    #define DEFAULT_CONFIG_DIR "."
#endif

typedef envire::core::GraphTraits::vertex_descriptor VertexDesc;


namespace mars
{
    namespace core
    {
        using std::string;
        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        void hard_exit(int signal)
        {
            exit(signal);
        }


        Simulator *Simulator::activeSimulator = 0;

        Simulator::Simulator(lib_manager::LibManager *theManager) :
            lib_manager::LibInterface{theManager},
            exit_sim{false}, allow_draw{true},
            sync_graphics{false}, physics_mutex_count{0},
            haveNewPlugin{false}
        {
            // TODO: Initialize instead of define
            config_dir = DEFAULT_CONFIG_DIR;
            calc_time = 0;
            avg_step_time = avg_log_time = 0;
            count = 0;
            config_dir = ".";

            std_port = 1600;

            // we don't want the physical calculation running from the beginning
            simulationStatus = STOPPED;
            was_running = false;
            // control a clean exit from the thread
            kill_sim = 0;
            sim_fault = false;
            // set the calculation step size in ms
            calc_ms      = 10; //defaultCFG->getInt("physics", "calc_ms", 10);
            avg_count_steps = 20;
            my_real_time = 0;
            // to synchronise drawing and physics
            sync_time = 40;
            sync_count = 0;
            load_option = OPEN_INITIAL;
            reloadGraphics = true;
            reloadSim = false;
            arg_run    = 0;
            arg_grid   = 0;
            arg_ortho  = 0;
            log_warning = false;
            log_debug = false;
            char *envText = getenv("BASE_LOG_LEVEL");
            if(envText)
            {
                string envString = envText;
                if(envString == "DEBUG" || envString == "ALL")
                {
                    log_warning = true;
                    log_debug = true;
                }
                else if(envString == "WARNING")
                {
                    log_debug = true;
                }
            }

            Simulator::activeSimulator = this; // set this Simulator object to the active one

            gravity = Vector{0.0, 0.0, -9.81}; // set gravity to earth conditions

            control = std::make_shared<ControlCenter>();

            // TODO: Can calling setupManagers be moved here and be combined with setting up of the evnireGraph? Then manually adding root frame to frameIDManager would be obsolete.

            // create envire graph
            control->envireGraph_ = std::make_shared<envire::core::EnvireGraph>();
            control->envireGraph_->addFrame(SIM_CENTER_FRAME_NAME);
            control->graphTreeView_ = std::make_shared<envire::core::TreeView>();
            control->envireGraph_->getTree(SIM_CENTER_FRAME_NAME, true, control->graphTreeView_.get());
            if(!ControlCenter::envireGraph)
            {
                ControlCenter::envireGraph = control->envireGraph_;
                ControlCenter::graphTreeView = control->graphTreeView_;
            }

            // add subscribe to be notified if some items have been added into the graph
            GraphItemEventDispatcher<envire::core::Item<::envire::types::World>>::subscribe(control->envireGraph_.get());
            //GraphEventDispatcher::subscribe(control->envireGraph_.get());

            absolutePoseExtender = std::unique_ptr<AbsolutePoseExtender>{new AbsolutePoseExtender{control->envireGraph_}};

            // build the factories
            ControlCenter::loadCenter = new LoadCenter{};
            control->sim = static_cast<SimulatorInterface*>(this);
            control->cfg = 0;//defaultCFG;
            dbSimTimePackage.add("simTime", 0.);
            dbSimDebugPackage.add("simUpdate", 0.);
            dbSimDebugPackage.add("worldStep", 0.);
            dbSimDebugPackage.add("logStep", 0.);

            checkOptionalDependency("data_broker");
            checkOptionalDependency("cfg_manager");
            checkOptionalDependency("mars_graphics");
            //checkOptionalDependency("log_console");

            // physics plugins to pass to the physics engine
            //checkOptionalDependency("envire_mls");
            //checkOptionalDependency("envire_mls_tests");

            setupManagers(theManager);
            setupPhysics();
            setupCollisions();

            getTimeMutex.lock();
            realStartTime = utils::getTime();
            getTimeMutex.unlock();

            if(control->cfg)
            {
                loadConfigurations();
            }
        }

        Simulator::~Simulator()
        {
            // unsubscribe from envire graph
            unsubscribe();
            for(auto &it: subWorlds)
            {
                it.second->stopThread = true;
                it.second->control->control = nullptr; // clear all shared pointer to ControlCenter
                while(it.second->isRunning())
                {
                    utils::msleep(1);
                }
            }
            {
                const auto* const simThread{dynamic_cast<Thread*>(this)};
                while(simThread->isRunning())
                {
                    utils::msleep(1);
                }
            }
            fprintf(stderr, "Delete mars_core\n");

            if(control->cfg)
            {
                auto saveFile = configPath.sValue;
                saveFile.append("/mars_Config.yaml");
                control->cfg->writeConfig(saveFile.c_str(), "Config");

                saveFile = configPath.sValue;
                saveFile.append("/mars_Physics.yaml");
                control->cfg->writeConfig(saveFile.c_str(), "Physics");

                saveFile = configPath.sValue;
                saveFile.append("/mars_Preferences.yaml");
                control->cfg->writeConfig(saveFile.c_str(), "Preferences");

                saveFile = configPath.sValue;
                saveFile.append("/mars_Simulator.yaml");
                control->cfg->writeConfig(saveFile.c_str(), "Simulator");
            }

            //libManager->releaseLibrary("log_console");
            // Due to event connections, we have to take care of the oder the memory is cleared
            absolutePoseExtender = nullptr;
            ControlCenter::motors = nullptr;
            ControlCenter::joints = nullptr;
            ControlCenter::sensors = nullptr;
            collisionManager->clear();
            ControlCenter::collision = nullptr;
            delete control->nodes;
            control->nodes = nullptr;
            ControlCenter::envireGraph = nullptr;
            ControlCenter::graphTreeView = nullptr;
            bool releaseGraphics = false;
            if(control->graphics)
            {
                releaseGraphics = true;
            }
            control = nullptr;
            subWorlds.clear();

            // TODO: do we need to delete control?
            libManager->releaseLibrary("mars_ode_physics");
            libManager->releaseLibrary("mars_ode_collision");
            if(releaseGraphics)
            {
                libManager->releaseLibrary("mars_graphics");
            }
            libManager->releaseLibrary("cfg_manager");
            libManager->releaseLibrary("data_broker");
        }

        void Simulator::newLibLoaded(const std::string &libName)
        {
            checkOptionalDependency(libName);
        }

        void Simulator::checkOptionalDependency(const string &libName)
        {
            if(libName == "data_broker")
            {
                setupDataBroker();
            }
            else if(libName == "cfg_manager")
            {
                setupCFGManager();
            }
            else if(libName == "mars_graphics" && !control->graphics)
            {
                setupMarsGraphics();
            }
            else if(libName == "log_console")
            {
                setupLogConsole();
            }
        }

        void Simulator::setupDataBroker()
        {
            ControlCenter::theDataBroker = libManager->getLibraryAs<data_broker::DataBrokerInterface>("data_broker");
            if(ControlCenter::theDataBroker)
            {
                // control->dataBroker is deprecated
                // TODO: we keep control->dataBroker for backwards compatibility
                control->dataBroker = ControlCenter::theDataBroker;

                // create streams
                getTimeMutex.lock();
                dbSimTimeId = ControlCenter::theDataBroker->pushData("mars_sim", "simTime",
                                                            dbSimTimePackage,
                                                            nullptr,
                                                            data_broker::DATA_PACKAGE_READ_FLAG);
                dbSimDebugId = ControlCenter::theDataBroker->pushData("mars_sim", "debugTime",
                                                                dbSimDebugPackage,
                                                                nullptr,
                                                                data_broker::DATA_PACKAGE_READ_FLAG);
                getTimeMutex.unlock();
                ControlCenter::theDataBroker->createTimer("mars_sim/simTimer");
                ControlCenter::theDataBroker->createTrigger("mars_sim/prePhysicsUpdate");
                ControlCenter::theDataBroker->createTrigger("mars_sim/postPhysicsUpdate");
                ControlCenter::theDataBroker->createTrigger("mars_sim/finishedDrawTrigger");

                // setup output
                ControlCenter::theDataBroker->registerSyncReceiver(this, "_MESSAGES_", "fatal",
                                                        data_broker::DB_MESSAGE_TYPE_FATAL);
                ControlCenter::theDataBroker->registerSyncReceiver(this, "_MESSAGES_", "error",
                                                        data_broker::DB_MESSAGE_TYPE_ERROR);
                ControlCenter::theDataBroker->registerSyncReceiver(this, "_MESSAGES_", "warning",
                                                        data_broker::DB_MESSAGE_TYPE_WARNING);
                ControlCenter::theDataBroker->registerSyncReceiver(this, "_MESSAGES_", "info",
                                                        data_broker::DB_MESSAGE_TYPE_INFO);
                ControlCenter::theDataBroker->registerSyncReceiver(this, "_MESSAGES_", "debug",
                                                        data_broker::DB_MESSAGE_TYPE_DEBUG);
                LOG_DEBUG("Simulator: no console loaded. output to stdout!");
            }
            else
            {
                fprintf(stderr, "ERROR: could not get DataBroker!\n");
            }
        }

        void Simulator::setupCFGManager()
        {
            control->cfg = libManager->getLibraryAs<cfg_manager::CFGManagerInterface>("cfg_manager");
        }

        void Simulator::setupMarsGraphics()
        {
            control->graphics = libManager->getLibraryAs<interfaces::GraphicsManagerInterface>("mars_graphics");
            if(control->graphics)
            {
                LOG_INFO("loaded mars_graphics");
                ControlCenter::loadCenter->loadMesh = control->graphics->getLoadMeshInterface();
                ControlCenter::loadCenter->loadHeightmap = control->graphics->getLoadHeightmapInterface();
                control->graphics->initializeOSG(nullptr);
            }
        }

        void Simulator::setupLogConsole()
        {
            const auto* const lib = libManager->getLibrary("log_console");
            if(ControlCenter::theDataBroker)
            {
                if(lib)
                {
                    LOG_DEBUG("Simulator: console loaded. stop output to stdout!");
                    ControlCenter::theDataBroker->unregisterSyncReceiver(this, "_MESSAGES_", "*");
                }
            }
            else
            {
                LOG_ERROR("Simulator: try to setup log_console, but there is no data broker.");
            }
        }

        void Simulator::setupManagers(lib_manager::LibManager* const libManager)
        {
            ControlCenter::motors = std::make_shared<MotorManager>(control.get());
            ControlCenter::joints = std::make_shared<JointManager>(control.get());
            ControlCenter::sensors = std::make_shared<SensorManager>(control.get());
            ControlCenter::nodes = new NodeManager{control.get(), libManager};
        }

        void Simulator::setupPhysics()
        {
            // TODO: add worldphysicsLoaderInterface to mars_interfaces
            physicsLoader = libManager->getLibraryAs<ode_physics::WorldPhysicsLoader>("mars_ode_physics", true);
            if(physicsLoader)
            {
                LOG_DEBUG("physics loaded");
                //physics = physicsLoader->createWorldInstance(control);
            }
            else
            {
                LOG_ERROR("no physics loaded");
            }
        }

        void Simulator::setupCollisions()
        {
            collisionManager = std::unique_ptr<CollisionManager>{new CollisionManager{control}};
            collisionManager->addCollisionHandler("mars_ode_collision", "mars_ode_collision", std::make_shared<ode_collision::CollisionHandler>());
            collisionSpaceLoader = libManager->getLibraryAs<ode_collision::CollisionSpaceLoader>("mars_ode_collision", true);
            if(collisionSpaceLoader)
            {
                LOG_DEBUG("collision space loaded");
                collisionSpace = collisionSpaceLoader->createCollisionSpace(control.get());
                collisionSpace->initSpace();
                ControlCenter::collision = collisionSpace;
                if(control->graphics)
                {
                    control->graphics->addGraphicsUpdateInterface(this);
                    contactLines = std::unique_ptr<osg_lines::Lines>{osg_lines::LinesFactory().createLines()};
                    contactLines->setLineWidth(3.0);
                    const auto color_transparent_red = osg_lines::Color{1.0, .0, .0, .5};
                    contactLines->setColor(color_transparent_red);
                    contactLines->drawStrip(false);
                }
            }
            else
            {
                LOG_ERROR("no collision space loaded");
            }
        }

        void Simulator::loadConfigurations()
        {
            configPath = control->cfg->getOrCreateProperty("Config", "config_path", config_dir);

            //control->cfg->getOrCreateProperty("Preferences", "resources_path",
            //                                  std::string(MARS_PREFERENCES_DEFAULT_RESOURCES_PATH));

            const auto simulatorFile = std::string{configPath.sValue + "/mars_Simulator.yaml"};
            control->cfg->loadConfig(simulatorFile.c_str());
            const auto physicsFile = std::string{configPath.sValue + "/mars_Physics.yaml"};
            control->cfg->loadConfig(physicsFile.c_str());

            // @loadLastSave: Used to receive value from config and hence not constexpr.
            bool loadLastSave = false;
            control->cfg->getPropertyValue("Config", "loadLastSave", "value", &loadLastSave);
            if (loadLastSave)
            {
                const auto saveOnCloseFile = std::string{configPath.sValue + "/mars_saveOnClose.yaml"};
                control->cfg->loadConfig(saveOnCloseFile.c_str());
            }

            initCfgParams();
            if(control->graphics)
            {
                if(cfgDrawContact.bValue)
                {
                    control->graphics->addOSGNode(contactLines->getOSGNode());
                }
            }
        }

        void Simulator::runSimulation(bool startThread)
        {

            // init the physics-engine
            //Convention startPhysics function
            // LOG_DEBUG("About to create the new physics world");
            // for (auto it = std::begin(physicsPlugins); it != std::end(physicsPlugins); it++)
            // {
            //     LOG_DEBUG("Physics plugin to be handed to the physics engine: "+it->name);
            // }
            // physics = PhysicsMapper::newWorldPhysics(control);
            // if (physicsPlugins.size() > 0)
            // {
            //     physics -> setPhysicsPlugins(physicsPlugins);
            // }

            gravity.x() = cfgGX.dValue;
            gravity.y() = cfgGY.dValue;
            gravity.z() = cfgGZ.dValue;
#ifndef __linux__
            this->setStackSize(16777216);
            fprintf(stderr, "INFO: set physics stack size to: %lu\n", getStackSize());
#endif

            while(!arg_v_scene_name.empty())
            {
                LOG_INFO("Simulator: scene to load: %s",
                         arg_v_scene_name.back().c_str());
                loadScene(arg_v_scene_name.back());
                arg_v_scene_name.pop_back();
            }
            if (arg_run)
            {
                simulationStatus = RUNNING;
                arg_run = 0;
                simStartTime = dbSimTimePackage[0].d;
                simRealTime = 0;
            }

            if(startThread)
            {
                this->start();
            }

            collisionSpace->updateTransforms();
        }

        std::shared_ptr<SubControlCenter> Simulator::createSubWorld(const std::string &name)
        {
            throw std::logic_error("Simulator::createSubWorld should be obsolete. If it is needed nontheless, check for common demoninator with itemAdded<World>.");
            // SubWorld * world = new SubWorld;

            // // use the control with new physic
            // //world->control = new ControlCenter;
            // //*(world->control) = *control;
            // world->control = std::make_shared<SubControlCenter>(interfaces::SubControlCenter());
            // world->control->setPrefix(name);

            // world->control->physics = physicsLoader->createWorldInstance();
            // world->control->physics->initTheWorld();
            // // the physics step_size is in seconds
            // world->control->physics->step_size = calc_ms/1000.;
            // world->control->physics->fast_step = cfgFaststep.bValue;
            // world->control->physics->world_erp = cfgWorldErp.dValue;
            // world->control->physics->world_cfm = cfgWorldCfm.dValue;
            // world->control->physics->world_gravity = gravity;

            // // create seperate collision space
            // world->control->collision = collisionSpaceLoader->createCollisionSpace(control.get());
            // world->control->collision->initSpace();

            // //world->control->physics->draw_contact_points = cfgDrawContact.bValue;
            // subWorlds[world->control->getPrefix()] = world;
            // world->start();

            // // store the control center of the subworld in its own frame in the graph
            // using SubControlItem = envire::core::Item<std::shared_ptr<interfaces::SubControlCenter>>;
            // SubControlItem::Ptr worldItemPtr(new SubControlItem(world->control));

            // envire::core::FrameId frameId = world->control->getFrameId();
            // control->envireGraph_->addFrame(frameId);
            // control->envireGraph_->addItemToFrame(frameId, worldItemPtr);
            // // TODO: add Identity() with current time to the envire::core::Transform
            // envire::core::Transform iniPose;
            // iniPose.transform.orientation = base::Quaterniond::Identity();
            // iniPose.transform.translation << 0.0, 0.0, 0.0;
            // iniPose.time = base::Time::now();
            // control->envireGraph_->addTransform(SIM_CENTER_FRAME_NAME, frameId, iniPose);

            // // add collision item into graph
            // CollisionInterfaceItem collisionItem;
            // collisionItem.collisionInterface = world->control->collision;
            // collisionItem.pluginName = "mars_ode_collision";
            // collisionManager->addCollisionInterfaceItem(collisionItem);
            // envire::core::Item<interfaces::CollisionInterfaceItem>::Ptr collisionItemPtr(new envire::core::Item<interfaces::CollisionInterfaceItem>(collisionItem));
            // control->envireGraph_->addItemToFrame(frameId, collisionItemPtr);

            // return world->control;
            return nullptr;
        }

        /**
         * This function is executing while the program is running.
         * It handles the physical simulation, if the physical simulation is started,
         * otherwise the function is in idle mode.
         *
         * pre:
         *     start the simulator thread and by the way the Physics loop
         *
         * post:
         *
         */
        void Simulator::run()
        {

            while (!kill_sim)
            {
                stepping_mutex.lock();
                if(simulationStatus == STOPPING)
                {
                    simulationStatus = STOPPED;
                }

                if(!isSimRunning())
                {
                    stepping_wc.wait(&stepping_mutex);
                    if(kill_sim)
                    {
                        stepping_mutex.unlock();
                        break;
                    }
                }

                if (sync_graphics && !sync_count)
                {
                    msleep(2);
                    stepping_mutex.unlock();
                    continue;
                }

                if(simulationStatus == STEPPING)
                {
                    simulationStatus = STOPPING;
                }
                stepping_mutex.unlock();

                if(my_real_time)
                {
                    myRealTime();
                }
                else if(physics_mutex_count > 0)
                {
                    myRealTime();
                    // if not in realtime this thread would lock the physicsThread right
                    // after releasing it. If an other thread is trying to lock
                    // it (physics_mutex_count > 0) we sleep so it has a chance.
                    msleep(1);
                }
                else
                {
                    myRealTime();
                }
                step();
            }
            simulationStatus = STOPPED;
            // here everything of the physical simulation can be closed
        }

        void Simulator::step(bool setState)
        {
            auto oldState = Status::UNKNOWN;
            long time = 0;
            static struct timespec tv;
            //struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_nsec = 100L;

            physicsThreadLock();

            if(setState)
            {
                oldState = simulationStatus;
                simulationStatus = STEPPING;
            }

            time = utils::getTime();

            if(ControlCenter::theDataBroker)
            {
                ControlCenter::theDataBroker->trigger("mars_sim/prePhysicsUpdate");
            }

            for(auto &it: subWorlds)
            {
                it.second->control->physics->clearPreviousStep();
            }
            collisionManager->handleContacts();

            contactLinesDataMutex.lock();
            contactLinesData.clear();
            for(const auto& contact: collisionManager->getContactVector())
            {
                const auto p = osg_lines::Vector{   contact.pos.x(),
                                                    contact.pos.y(),
                                                    contact.pos.z()};
                const auto n = osg_lines::Vector{   contact.pos.x()+contact.normal.x(),
                                                    contact.pos.y()+contact.normal.y(),
                                                    contact.pos.z()+contact.normal.z()};
                contactLinesData.push_back(p);
                contactLinesData.push_back(n);
                if(auto& body1 = contact.body1)
                {
                    body1->addContact(contact);
                }
                else if(auto& body2 = contact.body2)
                {
                    body2->addContact(contact);
                }
            }
            contactLinesDataMutex.unlock();

            for(const auto &it: subWorlds)
            {
                it.second->calcStep = true;
                // while(it.second->calcStep == true) {
                //     select(0, 0, 0, 0, &tv);
                // }
            }
            for(const auto &it: subWorlds)
            {
                while(it.second->calcStep == true)
                {
#ifdef __APPLE__
                    nanosleep(&tv, NULL);
#else
                    clock_nanosleep(CLOCK_MONOTONIC, 0, &tv, 0);
#endif
                    //select(0, 0, 0, 0, &tv);
                }
            }

            avg_step_time += static_cast<double>(getTimeDiff(time));

            // control->joints->updateJoints(calc_ms);
            control->motors->updateMotors(calc_ms);

            time = utils::getTime();

            getTimeMutex.lock();
            dbSimTimePackage[0].d += calc_ms;
            getTimeMutex.unlock();
            if(ControlCenter::theDataBroker)
            {
                ControlCenter::theDataBroker->pushData(dbSimTimeId, dbSimTimePackage);
                ControlCenter::theDataBroker->stepTimer("mars_sim/simTimer", calc_ms);
            }

            avg_log_time += static_cast<double>(getTimeDiff(time));
            if(++count > avg_count_steps)
            {
                avg_log_time /= count;
                avg_step_time /= count;
                count = 0;
                dbSimDebugPackage[1].d = avg_step_time;
                dbSimDebugPackage[2].d = avg_log_time;
                avg_step_time = avg_log_time = 0.0;
            }
            pluginLocker.lockForRead();

            // It is possible for plugins to call switchPluginUpdateMode during
            // the update call and get removed from the activePlugins list there.
            // We use erased_active to notify this loop about an erasure.
            for(size_t i = 0; i < activePlugins.size();)
            {
                erased_active = false;
                time = utils::getTime();

                activePlugins[i].p_interface->update(calc_ms);

                if(!erased_active)
                {
                    time = getTimeDiff(time);
                    activePlugins[i].timer += time;
                    activePlugins[i].t_count++;
                    if(activePlugins[i].t_count > avg_count_steps)
                    {
                        activePlugins[i].timer /= activePlugins[i].t_count;
                        activePlugins[i].t_count = 0;
                        //fprintf(stderr, "debug_time: %s: %g\n",
                        //        activePlugins[i].name.c_str(),
                        //        activePlugins[i].timer);
                        getTimeMutex.lock();
                        dbSimDebugPackage[i+3].d = activePlugins[i].timer;
                        getTimeMutex.unlock();
                        activePlugins[i].timer = 0.0;
                    }
                    ++i;
                }
            }
            pluginLocker.unlock();
            if(ControlCenter::theDataBroker)
            {
                ControlCenter::theDataBroker->pushData(dbSimDebugId,
                                              dbSimDebugPackage);
            }
            if (sync_graphics)
            {
                calc_time += calc_ms;
                if (calc_time >= sync_time)
                {
                    sync_count = 0;
                    if(control->graphics)
                    {
                        this->allowDraw();
                    }
                    calc_time = 0;
                }
            }
            if(ControlCenter::theDataBroker)
            {
                ControlCenter::theDataBroker->trigger("mars_sim/postPhysicsUpdate");
            }

            if(setState)
            {
                simulationStatus = oldState;
            }

            physicsThreadUnlock();
        }

        void Simulator::StartSimulation()
        {
            fprintf(stderr, "Simulation started ....\n");
            stepping_mutex.lock();
            simulationStatus = RUNNING;
            simStartTime = dbSimTimePackage[0].d;
            simRealTime = 0;
            stepping_wc.wakeAll();
            stepping_mutex.unlock();
        }

        /**
         * \return \c true if started, \c false if stopped
         */
        bool Simulator::startStopTrigger()
        {
            //LOG_INFO("Simulator start/stop command.");
            stepping_mutex.lock();

            switch(simulationStatus)
            {

            case RUNNING:
                // Allow update process to finish -> transition from 2 -> 0 in main loop
                simulationStatus = STOPPING;
                stepping_wc.wakeAll();
                break;
            case STOPPING:
                break;
            case STOPPED:
                simStartTime = dbSimTimePackage[0].d;
                simRealTime = 0;
                simulationStatus = RUNNING;
                stepping_wc.wakeAll();
                break;
            case STEPPING:
                simStartTime = dbSimTimePackage[0].d;
                simRealTime = 0;
                simulationStatus = RUNNING;
                stepping_wc.wakeAll();
            default: // UNKNOWN
                throw std::exception();
            }

            stepping_mutex.unlock();

            return simulationStatus != STOPPED;
        }

        //consider the case where the time step is smaller than 1 ms
        void Simulator::myRealTime()
        {
            static long myTime = utils::getTime();
            static long myTime2 = utils::getTime();
            long timeDiff = getTimeDiff(myTime);
            myTime += timeDiff;
            static double avgTime = 0;
            avgTime += static_cast<double>(getTimeDiff(myTime2));
            myTime2 = utils::getTime();

            simRealTime += timeDiff;
#ifdef __linux__  //__unix__, wenn Darwin das mitmacht.
            //used to remember last time this function was called
            //and as absolute (minimum) wake-up time.
            static struct timespec ts;
            static bool tsNeedsInit = true;
            if (tsNeedsInit)
            {
                const auto& retval = clock_gettime(CLOCK_MONOTONIC, &ts);
                if (retval != 0)
                {
                    throw std::runtime_error("clock_gettime(CLOCK_MONOTONIC, ...) failed");
                }
                tsNeedsInit = false;
            }

            //schedule minimum sleep time
            using ms_dur = std::chrono::duration<double, std::milli>;
            const auto calc_ns_dur = std::chrono::duration_cast<std::chrono::nanoseconds>(ms_dur{calc_ms});
            const auto calc_ns = static_cast<long>(calc_ns_dur.count());
            ts.tv_nsec += calc_ns;

            //the nsec value may not exceed max_ns_dur (one second)
            constexpr auto second_nsec_dur = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds{1});
            constexpr auto second_nsec = static_cast<long>(second_nsec_dur.count());

            // TODO: Profile alternative
            // if (ts.tv_nsec > second_nsec)
            // {
            //     ts.tv_sec += ts.tv_nsec / second_nsec; // integer division is intended!
            //     ts.tv_nsec %= second_nsec;
            // }
            while (ts.tv_nsec > second_nsec)
            {
                ts.tv_nsec -= second_nsec;
                ts.tv_sec += 1;
            }

            //sleep...
            if(my_real_time)
            {
                const auto& retval = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, 0);
                if (retval != 0)
                {
                    std::cerr << "WARNING: Your system is too slow!" << std::endl;
                }
            }
            //remember time
            const auto& retval = clock_gettime(CLOCK_MONOTONIC, &ts);
            if (retval != 0)
            {
                throw std::runtime_error("clock_gettime(CLOCK_MONOTONIC, ...) failed");
            }
#else
            if(my_real_time)
            {
                if(dbSimTimePackage[0].d-simStartTime > simRealTime)
                {
                    long valSleep = (dbSimTimePackage[0].d-simStartTime)-simRealTime;//calc_ms - timeDiff;
                    msleep(valSleep);
                }
                else
                {
                    msleep(1);
                }
            }
#endif
            //timeDiff = getTimeDiff(myTime);
            //avgTime += timeDiff;
            //myTime = utils::getTime();
            if(count+1 > avg_count_steps)
            {
                avgTime /= count+1;
                dbSimDebugPackage[0].d = avgTime;
                avgTime = 0;
            }
        }


        bool Simulator::isSimRunning() const
        {
            return (simulationStatus != STOPPED);
        }

        bool Simulator::sceneChanged() const
        {
            return b_SceneChanged;
        }

        std::string Simulator::getTmpPath() const
        {
#ifdef __linux__
            std::stringstream str;
            str << "/tmp/mars2/" << static_cast<int>(getpid()) << "/";
            return str.str();
#else
            return configPath.sValue + std::string("/tmp/");
#endif

        }

        void Simulator::sceneHasChanged(bool reseted)
        {
            b_SceneChanged = !reseted;
        }

        int Simulator::loadScene(const std::string &filename, const std::string &robotname, bool threadsave, bool blocking)
        {
            return loadScene(filename, false, robotname,threadsave,blocking);
        }

        int Simulator::loadScene(const std::string &filename,
                                 bool wasrunning, const std::string &robotname, bool threadsave, bool blocking)
        {
            if(!threadsave)
            {
                return loadScene_internal(filename,wasrunning, robotname);
            }

            //Loading is handles inside the mars thread itsels later
            externalMutex.lock();
            LoadOptions lo;
            lo.filename = filename;
            lo.wasRunning = wasrunning;
            lo.robotname = robotname;
            lo.zeroPose = true;
            filesToLoad.push_back(lo);
            externalMutex.unlock();

            while(blocking && !filesToLoad.empty())
            {
                msleep(10); // maybe std::this_thread::yield(); is better here?
            }
            return 1;
        }

        int Simulator::loadScene(const std::string &filename,
                                 const std::string &robotname,
                                 utils::Vector pos,
                                 utils::Vector rot, bool threadsave, bool blocking, bool wasrunning)
        {
            if(!threadsave)
            {
                return loadScene_internal(filename, robotname, pos, rot, wasrunning);
            }

            //Loading is handles inside the mars thread itsels later
            externalMutex.lock();
            LoadOptions lo;
            lo.filename = filename;
            lo.wasRunning = wasrunning;
            lo.robotname = robotname;
            lo.zeroPose = false;
            lo.pos = pos;
            lo.rot = rot;
            filesToLoad.push_back(lo);
            externalMutex.unlock();

            while(blocking && !filesToLoad.empty())
            {
                constexpr auto loadSleepStepDuration_ms = std::chrono::milliseconds{10};
                msleep(static_cast<unsigned long>(loadSleepStepDuration_ms.count()));
            }
            return 1;

        }

        int Simulator::loadScene_internal(const std::string &filename,
                                          const std::string &robotname,
                                          utils::Vector pos, utils::Vector rot,
                                          bool wasrunning)
        {

            LOG_DEBUG("[Simulator::loadScene_internal] Loading scene internal with given position\n");

            if(ControlCenter::loadCenter->loadScene.empty())
            {
                LOG_ERROR("Simulator:: no module to load scene found");
                return 0;
            }

            try
            {
                const auto& suffix = utils::getFilenameSuffix(filename);
                LOG_DEBUG("[Simulator::loadScene] suffix: %s", suffix.c_str());
                if( ControlCenter::loadCenter->loadScene.find(suffix) !=
                    ControlCenter::loadCenter->loadScene.end() )
                {
                    const bool loading_successful = ControlCenter::loadCenter->loadScene[suffix]->loadFile(filename.c_str(), getTmpPath().c_str(), robotname.c_str(), pos, rot);
                    if (!loading_successful)
                    {
                        return 0; //failed
                    }
                }
                else
                {
                    // no scene loader found
                    LOG_ERROR("Simulator: Could not find scene loader for: %s (%s)",
                              filename.c_str(), suffix.c_str());
                    return 0; //failed
                }
            } catch(SceneParseException& e)
            {
                LOG_ERROR("Could not parse scene: %s", e.what());
            }

            if (wasrunning)
            {
                startStopTrigger();//if the simulation has been stopped for loading, now it continues
            }
            constexpr bool sceneWasReseted = false;
            sceneHasChanged(sceneWasReseted);
            return 1;
        }

        int Simulator::loadScene_internal(const std::string &filename,
                                          bool wasrunning, const std::string &robotname)
        {

            LOG_DEBUG("Loading scene internal\n");

            if(ControlCenter::loadCenter->loadScene.empty())
            {
                LOG_ERROR("Simulator:: no module to load scene found");
                return 0;
            }

            try
            {
                const auto& suffix = utils::getFilenameSuffix(filename);
                if( ControlCenter::loadCenter->loadScene.find(suffix) !=
                    ControlCenter::loadCenter->loadScene.end() )
                {
                    const bool loading_successful = ControlCenter::loadCenter->loadScene[suffix]->loadFile(filename.c_str(), getTmpPath().c_str(), robotname.c_str());
                    if (!loading_successful)
                    {
                        return 0; //failed
                    }
                }
                else
                {
                    // no scene loader found
                    LOG_ERROR("Simulator: Could not find scene loader for: %s (%s)",
                              filename.c_str(), suffix.c_str());
                    return 0; //failed
                }
            }
            catch(SceneParseException& e)
            {
                LOG_ERROR("Could not parse scene: %s", e.what());
            }

            if (wasrunning)
            {
                startStopTrigger(); //if the simulation has been stopped for loading, now it continues
            }
            constexpr bool sceneWasReseted = false;
            sceneHasChanged(sceneWasReseted);
            return 1;
        }

        int Simulator::saveScene(const std::string &filename, bool wasrunning)
        {
            const auto& suffix = utils::getFilenameSuffix(filename);
            if (ControlCenter::loadCenter->loadScene[suffix]->saveFile(filename, getTmpPath())!=1)
            {
                LOG_ERROR("Simulator: an error somewhere while saving scene");
                return 0;
            }
            if (wasrunning)
            {
                startStopTrigger(); // resuming the simulation
            }
            constexpr bool sceneWasReseted = true;
            sceneHasChanged(sceneWasReseted);
            return 1;
        }

        void Simulator::addLight(LightData light)
        {
            // constexpr bool sceneWasReseted = false;
            // sceneHasChanged(sceneWasReseted);
            // if (control->graphics && control->controllers->isLoadingAllowed()) {
            //     unsigned long id = control->nodes->getID(light.node);
            //     if(id) {
            //         light.drawID = control->nodes->getDrawID(id);
            //     }
            //     control->graphics->addLight(light);
            // }
        }


        void Simulator::finishedDraw(void)
        {
            processRequests();

            if (reloadSim)
            {
                while (simulationStatus != STOPPED)
                {

                    if(simulationStatus == RUNNING)
                    {
                        StopSimulation();
                    }

                    constexpr auto simStoppingSleepDuration_ms = std::chrono::milliseconds{10};
                    msleep(simStoppingSleepDuration_ms.count());
                }
                reloadSim = false;
                //control->controllers->setLoadingAllowed(false);

                reloadWorld();

                //control->controllers->resetControllerData();
                //control->entities->resetPose();
                for (size_t i = 0; i < allPlugins.size(); i++)
                {
                    allPlugins[i].p_interface->reset();
                }
                //control->controllers->setLoadingAllowed(true);
                if (was_running)
                {
                    StartSimulation();
                }
                reloadGraphics = true;
            }
            allow_draw = 0;
            sync_count = 1;

            // Add plugins that have been added via Simulator::addPlugin
            if(haveNewPlugin)
            {
                pluginLocker.lockForWrite();
                for (size_t i = 0; i < newPlugins.size(); i++)
                {
                    if (pluginIsLoaded(newPlugins[i]))
                    {
                        const auto msg = std::string{"Plugin \""} + newPlugins[i].name + "\" is already loaded.";
                        LOG_WARN(msg.c_str()); // TODO: Make log_warn work.
                        std::cout << msg << std::endl;
                        continue;
                    }
                    allPlugins.push_back(newPlugins[i]);
                    activePlugins.push_back(newPlugins[i]);
                    getTimeMutex.lock();
                    dbSimDebugPackage.add(newPlugins[i].name, 0.0);
                    getTimeMutex.unlock();
                    newPlugins[i].p_interface->init();
                }
                newPlugins.clear();
                haveNewPlugin = false;
                pluginLocker.unlock();
            }

            pluginLocker.lockForRead();
            for (size_t i = 0; i < guiPlugins.size(); i++)
            {
                guiPlugins[i].p_interface->update(0);
                // TODO: fix time debuging for gui plugins
                /*
                  time = utils::getTime();


                  if(show_time) {
                  time = getTimeDiff(time);
                  guiPlugins[i].timer_gui += time;
                  guiPlugins[i].t_count_gui++;
                  if(guiPlugins[i].t_count_gui > 20) {
                  guiPlugins[i].timer_gui /= guiPlugins[i].t_count_gui;
                  guiPlugins[i].t_count_gui = 0;
                  fprintf(stderr, "debug_time_gui: %s: %g\n",
                  guiPlugins[i].name.data(),
                  guiPlugins[i].timer_gui);
                  guiPlugins[i].timer_gui = 0.0;
                  }
                  }
                */
            }
            pluginLocker.unlock();

            if (ControlCenter::theDataBroker)
            {
                ControlCenter::theDataBroker->trigger("mars_sim/finishedDrawTrigger");
            }

            // FIX: update Graph
            if(control->graphTreeView_->crossEdges.size() > 0)
            {
                const auto& source = control->envireGraph_->getSourceVertex(control->graphTreeView_->crossEdges[0].edge);
                const auto& target = control->envireGraph_->getTargetVertex(control->graphTreeView_->crossEdges[0].edge);
                const auto& sourceId = control->envireGraph_->getFrameId(source);
                const auto& targetId = control->envireGraph_->getFrameId(target);
                const auto msg = std::string{"Loop in tree detected: "} + sourceId + " --> " + targetId +
                    ". The physics plugin cannot handle loops in the graph";
                throw std::runtime_error(msg);
            }
            // update the graph from top to bottom
            // starts with the parent and go to children
            const VertexDesc originDesc = control->envireGraph_->vertex(SIM_CENTER_FRAME_NAME);

            physicsThreadLock();
            Simulator::updateChildPositions(originDesc, base::TransformWithCovariance::Identity(),
                                            control->envireGraph_, control->graphTreeView_);
            physicsThreadUnlock();
        }

        // TODO: replace typdef VertexDesc
        void Simulator::updateChildPositions(const envire::core::GraphTraits::vertex_descriptor vertex,
                                             const base::TransformWithCovariance& rootToFrame,
                                             std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                             std::shared_ptr<envire::core::TreeView> &graphTreeView)
        {
            if(graphTreeView->tree.find(vertex) != graphTreeView->tree.end())
            {
                const auto& children = graphTreeView->tree[vertex].children;
                for(const auto& child : children)
                {
                    Simulator::updatePositions(vertex, child, rootToFrame, envireGraph, graphTreeView);
                }
            }
        }

        void Simulator::updateChildPositions(const envire::core::GraphTraits::vertex_descriptor vertex,
                                             std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                             std::shared_ptr<envire::core::TreeView> &graphTreeView)
        {
            const auto& absolutePose = envireGraph->getItem<envire::core::Item<AbsolutePose>>(vertex, 0)->getData();
            const auto rootToFrame = base::TransformWithCovariance{static_cast<base::Position>(absolutePose.getPosition()), static_cast<base::Quaterniond>(absolutePose.getRotation())};
            Simulator::updateChildPositions(vertex, rootToFrame, envireGraph, graphTreeView);
        }

        // TODO: replace typdef, remove if containsItem condition
        void Simulator::updatePositions(const envire::core::GraphTraits::vertex_descriptor origin,
                                        const envire::core::GraphTraits::vertex_descriptor target,
                                        const base::TransformWithCovariance& rootToOrigin,
                                        std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                        std::shared_ptr<envire::core::TreeView> &graphTreeView)
        {
            auto tf = envireGraph->getTransform(origin, target);
            // TODO: instead of searching in the subworlds we should check for items of the frame that are of type dynamicobject

            if (envireGraph->containsItems<envire::core::Item<DynamicObjectItem>>(target))
            {
                // CAUTION: we assume that there is only one DynamicObjectItem in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple DynamicObjectItem for some reason
                const auto& it = envireGraph->getItem<envire::core::Item<DynamicObjectItem>>(target);
                const auto& object = it->getData().dynamicObject;

                // TODO: check if targetFrame is dynamic?
                base::TransformWithCovariance absolutTransform;
                object->getPosition(&absolutTransform.translation);
                object->getRotation(&absolutTransform.orientation);

                // calculate the relative pose of the frame
                tf.setTransform(rootToOrigin.inverse() * absolutTransform);
                tf.time = base::Time::now();
                envireGraph->updateTransform(origin, target, tf);
            }

            // set the absolute pose values
            // after all transformation of parent frames were updated
            if (envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(target))
            {
                // calculate the absolute pose of the frame
                tf = rootToOrigin * tf.transform;

                // CAUTION: we assume that there is only one AbsolutePose in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                const auto& it = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(target);
                it->getData().setPosition(tf.transform.translation);
                it->getData().setRotation(tf.transform.orientation);
            }

            Simulator::updateChildPositions(target, tf.transform, envireGraph, graphTreeView);
        }

        void Simulator::applyChildPositions(const envire::core::GraphTraits::vertex_descriptor vertex,
                                            const base::TransformWithCovariance& rootToFrame,
                                            std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                            std::shared_ptr<envire::core::TreeView> &graphTreeView)
        {
            if(graphTreeView->tree.find(vertex) != graphTreeView->tree.end())
            {
                const auto& children = graphTreeView->tree[vertex].children;
                for(const auto& child : children)
                {
                    Simulator::applyPositions(vertex, child, rootToFrame, envireGraph, graphTreeView);
                }
            }
        }

        void Simulator::applyChildPositions(const envire::core::GraphTraits::vertex_descriptor vertex,
                                            std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                            std::shared_ptr<envire::core::TreeView> &graphTreeView)
        {
            const auto& absolutePose = envireGraph->getItem<envire::core::Item<AbsolutePose>>(vertex, 0)->getData();
            const auto rootToFrame = base::TransformWithCovariance{static_cast<base::Position>(absolutePose.getPosition()), static_cast<base::Quaterniond>(absolutePose.getRotation())};
            Simulator::applyChildPositions(vertex, rootToFrame, envireGraph, graphTreeView);
        }

        // TODO: replace typdef, remove if containsItem condition
        void Simulator::applyPositions(const envire::core::GraphTraits::vertex_descriptor origin,
                                       const envire::core::GraphTraits::vertex_descriptor target,
                                       const base::TransformWithCovariance& rootToOrigin,
                                       std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                       std::shared_ptr<envire::core::TreeView> &graphTreeView)
        {
            const auto& localTransform = envireGraph->getTransform(origin, target);
            const auto globalTransform = envire::core::Transform{rootToOrigin} * localTransform;

            if (envireGraph->containsItems<envire::core::Item<DynamicObjectItem>>(target))
            {
                // CAUTION: we assume that there is only one DynamicObjectItem in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple DynamicObjectItem for some reason
                auto& object = envireGraph->getItem<envire::core::Item<DynamicObjectItem>>(target)->getData().dynamicObject;
                object->setPosition(globalTransform.transform.translation);
                object->setRotation(globalTransform.transform.orientation);
            }

            if (envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(target))
            {
                // CAUTION: we assume that there is only one AbsolutePose in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                auto& absolutePose = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(target)->getData();
                absolutePose.setPosition(globalTransform.transform.translation);
                absolutePose.setRotation(globalTransform.transform.orientation);
            }

            Simulator::applyChildPositions(target, globalTransform.transform, envireGraph, graphTreeView);
        }

        void Simulator::rotateRevolute( envire::core::FrameId origin,
                                        double angle,
                                        std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                        std::shared_ptr<envire::core::TreeView> &graphTreeView)
        {
            // TODO: Consider min and max position!
            rotateRevolute(envireGraph->getVertex(origin), angle, envireGraph, graphTreeView);
        }

        // TODO: currently we add the angle to the actual rotation
        // we should have an option to set an absolute rotation
        void Simulator::rotateRevolute( const envire::core::GraphTraits::vertex_descriptor origin,
                                        double angle,
                                        std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                        std::shared_ptr<envire::core::TreeView> &graphTreeView)
        {
            // first get the joint
            if (envireGraph->containsItems<envire::core::Item<envire::types::joints::Revolute>>(origin))
            {
                // CAUTION: we assume that there is only one DynamicObjectItem in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple DynamicObjectItem for some reason
                const auto& it = envireGraph->getItem<envire::core::Item<envire::types::joints::Revolute>>(origin);
                const auto& joint = it->getData();
                // get the joint axis
                //   - get the joint anchor (due to relative positioning and joints have their own frame
                //     the anchor is always (0,0,0)
                // the axis should be defined in the joint frame
                const auto& q = utils::angleAxisToQuaternion(angle, joint.getAxis());

                // get children
                const auto& children = graphTreeView->tree[origin].children;
                for(const auto& child : children)
                {
                    // apply rotation
                    auto tf = envireGraph->getTransform(origin, child);
                    tf.transform.translation = q*tf.transform.translation;
                    tf.transform.orientation = q*tf.transform.orientation;
                    envireGraph->updateTransform(origin, child, tf);
                }
                // Propagate change of childrens transforms
                const auto& absolutePose = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(origin)->getData();
                applyChildPositions(origin, base::TransformWithCovariance{static_cast<base::Position>(absolutePose.getPosition()), static_cast<base::Quaterniond>(absolutePose.getRotation())}, envireGraph, graphTreeView);
            }
        }

        void Simulator::rotateContinuous( envire::core::FrameId origin,
                                        double angle,
                                        std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                        std::shared_ptr<envire::core::TreeView> &graphTreeView)
        {
            rotateContinuous(envireGraph->getVertex(origin), angle, envireGraph, graphTreeView);
        }

        // TODO: currently we add the angle to the actual rotation
        // we should have an option to set an absolute rotation
        void Simulator::rotateContinuous( const envire::core::GraphTraits::vertex_descriptor origin,
                                        double angle,
                                        std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                        std::shared_ptr<envire::core::TreeView> &graphTreeView)
        {
            // first get the joint
            if (envireGraph->containsItems<envire::core::Item<envire::types::joints::Continuous>>(origin))
            {
                // CAUTION: we assume that there is only one DynamicObjectItem in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple DynamicObjectItem for some reason
                const auto& it = envireGraph->getItem<envire::core::Item<envire::types::joints::Continuous>>(origin);
                const auto& joint = it->getData();
                // get the joint axis
                //   - get the joint anchor (due to relative positioning and joints have their own frame
                //     the anchor is always (0,0,0)
                // the axis should be defined in the joint frame
                const auto& q = utils::angleAxisToQuaternion(angle, joint.getAxis());

                // get children
                const auto& children = graphTreeView->tree[origin].children;
                for(const auto& child : children)
                {
                    // apply rotation
                    auto tf = envireGraph->getTransform(origin, child);
                    tf.transform.translation = q*tf.transform.translation;
                    tf.transform.orientation = q*tf.transform.orientation;
                    envireGraph->updateTransform(origin, child, tf);
                }
                // Propagate change of childrens transforms
                const auto& absolutePose = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(origin)->getData();
                applyChildPositions(origin, base::TransformWithCovariance{static_cast<base::Position>(absolutePose.getPosition()), static_cast<base::Quaterniond>(absolutePose.getRotation())}, envireGraph, graphTreeView);
            }
        }

        void Simulator::applyDynamicRevoluteState(envire::core::FrameId origin,
                                                  double angularVelocity, Vector linearVelocity,
                                                  std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                                  std::shared_ptr<envire::core::TreeView> &graphTreeView)
        {
            envire::core::GraphTraits::vertex_descriptor node = envireGraph->getVertex(origin);
            // check if we realy have a revolute joint at the given frame
            if (envireGraph->containsItems<envire::core::Item<envire::types::joints::Revolute>>(node))
            {
                const auto& it = envireGraph->getItem<envire::core::Item<envire::types::joints::Revolute>>(node);
                const auto& joint = it->getData();
                // get the current axis of the joint
                const auto& absolutePose = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(node)->getData();
                Vector axis = absolutePose.getRotation()*joint.getAxis();
                Quaternion q = angleAxisToQuaternion(angularVelocity, axis.normalized());
                Vector pivot = absolutePose.getPosition();
                Simulator::applyDynamicState(node, pivot, q, linearVelocity,
                                             envireGraph, graphTreeView);
            }
        }

        // This methods adds a linear velocity and rotation velocity around the
        // pivot to all dynamic objects underneath origin
        void Simulator::applyDynamicState( const envire::core::GraphTraits::vertex_descriptor node,
                                           Vector pivot, Quaternion rotationVelocity, Vector linearVelocity,
                                           std::shared_ptr<envire::core::EnvireGraph> &envireGraph,
                                           std::shared_ptr<envire::core::TreeView> &graphTreeView)
        {
            // check if we have an dynamic object item
            using DynamicObjectEnvireItem = envire::core::Item<DynamicObjectItem>;
            if (envireGraph->containsItems<DynamicObjectEnvireItem>(node))
            {
                const auto& dynamicObject = envireGraph->getItem<DynamicObjectEnvireItem>(node)->getData().dynamicObject;
                // apply the state
                // positions of dynamic objects are stored in world coordinates
                Vector pos;
                dynamicObject->getPosition(&pos);
                Vector newPos = rotationVelocity*(pos-pivot) + pivot;
                Vector lVelocity;
                dynamicObject->getLinearVelocity(&lVelocity);
                lVelocity += linearVelocity + (newPos - pos);
                dynamicObject->setLinearVelocity(lVelocity);
                Vector aVelocity;
                dynamicObject->getAngularVelocity(&aVelocity);
                double v = aVelocity.norm();
                aVelocity.normalize();
                Quaternion q = angleAxisToQuaternion(v, aVelocity);
                // is the order correct, first q then the rotation the object already has?
                q = rotationVelocity*q;
                Eigen::AngleAxis<double> angleAxis(q);
                Vector aVel = angleAxis.axis().normalized()*angleAxis.angle();
                dynamicObject->setAngularVelocity(aVel);
            }

            // apply the the change to children
            if(graphTreeView->tree.find(node) != graphTreeView->tree.end())
            {
                const auto& children = graphTreeView->tree[node].children;
                for(const auto& child : children)
                {
                    Simulator::applyDynamicState(child, pivot,
                                                 rotationVelocity, linearVelocity,
                                                 envireGraph, graphTreeView);
                }
            }
        }

        void Simulator::newWorld(bool clear_all)
        {
            physicsThreadLock();
            // reset simTime
            realStartTime = utils::getTime();
            dbSimTimePackage[0].set(0.);

            control->sensors->clearAllSensors(clear_all);
            control->motors->clearAllMotors(clear_all);
            control->joints->clearAllJoints(clear_all);
            // control->nodes->clearAllNodes(clear_all, reloadGraphics);

            if(control->graphics)
            {
                control->graphics->clearDrawItems();
                if(reloadGraphics)
                {
                    control->graphics->reset();
                }
            }

            // constexpr bool sceneWasReseted = true;
            // sceneHasChanged(sceneWasReseted);
            for(auto &it: subWorlds)
            {
                it.second->control->physics->freeTheWorld();
                it.second->control->physics->initTheWorld();
            }
            physicsThreadUnlock();
        }

        void Simulator::resetSim(bool resetGraphics)
        {
            reloadSim = true;
            reloadGraphics = resetGraphics;
            stepping_mutex.lock();
            if(simulationStatus == RUNNING)
            {
                was_running = true;
            }
            else
            {
                was_running = false;
            }

            if(simulationStatus != STOPPED)
            {
                simulationStatus = STOPPING;
            }
            if(control->graphics)
            {
                this->allowDraw();
            }

            stepping_mutex.unlock();
        }


        void Simulator::reloadWorld(void)
        {
            realStartTime = utils::getTime();
            dbSimTimePackage[0].set(0.);

            // @clear_all: true would also clear envire-items, but these will be used for the later reload.
            constexpr bool clear_all = false;
            interfaces::ControlCenter::joints->clearAllJoints(clear_all);
            interfaces::ControlCenter::motors->clearAllMotors(clear_all);
            interfaces::ControlCenter::sensors->clearAllSensors(clear_all);

            collisionManager->clear();
            if (control->graphics)
            {
                control->graphics->clearDrawItems();
                if (reloadGraphics)
                {
                    control->graphics->reset();
                }
            }
            resetPoses();
            for(auto &it: subWorlds)
            {
                it.second->control->physics->freeTheWorld();
                it.second->control->physics->initTheWorld();
            }
            collisionManager->reset();

            reloadObjects();
            interfaces::ControlCenter::joints->reloadJoints();
            interfaces::ControlCenter::motors->reloadMotors();
            interfaces::ControlCenter::sensors->reloadSensors();
        }

        void Simulator::readArguments(int argc, char **argv)
        {
            int c;
            int option_index = 0;

            static struct option long_options[] = {
                {"help",no_argument,0,'h'},
                {"run",no_argument,0,'r'},
                {"show_grid",no_argument,0,'g'},
                {"ortho",no_argument,0,'o'},
                {"no-gui",no_argument,0,'G'},
                {"scenename", 1, 0, 's'},
                {"config_dir", required_argument, 0, 'C'},
                {"c_port",1,0,'c'},
                {0, 0, 0, 0}
            };

            // pipe arguments into cfg_manager
            if(control->cfg)
            {
                std::vector<std::string> arguments;
                for(int i = 0; i < argc; ++i)
                {
                    arguments.push_back(argv[i]);
                }
                char label[55];
                for(size_t i = 0; i < arguments.size(); ++i)
                {
                    size_t f = arguments[i].find("=");
                    if(f != std::string::npos)
                    {
                        control->cfg->getOrCreateProperty("Config", arguments[i].substr(0, f),
                                                          arguments[i].substr(f+1));
                    }
                    else
                    {
                        sprintf(label, "arg%zu", i);
                        control->cfg->getOrCreateProperty("Config", label, arguments[i]);
                    }
                }
            }

            while (1)
            {
                c = getopt_long(argc, argv, "hrgoGs:C:p:", long_options, &option_index);
                if (c == -1)
                    break;
                switch (c)
                {
                case 's':
                {
                    std::vector<std::string> tmp_v_s;
                    tmp_v_s = explodeString(';', optarg);
                    for(size_t i = 0; i < tmp_v_s.size(); ++i)
                    {
                        if(pathExists(tmp_v_s[i]))
                        {
                            arg_v_scene_name.push_back(tmp_v_s[i]);
                        }
                        else
                        {
                            LOG_ERROR("The given scene file does not exists: %s\n",
                                      tmp_v_s[i].c_str());
                        }
                    }
                }
                break;
                case 'C':
                    if(pathExists(optarg)) config_dir = optarg;
                    else printf("The given configuration Directory does not exists: %s\n", optarg);
                    break;
                case 'r':
                    arg_run = 1;
                    break;
                case 'c':
                    std_port = atoi(optarg);
                    break;
                case 'g':
                    arg_grid = 1;
                    break;
                case 'o':
                    arg_ortho = 1;
                    break;
                case 'G':
                    break;
                case 'h':
                default:
                    printf("\naccepted parameters are:\n");
                    printf("=======================================\n");
                    printf("-h             this screen:\n");
                    printf("-s <filename>  filename for scene to load\n");
                    printf("-r             start directly the simulation\n");
                    printf("-c             set standard controller port\n");
                    printf("-C             path to Configuration\n");
                    printf("-g             show 3d grid\n");
                    printf("-o             ortho perspective as standard\n");
                    printf("\n");
                }
            }

            return;
        }

        void Simulator::physicsThreadLock(void)
        {
            // physics_mutex_count is used to see how many threads are trying to
            // acquire the lock. Also see Simulator::run() on how this is used.
            physicsCountMutex.lock();
            physics_mutex_count++;
            physicsCountMutex.unlock();
            physicsMutex.lock();
        }

        void Simulator::physicsThreadUnlock(void)
        {
            // physics_mutex_count is used to see how many threads are trying to
            // acquire the lock. Also see Simulator::run() on how this is used.
            physicsCountMutex.lock();
            physics_mutex_count--;
            physicsCountMutex.unlock();
            physicsMutex.unlock();
        }

        std::shared_ptr<PhysicsInterface> Simulator::getPhysics(void) const
        {
            throw std::logic_error("Simulator::getPhysics is obsolete - now each subworld has its own physics simulation.");
        }

        void Simulator::preGraphicsUpdate(void)
        {
            contactLinesDataMutex.lock();
            contactLines->setData(contactLinesData);
            contactLinesDataMutex.unlock();
        }

        void Simulator::postGraphicsUpdate(void)
        {
            //finishedDraw();
        }

        void Simulator::exitMars(void)
        {
            stepping_mutex.lock();
            kill_sim = 1;
            stepping_wc.wakeAll();
            stepping_mutex.unlock();
            if(isCurrentThread())
            {
                return;
            }
            while(this->isRunning())
            {
                msleep(1);
            }
        }

        void Simulator::connectNodes(unsigned long id1, unsigned long id2)
        {
            // JointData connect_joint;
            // connect_joint.nodeIndex1 = id1;
            // connect_joint.nodeIndex2 = id2;
            // connect_joint.type = JOINT_TYPE_FIXED;
            // control->joints->addJoint(&connect_joint, true);
        }

        void Simulator::disconnectNodes(unsigned long id1, unsigned long id2)
        {
            //control->joints->removeJointByIDs(id1, id2);
        }

        void Simulator::rescaleEnvironment(sReal x, sReal y, sReal z)
        {
            // rescale all nodes
            // reset the anchor's positions
            //control->nodes->scaleReloadNodes(x, y, z);
            //control->joints->scaleReloadJoints(x, y ,z);
            resetSim();
        }


        void Simulator::singleStep(void)
        {
            stepping_mutex.lock();
            simulationStatus = STEPPING;
            stepping_wc.wakeAll();
            stepping_mutex.unlock();
        }

        void Simulator::switchPluginUpdateMode(int mode, PluginInterface *pl)
        {
            bool afound = false;
            bool gfound = false;
            bool bfound = false;
            data_broker::DataPackage tmpPackage;

            size_t i=0;
            for(auto p_iter=activePlugins.begin(); p_iter!=activePlugins.end();
                p_iter++, ++i)
            {
                if((*p_iter).p_interface == pl)
                {
                    afound = true;
                    if(!(mode & PLUGIN_SIM_MODE))
                    {
                        activePlugins.erase(p_iter);
                        erased_active = true;
                        bfound = true;
                    }
                    break;
                }
            }
            if(bfound)
            {
                size_t offset = 3;
                tmpPackage.add(dbSimDebugPackage[0]);
                tmpPackage.add(dbSimDebugPackage[1]);
                tmpPackage.add(dbSimDebugPackage[2]);
                for(size_t k = 0; k < activePlugins.size(); ++k)
                {
                    if(i==k)
                    {
                        offset = 4;
                    }
                    tmpPackage.add(dbSimDebugPackage[k+offset]);
                }
                getTimeMutex.lock();
                dbSimDebugPackage = tmpPackage;
                getTimeMutex.unlock();
            }
            for(auto p_iter=guiPlugins.begin(); p_iter!=guiPlugins.end();
                p_iter++)
            {
                if((*p_iter).p_interface == pl)
                {
                    gfound = true;
                    if(!(mode & PLUGIN_GUI_MODE))
                        guiPlugins.erase(p_iter);
                    break;
                }
            }

            for(auto p_iter=allPlugins.begin(); p_iter!=allPlugins.end();
                p_iter++)
            {
                if((*p_iter).p_interface == pl)
                {
                    if(mode & PLUGIN_SIM_MODE && !afound)
                    {
                        activePlugins.push_back(*p_iter);
                        getTimeMutex.lock();
                        dbSimDebugPackage.add(p_iter->name, 0.0);
                        getTimeMutex.unlock();
                    }
                    if(mode & PLUGIN_GUI_MODE && !gfound)
                        guiPlugins.push_back(*p_iter);
                    break;
                }
            }
        }

        /**
         * If \c true you cannot recover (currently) from this point without restarting the simulation.
         * To extend this, restart the simulator thread and reset the scene (untested).
         *
         * \return \c true if the simulation thread was not interrupted by ODE
         */
        bool Simulator::hasSimFault() const
        {
            return sim_fault;
        }

        void Simulator::handleError(PhysicsError error)
        {
            switch(error)
            {
            case PHYSICS_NO_ERROR:
            case PHYSICS_DEBUG:
            case PHYSICS_ERROR:
                break;
            case PHYSICS_UNKNOWN:
                LOG_WARN("looks like we caught a unknown exception from ODE.");
                break;
            }

            for(auto p_iter=allPlugins.begin(); p_iter!=allPlugins.end();
                p_iter++)
            {
                (*p_iter).p_interface->handleError();
            }

            //control->controllers->handleError();

            string onError;
            control->cfg->getPropertyValue("Simulator", "onPhysicsError",
                                           "value", &onError);
            std::transform(onError.begin(), onError.end(),
                           onError.begin(), ::tolower);
            if("abort" == onError || "" == onError)
            {
                abort();
            } else if("reset" == onError)
            {
                resetSim();
            } else if("warn" == onError)
            {
                // warning already happend in message handler
            } else if("shutdown" == onError)
            {
                //Killing the simulation thread, means the simulation gui still runs but the simulation thread get's stopped
                //In this state the the sim_fault is set to true which can be checked externally to react to this fault
                sim_fault = true;
                kill_sim = true;
            } else
            {
                LOG_WARN("unsupported config value for \"Simulator/onPhysicsError\": \"%s\"", onError.c_str());
                LOG_WARN("aborting by default...");
                abort();
            }
        }

        void Simulator::setGravity(const Vector &gravity)
        {
            if(control->cfg)
            {
                control->cfg->setPropertyValue("Simulator", "Gravity x", "value",
                                               gravity.x());
                control->cfg->setPropertyValue("Simulator", "Gravity y", "value",
                                               gravity.y());
                control->cfg->setPropertyValue("Simulator", "Gravity z", "value",
                                               gravity.z());
            }
        }


        void Simulator::noGUITimerUpdate(void)
        {
            finishedDraw();
        }


        ControlCenter* Simulator::getControlCenter(void) const
        {
            return control.get();
        }

        void Simulator::addPlugin(const pluginStruct& plugin)
        {
            pluginLocker.lockForWrite();
            newPlugins.push_back(plugin);
            haveNewPlugin = true;
            pluginLocker.unlock();
        }

        void Simulator::removePlugin(PluginInterface *pl)
        {
            pluginLocker.lockForWrite();

            size_t i=0;
            for(auto p_iter=activePlugins.begin(); p_iter!=activePlugins.end();
                p_iter++, ++i)
            {
                if((*p_iter).p_interface == pl)
                {
                    activePlugins.erase(p_iter);
                    data_broker::DataPackage tmpPackage;
                    size_t offset = 3;
                    tmpPackage.add(dbSimDebugPackage[0]);
                    tmpPackage.add(dbSimDebugPackage[1]);
                    tmpPackage.add(dbSimDebugPackage[2]);
                    for(size_t k=0; k<activePlugins.size(); ++k)
                    {
                        if(i==k)
                        {
                            offset = 4;
                        }
                        tmpPackage.add(dbSimDebugPackage[k+offset]);
                    }
                    getTimeMutex.lock();
                    dbSimDebugPackage = tmpPackage;
                    getTimeMutex.unlock();
                    break;
                }
            }

            for(auto p_iter=guiPlugins.begin(); p_iter!=guiPlugins.end();
                p_iter++)
            {
                if((*p_iter).p_interface == pl)
                {
                    guiPlugins.erase(p_iter);
                    break;
                }
            }

            for(auto p_iter=allPlugins.begin(); p_iter!=allPlugins.end();
                p_iter++)
            {
                if((*p_iter).p_interface == pl)
                {
                    allPlugins.erase(p_iter);
                    break;
                }
            }

            pluginLocker.unlock();
        }

        int Simulator::checkCollisions(void)
        {
            // TODO: reimplement on collision space
            return 0; //physics->checkCollisions();
        }

        void Simulator::sendDataToPlugin(int plugin_index, void* data)
        {
            if(plugin_index < 0 || plugin_index+1 > (int)allPlugins.size()) return;

            allPlugins[plugin_index].p_interface->getSomeData(data);
        }


        void Simulator::setSyncThreads(bool value)
        {
            sync_graphics = value;
        }

        /**
         * Calls GraphicsManager::update() method to redraw all OSG objects in the simulation.
         */
        void Simulator::updateSim()
        {
            if(control->graphics)
                control->graphics->update();
        }

        /**
         * This method is used for gui and simulation synchronization.
         */
        void Simulator::allowDraw(void)
        {
            allow_draw = 1;
        }

        /**
         * \return \c true if no external requests are open.
         */
        bool Simulator::allConcurrencysHandled()
        {
            return filesToLoad.empty();
        }

        /** This method is used for all calls that cannot be done from an external thread.
         * This means the requests have to be caches (like in loadScene) and have to be handled by
         * this method, which is called by Simulator::run().
         */
        void Simulator::processRequests()
        {
            externalMutex.lock();
            if(filesToLoad.size() > 0)
            {
                bool wasrunning = false;
                while (simulationStatus != STOPPED)
                {

                    if(simulationStatus == RUNNING)
                    {
                        StopSimulation();
                        wasrunning = true;
                    }

                    msleep(10);
                }

                for(size_t i = 0; i < filesToLoad.size(); i++)
                {
                    if (filesToLoad[i].zeroPose == true)
                    {
                        loadScene_internal(filesToLoad[i].filename, false,
                                           filesToLoad[i].robotname);
                    } else
                    {
                        loadScene_internal(filesToLoad[i].filename,
                                           filesToLoad[i].robotname,
                                           filesToLoad[i].pos, filesToLoad[i].rot,
                                           false);
                    }
                }
                filesToLoad.clear();

                if(wasrunning)
                {
                    StartSimulation();
                }
            }
            externalMutex.unlock();
        }


        void Simulator::exportScene(void) const
        {
            if(control->graphics)
            {
                string filename = "export.obj";
                control->graphics->exportScene(filename);
                filename = "export.osg";
                control->graphics->exportScene(filename);
            }
        }

        void Simulator::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property)
        {
            if(_property.paramId == cfgCalcMs.paramId)
            {
                // TODO: Make interaction between calc_ms and stepSize more obvious.
                // @calc_ms: getStepSizeS uses this to calculate the step size.
                calc_ms = _property.dValue;

                for(auto &it: subWorlds)
                {
                    it.second->control->physics->step_size = getStepSizeS();
                    //if(control->joints) control->joints->changeStepSize();
                }
                return;
            }

            if(_property.paramId == cfgFaststep.paramId)
            {
                for(auto &it: subWorlds)
                {
                    it.second->control->physics->fast_step = _property.bValue;
                }
                return;
            }

            if(_property.paramId == cfgRealtime.paramId)
            {
                my_real_time = _property.bValue;
                return;
            }

            if(_property.paramId == cfgSyncGui.paramId)
            {
                this->setSyncThreads(_property.bValue);
                return;
            }

            if(_property.paramId == cfgSyncTime.paramId)
            {
                sync_time = _property.dValue;
                return;
            }

            if(_property.paramId == cfgDrawContact.paramId)
            {
                for(const auto &it: subWorlds)
                {
                    CPP_UNUSED(it);
                    if (!contactLines)
                    {
                        LOG_ERROR("Contact lines are not properly set up.");
                        break;
                    }

                    //it.second->control->physics->draw_contact_points = _property.bValue;
                    if(cfgDrawContact.bValue != _property.bValue)
                    {
                        contactLinesDataMutex.lock();
                        cfgDrawContact.bValue = _property.bValue;
                        if(_property.bValue)
                        {
                            control->graphics->addOSGNode(contactLines->getOSGNode());
                        }
                        else
                        {
                            control->graphics->removeOSGNode(contactLines->getOSGNode());
                        }
                        contactLinesDataMutex.unlock();
                    }
                }
                return;
            }

            if(_property.paramId == cfgGX.paramId)
            {
                gravity.x() = _property.dValue;
                for(auto &it: subWorlds)
                {
                    it.second->control->physics->world_gravity = gravity;
                }
                return;
            }

            if(_property.paramId == cfgGY.paramId)
            {
                gravity.y() = _property.dValue;
                for(auto &it: subWorlds)
                {
                    it.second->control->physics->world_gravity = gravity;
                }
                return;
            }

            if(_property.paramId == cfgGZ.paramId)
            {
                gravity.z() = _property.dValue;
                for(auto &it: subWorlds)
                {
                    it.second->control->physics->world_gravity = gravity;
                }
                return;
            }

            if(_property.paramId == cfgWorldErp.paramId)
            {
                for(auto &it: subWorlds)
                {
                    it.second->control->physics->world_erp = _property.dValue;
                }
                return;
            }

            if(_property.paramId == cfgWorldCfm.paramId)
            {
                for(auto &it: subWorlds)
                {
                    it.second->control->physics->world_cfm = _property.dValue;
                }
                return;
            }

            if(_property.paramId == cfgVisRep.paramId)
            {
                if(control->graphics)
                {
                    bool collision = _property.iValue & 2;
                    collisionSpace->showDebugObjects(collision);
                }
                return;
            }

            if(_property.paramId == cfgAvgCountSteps.paramId)
            {
                avg_count_steps = _property.iValue;
                return;
            }

        }

        void Simulator::initCfgParams(void)
        {
            if(!control->cfg)
                return;
            cfgCalcMs = control->cfg->getOrCreateProperty("Simulator", "calc_ms",
                                                          calc_ms, this);
            calc_ms = cfgCalcMs.dValue;
            cfgFaststep = control->cfg->getOrCreateProperty("Simulator", "faststep",
                                                            false, this);
            cfgRealtime = control->cfg->getOrCreateProperty("Simulator", "realtime calc",
                                                            true, this);
            my_real_time = cfgRealtime.bValue;

            cfgDebugTime = control->cfg->getOrCreateProperty("Simulator", "debug time",
                                                             false, this);

            cfgSyncGui = control->cfg->getOrCreateProperty("Simulator", "sync gui",
                                                           false, this);

            cfgSyncTime = control->cfg->getOrCreateProperty("Simulator", "sync time",
                                                            40.0, this);

            cfgDrawContact = control->cfg->getOrCreateProperty("Simulator", "draw contacts",
                                                               false, this);

            cfgGX = control->cfg->getOrCreateProperty("Simulator", "Gravity x",
                                                      0.0, this);

            cfgGY = control->cfg->getOrCreateProperty("Simulator", "Gravity y",
                                                      0.0, this);

            cfgGZ = control->cfg->getOrCreateProperty("Simulator", "Gravity z",
                                                      -9.81, this);

            cfgWorldErp = control->cfg->getOrCreateProperty("Simulator", "world erp",
                                                            0.2, this);

            cfgWorldCfm = control->cfg->getOrCreateProperty("Simulator", "world cfm",
                                                            1e-5, this);

            cfgVisRep = control->cfg->getOrCreateProperty("Simulator", "visual rep.",
                                                          (int)1, this);

            cfgUseNow = control->cfg->getOrCreateProperty("Simulator", "getTime:useNow",
                                                          (bool)false, this);

            cfgAvgCountSteps = control->cfg->getOrCreateProperty("Simulator", "avg count steps",
                                                                 avg_count_steps, this);
            avg_count_steps = cfgAvgCountSteps.iValue;
            control->cfg->getOrCreateProperty("Simulator", "onPhysicsError",
                                              "abort", this);

        }

        void Simulator::receiveData(const data_broker::DataInfo &info,
                                    const data_broker::DataPackage &package,
                                    int callbackParam)
        {
            if(info.groupName != "_MESSAGES_")
            {
                fprintf(stderr, "got unexpected data broker package: %s %s\n",
                        info.groupName.c_str(), info.dataName.c_str());
                return;
            }
            // output to stdout
            std::string message;
            package.get(0, &message);
            switch(callbackParam)
            {
            case data_broker::DB_MESSAGE_TYPE_FATAL:
#ifndef WIN32
                fprintf(stderr, "\033[31mfatal: %s\033[0m\n", message.c_str());
#else
                fprintf(stderr, "fatal: %s\n", message.c_str());
#endif
                break;
            case data_broker::DB_MESSAGE_TYPE_ERROR:
#ifndef WIN32
                fprintf(stderr, "\033[1;31merror: %s\033[0m\n", message.c_str());
#else
                fprintf(stderr, "error: %s\n", message.c_str());
#endif
                break;
            case data_broker::DB_MESSAGE_TYPE_WARNING:
                if(log_warning)
                {
#ifndef WIN32
                    fprintf(stderr, "\033[0;32mwarning: %s\033[0m\n", message.c_str());
#else
                    fprintf(stderr, "warning: %s\n", message.c_str());
#endif
                }
                break;
            case data_broker::DB_MESSAGE_TYPE_INFO:
            case data_broker::DB_MESSAGE_TYPE_DEBUG:
                if(log_debug)
                {
#ifndef WIN32
                    fprintf(stderr, "\033[1;34minfo: %s\033[0m\n", message.c_str());
#else
                    fprintf(stderr, "info: %s\n", message.c_str());
#endif
                }
                break;
            default:
                fprintf(stderr, "???: %s\n", message.c_str());
                break;
            }
        }

        const utils::Vector& Simulator::getGravity(void)
        {
            return gravity;
        }

        unsigned long Simulator::getTime()
        {
            unsigned long returnTime = 0;
            getTimeMutex.lock();
            if(cfgUseNow.bValue)
            {
                returnTime = utils::getTime();
            }
            else
            {
                returnTime = realStartTime+dbSimTimePackage[0].d;
            }
            getTimeMutex.unlock();
            return returnTime;
        }

        /*

          TODO:
          Howto manage ode objects and sync them with envire representation:
          1. handle/manage map by ids (frameId, etc.)
          2. store ode objects in envire graph directly
          3. clone graph for physics representation

          General representation of data in envire graph?
          1. store visual, collision, inertial items seperately
          2. store geometry items that have annotations like physical properties, visual properties, etc.
          - Only store information vs. store objects with functionallity

         */

        // envire event callbacks
        // TODO: we might not add a physics frame for every envire frame (could call this function if inertial or joint is added)
        // void Simulator::frameAdded(const envire::core::FrameAddedEvent& e)
        // {
        // }

        std::shared_ptr<envire::core::EnvireGraph> Simulator::getGraph()
        {
            return control->envireGraph_;
        }


        std::string Simulator::getRootFrame()
        {
            return std::string{SIM_CENTER_FRAME_NAME};
        }

        std::shared_ptr<interfaces::SubControlCenter> Simulator::getSubControl(envire::core::FrameId frame)
        {
            // search for physics interface in graph
            bool done = false;
            while(!done)
            {
                const auto& vertex = control->envireGraph_->vertex(frame);
                const auto& parentVertex = control->graphTreeView_->tree[vertex].parent;
                // TODO: check if this check is correct
                if(parentVertex)
                {
                    frame = control->envireGraph_->getFrameId(parentVertex);
                    try
                    {
                        using SubControlItem = envire::core::Item<std::shared_ptr<interfaces::SubControlCenter>>;
                        const auto& it = control->envireGraph_->getItem<SubControlItem>(frame);
                        return it->getData();
                    }
                    catch (...)
                    {
                    }
                }
                else
                {
                    done = true;
                }
            }
            return nullptr;
        }

        void Simulator::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::World>>& e)
        {
            const auto& world = e.item->getData();

            auto* subWorld = new SubWorld{};
            // use the control with new physic
            subWorld->control = std::make_shared<SubControlCenter>();
            subWorld->control->control = control.get();
            subWorld->control->setPrefix(world.getPrefix());
            subWorld->control->setFrameId(e.frame);

            subWorld->control->physics = physicsLoader->createWorldInstance();
            subWorld->control->physics->initTheWorld();
            {
                // the physics step_size is in seconds
                using ms_dur = std::chrono::duration<double, std::milli>;
                using s_dur = std::chrono::duration<double>;
                const auto calc_s_dur = std::chrono::duration_cast<s_dur>(ms_dur{calc_ms});
                const auto calc_s = calc_s_dur.count();
                subWorld->control->physics->step_size = calc_s;
            }
            subWorld->control->physics->fast_step = cfgFaststep.bValue;
            subWorld->control->physics->world_erp = cfgWorldErp.dValue;
            subWorld->control->physics->world_cfm = cfgWorldCfm.dValue;
            subWorld->control->physics->world_gravity = gravity;
            subWorld->control->collision = collisionSpaceLoader->createCollisionSpace(control.get());
            subWorld->control->collision->initSpace();

            //world->control->physics->draw_contact_points = cfgDrawContact.bValue;
            subWorlds[subWorld->control->getPrefix()] = std::unique_ptr<SubWorld>(subWorld);
            subWorld->start();

            // store the control center of the subworld in its own frame in the graph
            using SubControlItem = envire::core::Item<std::shared_ptr<interfaces::SubControlCenter>>;
            auto subWorldItemPtr = SubControlItem::Ptr{new SubControlItem{subWorld->control}};

            control->envireGraph_->addItemToFrame(e.frame, subWorldItemPtr);

            // add collision item into graph
            CollisionInterfaceItem collisionItem;
            collisionItem.collisionInterface = subWorld->control->collision;
            collisionItem.pluginName = "mars_ode_collision";
            auto collisionItemPtr = envire::core::Item<interfaces::CollisionInterfaceItem>::Ptr{new envire::core::Item<interfaces::CollisionInterfaceItem>{collisionItem}};
            control->envireGraph_->addItemToFrame(e.frame, collisionItemPtr);
        }

        void Simulator::saveGraph(const std::string &fileName)
        {
            envire::core::GraphDrawing::write(*(control->envireGraph_.get()), fileName);
        }

        void Simulator::resetPoses()
        {
            auto* const c = control.get();
            auto resetPoseFunctor = [c](VertexDesc node, VertexDesc parent)
            {
                // TODO: This has implicit assumptions which are not enforced nor documented!
                //  * Root frame is assumed to not have an AbsolutePose.
                //  * Root frame is located at (0,0,0) with (1,0,0,0) rotation
                //  * Each other frame has exactly one Item of type AbsolutePose

                using AbsolutePoseEnvireItem = envire::core::Item<AbsolutePose>;
                if (c->envireGraph_->containsItems<AbsolutePoseEnvireItem>(node))
                {
                    // Calculate where node should be after potential change of parents position.
                    const auto parentAbsolutePose = c->envireGraph_->containsItems<AbsolutePoseEnvireItem>(parent) ? c->envireGraph_->getItem<AbsolutePoseEnvireItem>(parent)->getData() : AbsolutePose{};
                    const auto& transformation = c->envireGraph_->getTransform(parent, node);
                    const auto parentTransform = envire::core::Transform{parentAbsolutePose.getPosition(), parentAbsolutePose.getRotation()};
                    const auto rootToNode = parentTransform * transformation;

                    // Set where node should be after the reset of parent.
                    auto& currentAbsolutePose = c->envireGraph_->getItem<AbsolutePoseEnvireItem>(node)->getData();
                    currentAbsolutePose.setPosition(rootToNode.transform.translation);
                    currentAbsolutePose.setRotation(rootToNode.transform.orientation);

                    // Reset the position of node to the initial pose.
                    const auto transformationChange = envire::core::Transform{currentAbsolutePose.resetPose()};

                    // Adapt the transformation from parent to node to reflect the reset pose of node.
                    c->envireGraph_->setEdgeProperty(parent, node, transformation * transformationChange);

                    // Also remove potential dynamicobject from node.
                    itemRemover<interfaces::DynamicObjectItem>(c->envireGraph_.get(), node);
                }
            };

            physicsThreadLock();
            const auto& rootVertex = c->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            c->graphTreeView_->visitDfs(rootVertex, resetPoseFunctor);
            physicsThreadUnlock();

#if 0 // Tests if reset was performed correctly.
            auto absolutePoseCheckFunctor = [c](VertexDesc node, VertexDesc parent)
            {
                using AbsolutePoseEnvireItem = envire::core::Item<AbsolutePose>;
                if (c->envireGraph_->containsItems<AbsolutePoseEnvireItem>(node))
                {
                    const auto& absolutePose = c->envireGraph_->getItem<AbsolutePoseEnvireItem>(node)->getData();
                    if (!absolutePose.isInInitialPose())
                    {
                        std::cout << "Incorrect pose for frame " << control->envireGraph_->getFrameId(node) << std::endl;
                    }

                    using DynamicObjectEnvireItem = envire::core::Item<DynamicObjectItem>;
                    if (c->envireGraph_->containsItems<DynamicObjectEnvireItem>(node))
                    {
                        const auto& dynamicObject = c->envireGraph_->getItem<DynamicObjectEnvireItem>(node)->getData().dynamicObject;
                        utils::Vector position;
                        dynamicObject->getPosition(&position);
                        utils::Quaternion rotation;
                        dynamicObject->getRotation(&rotation);

                        constexpr double eps = 1e-6;
                        const bool positionMatches = position.isApprox(utils::Vector{absolutePose.getPosition()}, eps);
                        const bool rotationMatches = rotation.coeffs().isApprox(utils::Quaternion{absolutePose.getRotation()}.coeffs(), eps);
                        if (!positionMatches)
                        {
                            std::cout << "Incorrect position for dynamic object for frame " << control->envireGraph_->getFrameId(node) << std::endl;
                            std::cout << "dynamic object position: " << position << std::endl;
                            std::cout << "absolute pose position: " << absolutePose.getPosition() << std::endl;
                        }
                        if (!rotationMatches)
                        {
                            std::cout << "Incorrect rotation for dynamic object for frame " << control->envireGraph_->getFrameId(node) << std::endl;
                            std::cout << "dynamic object rotation: " << rotation.coeffs() << std::endl;
                            std::cout << "absolute pose position: " << absolutePose.getRotation().coeffs() << std::endl;
                        }
                    }
                }
            };
            c->graphTreeView_->visitDfs(rootVertex, absolutePoseCheckFunctor);
#endif
        }

        void Simulator::reloadObjects()
        {
            auto controlPtr = control.get();
            auto linkReadder = [controlPtr](VertexDesc node, VertexDesc parent)
            {
                itemReadder<envire::types::Link>(controlPtr->envireGraph_.get(), node);
                itemReadder<envire::types::Inertial>(controlPtr->envireGraph_.get(), node);
                itemReadder<envire::types::geometry::Box>(controlPtr->envireGraph_.get(), node);
                itemReadder<envire::types::geometry::Capsule>(controlPtr->envireGraph_.get(), node);
                itemReadder<envire::types::geometry::Cylinder>(controlPtr->envireGraph_.get(), node);
                itemReadder<envire::types::geometry::Mesh>(controlPtr->envireGraph_.get(), node);
                itemReadder<envire::types::geometry::Plane>(controlPtr->envireGraph_.get(), node);
                itemReadder<envire::types::geometry::Sphere>(controlPtr->envireGraph_.get(), node);
                itemReadder<envire::types::geometry::Heightfield>(controlPtr->envireGraph_.get(), node);
            };

            physicsThreadLock();
            const auto& rootVertex = control->envireGraph_->getVertex(SIM_CENTER_FRAME_NAME);
            control->graphTreeView_->visitDfs(rootVertex, linkReadder);
            physicsThreadUnlock();
        }

        bool Simulator::pluginIsLoaded(const interfaces::pluginStruct& p) const
        {
            const auto& name = p.name;
            const auto it = std::find_if(std::begin(allPlugins), std::end(allPlugins),
                [&name] (const interfaces::pluginStruct& x) { return x.name == name; });
            return it != std::end(allPlugins);
        }

        interfaces::sReal Simulator::getStepSizeS() const
        {
            static auto calc_s = [this]()
            {
                using ms_dur = std::chrono::duration<double, std::milli>;
                using s_dur = std::chrono::duration<double>;
                const auto calc_s_dur = std::chrono::duration_cast<s_dur>(ms_dur{calc_ms});
                return static_cast<sReal>(calc_s_dur.count());
            }();
            return calc_s;
        }

        interfaces::sReal Simulator::getVectorCollision(Vector position, Vector ray)
        {
            // todo: - order subworlds by bounding box ray collision
            //       - don't continue with subworlds if collision distance
            //         is already lower then bounding box collision
            double min = -1, d;
            for(auto &it: subWorlds)
            {
                d = it.second->control->collision->getVectorCollision(position, ray);
                if(min < 0 || min > d) min = d;
            }
            return d;
        }

    } // end of namespace core

    namespace interfaces
    {
        SimulatorInterface* SimulatorInterface::getInstance(lib_manager::LibManager *libManager)
        {
            return new core::Simulator{libManager};
        }
    } // end of namespace interfaces

} // end of namespace mars

DESTROY_LIB(mars::core::Simulator);
CREATE_LIB(mars::core::Simulator);
