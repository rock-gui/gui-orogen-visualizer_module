/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef VISUALIZER_MODULE_TASK_TASK_HPP
#define VISUALIZER_MODULE_TASK_TASK_HPP

#include "visualizer_module/TaskBase.hpp"

#include <envire/Core.hpp>
#include <envire/maps/Pointcloud.hpp>

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include <vizkit3d/EnvireVisualization.hpp>
#include <vizkit3d/RigidBodyStateVisualization.hpp>
#include <vizkit3d/TrajectoryVisualization.hpp>
#include <vizkit3d/MotionCommandVisualization.hpp>

namespace visualizer_module {

    /*! \class Task 
     * General visualization module which can be started and run independently from other modules.
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        envire::Environment mEnv;
        envire::Pointcloud* mpEnvirePointcloud;
        envire::FrameNode* mpFrameNodePointcloud;
        // Vizkit
        QtThreadedWidget<vizkit3d::Vizkit3DWidget> mVizkit3DWidget;
        envire::EnvireVisualization mEnvViz;
        vizkit3d::RigidBodyStateVisualization mRigidBodyViz;
        vizkit3d::TrajectoryVisualization mTrajectoryViz;
        vizkit3d::TrajectoryVisualization mTrajectorySplineViz;
        vizkit3d::MotionCommandVisualization mMotionCommandViz;

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "visualizer_module::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

