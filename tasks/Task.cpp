/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <envire/Orocos.hpp>

using namespace visualizer_module;

Task::Task(std::string const& name)
    : TaskBase(name),
        mEnv(),
        mpEnvirePointcloud(NULL),
        mpFrameNodePointcloud(NULL),
        mVizkit3DWidget(),
        mEnvViz(),
        mRigidBodyViz(),
        mTrajectoryViz(),
        mTrajectorySplineViz(),
        mMotionCommandViz()
{
}


Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine),
        mEnv(),
        mpEnvirePointcloud(NULL),
        mpFrameNodePointcloud(NULL),
        mVizkit3DWidget(),
        mEnvViz(),
        mRigidBodyViz(),
        mTrajectoryViz(),
        mTrajectorySplineViz(),
        mMotionCommandViz()
{
}


Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{ 
    if (! RTT::TaskContext::configureHook()) {
        return false;
    }
   
    mVizkit3DWidget.start();
    mVizkit3DWidget.getWidget()->addPlugin(&mEnvViz);
    mEnvViz.updateData(&mEnv);

    mVizkit3DWidget.getWidget()->addPlugin(&mRigidBodyViz);

    mVizkit3DWidget.getWidget()->addPlugin(&mTrajectoryViz);

    mTrajectorySplineViz.setColor(0.0, 1.0, 0.0, 1.0);
    mVizkit3DWidget.getWidget()->addPlugin(&mTrajectorySplineViz);

    mMotionCommandViz.setFrontAxis(vizkit::MotionCommandVisualization::FrontAxisX);
    mVizkit3DWidget.getWidget()->addPlugin(&mMotionCommandViz);
    
    // Add pointcloud
    mpEnvirePointcloud = new envire::Pointcloud;
    mEnv.attachItem(mpEnvirePointcloud);
    mpFrameNodePointcloud = new envire::FrameNode;
    mEnv.getRootNode()->addChild(mpFrameNodePointcloud);
    mpEnvirePointcloud->setFrameNode(mpFrameNodePointcloud);

    return true;
}


bool Task::startHook()
{ 
    if (! RTT::TaskContext::startHook())
        return false;

    return true;  
}


void Task::updateHook()
{
    RTT::TaskContext::updateHook();
    
    base::samples::RigidBodyState rbs;
    if(_robot_pose_in.read(rbs) == RTT::NewData) {
        mRigidBodyViz.updateRigidBodyState(rbs);
        mMotionCommandViz.updatePose(rbs.getPose());
        
        // Visualize pointcloud in front of the robot.
        mpFrameNodePointcloud->setTransform(rbs.getTransform());
    }

    // Update waypoints in vizkit
    std::vector<base::Vector3d> waypoints;
    if(_trajectory_in.read(waypoints) == RTT::NewData) {
        RTT::log(RTT::Info) << "Received new trajectory consisting of " << waypoints.size() 
                << " waypoints" << RTT::endlog();

        mTrajectoryViz.clear();
        for(unsigned int i=0; i < waypoints.size(); ++i) {
            base::Vector3d v = waypoints[i];
            mTrajectoryViz.updateTrajectory(v);    
        }
    }

    // Update spline which have been generated using the waypoints.
    std::vector<base::Trajectory> trajectory_spline_vector;
    if(_trajectory_spline_in.read(trajectory_spline_vector) == RTT::NewData) {
        RTT::log(RTT::Info) << "Received new spline trajectory" << RTT::endlog();

        mTrajectorySplineViz.clear();
        for(unsigned int i=0; i < trajectory_spline_vector.size(); ++i) {
            mTrajectorySplineViz.updateSpline(trajectory_spline_vector[i].spline);    
        }
    }

    // Update and visualize motion command
    base::MotionCommand2D motion_cmd;
    if(_motion_command_in.read(motion_cmd) == RTT::NewData) {
        RTT::log(RTT::Info) << "Received new motion command, translation " << 
                motion_cmd.translation << ", rotation " << motion_cmd.rotation << RTT::endlog();
        mMotionCommandViz.updateMotionCommand(motion_cmd);
    }

    // Update envire environment
    envire::OrocosEmitter::Ptr binary_event;
    if (_envire_environment_in.read(binary_event) == RTT::NewData)
    {
        RTT::log(RTT::Info) << "Received new envire environment" << RTT::endlog();
        mEnv.applyEvents(*binary_event);  
    }  
    
    // Update pointcloud
    base::samples::Pointcloud pointcloud;
    if(_pointcloud_in.read(pointcloud) == RTT::NewData) {
        RTT::log(RTT::Info) << "Received new pointcloud containing " << 
                pointcloud.points.size() << " points" << RTT::endlog();
                
        std::vector<base::Point>::iterator it = pointcloud.points.begin();
        mpEnvirePointcloud->clear();
        for(; it != pointcloud.points.end(); ++it) {
            mpEnvirePointcloud->vertices.push_back(*it);
        }
        mpEnvirePointcloud->itemModified();
    }
}


void Task::errorHook()
{
    RTT::TaskContext::errorHook();
}


void Task::stopHook()
{
    RTT::TaskContext::stopHook(); 
}


void Task::cleanupHook()
{
    RTT::TaskContext::cleanupHook();
}

