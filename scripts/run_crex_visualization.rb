require 'orocos'
require 'readline'

include Orocos

Orocos::CORBA.max_message_size = 8000000
Orocos::CORBA.name_service.ip = "10.250.3.129" # CREX PC
 
Orocos.initialize

Orocos.run 'visualizer_module_deployment',
	    "valgrind" => false, "wait" => 2000, "output" => nil do |*_|

    visualizer = TaskContext.get('visualizer_module')
    visualizer.configure
    visualizer.start

    pointcloud = TaskContext.get('pointcloud_creator')
    planner = TaskContext.get('simple_path_planner')
    follower = TaskContext.get('trajectory')
    vicon = TaskContext.get('vicon_deployment')

    pointcloud.envire_environment_out.connect_to(visualizer.envire_environment_in)
    planner.trajectory_out.connect_to(visualizer.trajectory_in)
    planner.trajectory_spline_out.connect_to(visualizer.trajectory_spline_in)
    follower.motion_command.connect_to(visualizer.motion_command_in)
    vicon.pose_samples.connect_to(visualizer.robot_pose_in)

=begin
    # Send pose to modules (will be replaced by Vicon)
    visualizer_pose_writer = visualizer.robot_pose_in.writer
    
    rbs = visualizer_pose_writer.new_sample
    rbs.sourceFrame = "robot"
    rbs.targetFrame = "world"
    rbs.position = Eigen::Vector3.new(0.0, 0.0, 0.0)
    rbs.orientation = Eigen::Quaternion.Identity
    while true
        visualizer_pose_writer.write(rbs)
        sleep 1
    end
=end

    Readline::readline("Press ENTER to exit ...") 
end
