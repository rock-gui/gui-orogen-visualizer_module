require 'orocos'
require 'readline'

include Orocos

Orocos::CORBA.max_message_size = 8000000
Orocos.initialize

Orocos.run 'visualizer_module_deployment',
	    "valgrind" => false, "wait" => 2000, "output" => nil do |*_|

    visualizer = TaskContext.get('visualizer_module')
    visualizer.configure
    visualizer.start

    crex_simu = TaskContext.get 'test_crex_simulation'
    pointcloud = TaskContext.get 'pointcloud_creator'
    planner = TaskContext.get 'simple_path_planner'
    follower = TaskContext.get 'trajectory'

    crex_simu.pose_out.connect_to(visualizer.robot_pose_in)
    pointcloud.envire_environment_out.connect_to(visualizer.envire_environment_in)
    planner.trajectory_out.connect_to(visualizer.trajectory_in)
    planner.trajectory_spline_out.connect_to(visualizer.trajectory_spline_in)
    follower.motion_command.connect_to(visualizer.motion_command_in)

    Readline::readline("Press ENTER to exit ...")
end
