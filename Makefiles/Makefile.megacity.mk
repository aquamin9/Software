megacity: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos megacity.launch"

tcp-server: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos tcp_server.launch"

virjoy-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh $*; source set_vehicle_name.sh $*; python misc/virtualJoy/virtualJoy.py"

parallel-autonomy: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos parallel_autonomy.launch"

formula-d-wheel-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh $*; source set_vehicle_name.sh $*; roslaunch duckietown_demos formula_D_steering_wheel.launch veh:=$*"

formula-d: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos formula_D.launch"

camera-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh $*; source set_vehicle_name.sh $*; rosrun image_view image_view image:='/$*/camera_node/image' _image_transport:=compressed"


set-in-charger-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh $*; source set_vehicle_name.sh $*; rostopic pub -1 /$*/maintenance_control_node/go_charging std_msgs/Bool True; rostopic pub -1 /$*/maintenance_control_node/set_state std_msgs/String 'CHARGING'; rosparam set /$*/charging_control_node/charger 4"

prepare: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; cd $(DUCKIEFLEET_ROOT); git pull; cd $(DUCKIETOWN_ROOT)"

prepare-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh $*; source set_vehicle_name.sh $*"
