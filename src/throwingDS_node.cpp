

#include "ros/ros.h"
#include "throwingMotionGeneration.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "throwingDS_node");

	ros::NodeHandle nh;
	double frequency = 500.0;

	// Parameters
	std::string input_topic_ds_model_path;
	std::string input_topic_desired_ee_pose;
	std::string input_topic_ee_pose;
	std::string input_topic_ee_velo;
	std::string output_topic_ee_velo;
	std::string output_topic_ee_accel;

	// get ros parameters
	if (!nh.getParam("input_topic_ds_model_path", input_topic_ds_model_path)) {
		ROS_ERROR("Couldn't retrieve the topic name for the input ds model. ");
		// return -1;
	}

	// get ros parameters
	if (!nh.getParam("input_topic_desired_ee_pose", input_topic_desired_ee_pose)) {
		ROS_ERROR("Couldn't retrieve the topic name for the input desired EE pose ");
		// return -1;
	}

	if (!nh.getParam("input_topic_ee_pose", input_topic_ee_pose))   {
		ROS_ERROR("Couldn't retrieve the topic name for the input EE pose. ");
		// return -1;
	}

	if (!nh.getParam("input_topic_ee_velo", input_topic_ee_velo))   {
		ROS_ERROR("Couldn't retrieve the topic name for the input EE velocity. ");
		// return -1;
	}

	if (!nh.getParam("output_topic_ee_velo", output_topic_ee_velo))   {
		ROS_ERROR("Couldn't retrieve the topic name for the output EE velocity. ");
		// return -1;
	}

	if (!nh.getParam("output_topic_ee_accel", output_topic_ee_accel))   {
		ROS_ERROR("Couldn't retrieve the topic name for the output EE acceleration. ");
		// return -1;
	}

	//
	throwingMotionGeneration ds_motion_generator( nh, 
									frequency,
	                                input_topic_ds_model_path,
	                                input_topic_desired_ee_pose,
									input_topic_ee_pose,
									input_topic_ee_velo,
									output_topic_ee_velo,
									output_topic_ee_accel);

	if (!ds_motion_generator.init()) {
		return -1;
	}
	else {
		ds_motion_generator.run();
	}

	return 0;
}