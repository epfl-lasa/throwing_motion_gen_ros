/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Authors: Mahdi Khoramshahi and Nadia Figueroa
 * email:   {mahdi.khoramshahi,nadia.figueroafernandez}@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the European Communitys Horizon 2020 Research and Innovation 
 * programme ICT-23-2014, grant agreement 644727-Cogimon and 643950-SecondHands.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __THROWING_MOTION_GENERATION_H__
#define __THROWING_MOTION_GENERATION_H__

#include <iomanip>
#include <sstream>
#include <fstream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PointStamped.h"
#include <std_msgs/Bool.h>
#include "nav_msgs/Path.h"

#include <vector>
#include <mutex>

// #include "MathLib.h"
// #include "GMRDynamics.h"
// #include "CDDynamics.h"
// #include <dynamic_reconfigure/server.h>
// #include <throwing_ds_ros/seDS_paramsConfig.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "Utils.hpp"
#include "throwingDS.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;


class throwingMotionGeneration
{
	private:

		// ROS system variables
	    ros::NodeHandle           nh_;
	    ros::Rate                 loop_rate_;

	    // Publishers/Subscriber
	    ros::Subscriber			sub_desired_ee_pose_;
	   	ros::Subscriber			sub_desired_toss_velo_;
	    ros::Subscriber			sub_retract_ee_pose_;
	   	ros::Subscriber			sub_ee_pose_;
	    ros::Subscriber			sub_ee_velo_;

	    ros::Publisher 			pub_desired_twist_;				// desired velocity of the robot end-effector
	    ros::Publisher 			pub_desired_acceleration_;		// desired velocity of the robot end-effector

	    // Topic Names
		std::string 			input_topic_ds_model_path_;
		std::string 			input_topic_desired_ee_pose_;	// release point position and associate orientation of EE
		std::string 			input_topic_desired_toss_velo_;
		std::string 			input_topic_retract_ee_pose_;
        std::string 			input_topic_ee_pose_;
        std::string 			input_topic_ee_velo_;
        std::string 			output_topic_ee_velo_;
        std::string 			output_topic_ee_accel_;
        std::string 			ds_type_;

        //
        geometry_msgs::Pose 	ee_desired_pose_;
        geometry_msgs::Twist 	ee_toss_velo_;
        geometry_msgs::Pose 	ee_current_pose_;
		geometry_msgs::Twist 	ee_current_velo_;
		geometry_msgs::Twist 	ee_desired_velo_;
		geometry_msgs::Twist 	ee_desired_accel_;

		//
		Eigen::Vector3d 		x_;							// current EE positon
		Eigen::Vector4d 		q_;							// current EE orientation (quaternion representation)
		Eigen::Vector3d 		xd_;						// desired EE positon (release position)
		Eigen::Vector4d 		qd_;						// desired EE orientation (release)
		Eigen::Matrix3d 		wRb_;             			// Orientation matrix (3x3)
		Eigen::Vector3d 		xr_;						// retract EE positon after throwing
		Eigen::Vector4d 		qr_;						// retract EE orientation fater throwing
		Eigen::Vector3d 		vd_;
		Eigen::Vector3d 		vtoss_;						// desired throwing velocity (release velocity)
		Eigen::Vector3d 		omegad_;
		Eigen::Matrix4d 		w_H_ce_;					// homogeneous transformation of current EE positon
		Eigen::Matrix4d 		w_H_de_;					// homogeneous transformation of desired EE positon (release position)
		Eigen::Matrix4d 		w_H_re_;					// homogeneous transformation of retract EE positon after throwing
		Eigen::Matrix3d 		BasisQ_;
		Vector6d 				V_ee_;
		Vector6d 				V_ee_d_;
		Vector6d 				A_ee_d_;
		Eigen::Vector3d 		toolOffsetFromEE_;

		double					modulRegion_[3];			
		Eigen::Matrix3d 		Kp_[3];
		Eigen::Matrix3d 		Dp_[3];
		Eigen::Matrix3d 		Ko_[3];
		Eigen::Matrix3d 		Do_[3];

		bool is2ndOrder_;
		// class of DS
		throwingDS dsThrowing2;

	public:

		throwingMotionGeneration( ros::NodeHandle &nh_,
	                double frequency,
	                std::string input_topic_ds_model_path,
	                std::string input_topic_desired_ee_pose,
	                std::string input_topic_desired_toss_velo,
	                std::string input_topic_retract_pose,
	                std::string input_topic_ee_pose,
	                std::string input_topic_ee_velo,
	                std::string output_topic_ee_velo,
	                std::string output_topic_ee_accel,
	                std::string ds_type
	                );

		~throwingMotionGeneration();
		bool init();
		void run();

	private:
		bool InitializeROS();
		bool InitializeDS();
		void generate_motion();
		void UpdateTargetPose(const geometry_msgs::Pose::ConstPtr& msg);
		void UpdateTargetVelocity(const geometry_msgs::Twist::ConstPtr& msg);
		void UpdateRetractPose(const geometry_msgs::Pose::ConstPtr& msg);
		void UpdateEEPose(const geometry_msgs::Pose::ConstPtr& msg);
		void UpdateEEVelocity(const geometry_msgs::Twist::ConstPtr& msg);
		void Publish_desired_motion();
		bool getDsParamFromYmlFile();
		bool getFirstOrderDsParamFromGMM();
		bool getSecondOrderDsParamFromGMM();	
};

#endif
