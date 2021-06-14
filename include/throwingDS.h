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

#ifndef __THROWING_DS_H__
#define __THROWING_DS_H__

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <stdio.h>

#include <vector>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <Utils.hpp>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

struct tossDsParam{
	double modulRegion[3]; 	// modulation parameters: distances [0] = radial, [1] = normal, [2] = tangent
	Eigen::Matrix3d Kp[3];  // Stiffness gains for position [0] : reaching  [1]: tossing [2]: retraction 
	Eigen::Matrix3d Dp[3]; 	// Damping gains   for position [0] : reaching  [1]: tossing [2]: retraction 
	Eigen::Matrix3d Ko[3]; 	// Stiffness gains for orienation [0] : reaching  [1]: tossing [2]: retraction 
	Eigen::Matrix3d Do[3]; 	// Damping gains   for orienation [0] : reaching  [1]: tossing [2]: retraction 
	bool is2ndOrder;				// boolean for the type of DS  True: second order, false: first order
};


class throwingDS
{
	public:
		// TASK ID: Reaching, tossing, going to retract
	    enum TASK_ID {REACH = 0, TOSS = 1, RETRACT = 2};
	  // MOTION ID
	    enum MOTION_ID {TRANSLATION = 0, ROTATION = 1};


	private:

		// nominal DS
		Eigen::Matrix3d Kp_[3];  // Stifness gains for position of reaching task    [0], modulated motion task, [1] go to retract task [2]
		Eigen::Matrix3d Dp_[3];  // Damping gains  for position of reaching task    [0], modulated motion task, [1] go to retract task [2]
		Eigen::Matrix3d Ko_[3];  // Stifness gains for orientation of reaching task [0], modulated motion task, [1] go to retract task [2]
		Eigen::Matrix3d Do_[3];  // Damping gains  for orientation of reaching task [0], modulated motion task, [1] go to retract task [2]
		//
		double rho_;
		double range_norm_;
		double range_tang_;

		double sw_proxim_;
		double sw_norm_;
		double sw_tang_;

		bool is2ndOrder_;

		Eigen::Matrix4d w_H_de_; 
		Eigen::Matrix4d w_H_re_; 	
		// Vector6d Vee_;

		Eigen::Vector3d v_toss_;
		Eigen::Vector3d w_toss_;
		Eigen::Matrix3d BasisQp_;
		Eigen::Matrix3d BasisQo_;
		//
		Vector6d V_ee_d_;
		Vector6d A_ee_d_;
		bool release_flag_;



	public:

		double a_proximity_;
		double a_normal_;
		double a_tangent_;
		double a_retract_;
		double coupling_;


		throwingDS();
		~throwingDS();

		bool init(double modulRegion[], Eigen::Matrix3d Kp[], Eigen::Matrix3d Dp[], Eigen::Matrix3d Ko[], Eigen::Matrix3d Do[], bool is2ndOrder);

		bool init(tossDsParam ds_param, Eigen::Vector3d releasePos, Eigen::Vector4d releaseOrient, Eigen::Vector3d releaseLinVel, Eigen::Vector3d releaseAngVel, 
								Eigen::Vector3d restPos, Eigen::Vector4d restOrient);

		/**
		 * @brief parameters for the throwing motion generation (2nd order)
		 * @param w_H_ce homogeneous transformation of current end effector pose
		 * @param Vee current velocity twist of the end-effector
		 * @param w_H_de homogeneous transformation of desired end effector pose
		 * @param w_H_re homogeneous transformation of rest end effector pose right after throwing
		 * @param Vdtoss desired throwing velocity of the end-effector (3x1)
		 * @param release_flag  a boolean flag to trigger the release of the object
		 */
		Vector6d generate_throwing_motion(Eigen::Matrix4d w_H_ce,  Vector6d Vee, Eigen::Matrix4d w_H_de, Eigen::Matrix4d w_H_re,  
									 								Eigen::Matrix3d BasisQ, Eigen::Vector3d Vdtoss, bool &release_flag);

		Eigen::Vector3d compute_modulated_motion(double activation, Eigen::Matrix3d BasisQ, Eigen::Vector3d Areach_ee, 
																							Eigen::Vector3d Amodul_ee_norm, Eigen::Vector3d Amodul_ee_tang);
		Eigen::Vector3d compute_angular_motion(double coupling, Eigen::Matrix4d w_H_c, Eigen::Vector3d Omega, 
																						Eigen::Matrix4d w_H_d, Eigen::Matrix3d Ko, Eigen::Matrix3d Do, bool is2ndOrder);
		Eigen::MatrixXd createOrthonormalMatrixFromVector(Eigen::VectorXd inVec);

		Vector6d apply(Eigen::Vector3d curPos, Eigen::Vector4d curOrient, Eigen::Vector3d curLinVel, Eigen::Vector3d curAngVel);
		//
		bool set_gains(int taskId, int motionId, Eigen::Matrix3d K, Eigen::Matrix3d D);
		bool set_modulationParameters(double new_modul_param[]);
		bool set_toss_linear_velocity(Eigen::Vector3d newLinVel);
		bool set_toss_angular_velocity(Eigen::Vector3d newAngVel);
		bool set_toss_pose(Eigen::Vector3d new_releasePos, Eigen::Vector4d new_releaseOrient);
		bool set_rest_pose(Eigen::Vector3d new_restPos, Eigen::Vector4d new_restOrient);
		bool get_release_flag();
};

#endif
