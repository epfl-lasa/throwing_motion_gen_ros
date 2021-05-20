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

#include <iomanip>
#include <sstream>
#include <fstream>

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

typedef Eigen::Matrix<double, 6, 1> Vector6d;


class throwingDS
{
	public:
		// TASK ID: Reaching, tossing, going to retract
	    enum TASK_ID {REACH = 0, TOSS = 1, RETRACT = 2};

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


	public:

		throwingDS();
		~throwingDS();

		bool init(double modulRegion[], Eigen::Matrix3d Kp[], Eigen::Matrix3d Dp[], Eigen::Matrix3d Ko[], Eigen::Matrix3d Do[]);

		/**
		 * @brief parameters for the throwing motion generation (2nd order)
		 * @param w_H_ce homogeneous transformation of current end effector pose
		 * @param Vee current velocity twist of the end-effector
		 * @param w_H_de homogeneous transformation of desired end effector pose
		 * @param w_H_re homogeneous transformation of rest end effector pose right after throwing
		 * @param Vdtoss desired throwing velocity of the end-effector (3x1)
		 * @param Ad_ee  desired acceleration command of the end-effector
		 */
		void generate_throwing_motion2(Eigen::Matrix4d w_H_ce,  Vector6d Vee, Eigen::Matrix4d w_H_de, Eigen::Matrix4d w_H_re,  
										 	Eigen::Matrix3d BasisQ, Eigen::Vector3d Vdtoss, Vector6d &Ad_ee);

		Eigen::Vector3d compute_modulated_motion2(double activation, Eigen::Matrix3d BasisQ, Eigen::Vector3d Areach_ee, 
													Eigen::Vector3d Amodul_ee_norm, Eigen::Vector3d Amodul_ee_tang);

		Eigen::Vector3d compute_angular_motion2(double coupling, Eigen::Matrix4d w_H_c, Eigen::Vector3d Omega, 
													Eigen::Matrix4d w_H_d, Eigen::Matrix3d Ko, Eigen::Matrix3d Do);
};

#endif
