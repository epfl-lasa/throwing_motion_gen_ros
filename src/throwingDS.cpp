

#include "throwingDS.h"


throwingDS::throwingDS(){
	//
	for(int i=0; i<3; i++){

		Kp_[i] = 5.0 *Eigen::MatrixXd::Identity(3,3);
		Dp_[i] = 2.0 *sqrt(Kp_[i](0,0))*Eigen::MatrixXd::Identity(3,3);
		//
		Ko_[i] = 2.5 *Eigen::MatrixXd::Identity(3,3);
		Do_[i] = 2.0 *sqrt(Ko_[i](0,0))*Eigen::MatrixXd::Identity(3,3);
	}
	//
	rho_		 = 0.10;
	range_norm_  = 0.03;
	range_tang_  = 0.03;

	sw_proxim_   = 100.0;
	sw_norm_     = 100.0;
	sw_tang_     = 100.0;

}
throwingDS::~throwingDS(){

}
bool throwingDS::init(double modulRegion[], Eigen::Matrix3d Kp[], Eigen::Matrix3d Dp[], Eigen::Matrix3d Ko[], Eigen::Matrix3d Do[], bool is2ndOrder){
	
	// initialize the gains
	memcpy(Kp_, &Kp[0], 3 * sizeof *Kp); 
	memcpy(Dp_, &Dp[0], 3 * sizeof *Dp); 
	memcpy(Ko_, &Ko[0], 3 * sizeof *Ko); 
	memcpy(Do_, &Do[0], 3 * sizeof *Do); 

	is2ndOrder_ = is2ndOrder;

	rho_        = modulRegion[0];
	range_norm_ = modulRegion[1];
	range_tang_ = modulRegion[2];

	// 
	std::cout << " rho_ is \n" << rho_ << std::endl;
	std::cout << " range_norm_ is \n" << range_norm_ << std::endl;
	std::cout << " range_tang_ is \n" << range_tang_ << std::endl;

	return true;
}

bool throwingDS::generate_throwing_motion(Eigen::Matrix4d w_H_ce,  Vector6d Vee, Eigen::Matrix4d w_H_de, Eigen::Matrix4d w_H_re,  
										 		Eigen::Matrix3d BasisQ, Eigen::Vector3d Vdtoss, Vector6d &V_ee_d, Vector6d &A_ee_d)
{
	// States and desired states
	Eigen::Vector3d X 	  = w_H_ce.block<3,1>(0,3);
	Eigen::Vector3d Xdot  = Vee.head(3);
	Eigen::Vector3d Omega = Vee.tail(3);
	Eigen::Vector3d Xdes  = w_H_de.block<3,1>(0,3);	
	Eigen::Vector3d Xretr = w_H_re.block<3,1>(0,3);	
	Eigen::Vector3d Xb    = Xdes + BasisQ*Eigen::Vector3d(-1.5*this->rho_, 0.0, 0.0);
	Eigen::Vector3d Xe    = Xdes + BasisQ*Eigen::Vector3d( 1.0*this->rho_, 0.0, 0.0);

	//=======================================================================
	// Modulation term
	//=======================================================================
	Eigen::Vector3d Xqb = BasisQ.transpose()*(X - Xb);
	Eigen::Vector3d Xqe = BasisQ.transpose()*(X - Xe);

	double dist2reach   = (X - Xb).norm();
	double dist2line    = Xqb.tail(2).norm();
	double dist2end     = Xqe.head(1).norm();
	
	double a_proximity	= 0.5*(std::tanh(this->sw_proxim_* (this->rho_ - dist2reach)) + 1.0 ); 	   		// scalar function of 3D distance  to initial (pre-modulation) position of attractor
	double a_normal   	= 0.5*(std::tanh(this->sw_norm_  * (this->range_norm_ - dist2line)) + 1.0 ); 	// scalar function of distance to the line of direction Vtoss and passing through the release position
	double a_tangent  	= 0.5*(std::tanh(this->sw_tang_  * (this->range_tang_ - dist2end))  + 1.0 ); 	// scalar function of distance to the stopping position of the throwing task
	double coupling     = exp(-0.5*dist2line/(2.0*range_norm_*range_norm_));
	// 
	double activation  = a_proximity + a_normal;
	activation = 0.5*(std::tanh(5.0 * (a_proximity + a_normal - 0.5)) + 1.0 );

	// state-dependent gain matrix
	// ----------------------------
	Vector6d Out_motion = Eigen::VectorXd::Zero(6,1);
	A_ee_d.setZero();
	V_ee_d.setZero();
	//
	if(this->is2ndOrder_){
		// Eigen::Vector3d Xstar = (1.0 - a_normal) * Xb + a_normal * (X + Km.inverse()*Dm*Xdot );
		Eigen::Vector3d Xstar = (1.0 - a_normal) * Xb + a_normal * (X + Vdtoss - (Eigen::MatrixXd::Identity(3,3)-Kp_[TOSS].inverse()*Dp_[TOSS] )*Xdot );
		// 
		Eigen::Vector3d Areach_ee      = Dp_[REACH]*Xdot + Kp_[REACH]*(X - Xb); 						// DS for approaching the tossing position
		Eigen::Vector3d Amodul_ee_norm = Dp_[TOSS]*Xdot  + Kp_[TOSS]*(X - Xb); 							// Modulated DS that aligned  the EE with the desired velocity
		Eigen::Vector3d Amodul_ee_tang = Dp_[TOSS]*Xdot  + Kp_[TOSS]*(X - Xstar); 						//this->computeModulatedAcceleration(Km, Dm, X, Xdot, Xstar);
		Eigen::Vector3d Aretrac_ee     = Dp_[RETRACT]*Xdot + Kp_[RETRACT]*(X - Xretr); 					// DS for retracting after the tossing position

		// get the modulated motion
		Out_motion.head(3) = (1.0-a_tangent)*this->compute_modulated_motion(activation, BasisQ, Areach_ee, Amodul_ee_norm, Amodul_ee_tang) + a_tangent * Aretrac_ee;
		// get angular motion
		Out_motion.tail(3) = (1.0-a_tangent)*this->compute_angular_motion(coupling, w_H_ce, Omega, w_H_de, Ko_[REACH], Do_[REACH], this->is2ndOrder_) 
							+   a_tangent   *this->compute_angular_motion(coupling, w_H_ce, Omega, w_H_re, Ko_[RETRACT], Do_[RETRACT], this->is2ndOrder_);

		// out_motion: Acceleration
		A_ee_d = Out_motion;
		// reconstruct desired velocity from compute acceleration
		V_ee_d.head(3) = -(1.0-a_tangent)*( ((1.0-activation)*Dp_[REACH].inverse() + activation * Dp_[TOSS].inverse())*A_ee_d.head(3) - Xdot) 
						 - a_tangent * (Dp_[RETRACT].inverse()*Out_motion.head(3)-Xdot);
		V_ee_d.tail(3) = -(1.0-a_tangent)*( ((1.0-activation)*Do_[REACH].inverse() + activation * Do_[TOSS].inverse())*A_ee_d.tail(3) - Xdot) 
						 - a_tangent * (Do_[RETRACT].inverse()*Out_motion.tail(3)-Xdot);

	}
	else{
		Eigen::Vector3d Xstar = (1.0 - a_normal) * Xb + a_normal * (X - Kp_[TOSS].inverse()*Vdtoss);
		// 
		Eigen::Vector3d Areach_ee      = Kp_[REACH]*(X - Xb); 						// DS for approaching the tossing position
		Eigen::Vector3d Amodul_ee_norm = Kp_[TOSS]*(X - Xb); 						// Modulated DS that aligned  the EE with the desired velocity
		Eigen::Vector3d Amodul_ee_tang = Kp_[TOSS]*(X - Xstar); 					// this->computeModulatedAcceleration(Km, Dm, X, Xdot, Xstar);
		Eigen::Vector3d Aretrac_ee     = Kp_[RETRACT]*(X - Xretr); 					// DS for retracting after the tossing position

		// get the modulated motion
		Out_motion.head(3) = (1.0-a_tangent)*this->compute_modulated_motion(activation, BasisQ, Areach_ee, Amodul_ee_norm, Amodul_ee_tang) + a_tangent * Aretrac_ee;
		// get angular motion
		Out_motion.tail(3) = (1.0-a_tangent)*this->compute_angular_motion(coupling, w_H_ce, Omega, w_H_de, Ko_[REACH], Do_[REACH], this->is2ndOrder_) 
							+   a_tangent   *this->compute_angular_motion(coupling, w_H_ce, Omega, w_H_re, Ko_[RETRACT], Do_[RETRACT], this->is2ndOrder_);

		// out_motion: Velocity
		V_ee_d = Out_motion;
		// reconstruct the desired acceleration based on the computed velocity (critically damped with same settling time)
		A_ee_d.head(3) = (1.0-a_tangent)*( ((1.0-activation)*Dp_[REACH] + activation * Dp_[TOSS])*Xdot) + a_tangent * (Dp_[RETRACT]*Xdot) + V_ee_d.head(3);
		A_ee_d.tail(3) = (1.0-a_tangent)*( ((1.0-activation)*Do_[REACH] + activation * Do_[TOSS])*Xdot) + a_tangent * (Do_[RETRACT]*Xdot) + V_ee_d.tail(3);
	}

	return true;
	
}

Eigen::Vector3d throwingDS::compute_modulated_motion(double activation, Eigen::Matrix3d BasisQ, Eigen::Vector3d Areach_ee, 
														Eigen::Vector3d Amodul_ee_norm, Eigen::Vector3d Amodul_ee_tang)
{
	//
	Eigen::MatrixXd den_temp  = Areach_ee.transpose() * Areach_ee;
	Eigen::RowVector3d Beta_j = 1.0/(den_temp(0,0)) * (Areach_ee.transpose() * BasisQ);

	Eigen::Matrix3d Lambda = Eigen::MatrixXd::Zero(3,3);
	Lambda.block<1,1>(0,0) = activation*( BasisQ.col(0).transpose() * Amodul_ee_tang * Beta_j(0) ) + (1.0-activation)*Eigen::MatrixXd::Identity(1,1);
	Lambda.block<1,1>(0,1) = activation*( BasisQ.col(0).transpose() * Amodul_ee_tang * Beta_j(1) );
	Lambda.block<1,1>(0,2) = activation*( BasisQ.col(0).transpose() * Amodul_ee_tang * Beta_j(2) );

	Lambda.block<1,1>(1,0) = activation*( BasisQ.col(1).transpose() * Amodul_ee_norm *Beta_j(0) )  + (1.0-activation)*Eigen::MatrixXd::Identity(1,1);
	Lambda.block<1,1>(1,1) = activation*( BasisQ.col(1).transpose() * Amodul_ee_norm *Beta_j(1) );
	Lambda.block<1,1>(1,2) = activation*( BasisQ.col(1).transpose() * Amodul_ee_norm *Beta_j(2) );

	Lambda.block<1,1>(2,0) = activation*( BasisQ.col(2).transpose() * Amodul_ee_norm *Beta_j(0) )  + (1.0-activation)*Eigen::MatrixXd::Identity(1,1);
	Lambda.block<1,1>(2,1) = activation*( BasisQ.col(2).transpose() * Amodul_ee_norm *Beta_j(1) );
	Lambda.block<1,1>(2,2) = activation*( BasisQ.col(2).transpose() * Amodul_ee_norm *Beta_j(2) );

	// computing the modulated second order DS (translation)
	return BasisQ * Lambda * BasisQ.transpose() * Areach_ee; 
}


Eigen::Vector3d throwingDS::compute_angular_motion(double coupling, Eigen::Matrix4d w_H_c, Eigen::Vector3d Omega, Eigen::Matrix4d w_H_d, Eigen::Matrix3d Ko, Eigen::Matrix3d Do, bool is2ndOrder){

	// Rotation motion
	//----------------
	Eigen::Matrix3d w_R_c   = w_H_c.block<3,3>(0,0);	// current orientation of the end effector
	Eigen::Matrix3d w_R_d   = w_H_d.block<3,3>(0,0);	// desired orientation of the end effector
	// coupling orientation task to position through spherical interpolation of the orientation 
	Eigen::Matrix3d w_R_d_t = Utils<double>::getCombinedRotationMatrix(coupling, w_R_c, w_R_d); 
	// relative transformation between desired and current frame
	Eigen::Matrix3d d_R_c_t = w_R_d_t.transpose() * w_R_c;
	// 3D Orientation Jacobian 
	Eigen::Matrix3d jacMuTheta = Utils<double>::getMuThetaJacobian(d_R_c_t) * w_R_c.transpose();
	// approaximation of the acceleration (neglecting  -jacMuTheta_dot * Omega)
	// return jacMuTheta.inverse() * ( Do*jacMuTheta*Omega + Ko * Utils<double>::getOrientationErrorCur2Des(d_R_c_t));

	if(is2ndOrder){
		// approaximation of the acceleration (neglecting  -jacMuTheta_dot * Omega)
		return jacMuTheta.inverse() * ( Do*jacMuTheta*Omega + Ko * Utils<double>::getOrientationErrorCur2Des(d_R_c_t));
	}
	else{
		return jacMuTheta.inverse() * (Ko * Utils<double>::getOrientationErrorCur2Des(d_R_c_t));
	}

}