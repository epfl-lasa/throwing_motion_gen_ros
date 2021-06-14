#include <throwingDS.h>


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
	rho_		     	= 0.20;
	range_norm_  	= 0.03;
	range_tang_  	= 0.01;

	sw_proxim_   	= 150.0;
	sw_norm_     	= 200.0;
	sw_tang_     	= 200.0;
	//
	a_proximity_ 	= 0.0;
	a_normal_ 		= 0.0;
	a_tangent_ 		= 0.0;
	a_retract_   	= 0.0;
	coupling_ 		= 0.0;
	//
	w_H_de_				= Eigen::MatrixXd::Identity(4,4);
	w_H_re_				= Eigen::MatrixXd::Identity(4,4);
	v_toss_				= Eigen::VectorXd::Zero(3);
	w_toss_				= Eigen::VectorXd::Zero(3);
	BasisQp_			= Eigen::MatrixXd::Identity(3,3);
	BasisQo_			= Eigen::MatrixXd::Identity(3,3);
	V_ee_d_ 			= Eigen::VectorXd::Zero(6);
	A_ee_d_ 			= Eigen::VectorXd::Zero(6);
	//
	release_flag_ = false;

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

	return true;
}

bool throwingDS::init(tossDsParam ds_param, Eigen::Vector3d releasePos, Eigen::Vector4d releaseOrient, 
											Eigen::Vector3d releaseLinVel, Eigen::Vector3d releaseAngVel, 
											Eigen::Vector3d restPos, Eigen::Vector4d restOrient)
{
	// initialize the gains
	memcpy(Kp_, &ds_param.Kp[0], 3 * sizeof *ds_param.Kp); 
	memcpy(Dp_, &ds_param.Dp[0], 3 * sizeof *ds_param.Dp); 
	memcpy(Ko_, &ds_param.Ko[0], 3 * sizeof *ds_param.Ko); 
	memcpy(Do_, &ds_param.Do[0], 3 * sizeof *ds_param.Do); 
	//
	is2ndOrder_ = ds_param.is2ndOrder;
	rho_        = ds_param.modulRegion[0];
	range_norm_ = ds_param.modulRegion[1];
	range_tang_ = ds_param.modulRegion[2];
	//
	v_toss_  = releaseLinVel;	
	w_toss_  = releaseAngVel;
	//
	w_H_de_ = Utils<double>::pose2HomoMx(releasePos, releaseOrient);
	w_H_re_ = Utils<double>::pose2HomoMx(restPos, restOrient);
	// 
	BasisQp_ = this->createOrthonormalMatrixFromVector(v_toss_);
	BasisQo_ = this->createOrthonormalMatrixFromVector(w_toss_);

	return true;	
}

Vector6d throwingDS::apply(Eigen::Vector3d curPos, Eigen::Vector4d curOrient, Eigen::Vector3d curLinVel, Eigen::Vector3d curAngVel){
	//
	Eigen::Matrix4d w_H_ce = Utils<double>::pose2HomoMx(curPos, curOrient);
	Vector6d Vee  	= Eigen::VectorXd::Zero(6);
	Vee.head(3) 	= curLinVel; 
	Vee.tail(3) 	= curAngVel; 
	bool release_flag = false;
	//
	return this->generate_throwing_motion(w_H_ce,  Vee, w_H_de_, w_H_re_, BasisQp_, v_toss_, release_flag);
}

Vector6d throwingDS::generate_throwing_motion(Eigen::Matrix4d w_H_ce,  Vector6d Vee, Eigen::Matrix4d w_H_de, Eigen::Matrix4d w_H_re,  
										 											Eigen::Matrix3d BasisQ, Eigen::Vector3d Vdtoss, bool &release_flag)
{
	// States and desired states
	Eigen::Vector3d X 	  = w_H_ce.block<3,1>(0,3);
	Eigen::Vector3d Xdot  = Vee.head(3);
	Eigen::Vector3d Omega = Vee.tail(3);
	Eigen::Vector3d Xdes  = w_H_de.block<3,1>(0,3);	
	Eigen::Vector3d Xretr = w_H_re.block<3,1>(0,3);	
	Eigen::Vector3d Xb    = Xdes + BasisQ*Eigen::Vector3d(-0.5*this->rho_, 0.0, 0.0);
	Eigen::Vector3d Xe    = Xdes + BasisQ*Eigen::Vector3d( 0.2*this->rho_, 0.0, 0.0);
	Eigen::Vector3d Xc    = Xdes + BasisQ*Eigen::Vector3d(-0.2*this->rho_, 0.0, 0.0);

	//=======================================================================
	// Modulation term
	//=======================================================================
	Eigen::Vector3d Xqb = BasisQ.transpose()*(X - Xb);
	Eigen::Vector3d Xqe = BasisQ.transpose()*(X - Xe);

	double dist2reach   = (X - Xc).norm();
	double dist2line    = Xqb.tail(2).norm();
	double dist2end     = Xqe.head(1).norm();
	
	a_proximity_	= 0.5*(std::tanh(this->sw_proxim_* (0.5*this->rho_ - dist2reach)) + 1.0 ); 	   		// scalar function of 3D distance  to initial (pre-modulation) position of attractor
	a_proximity_	*= (1.0-a_retract_);
	// a_proximity_ = 1.0;

	a_normal_   	= a_proximity_* 0.5*(std::tanh(this->sw_norm_  * (this->range_norm_ - dist2line)) + 1.0 ); 	// scalar function of distance to the line of direction Vtoss and passing through the release position
	a_tangent_  	= a_proximity_* 0.5*(std::tanh(this->sw_tang_  * (this->range_tang_ - dist2end))  + 1.0 ); 	// scalar function of distance to the stopping position of the throwing task
	coupling_     = exp(-0.5*dist2line/(2.0*range_norm_*range_norm_));
	coupling_ = 1.0;
	
	if(a_tangent_ >= 0.95){ 
		a_retract_   = 1.0;
	}
	else if((a_retract_ == 1.0) && (X - Xretr).norm() <= 0.01){  // tolerance within 1 cm
		// a_retract_    = 0.0;
	}
	// 
	if((X-Xdes).norm() <= 1e-2){  // release if the norm is within 1 cm
		release_flag_ = true;
	}

	release_flag = release_flag_;

	double activation   = a_proximity_;
	// state-dependent gain matrix
	// ----------------------------
	Vector6d Out_motion = Eigen::VectorXd::Zero(6,1);
	//
	if(this->is2ndOrder_){
		// Eigen::Vector3d Xstar = (1.0 - a_normal_) * Xb + a_normal_ * (X + Kp_[TOSS].inverse()*Dp_[TOSS]*Vdtoss );
		Eigen::Vector3d Xstar = (1.0 - a_normal_) * Xb + a_normal_ * (X + Vdtoss - (Eigen::MatrixXd::Identity(3,3)-Kp_[TOSS].inverse()*Dp_[TOSS] )*Xdot );
		// 
		Eigen::Vector3d Areach_ee      = Dp_[REACH]*Xdot + Kp_[REACH]*(X - Xb); 						// DS for approaching the tossing position
		Eigen::Vector3d Amodul_ee_norm = Dp_[TOSS]*Xdot  + Kp_[TOSS]*(X - Xb); 							// Modulated DS that aligned  the EE with the desired velocity
		Eigen::Vector3d Amodul_ee_tang = Dp_[TOSS]*Xdot  + Kp_[TOSS]*(X - Xstar); 					//this->computeModulatedAcceleration(Km, Dm, X, Xdot, Xstar);
		Eigen::Vector3d Aretrac_ee     = Dp_[RETRACT]*Xdot + Kp_[RETRACT]*(X - Xretr); 			// DS for retracting after the tossing position

		// get the modulated motion (out_motion: Acceleration)
		Out_motion.head(3) = (1.0-a_retract_)*this->compute_modulated_motion(activation, BasisQ, Areach_ee, Amodul_ee_norm, Amodul_ee_tang) + a_retract_ * Aretrac_ee;
		// get angular motion
		Out_motion.tail(3) = (1.0-a_retract_)*this->compute_angular_motion(coupling_, w_H_ce, Omega, w_H_de, Ko_[REACH], Do_[REACH], this->is2ndOrder_) 
												+   a_retract_   *this->compute_angular_motion(coupling_, w_H_ce, Omega, w_H_re, Ko_[RETRACT], Do_[RETRACT], this->is2ndOrder_);

	}
	else{
		Eigen::Vector3d Xstar = (1.0 - a_normal_) * Xb + a_normal_ * (X - Kp_[TOSS].inverse()*Vdtoss);
		// 
		Eigen::Vector3d Areach_ee      = Kp_[REACH]*(X - Xb); 					// DS for approaching the tossing position
		Eigen::Vector3d Amodul_ee_norm = Kp_[TOSS]*(X - Xb); 						// Modulated DS that aligned  the EE with the desired velocity
		Eigen::Vector3d Amodul_ee_tang = Kp_[TOSS]*(X - Xstar); 				// this->computeModulatedAcceleration(Km, Dm, X, Xdot, Xstar);
		Eigen::Vector3d Aretrac_ee     = Kp_[RETRACT]*(X - Xretr); 			// DS for retracting after the tossing position
		
		// get the modulated motion (out_motion: Velocity)
		Out_motion.head(3) = (1.0-a_retract_)*this->compute_modulated_motion(activation, BasisQ, Areach_ee, Amodul_ee_norm, Amodul_ee_tang) + a_retract_ * Aretrac_ee;
		// get angular motion
		Out_motion.tail(3) = (1.0-a_retract_)*this->compute_angular_motion(coupling_, w_H_ce, Omega, w_H_de, Ko_[REACH], Do_[REACH], this->is2ndOrder_) 
							+   a_retract_   *this->compute_angular_motion(coupling_, w_H_ce, Omega, w_H_re, Ko_[RETRACT], Do_[RETRACT], this->is2ndOrder_);
	}

	// return true;
	return Out_motion;
	
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

	Lambda.block<1,1>(1,0) = activation*( BasisQ.col(1).transpose() * Amodul_ee_norm *Beta_j(0) );
	Lambda.block<1,1>(1,1) = activation*( BasisQ.col(1).transpose() * Amodul_ee_norm *Beta_j(1) )  + (1.0-activation)*Eigen::MatrixXd::Identity(1,1);
	Lambda.block<1,1>(1,2) = activation*( BasisQ.col(1).transpose() * Amodul_ee_norm *Beta_j(2) );

	Lambda.block<1,1>(2,0) = activation*( BasisQ.col(2).transpose() * Amodul_ee_norm *Beta_j(0) );
	Lambda.block<1,1>(2,1) = activation*( BasisQ.col(2).transpose() * Amodul_ee_norm *Beta_j(1) );
	Lambda.block<1,1>(2,2) = activation*( BasisQ.col(2).transpose() * Amodul_ee_norm *Beta_j(2) )  + (1.0-activation)*Eigen::MatrixXd::Identity(1,1);

	// computing the modulated second order DS (translation)
	return BasisQ * Lambda * BasisQ.transpose() * Areach_ee; 
}


Eigen::Vector3d throwingDS::compute_angular_motion(double coupling_, Eigen::Matrix4d w_H_c, Eigen::Vector3d Omega, Eigen::Matrix4d w_H_d, Eigen::Matrix3d Ko, Eigen::Matrix3d Do, bool is2ndOrder){

	// Rotation motion
	//----------------
	Eigen::Matrix3d w_R_c   = w_H_c.block<3,3>(0,0);	// current orientation of the end effector
	Eigen::Matrix3d w_R_d   = w_H_d.block<3,3>(0,0);	// desired orientation of the end effector
	// coupling_ orientation task to position through spherical interpolation of the orientation 
	Eigen::Matrix3d w_R_d_t = Utils<double>::getCombinedRotationMatrix(coupling_, w_R_c, w_R_d); 
	// relative transformation between desired and current frame
	Eigen::Matrix3d d_R_c_t = w_R_d_t.transpose() * w_R_c;

	// 3D Orientation Jacobian 
	Eigen::Matrix3d jacMuTheta = Utils<double>::getMuThetaJacobian(d_R_c_t) * w_R_c.transpose();

	if(is2ndOrder){
		// approaximation of the acceleration (neglecting  -jacMuTheta_dot * Omega)
		return jacMuTheta.inverse() * ( Do*jacMuTheta*Omega + Ko * Utils<double>::getOrientationErrorCur2Des(d_R_c_t));
	}
	else{
		return jacMuTheta.inverse() * (Ko * Utils<double>::getOrientationErrorCur2Des(d_R_c_t));
	}

}

Eigen::MatrixXd throwingDS::createOrthonormalMatrixFromVector(Eigen::VectorXd inVec)
{
  //
  int n = inVec.rows();
  Eigen::MatrixXd basis = Eigen::MatrixXd::Random(n,n); 
  basis.col(0) = 1./inVec.norm() * inVec;

  assert(basis.rows() == basis.cols());
  uint dim = basis.rows();
  basis.col(0).normalize();
  for(uint i=1;i<dim;i++){
      for(uint j=0;j<i;j++)
          basis.col(i) -= basis.col(j).dot(basis.col(i))*basis.col(j);
      basis.col(i).normalize();
  }

  if (basis.rows() == 3){
  	Eigen::Vector3d u = basis.col(0);
  	Eigen::Vector3d v = basis.col(1);
  	Eigen::Vector3d w = u.cross(v);
  	basis.col(2) = w;
  }

  return basis;
} 

bool throwingDS::set_gains(int taskId, int motionId, Eigen::Matrix3d K, Eigen::Matrix3d D){

	if(motionId == TRANSLATION ){
		Kp_[taskId] = K;
		Dp_[taskId] = D;
	}
	else{
		Ko_[taskId] = K;
		Do_[taskId] = D;
	}

	return true;	
}

bool throwingDS::set_modulationParameters(double new_modul_param[]){

	rho_        = new_modul_param[0];
	range_norm_ = new_modul_param[1];
	range_tang_ = new_modul_param[2];

	return true;
}

bool throwingDS::set_toss_linear_velocity(Eigen::Vector3d newLinVel){
	//
	v_toss_ = newLinVel;
	BasisQp_ = this->createOrthonormalMatrixFromVector(v_toss_);
	return true;
}

bool throwingDS::set_toss_angular_velocity(Eigen::Vector3d newAngVel){
	//
	w_toss_  = newAngVel;
	BasisQo_ = this->createOrthonormalMatrixFromVector(w_toss_);
	return true;
}

bool throwingDS::set_toss_pose(Eigen::Vector3d new_releasePos, Eigen::Vector4d new_releaseOrient){
	//
	w_H_de_ = Utils<double>::pose2HomoMx(new_releasePos, new_releaseOrient);
	return true;
}

bool throwingDS::set_rest_pose(Eigen::Vector3d new_restPos, Eigen::Vector4d new_restOrient){
	//
	w_H_re_ = Utils<double>::pose2HomoMx(new_restPos, new_restOrient);
	return true;
}

bool throwingDS::get_release_flag(){
	return release_flag_;
}
