

#include "throwingMotionGeneration.h"


Eigen::MatrixXd createOrthonormalMatrixFromVector(Eigen::VectorXd inVec)
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


throwingMotionGeneration::throwingMotionGeneration( ros::NodeHandle &nh,
		                double frequency,
		                std::string input_topic_ds_model_path,
		                std::string input_topic_desired_ee_pose,
		                std::string input_topic_desired_toss_velo,
		                std::string input_topic_retract_pose,
		                std::string input_topic_ee_pose,
		                std::string input_topic_ee_velo,
		                std::string output_topic_ee_velo,
		                std::string output_topic_ee_accel,
		                std::string dsType
		                )
		: nh_(nh),
		loop_rate_(frequency),
		input_topic_ds_model_path_(input_topic_ds_model_path),
		input_topic_desired_ee_pose_(input_topic_desired_ee_pose),
		input_topic_desired_toss_velo_(input_topic_desired_toss_velo),
		input_topic_retract_ee_pose_(input_topic_retract_pose),
		input_topic_ee_pose_(input_topic_ee_pose),
		input_topic_ee_velo_(input_topic_ee_velo),
		output_topic_ee_velo_(output_topic_ee_velo),
		output_topic_ee_accel_(output_topic_ee_accel), 
		ds_type_(dsType) {
	
	ROS_INFO_STREAM("Motion generator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}

throwingMotionGeneration::~throwingMotionGeneration(){
	 ROS_INFO_STREAM("In destructor.. throwing DS was tarminated! ");
}

bool throwingMotionGeneration::init(){
	//
	x_.setZero();
	q_.setZero();
	xd_.setZero();
	qd_.setZero();
	vd_.setZero();
	xr_.setZero();
	qr_.setZero();
	omegad_.setZero();
	V_ee_.setZero();
	V_ee_d_.setZero();
	A_ee_d_.setZero();
	wRb_.setIdentity();      
	w_H_ce_.setIdentity();
	w_H_de_.setIdentity();
	w_H_re_.setIdentity();
	BasisQ_.setIdentity();
	//
	modulRegion_[0] = 0.150;
	modulRegion_[1] = 0.03;
	modulRegion_[2] = 0.03;
	//
	toolOffsetFromEE_.setZero();
	is2ndOrder_ = false;

	
	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	if (!InitializeDS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}


	return true;
}
void throwingMotionGeneration::run(){

	while (nh_.ok()) {

        generate_motion();
        Publish_desired_motion();

		ros::spinOnce();
		loop_rate_.sleep();
	}
    nh_.shutdown();

}

bool throwingMotionGeneration::InitializeROS(){

    sub_desired_ee_pose_	= 	nh_.subscribe( input_topic_desired_ee_pose_ , 1000, &throwingMotionGeneration::UpdateTargetPose, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_desired_toss_velo_  =  	nh_.subscribe( input_topic_desired_toss_velo_ , 1000, &throwingMotionGeneration::UpdateTargetVelocity, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_retract_ee_pose_	= 	nh_.subscribe( input_topic_retract_ee_pose_ , 1000, &throwingMotionGeneration::UpdateRetractPose, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_ee_pose_ 			=	nh_.subscribe( input_topic_ee_pose_ , 1000, &throwingMotionGeneration::UpdateEEPose, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_ee_velo_ 			=  	nh_.subscribe( input_topic_ee_velo_ , 1000, &throwingMotionGeneration::UpdateEEVelocity, this, ros::TransportHints().reliable().tcpNoDelay());
	    
    pub_desired_twist_        = nh_.advertise<geometry_msgs::Twist>(output_topic_ee_velo_, 1);    
	pub_desired_acceleration_ = nh_.advertise<geometry_msgs::Twist>(output_topic_ee_accel_, 1);

	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The Motion generator is ready.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}
bool throwingMotionGeneration::InitializeDS(){
	//

	// select the type of DS
	// =====================
	char c = ds_type_[1];
	switch(c){

		// Analytic DS
		//------------
		case '0': {							// First order
			getDsParamFromYmlFile();
			is2ndOrder_ = false;
		} break;

		case '1': {							// Second order
			getDsParamFromYmlFile();
			is2ndOrder_ = true;
		} break;

		// Learned DS
		//------------
		case '2': {							// First order
			getFirstOrderDsParamFromGMM(); 							// TO BE COMPLETED
			is2ndOrder_ = false;
		} break;

		case '4': {							// Second order
			getSecondOrderDsParamFromGMM(); 						// TO BE COMPLETED
			is2ndOrder_ = true;
		} break;
	}

	
	dsThrowing2.init(modulRegion_, Kp_, Dp_, Ko_, Do_, is2ndOrder_);

	return true;
}
void throwingMotionGeneration::generate_motion() {
	//
	BasisQ_ = createOrthonormalMatrixFromVector(vtoss_);
	bool release_flag;
	Vector6d motion = dsThrowing2.generate_throwing_motion(w_H_ce_,  V_ee_, w_H_de_, w_H_re_, BasisQ_, vtoss_, release_flag);
	
	if(!is2ndOrder_) {
		V_ee_d_ = motion;
	}
	else {
		A_ee_d_ = motion;
	}
}

void throwingMotionGeneration::UpdateEEPose(const geometry_msgs::Pose::ConstPtr& msg){
	// Update end effecotr pose (position+orientation)
	x_ << 	msg->position.x,	msg->position.y, 	msg->position.z;						// position
	q_ << 	msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;	// orientation
	//
	wRb_   = Utils<double>::quaternionToRotationMatrix(q_);
	x_     = x_ + wRb_ * toolOffsetFromEE_;
	w_H_ce_.block<3,1>(0,3) = x_;
	w_H_ce_.block<3,3>(0,0) = wRb_;
}

void throwingMotionGeneration::UpdateTargetPose(const geometry_msgs::Pose::ConstPtr& msg){
	// Update end effecotr pose (position+orientation)
	xd_ << 	msg->position.x,	msg->position.y, 	msg->position.z;						// position
	qd_ << 	msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;	// orientation

	w_H_de_.block<3,1>(0,3) = xd_;
	w_H_de_.block<3,3>(0,0) = Utils<double>::quaternionToRotationMatrix(qd_);
}

void throwingMotionGeneration::UpdateRetractPose(const geometry_msgs::Pose::ConstPtr& msg){
	// Update end effecotr pose (position+orientation)
	xr_ << 	msg->position.x,	msg->position.y, 	msg->position.z;						// position
	qr_ << 	msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;	// orientation

	w_H_re_.block<3,1>(0,3) = xr_;
	w_H_re_.block<3,3>(0,0) = Utils<double>::quaternionToRotationMatrix(qr_);
}

void throwingMotionGeneration::UpdateEEVelocity(const geometry_msgs::Twist::ConstPtr& msg){
	V_ee_ << msg->linear.x,		msg->linear.y, 		msg->linear.z, 		// EE linear velocity
			 msg->angular.x, 	msg->angular.y, 	msg->angular.z;		// EE angular velocity
}

void throwingMotionGeneration::UpdateTargetVelocity(const geometry_msgs::Twist::ConstPtr& msg){
	vtoss_ << msg->linear.x,		msg->linear.y, 		msg->linear.z; 		// tossing linear velocity
}


void throwingMotionGeneration::Publish_desired_motion(){

	geometry_msgs::Twist msgDesiredAcceleration;
    msgDesiredAcceleration.linear.x  = A_ee_d_(0);
    msgDesiredAcceleration.linear.y  = A_ee_d_(1);
    msgDesiredAcceleration.linear.z  = A_ee_d_(2);
    msgDesiredAcceleration.angular.x = A_ee_d_(3);
    msgDesiredAcceleration.angular.y = A_ee_d_(4);
    msgDesiredAcceleration.angular.z = A_ee_d_(5);

    geometry_msgs::Twist msgDesiredTwist;
    msgDesiredTwist.linear.x  = V_ee_d_(0);
    msgDesiredTwist.linear.y  = V_ee_d_(1);
    msgDesiredTwist.linear.z  = V_ee_d_(2);
    msgDesiredTwist.angular.x = V_ee_d_(3);
    msgDesiredTwist.angular.y = V_ee_d_(4);
    msgDesiredTwist.angular.z = V_ee_d_(5);

	pub_desired_twist_.publish(msgDesiredTwist);
}


bool throwingMotionGeneration::getDsParamFromYmlFile(){
	//
	std::vector<double> Gain_0, Gain_1, Gain_2;
	// ==================
	// Position task
	// ==================
	if (nh_.getParam("Kp_0", Gain_0) && nh_.getParam("Kp_1", Gain_1) && nh_.getParam("Kp_2", Gain_2)) {

		Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Gn0(Gain_0.data(),3, 3);
		Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Gn1(Gain_1.data(),3, 3);
		Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Gn2(Gain_2.data(),3, 3);

 		Kp_[0] = Gn0; 	Kp_[1] = Gn1; 	Kp_[2] = Gn2;
 	}
 	else{
		ROS_ERROR("Couldn't retrieve the DS stiffness parameters for translation. ");
		return false;
	}
	//
	Gain_0.clear();  Gain_1.clear();  Gain_2.clear(); 
	//
	if (nh_.getParam("Dp_0", Gain_0) && nh_.getParam("Dp_1", Gain_1) && nh_.getParam("Dp_2", Gain_2)) {

		Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Gn0(Gain_0.data(),3, 3);
		Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Gn1(Gain_1.data(),3, 3);
		Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Gn2(Gain_2.data(),3, 3);

 		Dp_[0] = Gn0; 	Dp_[1] = Gn1; 	Dp_[2] = Gn2;
 	}
 	else{
		ROS_ERROR("Couldn't retrieve the DS damping parameters for translation. ");
		return false;
	}

	// ==================
	// Orientation task
	// ==================
	Gain_0.clear();  Gain_1.clear();  Gain_2.clear(); 
	if (nh_.getParam("Ko_0", Gain_0) && nh_.getParam("Ko_1", Gain_1) && nh_.getParam("Ko_2", Gain_2)) {

		Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Gn0(Gain_0.data(),3, 3);
		Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Gn1(Gain_1.data(),3, 3);
		Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Gn2(Gain_2.data(),3, 3);

 		Ko_[0] = Gn0; 	Ko_[1] = Gn1; 	Ko_[2] = Gn2;
 	}
 	else{

		ROS_ERROR("Couldn't retrieve the DS stiffness parameters for Orientation. ");
		return false;
	}
	//
	Gain_0.clear();  Gain_1.clear();  Gain_2.clear(); 
	//
	if (nh_.getParam("Do_0", Gain_0) && nh_.getParam("Do_1", Gain_1) && nh_.getParam("Do_2", Gain_2)) {

		Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Gn0(Gain_0.data(),3, 3);
		Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Gn1(Gain_1.data(),3, 3);
		Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Gn2(Gain_2.data(),3, 3);

 		Do_[0] = Gn0; 	Do_[1] = Gn1; 	Do_[2] = Gn2;
 	}
 	else{
		ROS_ERROR("Couldn't retrieve the DS damping parameters for Orientation. ");
		return false;
	}

}


bool throwingMotionGeneration::getFirstOrderDsParamFromGMM(){
	return true;
}
bool throwingMotionGeneration::getSecondOrderDsParamFromGMM(){
	return true;
}