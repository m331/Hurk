
#include <XMLConfiguration.h>
#include <threemxl/dxlassert.h>
#include <arm/arm.h>

const double Arm::default_speed_ = 0.3;
const double Arm::default_accel_ = 0.5;


const double Arm::elbow_upper_limit_ = 2.3;
const double Arm::elbow_lower_limit_ = -2.1;

const double Arm::shoulder_length_ = 0.3;
const double Arm::elbow_length_ = 0.3;
const double Arm::lambda_ = 0.001;

double  p1 =       -2.24 ;
double 	p2 =       16.59 ;
double 	q1 =     0.09055 ;

#define CLIP(x, l, u) ((x)<(l))?(l):((x)>(u))?(u):(x)

Arm::~Arm()
{
	ROS_ERROR("Shutting down Arm ");
	initablePosition();
	delete shoulder_motor_;
	delete elbow_motor_;
	
	ROS_ERROR("Shutting down done");
	nh_.shutdown();
}
void Arm::init()
{
	ROS_INFO("Initializing Arm");

	ik_counter_ = -1;
	// Read parameters
	std::string motor_port_name, motor_config_name;

	ROS_ASSERT(nh_.getParam("motor_port", motor_port_name));
	ROS_ASSERT(nh_.getParam("motor_config", motor_config_name));

	// Subscript to velocity command topic
	vel_sub_ = nh_.subscribe("cmd_vel", 1, &Arm::velocityCallback, this);
	// Subscript to position command topic
	pos_sub_ = nh_.subscribe("cmd_pos", 1, &Arm::positionCallback, this);

	// Service calls for predef pos and set height
	predef_pos_service_ = nh_.advertiseService("predef_pos_service",&Arm::predefCallback, this);
	set_height_service_ = nh_.advertiseService("setheight", &Arm::setheightCallback, this);

	// Subscribe to the ikvel commands topic
	ikvel_sub_ = nh_.subscribe("ik_vel", 1, &Arm::ikvelCallback, this);

	// Advertise on the status topic
	status_pub_ = nh_.advertise<arm::ArmStatus>("status",1);
	joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

	// Load motor configuration
	CXMLConfiguration motor_config_xml;
	ROS_ASSERT(motor_config_xml.loadFile(motor_config_name));

	CDxlConfig motor_config_shoulder;
	motor_config_shoulder.readConfig(motor_config_xml.root().section("shoulder"));
	shoulder_motor_ = new C3mxlROS(motor_port_name.c_str());
	shoulder_motor_->setConfig(&motor_config_shoulder);

	CDxlConfig motor_config_elbow;
	motor_config_elbow.readConfig(motor_config_xml.root().section("elbow"));
	elbow_motor_ = new C3mxlROS(motor_port_name.c_str());
	elbow_motor_->setConfig(&motor_config_elbow);

	ros::Rate init_rate(1);

	// Initialize elbow motor
	while (ros::ok() && elbow_motor_->init() != DXL_SUCCESS)
	{
		ROS_WARN_ONCE("Couldn't connect to elbow motor, will continue trying every second");
		init_rate.sleep();
	}

	// Initialize shoulder motor
	while (ros::ok() && shoulder_motor_->init() != DXL_SUCCESS)
	{
		ROS_WARN_ONCE("Couldn't connect to shoulder motor, will continue trying every second");
		init_rate.sleep();
	}

	// Perform motor initialization

	int sstat;
	int estat;
	ros::Rate loop_rate(10);

	// Initialize Shoulder
	DXL_SAFE_CALL(shoulder_motor_->set3MxlMode(EXTERNAL_INIT));
	DXL_SAFE_CALL(shoulder_motor_->setSpeed(0.3));
	DXL_SAFE_CALL(shoulder_motor_->setAcceleration(1));
	DXL_SAFE_CALL(shoulder_motor_->setTorque(0.1));


	while (ros::ok())
	{
		// Check for status
		DXL_SAFE_CALL(shoulder_motor_->getStatus());

		sstat = shoulder_motor_->presentStatus();


		if (sstat != M3XL_STATUS_INITIALIZE_BUSY)
		{
			break;
		}

		loop_rate.sleep();
	}

	if (sstat != M3XL_STATUS_INIT_DONE)
	{
		ROS_FATAL_STREAM("Couldn't initialize shoulder: " << shoulder_motor_->translateErrorCode(sstat));
		ROS_ISSUE_BREAK();
	}

	// Initialize elbow
	DXL_SAFE_CALL(elbow_motor_->set3MxlMode(EXTERNAL_INIT));
	DXL_SAFE_CALL(elbow_motor_->setAcceleration(1));
	DXL_SAFE_CALL(elbow_motor_->setSpeed(-0.3));
	DXL_SAFE_CALL(elbow_motor_->setTorque(-0.1));
	while (ros::ok())
	{
		DXL_SAFE_CALL(elbow_motor_->getStatus());

		estat = elbow_motor_->presentStatus();

		if (estat != M3XL_STATUS_INITIALIZE_BUSY)
		{
			break;
		}

		loop_rate.sleep();
	}
	// Set to preconfigured mode
	DXL_SAFE_CALL(shoulder_motor_->set3MxlMode(motor_config_shoulder.m3mxlMode));
	DXL_SAFE_CALL(elbow_motor_->set3MxlMode(motor_config_elbow.m3mxlMode));

	mode_pos_ = true;
	homePosition();
}

void Arm::homePosition()
{
	if(!mode_pos_){

		shoulder_motor_->set3MxlMode(POSITION_MODE);
		shoulder_motor_->get3MxlMode();

		elbow_motor_->set3MxlMode(POSITION_MODE);
		elbow_motor_->get3MxlMode();

		mode_pos_ = true;
	}

	motor_command_.shoulder.position = 0.3;
	motor_command_.elbow.position = -1.57;

	shoulder_motor_->setPos(motor_command_.shoulder.position,default_speed_);
	elbow_motor_->setPos(motor_command_.elbow.position,default_speed_);

}


void Arm::navPosition()
{
	if(!mode_pos_){

		shoulder_motor_->set3MxlMode(POSITION_MODE);
		shoulder_motor_->get3MxlMode();

		elbow_motor_->set3MxlMode(POSITION_MODE);
		elbow_motor_->get3MxlMode();

		mode_pos_ = true;
	}

	motor_command_.shoulder.position = 0.8;
	motor_command_.elbow.position = -0.4;

	shoulder_motor_->setPos(motor_command_.shoulder.position,default_speed_);
	elbow_motor_->setPos(motor_command_.elbow.position,default_speed_);

}

void Arm::initablePosition()
{
	if(!mode_pos_){

		shoulder_motor_->set3MxlMode(POSITION_MODE);
		shoulder_motor_->get3MxlMode();

		elbow_motor_->set3MxlMode(POSITION_MODE);
		elbow_motor_->get3MxlMode();

		mode_pos_ = true;
	}

	motor_command_.shoulder.position = -0.2;
	motor_command_.elbow.position = -1.2;

	shoulder_motor_->setPos(motor_command_.shoulder.position,default_speed_);
	elbow_motor_->setPos(motor_command_.elbow.position,default_speed_);
}

bool Arm::setikVel(Eigen::Vector2f dX)
{
	Eigen::Matrix2f J;
	float th_s, th_e;
	th_s = status_msg_.shoulder.position;
	th_e = status_msg_.elbow.position;
	// Creating the Jacobian matrix
	J << shoulder_length_*cos(th_s), elbow_length_*cos(th_e),
			shoulder_length_*sin(th_s), elbow_length_*sin(th_e);

	//	// Straight forward inverse of the matrix
	//	dTh = J.inverse()*dX;

	//Handling singularities (the DLS method)
	Eigen::Vector2f dTh;
	Eigen::Matrix2f temp, eye;
	eye << 1,0,0,1;
	temp = (J*J.transpose() + (pow(lambda_,2)*eye));
	dTh = J.transpose()*temp.inverse()*dX;

	//	ROS_INFO("Shoulder: %f\t Elbow: %f", dTh(0), dTh(1));

	dTh(0) = CLIP(dTh(0), -0.3, 0.3);
	dTh(1) = CLIP(dTh(1), -0.3, 0.3);

	motor_command_.shoulder.speed = dTh(0);
	motor_command_.elbow.speed = dTh(1);

	if(mode_pos_)
	{
		shoulder_motor_->set3MxlMode(SPEED_MODE);
		shoulder_motor_->get3MxlMode();

		elbow_motor_->set3MxlMode(SPEED_MODE);
		elbow_motor_->get3MxlMode();

		mode_pos_ = false;
	}


	if(elbow_rel_pos_ > elbow_upper_limit_) {
		motor_command_.elbow.speed = -1.5 * fabs(motor_command_.shoulder.speed);
	}
	if(elbow_rel_pos_ < elbow_lower_limit_) {
		motor_command_.elbow.speed = 1.5 * fabs(motor_command_.shoulder.speed);
	}

	shoulder_motor_->setSpeed(motor_command_.shoulder.speed);
	elbow_motor_->setSpeed(motor_command_.elbow.speed);

	//	ROS_INFO("Shoulder: %f\t Elbow: %f", dTh(0), dTh(1));
	ik_counter_ = 15;

	return true;
}
void Arm::ikvelCallback(const geometry_msgs::Point::ConstPtr &msg)
{
	// Considering only two DoF in the arm (shoulder and elbow)
	// msg->x : dx, msg->z: dz
	Eigen::Vector2f dX;
	dX << -msg->x, msg->z;

	setikVel(dX);

}



bool Arm::setheightCallback(arm::SetHeight::Request &req, arm::SetHeight::Response &res)
{
	double start_time = ros::Time::now().toSec();
	bool tf_available = false;
	ROS_INFO("Setting Arm height to: %f", req.height);

	while((ros::Time::now().toSec() - start_time) < req.time_limit)
	{

		ros::Time time_now = ros::Time::now();
		try{
			tf_available = tf_listener_.waitForTransform("/gripper", "/base_link",time_now, ros::Duration(2.0));
		}
		catch(tf::TransformException &te)
		{
			ROS_ERROR("SetHeight Tf Exception %s", te.what());
		}

		bool height_reached = false;
		if(tf_available && !height_reached)
		{
			tf::StampedTransform gripper_base;
			tf_listener_.lookupTransform("/gripper", "/base_link", ros::Time(0),gripper_base);

			double height_error = req.height + gripper_base.getOrigin().getZ() ;

			ros::Rate update_rate(50);
			while(fabs(height_error) > 0.01)
			{
				Eigen::Vector2f dX;
				dX << 0.0 , copysign(0.04, height_error);

				setikVel(dX);

				//statusPublish();

				update_rate.sleep();
				tf_listener_.lookupTransform("/gripper", "/base_link", ros::Time(0),gripper_base);
				height_error = req.height + gripper_base.getOrigin().getZ() ;

				if((ros::Time::now().toSec() - start_time) > req.time_limit)
				{
					ROS_INFO("Time up for setting height");
					res.reached = false;
					return true;
				}
			}
			res.reached = true;
			return true;
		}
	}
	return false;
}

bool Arm::predefCallback(arm::PredefPos::Request &req, arm::PredefPos::Response &res)
{
	switch(req.predef_pos){
	case 1:
		navPosition();
		break;
	case -1:
		initablePosition();
		break;
	default:
	case 0:
		homePosition();
		break;
	}

	//		ROS_INFO("Checking time and joint limits");
	// Checking if the joints have reached their requested position
	int sstat, estat;
	DXL_SAFE_CALL(shoulder_motor_->getStatus());
	sstat = shoulder_motor_->presentStatus();

	DXL_SAFE_CALL(elbow_motor_->getStatus());
	estat = elbow_motor_->presentStatus();

	ros::Rate poscheckrate(20);
	double start_time = ros::Time::now().toSec();
	//		ROS_INFO("Status: %d, %d, %d, %d", sstat, estat);

	while ((sstat!= M3XL_STATUS_POS_MODE_DONE) || (estat!=M3XL_STATUS_POS_MODE_DONE))
	{
		DXL_SAFE_CALL(shoulder_motor_->getStatus());
		sstat = shoulder_motor_->presentStatus();
		DXL_SAFE_CALL(elbow_motor_->getStatus());
		estat = elbow_motor_->presentStatus();

		if((ros::Time::now().toSec() - start_time) > req.time)
		{
			res.status = false;
			return true;
		}

		poscheckrate.sleep();
	}

	res.status = true;
	return true;
}

void Arm::spin()
{
	ROS_INFO("Spinning");

	ros::Rate r(30);

	while(ros::ok())
	{

		ros::spinOnce();
		//statusPublish();
		if(ik_counter_ > 0)
			ik_counter_ --;
		else if(ik_counter_ == 0)
		{
			ik_counter_ = -1;
			shoulder_motor_->setSpeed(0);
			elbow_motor_->setSpeed(0);
		}

		r.sleep();
	}

	ros::spin();
}



void Arm::velocityCallback(const arm::Jolt4DOF::ConstPtr &msg)
{
	if(mode_pos_)
	{
		shoulder_motor_->set3MxlMode(SPEED_MODE);
		shoulder_motor_->get3MxlMode();

		elbow_motor_->set3MxlMode(SPEED_MODE);
		elbow_motor_->get3MxlMode();

		mode_pos_ = false;
	}

	if(isnan(msg->elbow.speed) || isnan(msg->shoulder.speed) || isinf(msg->elbow.speed) || isinf(msg->shoulder.speed))
	{
		ROS_WARN("The arm cannot go at GOD speed. Sorry");
		return;
	}



	motor_command_ = *msg;

	motor_command_.elbow.speed = CLIP(msg->elbow.speed, -0.3, 0.3);
	motor_command_.shoulder.speed = CLIP(msg->shoulder.speed, -0.3, 0.3);

	if(elbow_rel_pos_ > elbow_upper_limit_) {
		motor_command_.elbow.speed = -1.5 * fabs(motor_command_.shoulder.speed);
	}
	if(elbow_rel_pos_ < elbow_lower_limit_) {
		motor_command_.elbow.speed = 1.5 * fabs(motor_command_.shoulder.speed);
	}

	shoulder_motor_->setSpeed(motor_command_.shoulder.speed);
	elbow_motor_->setSpeed(motor_command_.elbow.speed);

	ROS_DEBUG_STREAM("Arm [shoulder,elbow] velocity set to ["
			<< msg->shoulder << ", " << msg->elbow  << "]");
}

void Arm::positionCallback(const arm::Jolt4DOF::ConstPtr &msg)
{
	if(!mode_pos_){

		shoulder_motor_->set3MxlMode(POSITION_MODE);
		shoulder_motor_->get3MxlMode();

		elbow_motor_->set3MxlMode(POSITION_MODE);
		elbow_motor_->get3MxlMode();

		mode_pos_ = true;
	}

	motor_command_ = *msg;
	if(msg->shoulder.speed == 0) {
		motor_command_.shoulder.speed = default_speed_;
		motor_command_.elbow.speed = default_speed_;
	}

	shoulder_motor_->setPos(motor_command_.shoulder.position,motor_command_.shoulder.speed);
	elbow_motor_->setPos(motor_command_.elbow.position,motor_command_.elbow.speed);


	ROS_DEBUG_STREAM("Arm [shoulder,elbow] position set to ["
			<< msg->shoulder << ", " << msg->elbow << "]");
}

/*
void Arm::statusPublish()
{
	
	status_msg_.shoulder = Arm::getStatus(shoulder_motor_);
	status_msg_.elbow = getStatus(elbow_motor_);
	//status_msg_.wrist_pitch = getStatus(wrist_pitch_motor_);
	//status_msg_.wrist_roll = getStatus(wrist_roll_motor_);

	status_pub_.publish(status_msg_);

	sensor_msgs::JointState joint_state;

	//update joint_state
	joint_state.header.stamp = ros::Time::now();

	shoulder_abs_pos_ 		= status_msg_.shoulder.position;
	shoulder_rel_pos_		= shoulder_abs_pos_;
	elbow_abs_pos_ 			= status_msg_.elbow.position;
	elbow_rel_pos_			= elbow_abs_pos_ - shoulder_abs_pos_;
	//wrist_pitch_abs_pos_ 	= status_msg_.wrist_pitch.position;
	//wrist_pitch_rel_pos_	= wrist_pitch_abs_pos_ - elbow_abs_pos_ - 1.57;
	//wrist_roll_abs_pos_ 	= status_msg_.wrist_roll.position;

	joint_state.name.resize(4);
	joint_state.position.resize(4);
	joint_state.name[0] ="shoulder";
	joint_state.position[0] = shoulder_rel_pos_;
	joint_state.name[1] ="elbow";
	joint_state.position[1] = elbow_rel_pos_;
	//joint_state.name[2] ="wrist_roll_joint";
	//joint_state.position[2] = wrist_roll_abs_pos_;
	//joint_state.name[3] ="wrist_pitch_joint";
	//joint_state.position[3] = wrist_pitch_rel_pos_;

	joint_pub_.publish(joint_state);

	if(mode_pos_) {
		bool bool_stop = false;
		if(	elbow_rel_pos_ > elbow_upper_limit_ ||
				elbow_rel_pos_ < elbow_lower_limit_ ||
				//wrist_pitch_rel_pos_ > wrist_pitch_upper_limit_ ||
				//wrist_pitch_rel_pos_ < wrist_pitch_lower_limit_)
			bool_stop = true;

		if(bool_stop) {
			shoulder_motor_->set3MxlMode(SPEED_MODE);
			shoulder_motor_->get3MxlMode();

			elbow_motor_->set3MxlMode(SPEED_MODE);
			elbow_motor_->get3MxlMode();

			//wrist_pitch_motor_->set3MxlMode(SPEED_MODE);
			//wrist_pitch_motor_->get3MxlMode();

			mode_pos_ = false;
			shoulder_motor_->setSpeed(0.0);
			elbow_motor_->setSpeed(0.0);
			//wrist_pitch_motor_->setSpeed(0.0);

			bool_stop = false;
			ROS_ERROR("Position callback resulted in a floating limit");
		}
	}


}*/

/*
Arm::MotorStatus Arm::getStatus(C3mxlROS *motor)
{
	motor->getState();
	motor->getStatus();

	Arm::MotorStatus stat;
	stat.speed = motor->presentSpeed();
	stat.position = motor->presentPos();
	stat.status = motor->presentStatus();

	return stat;
}
*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm");

	Arm arm;
	arm.spin();

	return 0;
}


