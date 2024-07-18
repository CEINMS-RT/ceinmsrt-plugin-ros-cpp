#include <PluginROS.h>

PluginROS::PluginROS() : record_(false), outDirectory_("output"), threadStop_(false)
{

}

PluginROS::~PluginROS()
{
	delete torquePublisher_, positionSub_, rosNode_, muscleForcePublisher_, muscleFiberLengthPublisher_,
	 muscleFiberVelocityPublisher_, muscleForceActivePublisher_, muscleForcePassivePublisher_, tendonStrainPublisher_, torqueSub_;
}

void PluginROS::init(std::string& executableXMLFileName)
{
	ExecutionXmlReader executionCfg(executableXMLFileName);
	init(atoi(executionCfg.getComsumerPort().c_str()));
}

void PluginROS::init(int portno)
{
	timeStamp_ = rtb::getTime();
	std::map<std::string, std::string> rosArg;
	ros::init(rosArg, "nms_model", ros::init_options::NoSigintHandler);
	rosNode_ = new ros::NodeHandle;
	std::cout<< muscleName_.size() << std::endl;
	torquePublisher_ = new ros::Publisher(rosNode_->advertise<sensor_msgs::JointState>("joint_state", dofName_.size()));
	muscleForcePublisher_ = new ros::Publisher(rosNode_->advertise<sensor_msgs::JointState>("muscle_state", muscleName_.size()));
	muscleForceActivePublisher_ = new ros::Publisher(rosNode_->advertise<sensor_msgs::JointState>("muscle_state_active", muscleName_.size()));
	muscleForcePassivePublisher_ = new ros::Publisher(rosNode_->advertise<sensor_msgs::JointState>("muscle_state_passive", muscleName_.size()));
	tendonStrainPublisher_ = new ros::Publisher(rosNode_->advertise<sensor_msgs::JointState>("tendon_strain", muscleName_.size()));
	positionSub_ = new ros::Subscriber(rosNode_->subscribe("joint_position", dofName_.size(), &PluginROS::JointPositionCallBackROS, this));
	torqueSub_ = new ros::Subscriber(rosNode_->subscribe("joint_torque_ext", dofName_.size(), &PluginROS::JointTorqueExtCallBackROS, this));
	std::cout << "publisher created" << std::endl;

	if (record_)
	{
		logger_ = new OpenSimFileLogger<double>(outDirectory_);
		logger_->addLog(Logger::IK, dofName_);
		logger_->addLog(Logger::ID, dofName_);
		logger_->addLog(Logger::RandomSignal);
	}
}

void PluginROS::JointPositionCallBackROS(const sensor_msgs::JointState& msg)
{
	for (std::vector<std::string>::const_iterator it = msg.name.begin(); it != msg.name.end(); it++)
		dataAngleXsens_[*it] = msg.position.at(std::distance<std::vector<std::string>::const_iterator>(msg.name.begin(), it));

	dataMutexPosition_.lock();
	dataAngle_.clear();
	dataAngle_["L5_S1_Flex_Ext"] = -dataAngleXsens_["jt_L5_S1_y"];
	dataAngle_["L5_S1_axial_rotation"] = -dataAngleXsens_["jt_L5_S1_z"];
	dataMutexPosition_.unlock();

	
	dataMutexTime_.lock();
	timeStamp_ = rtb::getTime();
	dataMutexTime_.unlock();

	std::vector<double> positionTemp;
	for (std::vector<std::string>::const_iterator it = dofName_.begin(); it != dofName_.end(); it++)
	{
		if(*it == "L5_S1_Flex_Ext")
			positionTemp.push_back(dataAngleXsens_["jt_L5_S1_y"]);
		else if(*it == "L5_S1_axial_rotation")
			positionTemp.push_back(dataAngleXsens_["jt_L5_S1_z"]);
		else
			positionTemp.push_back(0);
	}
	msgTorque_.position = positionTemp;

	if (record_)
	{
		logger_->log(Logger::IK, timeStamp_, positionTemp);
		double msgID = msgTorque_.header.seq;
		logger_->log(Logger::RandomSignal, timeStamp_, msgID);
	}
}

void PluginROS::JointTorqueExtCallBackROS(const sensor_msgs::JointState& msg)
{
	std::map<std::string, double> data;
	for (std::vector<std::string>::const_iterator it = msg.name.begin(); it != msg.name.end(); it++)
		data[*it] = msg.effort.at(std::distance<std::vector<std::string>::const_iterator>(msg.name.begin(), it));

	data["L5_S1_axial_rotation"] = 0;
	std::vector<double> torqueExtTemp;
	for (std::vector<std::string>::const_iterator it = dofName_.begin(); it != dofName_.end(); it++)
		torqueExtTemp.push_back(data.at(*it));


	dataMutexTorqueExt_.lock();
	dataTorque_.clear();
	dataTorque_ = data;
	dataMutexTorqueExt_.unlock();

	if (record_)
		logger_->log(Logger::ID, timeStamp_, torqueExtTemp);
}

void PluginROS::setMuscleForce(const std::vector<double>& muscleForce)
{
	sensor_msgs::JointState msg;
	msg.name = muscleName_;
	msg.effort = muscleForce;
	msg.position = fiberLength_;
	msg.velocity = fiberVelocity_;
	muscleForcePublisher_->publish(msg);
}

void PluginROS::setMuscleForcePassive(const std::vector<double>& muscleForcePassive)
{
	sensor_msgs::JointState  msg;
	msg.name = muscleName_;
	msg.position = fiberLength_;
	msg.velocity = fiberVelocity_;
	msg.effort = muscleForcePassive;
	muscleForcePassivePublisher_->publish(msg);
}

void PluginROS::setMuscleForceActive(const std::vector<double>& muscleForceActive)
{
	sensor_msgs::JointState msg;
	msg.name = muscleName_;
	msg.position = fiberLength_;
	msg.velocity = fiberVelocity_;
	msg.effort = muscleForceActive;
	muscleForceActivePublisher_->publish(msg);
}

void PluginROS::setTendonStrain(const std::vector<double>& tendonStrain)
{
	sensor_msgs::JointState msg;
	msg.name = muscleName_;
	msg.effort = tendonStrain;
	tendonStrainPublisher_->publish(msg);
}

void PluginROS::setMuscleFiberLength(const std::vector<double>& muscleFiberLength)
{
	fiberLength_ = muscleFiberLength;
}

void PluginROS::setMuscleFiberVelocity(const std::vector<double>& muscleFiberVelocity)
{
	fiberVelocity_ = muscleFiberVelocity;
}

void PluginROS::setDofTorque(const std::vector<double>& dofTorque)
{
	msgTorque_.name = dofName_;
	msgTorque_.effort = dofTorque;
	torquePublisher_->publish(msgTorque_);
}

const std::map<std::string, double>& PluginROS::GetDataMap()
{
	ros::spinOnce();
	dataMutexPosition_.lock();
	dataAngleSafe_ = dataAngle_;
	dataMutexPosition_.unlock();
	return dataAngleSafe_;
}

const std::map<std::string, double>& PluginROS::GetDataMapTorque()
{
	//No ros::spinOnce since GetDataMap is called just before this one.
	dataMutexTorqueExt_.lock();
	dataTorqueExtSafe_ = dataTorqueExt_;
	dataMutexTorqueExt_.unlock();
	return dataTorqueExtSafe_;
}

const double& PluginROS::GetAngleTimeStamp()
{
	dataMutexTime_.lock();
	timeStampSafe_ = timeStamp_;
	dataMutexTime_.unlock();
	return timeStampSafe_;
}

void PluginROS::stop()
{
	if (record_)
	{
		logger_->stop();
		delete logger_;
	}
}

void PluginROS::setDirectory(std::string outDirectory, std::string inDirectory)
{
	outDirectory_ = outDirectory;
}

#ifdef __linux__
extern "C" AngleAndComsumerPlugin* create() {
	return new PluginROS;
}

extern "C" void destroy(AngleAndComsumerPlugin* p) {
	delete p;
}
#endif

#ifdef WIN32 // __declspec (dllexport) id important for dynamic loading
extern "C" __declspec (dllexport) AngleAndComsumerPlugin* __cdecl create() {
	return new PluginROS;
}

extern "C" __declspec (dllexport) void __cdecl destroy(AngleAndComsumerPlugin* p) {
	delete p;
}
#endif