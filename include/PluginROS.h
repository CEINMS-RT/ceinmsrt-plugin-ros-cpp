#ifndef PluginROS_H
#define PluginROS_H


#include "execution.hxx"
#include "XMLInterpreter.h"
#include <AngleAndComsumerPlugin.h>
#include "ExecutionXmlReader.h"
#include "OpenSimFileLogger.h"
#include <getTime.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <ros/master.h>

#include <thread>
#include <chrono>
#include <mutex>
#define M_PI 3.14159265359

#ifdef WIN32
	class __declspec(dllexport) PluginROS : public AngleAndComsumerPlugin {
#endif
#ifdef __linux__ 
	class  PluginROS : public AngleAndComsumerPlugin {
#endif
public:
	PluginROS();
	~PluginROS();

	void init(std::string& executableXMLFileName);


	void init(int portno);


	void setMuscleForce(const std::vector<double>& muscleForce);

	void setMuscleFiberLength(const std::vector<double>& muscleFiberLength);

	void setMuscleFiberVelocity(const std::vector<double>& muscleFiberVelocity);

	void setDofName(const std::vector<std::string>& dofName)
	{
		dofName_ = dofName;
	}

	void setDofTorque(const std::vector<double>& dofTorque);

	void setDofStiffness(const std::vector<double>& dofStiffness)
	{

	}

	void setMuscleName(const std::vector<std::string>& muscleName)
	{
		muscleName_ = muscleName;
	}

	void setOutputTimeStamp(const double& timeStamp)
	{

	}


	void setMuscleForcePassive(const std::vector<double>& muscleForcePassive);

	void setMuscleForceActive(const std::vector<double>& muscleForceActive);
	
	void setTendonStrain(const std::vector<double>& tendonStrain);


	const double& GetAngleTimeStamp();

	const std::vector<std::string>& GetDofName()
	{
		return dofName_;
	}

	const std::map<std::string, double>& GetDataMap();

	const std::map<std::string, double>& GetDataMapTorque();

	void stop();

	void setDirectory(std::string outDirectory, std::string inDirectory = std::string());

	void setVerbose(int verbose)
	{
	}

	void setRecord(bool record)
	{
		record_ = record;
	}

protected:

	void JointPositionCallBackROS(const sensor_msgs::JointState& msg);
	
	void JointTorqueExtCallBackROS(const sensor_msgs::JointState& msg);

	std::vector<std::string> dofName_;
	std::vector<std::string> muscleName_;
	std::vector<double> muscleForcePassive_;
	std::vector<double> muscleForceActive_;
	std::vector<double> tendonStrain_;
	std::vector<double> fiberLength_;
	std::vector<double> fiberVelocity_;
	std::vector<double> muscleFiberLength_;
	double timeStamp_;
	double timeStampSafe_;
	std::vector<unsigned long> varNameVect_;
	OpenSimFileLogger<double>* logger_;
	std::map<std::string, double> dataAngle_;
	std::map<std::string, double> dataAngleSafe_;
	std::map<std::string, double> dataAngleXsens_;
	std::map<std::string, double> dataTorque_;
	std::map<std::string, double> dataTorqueExtSafe_;
	std::map<std::string, double> dataTorqueExt_;
	std::string outDirectory_;
	bool record_;
	std::mutex dataMutexPosition_;
	std::mutex dataMutexTorqueExt_;
	std::mutex dataMutexTime_;

	bool threadStop_;
	int cpt_;

	ros::NodeHandle *rosNode_;
	ros::Publisher *torquePublisher_;
	ros::Publisher *muscleForcePublisher_;
	ros::Publisher *muscleFiberLengthPublisher_;
	ros::Publisher *muscleFiberVelocityPublisher_;
	ros::Publisher *muscleForceActivePublisher_;
	ros::Publisher *muscleForcePassivePublisher_;
	ros::Publisher *tendonStrainPublisher_;
	ros::Subscriber *positionSub_;
	ros::Subscriber *torqueSub_;
	sensor_msgs::JointState msgTorque_;

};

#endif