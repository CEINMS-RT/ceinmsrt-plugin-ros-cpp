#ifndef PluginEMGROS_H
#define PluginEMGROS_H

#include "execution.hxx"
#include "XMLInterpreter.h"
#include "ExecutionEmgXml.h"
#include <ProducersPluginVirtual.h>
#include "NMSmodel.hxx"
#include "OpenSimFileLogger.h"
#include <getTime.h>
#include <thread>
#include <mutex>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <ros/master.h>

#define M_PI 3.14159265359

#ifdef WIN32
	class __declspec(dllexport) PluginEMGROS : public ProducersPluginVirtual {
#endif
#ifdef UNIX
	class  PluginEMGROS : public ProducersPluginVirtual {
#endif
public:
	PluginEMGROS();
	~PluginEMGROS();

	void init(std::string xmlName, std::string executionName);

	void reset()
	{
	}

	/**
	* Get the time stamp of the EMG capture.
	*/
	const double& getTime()
	{
		return  timeStamp_;
	}

	/**
	* Get a set of the channel name
	*/
	const std::set<std::string>& GetNameSet()
	{
		return nameSet_;
	}
	const std::map<std::string, double>& GetDataMap();

	const std::map<std::string, double>& GetDataMapTorque()
	{
		return _torque;
	}

	void stop();

	void setDirectories(std::string outDirectory, std::string inDirectory = std::string())
	{
		_outDirectory = outDirectory;
		_inDirectory = inDirectory;
	}

	void setVerbose(int verbose)
	{
	}

	void setRecord(bool record)
	{
		record_ = record;
	}

protected:

	std::string _outDirectory;
	std::string _inDirectory;

	void emgCallBackROS(const sensor_msgs::JointState& msg);

	ros::NodeHandle *rosNode_;
	double timeStamp_;
	double timeStampSafe_;
	std::vector<unsigned long> varNameVect_;
	OpenSimFileLogger<double>* logger_;
	bool record_;
	std::set<std::string> nameSet_;
	std::map<std::string, double> dataEMG_;
	std::map<std::string, double> dataEMGSafe_;
	ExecutionEmgXml* _executionEmgXml;
	std::vector<std::string> nameVect_; //!< Vector of channel names
	std::map<std::string, double> _torque;
	std::vector<double> maxAmp_;
	ros::Subscriber *emgSub_;

	std::mutex dataMutex_;
};

#endif