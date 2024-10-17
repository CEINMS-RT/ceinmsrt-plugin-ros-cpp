#include <PluginEMGROS.h>

PluginEMGROS::PluginEMGROS() : record_(false), _outDirectory("output")
{

}

PluginEMGROS::~PluginEMGROS()
{
	delete emgSub_, rosNode_;
}

void PluginEMGROS::init(std::string xmlName, std::string executionName)
{
	timeStamp_ = rtb::getTime();
	//pointer to subject XML
	std::auto_ptr<NMSmodelType> subjectPointer;

	try
	{
		subjectPointer = std::auto_ptr<NMSmodelType>(subject(xmlName, xml_schema::flags::dont_initialize));
	}
	catch (const xml_schema::exception& e)
	{
		cout << e << endl;
		exit(EXIT_FAILURE);
	}

	NMSmodelType::Channels_type& channels(subjectPointer->Channels());
	ChannelsType::Channel_sequence& channelSequence(channels.Channel());

	//std::cout << "Reading Execution XML from: " << executionName << std::endl;

	// Configuration XML
	std::auto_ptr<ExecutionType> executionPointer;

	try
	{
		std::auto_ptr<ExecutionType> temp(execution(executionName, xml_schema::flags::dont_initialize));
		executionPointer = temp;
	}
	catch (const xml_schema::exception& e)
	{
		cout << e << endl;
		exit(EXIT_FAILURE);
	}

	// Get the EMG XML configuration
	const std::string& EMGFile = executionPointer->ConsumerPlugin().EMGDeviceFile().get();

	// EMG XML class reader
	_executionEmgXml = new ExecutionEmgXml(EMGFile);

	//Ip and Port from the EMG xml
	std::string port = _executionEmgXml->getPort();


	//Get the EMG channel from the subject xml
	for (ChannelsType::Channel_iterator it = channelSequence.begin(); it != channelSequence.end(); it++)
	{
		//get name channel
		nameSet_.insert(it->name());
		nameVect_.push_back(it->name());

	}

	maxAmp_ = _executionEmgXml->getMaxEmg();

	if (maxAmp_.size() < nameVect_.size()) // If we want to record more channels than those indicated in <maxEMG> (executionEMG.xml), add additional ones
	{
		int extraElectrodes = nameVect_.size() - maxAmp_.size();
		std::cout << "Number of Max amplitude EMG < number of EMGs. Adding " << extraElectrodes << " additional max amplitude entries." << std::endl;
		for (int i = 0; i < extraElectrodes; i++)
		{
			maxAmp_.push_back(0.0000001);
		}
	}
	else if (maxAmp_.size() > nameVect_.size()) // If there are already more channels specified than those we want to record then, add only those needed.
	{
		maxAmp_.clear();
		for (size_t i = 0; i < nameVect_.size(); i++)
		{
			maxAmp_.push_back(0.0000001);
		}
	}

	for (std::vector<std::string>::const_iterator it = nameVect_.begin(); it != nameVect_.end(); it++)
		dataEMG_[*it] = 0;


	if (record_)
	{
		logger_ = new OpenSimFileLogger<double>(_outDirectory);
		logger_->addLog(Logger::EmgsFilter, nameVect_);
	}

	std::map<std::string, std::string> rosArg;
	ros::init(rosArg, "EMG_CEINMS", ros::init_options::NoSigintHandler);
	rosNode_ = new ros::NodeHandle;
	emgSub_ = new ros::Subscriber(rosNode_->subscribe("emg", nameVect_.size(), &PluginEMGROS::emgCallBackROS, this));
}

void PluginEMGROS::emgCallBackROS(const sensor_msgs::JointState& msg)
{
	std::map<std::string, double> emgDataLocal;
	double timeLocal;

	timeLocal = rtb::getTime();
	std::vector<double> data, dataSave;

	for (std::vector<std::string>::const_iterator it = msg.name.begin(); it != msg.name.end(); it++)
	{
		//std::cout << "msg.name " << *it<<std::endl;
		emgDataLocal[*it] = msg.position.at(std::distance<std::vector<std::string>::const_iterator>(msg.name.begin(), it));
	}

	for (std::vector<std::string>::const_iterator it = nameVect_.begin(); it != nameVect_.end(); it++)
	{
		//std::cout<< "nameVect_ " << *it<<std::endl;
		data.push_back(emgDataLocal.at(*it));
	}

//#ifdef NORMEMG
	for (int i = 0; i < nameVect_.size(); i++) 
	{
		if (maxAmp_[i] < data[i])
			maxAmp_[i] = data[i];
//#ifdef MAXEMG
		emgDataLocal[nameVect_[i]] = data[i] / maxAmp_[i];
//#else
		//emgDataLocal[nameVect_[i]] = data[i];
//#endif
		dataSave.push_back(emgDataLocal[nameVect_[i]]);
	}
//#else
//	dataSave = data;
//#endif

	if (record_)
		logger_->log(Logger::EmgsFilter, timeStamp_, dataSave);

	dataMutex_.lock();
	dataEMG_ = emgDataLocal;
	dataMutex_.unlock();

	timeStamp_ = timeLocal;
}

const std::map<std::string, double>& PluginEMGROS::GetDataMap()
{
	ros::spinOnce();
	dataMutex_.lock();
	dataEMGSafe_ = dataEMG_;
	dataMutex_.unlock();
	return dataEMGSafe_;
}


void PluginEMGROS::stop()
{
	if (record_)
	{
		logger_->stop();
		delete logger_;
	}
	_executionEmgXml->setMaxEmg(maxAmp_);
	_executionEmgXml->UpdateEmgXmlFile();
}

#ifdef UNIX
extern "C" ProducersPluginVirtual* create() {
	return new PluginEMGROS;
}

extern "C" void destroy(ProducersPluginVirtual* p) {
	delete p;
}
#endif

#ifdef WIN32 // __declspec (dllexport) id important for dynamic loading
extern "C" __declspec (dllexport) ProducersPluginVirtual* __cdecl create() {
	return new PluginEMGROS;
}

extern "C" __declspec (dllexport) void __cdecl destroy(ProducersPluginVirtual* p) {
	delete p;
}
#endif