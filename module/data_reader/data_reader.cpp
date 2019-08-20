#include <windows.h>

#include "data_reader.hpp"
#include <fstream>
namespace agv_robot {

void DataReader::InitLog()
{
	google::InitGoogleLogging("");
#ifdef DEBUG_MODE
	google::SetStderrLogging(google::GLOG_INFO); 
#else
	google::SetStderrLogging(google::GLOG_ERROR);
#endif
	FLAGS_colorlogtostderr = true; 
	//FLAGS_servitysinglelog = true;
	google::SetLogDestination(google::GLOG_FATAL, "./log/log_fatal_"); 
	google::SetLogDestination(google::GLOG_ERROR, "./log/log_error_"); 
	google::SetLogDestination(google::GLOG_WARNING, "./log/log_warning_"); 
	google::SetLogDestination(google::GLOG_INFO, "./log/log_info_"); 
	FLAGS_logbufsecs = 0; 
	FLAGS_max_log_size = 100; 
	FLAGS_stop_logging_if_full_disk = true;
}

DataReader::DataReader(ConfigFile& cfg)
{
	InitLog();
	data_file_name_ = (string)cfg.value("protodatareader", "filename", "recoder200712-1104.msg");
	max_speed_      = cfg.value("protodatareader", "max_speed", 300);
	min_speed_      = cfg.value("protodatareader", "min_speed", 25);

	fstream input(data_file_name_, ios::in | ios::binary);
	LOG_IF(FATAL, !input) << "can't open " << data_file_name_;

	if (!data_.ParseFromIstream(&input))
	{
		LOG(FATAL) << "can't read the data file, please check if the file was encrypted or broken";
	}
	LOG(INFO) << "";

	current_index_ = 0;
	speed_ = 50;

	Set("Backward", &DataReader::Backward, this);
	Set("Forward", &DataReader::Forward, this);
	Set("Accelerate", &DataReader::Accelerate, this);
	Set("Decelerate", &DataReader::Decelerate, this);
}

stdmsg::String DataReader::Backward(const stdmsg::Pose& pos)
{
	cout << "rpc call success" << endl;

	current_index_ -= 20;

	if (current_index_ < 0) current_index_ = 0;

	stdmsg::String str;
	str.set_str("OK");
	return str;
}

stdmsg::String DataReader::Forward(const stdmsg::Pose& pose)
{
	cout << "rpc call success" << endl;

	current_index_ += 20;

	if (current_index_ > data_.scans_size()) 
		current_index_ = data_.scans_size();

	stdmsg::String str;
	str.set_str("OK");
	return str;
}

stdmsg::String DataReader::Accelerate(const stdmsg::Pose& pose)
{
	cout << "rpc call success" << endl;

	speed_ /= 2;

	if (speed_ > max_speed_)
		speed_ = max_speed_;

	stdmsg::String str;
	str.set_str("OK");
	return str;
}

stdmsg::String DataReader::Decelerate(const stdmsg::Pose& pose)
{
	cout << "rpc call success" << endl;

	speed_ *= 2;

	if (speed_ < min_speed_)
		speed_ = min_speed_;

	stdmsg::String str;
	str.set_str("OK");
	return str;
}

void DataReader::Update(vector<Message*> input,
	                         vector<Message*> output)
{
	if (current_index_ < data_.scans().size())
	{
		stdmsg::Laser_Scan* scan = (stdmsg::Laser_Scan*)output[0];
		scan->CopyFrom(data_.scans().Get(current_index_));
		current_index_++;
		cout << current_index_ << endl;
		Sleep(speed_);
	}
	else //current_index_ > data_.scans().size()
	{
		cout << "data has been run out" << endl;
		cout << "press 'y' to replay the data or press 'n' to end the program" << endl;
		char choice;
		cin >> choice;
		if (choice == 'y') current_index_ = 0;
		if (choice == 'n') exit(0);
	}
}

EXPORT_INSTANCE(DataReader)
}

