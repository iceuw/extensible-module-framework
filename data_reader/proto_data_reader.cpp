#include <windows.h>

#include "proto_data_reader.hpp"
#include <fstream>
namespace agv_robot {

void ProtoDataReader::InitLog()
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

ProtoDataReader::ProtoDataReader(ConfigFile& cfg)
{
	//InitLog();
	//data_file_name_ = (string)cfg.value("protodatareader", "filename", "recoder200712-1104.msg");

	//fstream input(data_file_name_, ios::in | ios::binary);
	//LOG_IF(FATAL, !input) << "can't open " << data_file_name_;

	//if (!data_.ParseFromIstream(&input))
	//{
	//	LOG(FATAL) << "can't read the data file, please check if the file was encrypted or broken";
	//}
	//LOG(INFO) << "";

	//current_index_ = 0;
	Set("test", &ProtoDataReader::Backward, this);
}

stdmsg::String ProtoDataReader::Backward(const stdmsg::Pose& pos)
{
	cout << "rpc call success" << endl;
	stdmsg::String str;
	str.set_str("OK");
	return str;
}

void ProtoDataReader::Update(vector<Message*> input,
	                         vector<Message*> output)
{
	//if (current_index_ < data_.scans().size())
	//{
	//	stdmsg::Laser_Scan* scan = (stdmsg::Laser_Scan*)output[0];
	//	scan->CopyFrom(data_.scans().Get(current_index_));
	//	current_index_++;
	//	cout << current_index_ << endl;
	//	Sleep(1000);
	//}
	//else //current_index_ > data_.scans().size()
	//{
	//	cout << "data has been run out" << endl;
	//	cout << "press 'y' to replay the data or press 'n' to end the program" << endl;
	//	char choice;
	//	cin >> choice;
	//	if (choice == 'y') current_index_ = 0;
	//	if (choice == 'n') exit(0);
	//}
	Sleep(1000);
}

EXPORT_INSTANCE(ProtoDataReader)
}

