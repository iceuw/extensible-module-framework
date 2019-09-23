#include "system.hpp"
#include "msg_parser.hpp"
#include <windows.h>
//#include "proto_data_reader.hpp"
//#include "recoder.hpp"

namespace agv_robot
{
 void GlogInit()
 {
 	google::InitGoogleLogging("");
 #ifdef DEBUG_MODE
 	google::SetStderrLogging(google::GLOG_INFO); //设置级别高于 google::INFO 的日志同时输出到屏幕
 #else
 	google::SetStderrLogging(google::GLOG_FATAL);//设置级别高于 google::FATAL 的日志同时输出到屏幕
 #endif
 	FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
 	//FLAGS_servitysinglelog = true;// 用来按照等级区分log文件
 	google::SetLogDestination(google::GLOG_FATAL, "./log/log_fatal_"); // 设置 google::FATAL 级别的日志存储路径和文件名前缀
 	google::SetLogDestination(google::GLOG_ERROR, "./log/log_error_"); //设置 google::ERROR 级别的日志存储路径和文件名前缀
 	google::SetLogDestination(google::GLOG_WARNING, "./log/log_warning_"); //设置 google::WARNING 级别的日志存储路径和文件名前缀
 	google::SetLogDestination(google::GLOG_INFO, "./log/log_info_"); //设置 google::INFO 级别的日志存储路径和文件名前缀
 	FLAGS_logbufsecs = 0; //缓冲日志输出，默认为30秒，此处改为立即输出
 	FLAGS_max_log_size = 100; //最大日志大小为 100MB
 	FLAGS_stop_logging_if_full_disk = true; //当磁盘被写满时，停止日志输出
 											//google::SetLogFilenameExtension("91_"); //设置文件名扩展，如平台？或其它需要区分的信息
 											//google::InstallFailureSignalHandler(); //捕捉 core dumped (linux)
 											//google::InstallFailureWriter(&Log); //默认捕捉 SIGSEGV 信号信息输出会输出到 stderr (linux)
 }

System::System(ConfigFile& cfg, stdmsg::Net net_param) : rpc_thread(this)
{
	GlogInit();
	InitNode();
	map<string, int> msg_name_to_idx;
	set<string> available_msgs;

	//TODO switch the size to param.size
	int block_size = net_param.block_size();
	input_msgs_.resize(block_size);
	input_msg_ids_.resize(block_size);
	output_msgs_.resize(block_size);
	output_msg_ids_.resize(block_size);

	//TODO
	//add param to init the blocks
	for (int block_id = 0; block_id < block_size; ++block_id)
	{
		stdmsg::Block block_param = net_param.block().Get(block_id);
		string DLL_name = block_param.dll();
		string block_name = block_param.object();
		shared_ptr<FunctionBlock> block = ImportBlock(DLL_name, block_name, cfg);
		blocks_.push_back(block);
	
		rpc_->add_handle(block->_rhandlertable);
		for (int output_id = 0; output_id < block_param.output_size(); ++output_id)
		{
			AppendOutput(block_param.output(output_id), block_param.output_type(output_id),
				block_id, output_id, &available_msgs, &msg_name_to_idx);
		}
	}	
	for (int block_id = 0; block_id < block_size; ++block_id)
	{
		stdmsg::Block block_param = net_param.block().Get(block_id);
		for (int input_id = 0; input_id < block_param.input_size(); ++input_id)
		{
			AppendInput(block_param.input(input_id), block_param.input_type(input_id), 
				block_id, input_id, &available_msgs, &msg_name_to_idx);
		}
	}	

		
	//shared_ptr<FunctionBlock> func(new ProtoDataReader(cfg));
	//blocks_.push_back(func);
	////for(int output_id = 0; output_id < block_param.output_size(); ++ output_id)
	////AppendTop(param, layer_id, top_id, &available_blobs, &blob_name_to_idx);
	//AppendOutput("stdmsg.Laser_Scan", 0, 0, &available_msgs, &msg_name_to_idx);

	////shared_ptr<FunctionBlock> func2(new Recoder(cfg));

	//AppendInput("stdmsg.Laser_Scan", 1, 0, &available_msgs, &msg_name_to_idx);

	

}

//TODO
//just for test
//need to enhance robust
void System::InitNode()
{
	rpc_ = new RPC();
	rpc_->bind("tcp://*:9000");
	rpc_->connect("tcp://192.168.1.108:9000");
	rpc_->set("rpctest", &System::RpcTest, this);
	rpc_thread.start();
}

void System::AppendInput(const string msg_name, const string msg_type, const int block_id, 
	const int input_id, set<string>* available_msgs, map<string, int>* msg_name_to_index)
{
	if (available_msgs->find(msg_name) == available_msgs->end())
	{
		LOG(FATAL) << "找不到输入消息 ' " << msg_name << " '";
		//cout << "找不到消息" << msg_name << endl;
		system("pause");
		exit(-1);
	}
	const int msg_id = (*msg_name_to_index)[msg_name];

	input_msgs_[block_id].push_back(msgs_[msg_id].get());
	input_msg_ids_[block_id].push_back(msg_id);
	//available_msgs->erase(msg_name);
}

void System::AppendOutput(const string out_msg_name, const string out_msg_type, const int block_id, 
	const int output_id, set<string>* available_msgs, map<string, int>* msg_name_to_index)
{
	shared_ptr<Message> out_msg(createMessage(out_msg_type));
	const int msg_id = msgs_.size();
	msgs_.push_back(out_msg);
	msg_names_.push_back(out_msg_name);
	if (msg_name_to_index) (*msg_name_to_index)[out_msg_name] = msg_id;
	output_msgs_[block_id].push_back(out_msg.get());
	output_msg_ids_[block_id].push_back(msg_id);
	if (available_msgs) available_msgs->insert(out_msg_name);
}

void System::Run()
{
	double time_;
	LARGE_INTEGER nFreq_;//计时频率
	LARGE_INTEGER nBeginTime_;
	LARGE_INTEGER nEndTime_;
	QueryPerformanceFrequency(&nFreq_);
	QueryPerformanceCounter(&nBeginTime_);//开始计时 	
	while (1)
	{
		for (int i = 0; i < blocks_.size(); ++i)
		{
			blocks_[i]->Update(input_msgs_[i], output_msgs_[i]);
			QueryPerformanceCounter(&nEndTime_);//停止计时  
			time_ = (double)(nEndTime_.QuadPart - nBeginTime_.QuadPart) / (double)nFreq_.QuadPart;//计算程序执行时间单位为s  
			nBeginTime_ = nEndTime_;
			//cout << i << "  用时：" << time_ * 1000 << " ms" << endl;
		}
		//cout << endl;
	}
}

System::~System()
{

}

}
