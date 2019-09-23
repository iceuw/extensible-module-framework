#ifndef SYSTEM_
#define SYSTEM_

#include <vector>
#include <memory>

#include "function_block.hpp"
#include "cfg_utils.hpp"
#include "proto/stdmsg.pb.h"
#include "glog/logging.h"
#include "node.hpp"
#include "thread_utils.hpp"

using namespace std;
using ::google::protobuf::Message;

namespace agv_robot {


class System {
private:
	
	struct RPCThread : public BThread
	{
		System* handle;
		RPCThread(System* sys)
		{
			handle = sys;
		}
		~RPCThread()
		{
			kill();
		}
		void run()
		{
			try
			{
				while (true) handle->rpc_->run();
			}
			catch (const std::exception& e)
			{
				LOG(ERROR) << "communication error: " << e.what();
			}
		}
	}rpc_thread;

	vector<shared_ptr<Message>> msgs_;
	vector<string> msg_names_;

	vector<vector<Message*>> output_msgs_;
	vector<vector<int>> output_msg_ids_;

	vector<vector<Message*>> input_msgs_;
	vector<vector<int>> input_msg_ids_;

	map<string, int> msg_name_index_;

	vector<shared_ptr<FunctionBlock>> blocks_;

	RPC* rpc_;
public:
	System(ConfigFile& cfg, stdmsg::Net net_param);

	void InitNode();

	stdmsg::String RpcTest(const stdmsg::Pose& pos)
	{
		cout << "this is system rpc test" << endl;
		stdmsg::String str;
		str.set_str("OK");
		return str;
	}

	void AppendInput(const string msg_name, const string msg_type, const int block_id,
		const int input_id, set<string>* available_msgs, map<string, int>* msg_name_to_index);
	void AppendOutput(const string msg_name, const string msg_type, const int block_id, 
		const int output_id, set<string>* available_msgs, map<string, int>* msg_name_to_index);

	void Run();
	~System();
};
}
#endif // !SYSTEM_
