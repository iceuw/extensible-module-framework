#ifndef SYSTEM_
#define SYSTEM_

#include <vector>
#include <memory>

#include "function_block.hpp"
#include "configfile.h"
#include "stdmsg.pb.h"
//#include "glog/logging.h"

using namespace std;
using ::google::protobuf::Message;

namespace agv_robot {

class System {
private:
	vector<shared_ptr<Message>> msgs_;
	vector<string> msg_names_;

	vector<vector<Message*>> output_msgs_;
	vector<vector<int>> output_msg_ids_;

	vector<vector<Message*>> input_msgs_;
	vector<vector<int>> input_msg_ids_;

	map<string, int> msg_name_index_;

	vector<shared_ptr<FunctionBlock>> blocks_;
public:
	System(ConfigFile& cfg, stdmsg::Net net_param);
	void AppendInput(const string msg_name, const int block_id, const int input_id, 
		set<string>* available_msgs, map<string, int>* msg_name_to_index);
	void AppendOutput(const string msg_name, const int block_id, const int output_id, 
		set<string>* available_msgs, map<string, int>* msg_name_to_index);
	void Run();
	~System();
};
}
#endif // !SYSTEM_
