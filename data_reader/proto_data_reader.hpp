#ifndef PROTO_DATA_READER_
#define PROTO_DATA_READER_

#include "system/function_block.hpp"
#include "proto/stdmsg.pb.h"
#include "glog/logging.h"
#include "utils/configfile.h"

using namespace std;

namespace agv_robot {
class ProtoDataReader : public FunctionBlock
{
private:
	void InitLog();
	string data_file_name_;
	stdmsg::LaserList data_;
	int current_index_;

public:
	ProtoDataReader(ConfigFile& cfg);
	stdmsg::String Backward(const stdmsg::Pose& pos);
	void Forward(int nu);
	void Accelerate(int k);
	void Decelerate(int k);
	void Update(vector<Message*> input,
		        vector<Message*> output);

};
}
#endif