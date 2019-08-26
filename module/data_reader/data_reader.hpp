#ifndef PROTO_DATA_READER_
#define PROTO_DATA_READER_

#include "function_block.hpp"
#include "proto/stdmsg.pb.h"
#include "glog/logging.h"
#include "cfg_utils.hpp"

using namespace std;

namespace agv_robot {
class DataReader : public FunctionBlock
{
private:
	void InitLog();
	string data_file_name_;
	stdmsg::LaserList data_;
	int current_index_;
	int speed_;

	int max_speed_;
	int min_speed_;

	bool play_;

public:
	DataReader(ConfigFile& cfg);
	stdmsg::String Backward(const stdmsg::Pose& pos);
	stdmsg::String Forward(const stdmsg::Pose& pos);
	stdmsg::String Accelerate(const stdmsg::Pose& pos);
	stdmsg::String Decelerate(const stdmsg::Pose& pos);
	stdmsg::String StopAndPlay(const stdmsg::Pose& pos);
	void Update(vector<Message*> input,
		        vector<Message*> output);

};
}
#endif