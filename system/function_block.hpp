#ifndef FUNCTION_BLOCK_
#define FUNCTION_BLOCK_ 

#include "proto/stdmsg.pb.h"

#ifdef __WINDOWS_
#define EXPORT_API extern "C" __declspec(dllexport)
#else
#define EXPORT_API extern "C" __attribute((visibility("default")))
#endif

using std::vector;
using ::google::protobuf::Message;

namespace agv_robot{

class FunctionBlock{

public:
	FunctionBlock() {};
	virtual void update(vector<Message*> input, vector<Message*> output) = 0;
};

#define EXPORT_INSTANCE(block)	\
EXPORT_API FunctionBlock* Create##block(ConfigFile& cfg)\
{\
    return new block(cfg);	\
}\

}
#endif

