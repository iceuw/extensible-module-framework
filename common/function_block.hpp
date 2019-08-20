#ifndef FUNCTION_BLOCK_
#define FUNCTION_BLOCK_ 

#include "proto/stdmsg.pb.h"
#include "rpc_utils.hpp"
#include "cfg_utils.hpp"

#ifdef _WINDOWS
#define EXPORT_API extern "C" __declspec(dllexport)
#else
#define EXPORT_API extern "C" __attribute((visibility("default")))
#endif

using std::vector;
using std::map;
using std::string;

using ::google::protobuf::Message;

namespace agv_robot{

class FunctionBlock : public RPCUtils
{
public:
	FunctionBlock() {};

	FunctionBlock(const ConfigFile& cfg) {};

	virtual void Update(vector<Message*> input, vector<Message*> output) = 0;

	///remote process call function. Used in ui.
	///
	///Use method "Set" Function. For example:
	///
	// stdmsg::String DefaultCallBack(const stdmsg::String& msg)
	// {
	// 	std::cout << "this is default call back" << std::endl;
	// 	stdmsg::String str;
	// 	str.set_str("ok");
	// 	return str;
	// }
	///
	///attention:
	///this rpc library only support below type function:
	///stdmsg::(used message class) fun(const stdmsg::(used message class)& msg)
	///if you has nothing to return. You can random choose a message type as your 
	///function return.
	template<typename BASECLASS, typename TYPE, typename RET >
	void Set(const std::string &name,
		RET(BASECLASS::*_func)(const TYPE&),
		BASECLASS* instance, int size = 1)
	{
		_rcallback2<BASECLASS, TYPE, RET >* func = new _rcallback2<BASECLASS, TYPE, RET >(_func, instance);

		add_handle(name, func);
	}

};

#define EXPORT_INSTANCE(block)	\
EXPORT_API FunctionBlock* Create##block(ConfigFile& cfg)\
{\
    return new block(cfg);	\
}\

}
#endif