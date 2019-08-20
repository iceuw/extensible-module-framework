#ifndef MSG_PARSER_
#define MSG_PARSER_

#ifdef _WINDOWS
#include <windows.h>
#elif defined linux
#include <dlfcn.h>
#endif

#include "proto/stdmsg.pb.h"
#include "cfg_utils.hpp"
#include "node.hpp"

namespace agv_robot
{

Message* createMessage(const std::string& typeName)
{
	string msg_name = typeName;
	int pos = msg_name.find("stdmsg");
	if (pos == string::npos)
	{
		msg_name = "stdmsg." + msg_name;
	}
	google::protobuf::Message* message = NULL;
	const google::protobuf::Descriptor* descriptor =
		google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(msg_name);
	assert(descriptor != NULL);
	if (descriptor)
	{
		const google::protobuf::Message* prototype = 
			google::protobuf::MessageFactory::generated_factory()->GetPrototype(descriptor);
		if (prototype)
		{
			message = prototype->New();
		}
	}
	return message;
}

shared_ptr<FunctionBlock> ImportBlock(string DLL_name, string block_name, ConfigFile& cfg)
{
	typedef FunctionBlock* (CREATEBLK)(ConfigFile& cfg);
#ifdef _WINDOWS

	HINSTANCE hd = ::LoadLibrary(DLL_name.c_str());
	LOG_IF(FATAL, !hd) << "can't find DLL: " << DLL_name;

	string creator_name = "Create" + block_name;
	CREATEBLK* creator = (CREATEBLK*)::GetProcAddress(hd, creator_name.c_str());
	LOG_IF(FATAL, !creator) << DLL_name << "can't find creator: " << creator_name;

	return shared_ptr<FunctionBlock>((*creator)(cfg));

#elif defined linux
    //load the dynamic library right now
    void* hd = dlopen(DLL_name.c_str(), RTLD_NOW);

	string creator_name = "Create" + block_name;
	CREATEBLK* creator = (CREATEBLK*)dlsym(hd, creator_name.c_str());

	return shared_ptr<FunctionBlock>((*creator)(cfg));
#endif
}

} //end namespace agv_robot
#endif