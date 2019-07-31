#include "system/function_block.hpp"
#include "node.hpp"
#include "utils/configfile.h"

#include <iostream>

using namespace std;

namespace agv_robot
{
class Sender : public FunctionBlock
{
private:
	middleware::Node* node_;
	string publish_topic_;
	int publish_interval_;
public:
	Sender(ConfigFile& cfg);
	void update(vector<Message*> input, vector<Message*> output);
};
}