#include "function_block.hpp"
#include "node.hpp"
#include "cfg_utils.hpp"

#include <iostream>

using namespace std;

namespace agv_robot
{
class DataSender : public FunctionBlock
{
private:
	Node* node_;
	string publish_topic_;
	int publish_interval_;
public:
	DataSender(ConfigFile& cfg);
	void Update(vector<Message*> input, vector<Message*> output);
};
}