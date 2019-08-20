#include "data_sender.hpp"

namespace agv_robot
{
DataSender::DataSender(ConfigFile& cfg)
{
	string bind = cfg.value("node", "bind", "tcp://*:9001");
	
	node_ = new Node(bind);

	string connect = cfg.value("node", "connect", "tcp://192.168.1.108:7871");
	while (connect != "")
	{
		int pos = connect.find(";");
		node_->connect(connect.substr(0, pos));

		if (pos != std::string::npos) connect = connect.substr(pos + 1);
		else connect = "";
	}

	publish_topic_ = cfg.value("node", "publish_topic", "laser");
	publish_interval_ = cfg.value("node", "publish_interval", "1");
}
void DataSender::Update(vector<Message*> input, vector<Message*> output)
{
	stdmsg::Laser_Scan* scan = (stdmsg::Laser_Scan*)input[0];
	if (scan->seq() % publish_interval_ == 0 && this->node_)
	{
		node_->publish(publish_topic_, *scan);
	}
}

EXPORT_INSTANCE(DataSender)

}
