#ifndef RECODER_
#define RECODER_

#include <stdmsg.pb.h>
#include <utils/configfile.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <queue>
#include "glog/logging.h"
#include "function_block.hpp"

namespace agv_robot
{

class Recoder : public FunctionBlock
{
private:
	std::queue<stdmsg::Laser_Scan*> cache;
	stdmsg::LaserList laser_scans;
	std::string save_file;
	std::fstream* output_;
	int max_save;
public:
	Recoder(ConfigFile& cfg)
	{
		max_save = cfg.value("recoder", "max_save", 5000);
		save_file = (std::string)cfg.value("recoder", "save_file", "recoder.dat");

		//添加时间后缀
		time_t raw_time;
		time(&raw_time);
		tm* time_info = localtime(&raw_time);
		char suffix[80];
		strftime(suffix, 80, "%C%m%d-%H%M", time_info);

		std::string filename = save_file;
		auto dot = filename.find_first_of(".");
		filename.insert(dot, suffix);

		output_ = new std::fstream(filename,
			std::ios::out | std::ios::trunc | std::ios::binary);
	}
	~Recoder()
	{
		output_->close();
		delete output_;
	}

	void update(vector<Message*> input, vector<Message*> output)
	{
		//LOG(INFO) << "开始记录scan数据";
		stdmsg::Laser_Scan* scan = (stdmsg::Laser_Scan*)input[0];
		laser_scans.add_scans()->CopyFrom(*scan);
		if (!laser_scans.SerializeToOstream(output_))
		{
			std::cerr << "Failed to write the laser list" << std::endl;
			laser_scans.Clear();
		}
		laser_scans.Clear();
	}

};

}
#endif