#include "odometry.hpp"
#include "glog/logging.h"
#include <thread>

namespace agv_robot
{
Odometry::Odometry(ConfigFile& cfg) :
	ser_(NULL),
	x_(0), y_(0), theta_(0),
	v_(0), w_(0),
	last_x_(0), last_y_(0), last_theta_(0),
	origin_x_(0), origin_y_(0), origin_theta_(0),
	send_index_(0),
	update_odom_thread_(this)
{
	port_ = (string)cfg.value("odometry", "port", "COM2");
	baudrate_ = cfg.value("odometry", "baud", 19200);

	ConfigFile input;
	if (!input.read("input"))
	{
		LOG(WARNING) << "Can't read the origin info";
	}
	origin_x_ = (double)input.value("Origin", "x", 0);
	origin_y_ = (double)input.value("Origin", "y", 0);
	origin_theta_ = (double)input.value("Origin", "theta", 0);

	OpenPort();
	update_odom_thread_.start();
}
void Odometry::OpenPort()
{
	if (!ser_)
	{
		try
		{
			ser_ = new serial::Serial(port_, baudrate_);
		}
		catch (std::exception e)
		{
			LOG(ERROR) << "can't open port: " << e.what();
			cout << e.what();
			ser_ = NULL;
		}
	}
}

void Odometry::Write2Ser(stdmsg::WriteToOdometryMsg write_msg)
{
	float x = write_msg.pose().position().x();
	float y = write_msg.pose().position().y();
	float theta = write_msg.pose().orentation().yaw();
	char pflg = write_msg.pflg()[0];
	short send_index = send_index_ + 1;
	if (send_index >= 10000)
		send_index_ = 0;

	short match = (short)(write_msg.match() * 100);

	//坐标转换
	x = (x - origin_x_)*cos(origin_theta_) + (y - origin_y_)*sin(origin_theta_);
	y = (y - origin_y_)*cos(origin_theta_) - (x - origin_x_)*sin(origin_theta_);
	theta = theta - origin_theta_;

	if (ser_->isOpen())
	{
		unsigned char buffer[1024];
		char head[2] = { 0xEE, 0xAA };
		char tail[4] = { 0x02, 0x00, 0x00, 0xBB };

		//转换字符串
		unsigned char *str_x = (unsigned char*)&x;
		unsigned char *str_y = (unsigned char*)&y;
		unsigned char *str_theta = (unsigned char*)&theta;
		unsigned char *str_pflg = (unsigned char*)&pflg;
		unsigned char *str_sendfg = (unsigned char*)&send_index;
		unsigned char *str_match = (unsigned char*)&match;

		//转16进制
		for (int i = 0; i < 22; i++)
		{
			if (i < 2)
				buffer[i] = head[i];
			if (i >= 2 && i < 6)
				buffer[i] = str_x[3 - (i - 2)];
			if (i >= 6 && i < 10)
				buffer[i] = str_y[3 - (i - 6)];
			if (i >= 10 && i < 14)
				buffer[i] = str_theta[3 - (i - 10)];
			if (i >= 14 && i < 16)
				buffer[i] = str_pflg[1 - (i - 14)];
			if (i >= 16 && i < 18)
				buffer[i] = str_sendfg[1 - (i - 16)];
			if (i >= 18 && i < 20)
				buffer[i] = str_match[1 - (i - 18)];
			if (i >= 20 && i < 24)
				buffer[i] = tail[i - 20];
			//std::cout << std::hex << (buffer[i] & 0xff) << " ";
		}

		try
		{
			ser_->write(buffer, 24);
		}
		catch (std::exception e)
		{
			LOG(ERROR) << e.what();
		}
	}
	else
	{
		if (!ser_)
		{
			delete ser_;
			ser_ = NULL;
			OpenPort();
		}
	}
}

void Odometry::ReadFromSer()
{
	string rst = "";
	if (ser_->isOpen())
	{
		while (true)
		{
			string rec_data = ser_->read(MAX_READ_BYTE);
			rst += rec_data;

			static const uint8_t head[] = { 0xee, 0xaa }, tail[] = { 0x00, 0xbb };
			std::string head_str = (char*)head;
			std::string tail_str = (char*)tail;

			int start = rst.find(head_str);
			int end = rst.find(tail_str);
			if (start == std::string::npos)
			{
				rst = "";
				continue;
			}
			if (end == std::string::npos)
			{
				//rst.data = "";
				continue;
			}
			if (start + 38 > rst.length())
			{
				continue;
			}
			lock_.lock();
			string sub = rst.substr(start + 2, 36);
			const char* data = sub.c_str();

			float not_use;
			//The fisrt three value is reserved value. 
			//Have not been used yet.
			memcpy(&not_use, data, 4);
			memcpy(&not_use, data + 4, 4);
			memcpy(&not_use, data + 8, 4);
			
			memcpy(&x_, data + 12, 4);
			memcpy(&y_, data + 16, 4);
			memcpy(&theta_, data + 20, 4);
			memcpy(&v_, data + 24, 4);
			memcpy(&w_, data + 28, 4);

			x_ /= 1000;
			y_ /= 1000;
			lock_.unlock();

			break;
		}
	}
	else
	{
		if (!ser_)
		{
			delete ser_;
			ser_ = NULL;
			OpenPort();
		}
	}
}

void Odometry::Update(vector<Message*> input, vector<Message*> output)
{
	LOG_IF(FATAL, input.size() != 2) << "Odometry's input msgs number"
		<< " not equal 2";
	stdmsg::Laser_Scan* laser_msg = (stdmsg::Laser_Scan*)input[0];
	stdmsg::WriteToOdometryMsg* write_msg = (stdmsg::WriteToOdometryMsg*)input[1];
	stdmsg::Laser_Scan* out = (stdmsg::Laser_Scan*)output[0];

	//Write2Ser(write_msg);
	msg_.CopyFrom(*write_msg);
	out->CopyFrom(*laser_msg);

	//lock_.lock();
	out->mutable_robot()->mutable_position()->set_x(x_);
	out->mutable_robot()->mutable_position()->set_y(y_);
	out->mutable_robot()->mutable_orentation()->set_yaw(theta_);
	//lock_.unlock();
}
}


