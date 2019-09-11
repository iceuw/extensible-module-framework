#ifndef ODOMETRY_
#define ODOMETRY_


#include "function_block.hpp"
#include "thread_utils.hpp"
#include "serial/serial.h"

namespace agv_robot
{

#define MAX_READ_BYTE 128

class Odometry : public FunctionBlock 
{
public:
	Odometry(ConfigFile& cfg);
	void Update(vector<Message*> input, vector<Message*> output);
	stdmsg::WriteToOdometryMsg msg_;
private:
	struct UpdateOdomThread : public BThread
	{
		UpdateOdomThread(Odometry* handle)
		{
			handle_ = handle;
		}
		~UpdateOdomThread()
		{
			kill();
		}
		void run()
		{
			while (true)
			{
				if (handle_) {
					handle_->ReadFromSer();
					handle_->Write2Ser(handle_->msg_);
				}
			}
		}
		Odometry* handle_ = NULL;
	}update_odom_thread_;

	void OpenPort();
	void Write2Ser(stdmsg::WriteToOdometryMsg msg);
	void ReadFromSer();
	string port_;
	int baudrate_;

	serial::Serial* ser_;
	BMutex lock_;

	float x_, y_, theta_;
	float v_, w_;
	float last_x_, last_y_, last_theta_;
	float origin_x_, origin_y_, origin_theta_;
	

	int send_index_;
};
EXPORT_INSTANCE(Odometry)
}//end namespace

#endif