#ifndef LASER_HPP_
#define LASER_HPP_

#include "system/function_block.hpp"

#include <string>
#include <vector>
#include <iostream>

#include "thread.hpp"
#include "net/sockets.hpp"
#include "utils/configfile.h"

namespace agv_robot{

#define helper__(type, x)           \
private:                            \
type x##_;                          \
public:                             \
inline type x(){                    \
    lock_.lock();type tmp = x##_;   \
    lock_.unlock();                 \
    return tmp;                     \
}

class Laser : private TD::Socket, public FunctionBlock
{
private:

    struct UpdateThread : public BThread {
        Laser* handle;
        UpdateThread(Laser* p){
            handle = p;
        }
        ~UpdateThread(){
            kill();
        }
        void run(){
            while(true){
                if(handle){
                    handle->update();
                    handle->seq_++;
                }
            }
        }
    }update_thread_;

	//Laser(const std::string& ip, int port);
	void update();
    void Start();

	helper__(double, angle_min);
	helper__(double, angle_max);
	helper__(double, angle_increment);
	helper__(double, range_max);
	helper__(std::vector<double>, ranges);
	helper__(std::vector<double>, ranges_rssi);

	BMutex lock_;
    bool is_working_;
    std::string ip_;
    int port_;
    int seq_;

public:

    Laser(ConfigFile& cfg);
	~Laser();
    
    void Update(vector<Message*> input, vector<Message*> output);

};

EXPORT_INSTANCE(Laser)


#undef helper__

}

#endif //end LASER_HPP_