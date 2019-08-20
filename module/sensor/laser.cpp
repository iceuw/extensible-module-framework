#include "laser.hpp"

#include <sstream>

namespace agv_robot{

Laser::Laser(ConfigFile& cfg) : is_working_(false), seq_(0), update_thread_(this)
{
    ip_ = (std::string)cfg.value("laser", "ip", "192.168.0.1");
    port_ = cfg.value("laser", "port", 9001);
	std::cout << "laser's ip is" << ip_ << std::endl;
	std::cout << "laser's port is " << port_ << std::endl;
	range_max_ = 40.0;
    this->Start();
	this->update_thread_.start();
}
void Laser::Start()
{
    if(this->is_working_)
        return;
    try
    {
		std::cout << "start connect laser..." << std::endl;
        this->connect(ip_.c_str(), port_);
    }catch(int &e)
    {
        std::cerr << "connect exception: "  <<ip_<<":"<<port_<< ", try to reconnect ..." <<std::endl;
        return;
    }

    this->sendAll( "\x02sRN LMPscancfg\x03" );
    std::string recvd = recv(100);
    for( int i = 0; i < recvd.length(); i++)
        if(recvd[i] == '\x03') recvd[i] = ' ';
    std::stringstream data( recvd );

    std::string hex;
    data >> hex >> hex >> hex >> hex;

    #define helper__(x) {               \
        unsigned int tmp;               \
        data >> std::hex >> tmp;        \
        int v = tmp;                    \
        if(tmp >0x8000000)              \
            v = tmp - 1 - 0xFFFFFFFF;   \
        x = v;                          \
    }                                   \

    helper__(angle_increment_);//4
    angle_increment_ = angle_increment_/10000.0 * 3.141592653/180;
    helper__(angle_min_);//5
    angle_min_ = angle_min_/10000.0 *3.141592653/180 - 3.141592653/2;
    helper__(angle_max_);//6
    angle_max_ = angle_max_/10000.0 *3.141592653/180 - 3.141592653/2;


    ranges_.resize(int((angle_max_ - angle_min_)/angle_increment_ + 1));
	ranges_rssi_.resize(ranges_.size());

    sendAll("\x02sWN LMDscandatacfg 01 00 1 1 0 00 00 0 0 0 1 +1\x03");
    recv(100);
    sendAll("\x02sMN LMCstartmeas\x03");

    int status = 0;
    do
    {
        sendAll("\x02sRN STlms\x03");
        std::string recvd = recv(100);
        std::stringstream data( recvd );
        data>>hex>>hex>>status;
        sendAll("\x02sEN LMDscandata 1\x03");
        recv(100);

    }while(status != 7);

    fprintf(stderr,"angle = %.4f:%.4f:%.4f %dbeams ", 
                    angle_min_, 
                    angle_increment_, 
                    angle_max_, 
                    ranges_.size() );

    fprintf(stderr,"Laser is working now! \n");

    this->is_working_ = true;
}
Laser::~Laser()
{
}
void Laser::update()
{
    static char buffer_[2048];
    std::string buffer, rssi;
    int pos1 = 0, pos2 = 0;
    while ( (pos1 = buffer.find("DIST1")) == std::string::npos
            || (pos2 = buffer.find("\x03", pos1 ))  == std::string::npos )
    {
        int n;
        try{
            n = recv(buffer_, 2047);
        }catch(...){
            std::cerr<<"Laser is off, try to reconnect!"<<std::endl;
            this->is_working_ = false;
            this->Start();
            continue;
        }

        buffer_[n] = 0;
        buffer += buffer_;
    }
    buffer = buffer.substr( pos1 + strlen("DIST1"), pos2 );
	rssi = buffer.substr(buffer.find("RSSI1") + strlen("RSSI1"), pos2);

    char *p = &buffer[0];
	this->lock_.lock();
	try
	{
        strtok(p   , " ");
        strtok(NULL, " ");
        strtok(NULL, " ");
        strtok(NULL, " ");
        strtok(NULL, " ");
		for( int i = 0; i < ranges_.size(); i++)
		{
			unsigned int tmp; 
            sscanf( strtok(NULL, " "), "%x", &tmp );
			ranges_[i] = tmp / 1000.0;
            if(ranges_[i] < 0.2)
                ranges_[i] = this->range_max_;
		}
		p = &rssi[0];
		strtok(p, " ");
		for (int i = 0; i < 4; ++i)
			strtok(NULL, " ");
		for (int i = 0; i < ranges_rssi_.size(); i++)
		{
			unsigned int tmp;
			sscanf(strtok(NULL, " "), "%x", &tmp);
			ranges_rssi_[i] = tmp;
		}
        //for(int i =540; i < _ranges.size(); i++)
        //{
        //    _ranges[i] = this->_range_max;
        //}
        for(int i = 0; i < 0 ; i++)
        {
            ranges_[i] = this->range_max_;
        }
	}catch(...)
	{
		std::cerr << "Laser frame has no enough data"<<std::endl;
	}
	this->lock_.unlock();
}

void Laser::Update(vector<Message*> input, vector<Message*> output){
    //transform the output ptr to Laser_Scan msg
    stdmsg::Laser_Scan* scan = (stdmsg::Laser_Scan*)output[0];

    this->lock_.lock();

    if (scan->config().angle_max() != angle_max() ||
        scan->ranges_size() < ranges().size()) {
        scan->mutable_config()->set_angle_min(angle_min());
        scan->mutable_config()->set_angle_max(angle_max());
        scan->mutable_config()->set_angle_increment(angle_increment());
        scan->mutable_config()->set_range_max(range_max());
        for(int i = scan->ranges_size(); i < ranges_.size(); i ++)
        {
            scan->mutable_ranges()->Add();
            scan->mutable_ranges_rssi()->Add();
        }
    }

    for (int i = 0; i < ranges_.size(); ++i){
        scan->mutable_ranges()->Set(i, ranges_[i]);
        scan->mutable_ranges_rssi()->Set(i, ranges_rssi_[i]);
    }

    this->lock_.unlock();
}

} //end namespace agv_robot