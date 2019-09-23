#ifndef NODEH
#define NODEH
#include <string>
#include <map>
#include <vector>
#include <iostream>
#include "rpc_utils.hpp"
#include "zmq.hpp"


namespace agv_robot
{

class Node
{
public:
    Node( const std::string& name );
    
    virtual ~Node();
    
    template <class TYPE>
    void subscrible(const std::string& name,void (*_func)(const TYPE&) , int size = 1)
    {
        this->_subscriber.setsockopt( ZMQ_SUBSCRIBE, name.data(), size);
        _callback1<TYPE > *func = new _callback1<TYPE >(_func);
        add_handle(name, func);
    };
    
    template<typename BASECLASS, typename TYPE >
    void subscrible(const std::string &name, 
                    void (BASECLASS::*_func)(const TYPE&), 
                    BASECLASS* instance, int size = 1)
    {
        this->_subscriber.setsockopt( ZMQ_SUBSCRIBE, name.data(), size);
        _callback2<BASECLASS, TYPE >* func = new _callback2<BASECLASS, TYPE >(_func, instance);
        
        add_handle(name, func);
    }
    

    /* publish data, the data should be serializable, 
    here with google protobuf */
    template<typename TYPE>
    void publish(const std::string &name, const TYPE& data)
    {
        bool rc;
        
        rc = this->_publisher.send (name.c_str(),name.size(), ZMQ_SNDMORE);
        assert(rc == true);
        
        void *p = new unsigned char [data.ByteSize()];
        data.SerializeToArray(p, data.ByteSize());

        rc = this->_publisher.send (p, data.ByteSize());
        delete[] p;
        assert(rc == true);
    }
    
    int run(double timeout = -1);
    void connect(const std::string &uri);
	
protected:
    void add_handle(const std::string &name, _callback_func* func);
private:
    std::map<std::string, std::vector<_callback_func*> > _handlertable; 
    zmq::context_t _context;
    zmq::socket_t _publisher;
    zmq::socket_t _subscriber;
};

class RPC
{

private:
	#define _RPC "RPC"
	//RPC\3method\3google_protocol_buffer_data           //no space allowed between section and data:
	zmq::context_t _context;
	zmq::socket_t *_server;
	zmq::socket_t *_client;
	std::string _address;
	std::map<std::string, _rcallback_func* > _handlertable; 
public:
	RPC();
	~RPC();
	int run(double timeout = -1);
    void connect(const std::string& address);
	void bind(const std::string& name);

	/*
	 * timeout should be bigger then 1 second
	 * before abandon, the client will try trise_times
	 */
	template <class TYPE1,class TYPE2>
    TYPE1 call(const std::string& name, TYPE2 data, int tries_time = 1, double timeout = 1.5)
    {
		if( _address == "" )
			throw exception("not connect to the server, please give the adress and connect it");
		TYPE1 ret;

		//serialize the parameter to string
		std::string head = _RPC;
		head = head + "\n" + name + "\n";
		char *p = new char [ head.length() + data.ByteSize()];
		memcpy(p, head.c_str(), head.length());
		data.SerializeToArray(p + head.length(), data.ByteSize());
		/*
		for(int i = 0; i <  head.length() + data.ByteSize(); ++i)
			printf("%02x ", p[i]);
		std::cout<<std::endl;*/

		bool expect_reply = false;
		while(tries_time--)
		{
			//send the string to the server
			int rc = this->_client->send (p, head.length() + data.ByteSize());
			assert(rc == true);

			//try to receive from the server
			zmq::pollitem_t items[] = { { *_client, 0, ZMQ_POLLIN, 0 } };
            zmq::poll (&items[0], 1, timeout  * 1000);
			
			zmq::message_t msg;
			//  If we got a reply, process it
            if (items[0].revents & ZMQ_POLLIN) 
			{
				expect_reply = true;
				this->_client->recv(&msg);
				ret.ParseFromArray(msg.data(), msg.size());
            }else
			{
				//reconnect
				_client = new zmq::socket_t (_context, ZMQ_REQ);
				_client->connect ( _address.c_str() );
				//  Configure socket to not wait at close time
				int linger = 0;
				_client->setsockopt (ZMQ_LINGER, &linger, sizeof (linger));
			}
		}
		
        delete[] p;
		
		if(!expect_reply)
			throw exception("server seems not running now!");
		else 
			return ret;
        
    };
    
    template<typename BASECLASS, typename TYPE, typename RET >
    void set(const std::string &name, 
                    RET (BASECLASS::*_func)(const TYPE&), 
                    BASECLASS* instance, int size = 1)
    {
        _rcallback2<BASECLASS, TYPE, RET >* func = new _rcallback2<BASECLASS, TYPE, RET >(_func, instance);
       
        add_handle(name, func);
    }

	template <class RET, class TYPE>
	void set(const std::string& name, RET (*_func)(const TYPE&) , int size = 1)
    {
        _rcallback1<TYPE, RET > *func = new _rcallback1<TYPE, RET>(_func);
        add_handle(name, func);
    };
    void add_handle(const std::string &name, _rcallback_func* func);
	void add_handle(const std::map<std::string, _rcallback_func* > rcallback_funcs);
protected:
};
};

#endif