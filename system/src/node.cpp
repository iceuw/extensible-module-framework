#include "node.hpp"
#include <sstream>
#include <string.h>
namespace agv_robot 
{

void Node:: connect(const std::string &uri)
{
    this->_subscriber.connect(uri.c_str());
}
int Node::run(double timeout)
{
    /* check if data have arrived with timeout */
    if( timeout > 0 )
    {
        zmq::pollitem_t items =  { this->_subscriber, 0, ZMQ_POLLIN, 0 } ;
        int rc = zmq::poll(&items, 1, 1000 * timeout);
        assert(rc == 0);
        if ( ! (items.revents & ZMQ_POLLIN) )
            return -1;
    }
    
    /* receive msg with topic */
    zmq::message_t topic, message;
    this->_subscriber.recv(&topic);
    this->_subscriber.recv(&message);
    
    /* lookup to table to find callback function */
    std::map< std::string, std::vector<_callback_func*> >::iterator iter; 
    iter = this->_handlertable.find( std::string(static_cast<char*>(topic.data()), topic.size()) );
    
    if (iter == this->_handlertable.end() )
        return -1;
    
    //std::string data(static_cast<char*>(message.data()), message.size());
    std::vector<_callback_func*>::iterator it;
    it = iter->second.begin();
    for ( ; it != iter->second.end(); it++)
        (**it) ( message.data(), message.size() );
	return 0;
}

Node::Node( const std::string& name ):
    _context(), _publisher(_context, ZMQ_PUB), _subscriber (_context, ZMQ_SUB)
{
    this->_publisher.bind(name.c_str());
}
Node::~Node()
{
    std::map< std::string, std::vector<_callback_func*> >::iterator iter; 
    iter = this->_handlertable.begin();
    for(;iter!=this->_handlertable.end();++iter)
    {        
        std::vector<_callback_func*>::iterator it;
        it = iter->second.begin();
        for ( ;it != iter->second.end(); it++)
            delete (*it);
    }
}
    
void Node::add_handle(const std::string &name, _callback_func* func)
{
    std::map< std::string, std::vector<_callback_func*> >::iterator iter;
    
    iter = this->_handlertable.find(name);
    
    if( iter != this->_handlertable.end() ){
        iter->second.push_back( func );
    }
    else
    {
        std::vector<_callback_func*> second;
        second.push_back( func );
        this->_handlertable[name] = second;
    }
    
}


RPC::RPC():
	_context(), _server( NULL ), _client (NULL)
{
		
};

RPC::~RPC()
{
	if(_client)
		delete _client;
	if(_server)
		delete _server;
}

int RPC::run(double timeout)
{
	if(!_server)
		throw exception  ("no server is runing!");

	/* check if data have arrived with timeout */
	if( timeout > 0 )
	{
		zmq::pollitem_t items[] = { { *_server, 0, ZMQ_POLLIN, 0 } };
        int rc = zmq::poll (&items[0], 1, timeout * 1000 );
		std::cout<<rc<<std::endl;
		assert(rc == 0);
		if ( ! (items[0].revents & ZMQ_POLLIN) )
			return -1;
	}

	/* receive msg  */
	zmq::message_t  message;
	this->_server->recv(&message);
	char *buffer = static_cast<char*>( message.data() );
	/*for(int i = 0; i < message.size(); ++i)
			printf("%02x ", buffer[i]);
	std::cout<<std::endl;
	*/

	/* lookup to table to find callback function */
	// split the buffer to two part
	char *p;
	if( p = strchr(buffer,'\n') )
		*p = 0;

	if(strncmp( buffer, _RPC, message.size() ) != 0) 
	{
		throw exception ("RPC frame fatal error!");
	}
	buffer = ++p;

	if( p = strchr(buffer,'\n') )
		*p = 0;
	char* api = buffer;
	p++;
	unsigned long data_size = static_cast<char*>( message.data() ) + message.size() - p;

	//std::cout<<api<<std::endl;
	std::map< std::string,_rcallback_func* >::iterator iter; 
	iter = this->_handlertable.find( std::string( api ) );
    
	if (iter == this->_handlertable.end() )
		return -1;
    
	_rcallback_func* it = iter->second;
	if(it)
	{
		_return ret;
		try{
			ret = (*it) ( p, data_size );
		}catch(...)
		{
			if(ret._buffer)
				delete []ret._buffer;
			throw exception ( "RPC call failed");
		}

		// reply to the client
		this->_server->send( ret._buffer, ret._size);
		if(ret._buffer)
			delete []ret._buffer;
	}
	return 0;
}
void RPC::connect(const std::string& address)
{
	if(_client )
		delete _client;
	_address = address;

	_client = new zmq::socket_t (_context, ZMQ_REQ);
	_client->connect ( _address.c_str() );
	//  Configure socket to not wait at close time
	int linger = 0;
	_client->setsockopt (ZMQ_LINGER, &linger, sizeof (linger));
}
void RPC::bind(const std::string& name)
{
	if(_server)
		delete _server;
	_server = new zmq::socket_t(_context, ZMQ_REP );
	_server->bind( name.c_str() );
}

void RPC::add_handle(const std::string &name, _rcallback_func* func)
{
	std::map< std::string, _rcallback_func* >::iterator iter;
    
	iter = this->_handlertable.find(name);
    
	if( iter != this->_handlertable.end() ){
		iter->second = func;
	}
	else
	{
		this->_handlertable[name] = func;
	}
}

void RPC::add_handle(std::map<std::string, _rcallback_func *> rcallback_funcs)
{
	for (auto it = rcallback_funcs.begin();
		it != rcallback_funcs.end();
		++it)
	{
		add_handle(it->first, it->second);
	}
}
}
