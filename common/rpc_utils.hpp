#ifndef RPC_UTILS_
#define RPC_UTILS_
#include <string>
#include <map>
#include <vector>
#include <iostream>


namespace agv_robot 
{

class exception:public std::exception
{
public:
	inline exception(const std::string& name)
	{
		m_errmsg = name;
	}
	inline ~exception() throw()
	{};
	inline virtual const char* what() const throw()
	{
		return m_errmsg.c_str();
	}
private:
	std::string m_errmsg;
};

struct _callback_func 
{
    virtual void operator()(void *p, int size ) const = 0;
};

template <class TYPE>
struct _callback1 : public _callback_func 
{
    _callback1(void (*func)(const TYPE& data))
    {
        this->_func = func;
        this->_data = new TYPE;
    }
    ~_callback1()
    {
        delete this->_data;
    }
    virtual void operator()(void *p, int size ) const
    {
        //for(int i = 0; i < size; i++)
        //    printf("%2x ",((char*)p)[i]);
        //std::cout<<std::endl;
        _data->ParseFromArray(p, size);
        this->_func(*this->_data);
    }
    
    void (*_func)(const TYPE& data);
    TYPE* _data;
};
template <class BASECLASS, class TYPE>
struct _callback2 : public _callback_func 
{
    _callback2(void (BASECLASS::*func)(const TYPE& data), BASECLASS *instance)
    {
        this->_instance = instance;
        this->_func = func;
        this->_data = new TYPE;
    }
    ~_callback2()
    {
        delete this->_data;
    }
    virtual void operator()(void *p, int size ) const
    {

        _data->ParseFromArray(p, size);
        (_instance->*_func) (*this->_data);
    }
    
    void (BASECLASS::*_func)(const TYPE& data);
    BASECLASS *_instance;
    TYPE *_data;
};

struct _return
{
	inline _return(void *p = NULL, unsigned int size = 0)
		:_buffer((char*)p),_size(size)
	{
	}
	inline _return(const _return& ret)
		:_buffer(ret._buffer), _size(ret._size)
	{
	}
	inline _return& operator = (const _return & ret)
	{
		_buffer = ret._buffer;
		_size = ret._size;
		return * this;
	}
	char *_buffer;
	int _size;
};
struct _rcallback_func 
{
    virtual _return operator()(void *p, int size ) const = 0;
};
template < class TYPE, class RET>
struct _rcallback1 : public _rcallback_func 
{
    _rcallback1(RET (*func)(const TYPE& data))
    {
        this->_func = func;
        this->_data = new TYPE;
    }
    ~_rcallback1()
    {
        delete this->_data;
    }
    virtual _return operator()(void *p, int size ) const
    {
		_data->ParseFromArray(p, size);
        RET data = this->_func (*this->_data);
		void *rp = new unsigned char [data.ByteSize()];
        data.SerializeToArray(rp, data.ByteSize());
		return _return( rp, data.ByteSize() );
    }
    
    RET (*_func)(const TYPE& data);
    TYPE* _data;
};
template < class BASECLASS, class TYPE, class RET>
struct _rcallback2 : public _rcallback_func 
{
    _rcallback2(RET (BASECLASS::*func)(const TYPE& data), BASECLASS *instance)
    {
        this->_instance = instance;
        this->_func = func;
        this->_data = new TYPE;
    }
    ~_rcallback2()
    {
        delete this->_data;
    }
    virtual _return operator()(void *p, int size ) const
    {
        _data->ParseFromArray(p, size);
        RET data = (_instance->*_func) (*this->_data);
		void *rp = new unsigned char [data.ByteSize()];
        data.SerializeToArray(rp, data.ByteSize());
		return _return( rp, data.ByteSize() );
    }
    
    RET (BASECLASS::*_func)(const TYPE& data);
    BASECLASS *_instance;
    TYPE *_data;
};

class RPCUtils
{
public:
	std::map<std::string, _rcallback_func* > _rhandlertable; 
    //std::map<std::string, std::vector<_callback_func*> > _handlertable; 

	void add_handle(const std::string &name, _rcallback_func* func)
	{
		std::map< std::string, _rcallback_func* >::iterator iter;

		iter = this->_rhandlertable.find(name);

		if (iter != this->_rhandlertable.end())
		{
			iter->second = func;
		}
		else
		{
			this->_rhandlertable[name] = func;
		}
	}
	//template<typename BASECLASS, typename TYPE >
	//void Set(const std::string &name,
	//	void (BASECLASS::*_func)(const TYPE&),
	//	BASECLASS* instance, int size = 1)
	//{
	//	//this->_subscriber.setsockopt(ZMQ_SUBSCRIBE, name.data(), size);
	//	_callback2<BASECLASS, TYPE >* func = new _callback2<BASECLASS, TYPE >(_func, instance);

	//	add_handle(name, func);
	//}

	//void add_handle(const std::string &name, _callback_func* func)
	//{
	//	std::map< std::string, std::vector<_callback_func*> >::iterator iter;

	//	iter = this->_handlertable.find(name);

	//	if (iter != this->_handlertable.end())
	//	{
	//		iter->second.push_back(func);
	//	}
	//	else
	//	{
	//		std::vector<_callback_func*> second;
	//		second.push_back(func);
	//		this->_handlertable[name] = second;
	//	}
	//}
};
};

#endif