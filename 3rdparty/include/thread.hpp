#ifndef BTHREAD_HPP
#define BTHREAD_HPP


// Use Win or Posix
#ifdef _WIN32
#define WIN32
#endif
#ifdef WIN32 
	#define WINDOWS
	#include <windows.h>
#else
    #define POSIX
	#ifndef POSIX
		#warning POSIX will be used (but you did not define it)
	#endif
	#include <pthread.h>
	#include <signal.h>
#endif



class BThread{
private:
	/**
	* @brief Denied equality operator
	* @param none
	*/
	void operator=(const BThread &){}
	/**
	* @brief Denied copy constructor
	* @param none
	*/
	inline BThread(const BThread &){}

#ifdef WINDOWS
 	HANDLE _handle;
#else
	pthread_t _thread; /**< Posix Thread*/
#endif
	
	volatile bool _isRunning;  /**< Fast bool lookup */


	/**
	* @brief Static starter function to execute posix thread
	* @brief This function set thread->isRunning to false
	*/
	#ifdef WINDOWS
 	static DWORD WINAPI Starter(LPVOID in_thread){
	#else
	static void* Starter(void* in_thread){
	#endif
		BThread * thread = static_cast< BThread * >(in_thread);
		thread->_isRunning = true;
		thread->run();
		thread->_isRunning = false;
		
		return 0x00;
	}

public:
	/**
	* @brief Constructor
	*/
	inline BThread(){
	#ifdef WINDOWS
 		_handle = 0x00;
	#else
	#endif
		_isRunning = false;
	}
	/**
	* @brief Destructor, Warning, it waits the end of the current thread
	*/
	inline virtual ~BThread(){
		if(!_isRunning) return;
        
        // if we destroy the thread until it has finished
        // there is a problem in your implementation algorithme
        // So we wait before destroying the thread!
        wait();
	#ifdef WINDOWS
		CloseHandle (_handle);
	#else
	#endif
	}

	/**
	* @brief start the thread
	* @return true if success else false
	*/
	inline bool start(){
		if(_isRunning) return false;		
	#ifdef WINDOWS
		_handle = CreateThread( 0x00, 0x00,BThread::Starter, static_cast< void* >(this), 0x00, 0x00);
		return _handle != NULL;
	#else
		return pthread_create(&_thread, NULL, BThread::Starter, static_cast< void* >(this)) == 0;
	#endif
	}

	/**
	* @brief Fast look up to know if a thread is running
	* @return true if running else false
	*/
	inline volatile bool isRunning() const{
		return _isRunning;
	}

	/**
	* @brief Wait the end of a thread
	* @return false in case of error, true if all right
	*/
	inline bool wait() const{
 		if(!_isRunning) return false;
	#ifdef WINDOWS
		return WaitForSingleObject(_handle,INFINITE) == 0x00000000L;
	#else
		return pthread_join(_thread, NULL) == 0;
	#endif

	}

	/**
	* @brief the function is called when thread is starting
	* @must You must implement this methode!
	*/
	inline virtual void run() = 0;
	
	/**
	*
	*/
	inline bool kill(){
		if(!_isRunning) return false;	
		
        _isRunning = false;	
	#ifdef WINDOWS
		bool success = TerminateThread(_handle,1) && CloseHandle(_handle);
		_handle = 0x00;
 		return success;
	#else
 		return pthread_kill( _thread, SIGKILL) == 0;
	#endif
	}

};


class BMutex{
private:

#ifdef WINDOWS
	CRITICAL_SECTION _mutex; /**< Window mutex */
#else
	pthread_mutex_t _mutex; /**< posix mutex */
#endif

	bool _locked;			/**< Fast locked look up used for copying */
	
	inline void init(){
	#ifdef WINDOWS
		InitializeCriticalSection(&_mutex);
	#else
		pthread_mutexattr_t attr;
		pthread_mutexattr_init(&attr);
		pthread_mutexattr_settype(&attr,PTHREAD_MUTEX_RECURSIVE);
		pthread_mutex_init(&_mutex,&attr);
		pthread_mutexattr_destroy(&attr);
	#endif
		_locked = false;
	}

public:	

	/**
	* @brief Construct a BMutex
	* @brief Posix and Win mutex
	*/
	inline BMutex(){
		init();
	}
	/**
	* @brief Copy Constructor a mutex (copy the locked state only)
	* @param Based mutex
	* 
	*/
	inline BMutex( const BMutex &in_mutex ) {
		init();
			
		if(in_mutex._locked && !_locked) lock();
		else if(!in_mutex._locked && _locked) unlock();
	}
	
	/**
	* @brief Copy a mutex (copy the locked state only)
	* @param Based mutex
	* @return Current mutex
	*/
	inline BMutex& operator=(const BMutex &in_mutex) {
		if(in_mutex._locked && !_locked) lock();
		else if(!in_mutex._locked && _locked) unlock();
		return *this;
	}
	
	
	/**
	* @brief Destructor
	*/
	inline virtual ~BMutex(){
	#ifdef WINDOWS
		DeleteCriticalSection(&_mutex);
	#else
		pthread_mutex_unlock(&_mutex);
		pthread_mutex_destroy(&_mutex);
	#endif
	}

	/**
	* @brief lock a mutex
	* @return WIN true
	* @return POSIX true if success
	*/
	inline bool lock(){ 
		_locked = true;
	#ifdef WINDOWS
		EnterCriticalSection(&_mutex);
		return true;
	#else
		return pthread_mutex_lock(&_mutex) == 0; 
	#endif
	}

	/**
	* @brief lock a mutex
	* @return true if success else false (if busy or error)
	*/
	inline bool tryLock(){ 
		_locked = true;
	#ifdef WINDOWS
		return TryEnterCriticalSection(&_mutex); 
	#else
		return pthread_mutex_trylock(&_mutex) == 0;
	#endif
	}

	/**
	* @brief unlock a mutex
	* @return WIN true in every cases
	* @return POSIX true if success
	*/
	inline bool unlock(){ 
		_locked = false;
	#ifdef WINDOWS
		LeaveCriticalSection(&_mutex);
		return true;
	#else
		return pthread_mutex_unlock(&_mutex) == 0;
	#endif
	}
	
	/**
	* @brief Fast locked look up
	* @return true if locked else falses
	* This methode use the fast look up variable but still CONST
	* if you want to test perfectly do :
	* if(myMutex.tryLock()){
	* 	myMutex.unlock();
	* 	// I am sure that the mutex is not locked
	* }
	* else{
	* 	// The mutex is locked
	* }
	*/
	inline bool isLocked() const{
		return _locked;
	}


};

#ifndef WINDOWS
class BCond
{
private:
    pthread_cond_t _ready;
    pthread_mutex_t _mutex;
	//volatile bool is_setup;
public:
    inline BCond()
    {
        pthread_mutex_init(&_mutex,NULL);
        pthread_cond_init (&(this->_ready), NULL);
    }
    inline ~BCond()
    {
        pthread_cond_destroy(&(this->_ready));
        pthread_mutex_unlock(&_mutex);
        pthread_mutex_destroy(&_mutex);
    }
    inline void wait(double timeout = -1)
    {
        if(timeout<0)
            pthread_cond_wait(&(this->_ready),   &(this->_mutex) );
        else
        {
            struct timespec abstime;
            abstime.tv_sec = int(timeout);
            abstime.tv_nsec = int( (timeout - int(timeout)) * 1e9);
            pthread_cond_timedwait(&(this->_ready),   &(this->_mutex), &abstime) ;
        }
		//is_setup = false;
    }
    inline void wakeup()
    {
		//if(!is_setup){
	        pthread_cond_signal(&(this->_ready));
			//is_setup = true;
		//}
    }
};
#else
class BCond
{
private:
    HANDLE hEvent;
	//volatile bool is_setup;
public:
    inline BCond()//:is_setup(false)
    {
        hEvent = CreateEvent(NULL, TRUE, TRUE, NULL);
		ResetEvent(hEvent);
    }
    inline ~BCond()
    {
        CloseHandle(hEvent);
    }
    inline void wait(double timeout = -1)
    {
        if(timeout<0)
            DWORD dReturn = WaitForSingleObject(hEvent,INFINITE);
        else
        {
            DWORD dReturn = WaitForSingleObject(hEvent,INFINITE);
        }
		ResetEvent(hEvent);
		//is_setup = false;
    }
    inline void wakeup()
    {
		//if(!is_setup){
	        SetEvent(hEvent);
		//	is_setup = true;
		//}
    }
};
#endif


#endif
