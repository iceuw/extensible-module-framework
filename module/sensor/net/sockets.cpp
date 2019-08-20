
#include "sockets.hpp"

#ifdef WIN32
typedef int socklen_t;
#endif

namespace TD
{

Socket::Socket(int type, int domain, int protocol)
{       
#ifdef WIN32 // Brain fuc*
    int n= WSAStartup(MAKEWORD(2, 0), &wsa_data);
    if(n == -1) error(n);
#endif

    sockfd = socket(domain, type, protocol);
    if(sockfd <= 0) error(sockfd);

    char yes = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    my_addr.sin_family = domain;
    buffer_size = 0;
    allocate_size = 1024;
    buffer = new char[allocate_size];
}

Socket::Socket(int new_fd, bool flag)
{
    sockfd = new_fd;
    buffer_size = 0;
    allocate_size = 1024;
    buffer = new char[allocate_size];
}

Socket::~Socket()
{
    if(buffer)
        delete buffer;
}

void Socket::bind(int port)
{
    my_addr.sin_port = htons(port);
    my_addr.sin_addr.s_addr = INADDR_ANY;
    memset(my_addr.sin_zero, '\0', sizeof my_addr.sin_zero);

    int n = ::bind(sockfd, (struct sockaddr*)&my_addr, sizeof(my_addr));
    if(n == -1) error(n);
}

void Socket::listen(int backlog)
{
    int n = ::listen(sockfd, backlog);
    if(n == -1) error(n);
}

Socket Socket::accept()
{
    int new_fd;
    struct sockaddr_in their_addr;

    socklen_t size = sizeof(their_addr);

    new_fd = ::accept(sockfd, (struct sockaddr*)&their_addr, &size);
    if(new_fd <= 0) error(new_fd);

    return Socket(new_fd, true);
}

struct in_addr* Socket::getHostByName(const char* server)
{
    struct hostent *h;
    h = gethostbyname(server);

    if(h == 0) error(-1);
    return (struct in_addr*)h->h_addr;
}

const char* Socket::getPeerName()
{       
    struct sockaddr_in peer;
    socklen_t size = sizeof(peer);

    int n = getpeername(sockfd, (struct sockaddr*)&peer, &size);
    if(n == -1) throw(n);

    return inet_ntoa(peer.sin_addr);
}

const char* Socket::getHostName()
{
    char name[256];

    int n = gethostname(name, sizeof(name));
    if(n == -1) throw(n);

    return inet_ntoa(*getHostByName(name));
}

void Socket::connect(const char* server, int port)
{
    my_addr.sin_port = htons(port);
    my_addr.sin_addr = *getHostByName(server);
    memset(my_addr.sin_zero, '\0', sizeof(my_addr.sin_zero));
    
    int n = ::connect(sockfd, (struct sockaddr *)&my_addr, sizeof(my_addr));
    if(n == -1) error(n);
}

int Socket::send(std::string& msg)
{
    int ret;
    ret = ::send(sockfd, msg.c_str(), msg.size(), 0);
    if(ret <= 0) error(ret);

    return ret;
}

void Socket::setTimeout(unsigned int timeout)
{
    struct linger lin;

    if(timeout == 0)
        lin.l_onoff = 0;
    else
        lin.l_onoff = 1;

    lin.l_linger = timeout;

#ifdef WIN32
    setsockopt(sockfd, SOL_SOCKET, SO_LINGER, (char*)&lin, sizeof(lin));
#else
    setsockopt(sockfd, SOL_SOCKET, SO_LINGER, &lin, sizeof(lin));
#endif
}

int Socket::send(const char* msg)
{
    int ret;
    ret = ::send(sockfd, msg, strlen(msg), 0);
    if(ret <= 0) error(ret);

    return ret;
}

int Socket::sendAll(std::string& msg)
{
    int total = 0;
    int len = msg.size();
    int bytesleft = len;
    int n = -1;

    while(total < len)
    {
        n = ::send(sockfd, msg.c_str() + total, bytesleft, 0);
        if (n <= 0) { throw(n); }
        total += n;
        bytesleft -= n;
    }
    
    return len;
}

int Socket::sendAll(const char* msg)
{
    int total = 0;
    int len = strlen(msg);
    int bytesleft = len;
    int n = -1;

    while(total < len)
    {
        n = ::send(sockfd, msg + total, bytesleft, 0);
        if (n <= 0) { throw(n); }
        total += n;
        bytesleft -= n;
    }

    return len;
}

int Socket::sendln(const char* msg)
{
    return (sendAll(msg) + sendAll("\n"));
}

int Socket::sendln(std::string& msg)
{
    return (sendAll(msg) + sendAll("\n"));
}

int Socket::recv(char* buffer, int len, int flags)
{

    int n = ::recv(sockfd, buffer, len, flags);
    if(n <= 0) error(n);

    return n;
}
std::string Socket::recv( int len, int flags)
{
    char* buffer = new char[len];
    int n = ::recv(sockfd, buffer, len, flags);
    if(n <= 0) error(n);
    std::string ret(buffer);
    delete []buffer;
    return ret;
}

void Socket::close()
{
#ifdef WIN32
    WSACleanup();
    closesocket(sockfd);
#else
    ::close(sockfd);
#endif
}

void Socket::shutdown(int type)
{
    int n;

#ifdef WIN32
    WSACleanup();
#endif

    n = ::shutdown(sockfd, type);

    if(n != 0) error(n);
}

void Socket::error(int code)
{
    throw(code);
}

void Socket::clearBuffer()
{       
    if(buffer_size > 0)
    {
        free(buffer);
        buffer_size = 0;
    }
}

}

