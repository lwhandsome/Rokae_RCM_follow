#include <iostream>
#include <array>
#include <mutex>
#include <thread>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

class Recver
{
public:
    Recver();
    ~Recver();
    bool setup();
    bool getvec(std::array<double, 2> &data);
    bool reset();

private:
    int sfd, conn, len;
    char recvbuff[1024];
    struct sockaddr_in addr;
};

class LoopRecv
{
public:
    LoopRecv();
    void getParam(std::array<double, 2> &p);
    void stop();

private:
    void getVec();

private:
    Recver rec;
    std::array<double, 2> data, param;
    bool setup_flag, stop_flag;
    std::mutex mtx;
    std::thread t_get;
};