#include "tcp_recv.h"

Recver::Recver()
{
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(31901);
}

Recver::~Recver()
{
    close(conn);
    close(sfd);
}

bool Recver::reset()
{
    close(conn);
    close(sfd);
    return true;
}

bool Recver::setup()
{
    if ((sfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        std::cout << "FAIL to create socket!" << std::endl;
        return false;
    }
    if (bind(sfd, (struct sockaddr *)&addr, sizeof(addr)) == -1)
    {
        std::cout << "FAIL to bind socket!" << std::endl;
        return false;
    }
    if (listen(sfd, 1) == -1)
    {
        std::cout << "FAIL to listen to port!" << std::endl;
        return false;
    }
    if ((conn = accept(sfd, (struct sockaddr *)NULL, NULL)) == -1)
    {
        std::cout << "FAIL to accept!" << std::endl;
        return false;
    }
    return true;
}

bool Recver::getvec(std::array<double, 2> &data)
{
    if ((len = recv(conn, recvbuff, 1024, 0)) == -1)
    {
        std::cout << "FAIL to receieve!" << std::endl;
        return false;
    }

    recvbuff[len] = '\0';
    // std::cout << "receieved: " << recvbuff << std::endl;
    if (!strcmp(recvbuff, "end") || len == 0)
        return false;    

    bool valid = false;
    int sec = -1;    
    for (int i = len - 1; i >= 0; i--)
    {             
        if (recvbuff[i] == ']')
            valid = true;
        if (valid && recvbuff[i] == ',')
            sec = i;
        if (valid && sec > 0 && recvbuff[i] == '[')
        {            
            data[0] = strtod(recvbuff+i+1, NULL);
            data[1] = strtod(recvbuff+sec+1, NULL);                  
            break;
        }
    }
    return true;
}

LoopRecv::LoopRecv()
{
    stop_flag = false;
    data.fill(0);
    if (!rec.setup())
    {
        std::cout << "FAIL to setup socket!" << std::endl;
        setup_flag = false;
    }
    else
    {
        setup_flag = true;
        t_get = std::thread(&LoopRecv::getVec, this);
    }
}

void LoopRecv::getParam(std::array<double, 2>& p)
{
    std::lock_guard<std::mutex> lock(mtx);
    p = data;
}

void LoopRecv::stop()
{
    stop_flag = true;
    t_get.join();
}

void LoopRecv::getVec()
{
    while(setup_flag && !stop_flag)
    {
        if (!rec.getvec(param))
        {
            std::cout << "FAIL to receieve params!" << std::endl;
            setup_flag = false;
            break;
        }
        else
        {
            mtx.lock();
            data = param;
            mtx.unlock();
        }
        sleep(0.005);
    }
}