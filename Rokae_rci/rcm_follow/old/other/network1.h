#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
using namespace std;

class CtrlParam
{
public:
    double x;
    double y;
};

class Recver
{
private:
    int sfd, conn, len;
    char recvbuff[1024];
    struct sockaddr_in addr;

public:
    Recver(/* args */);
    ~Recver();
    bool setup();
    bool getvec(CtrlParam &params);
    bool reset();
};

Recver::Recver(/* args */)
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
        cout << "FAIL to create socket!" << endl;
        return false;
    }
    if (bind(sfd, (struct sockaddr *)&addr, sizeof(addr)) == -1)
    {
        cout << "FAIL to bind socket!" << endl;
        return false;
    }
    if (listen(sfd, 1) == -1)
    {
        cout << "FAIL to listen to port!" << endl;
        return false;
    }
    if ((conn = accept(sfd, (struct sockaddr *)NULL, NULL)) == -1)
    {
        cout << "FAIL to accept!" << endl;
        return false;
    }
    return true;
}

bool Recver::getvec(CtrlParam &params)
{
    if ((len = recv(conn, recvbuff, 1024, 0)) == -1)
    {
        cout << "FAIL to receieve!" << endl;
        return false;
    }

    recvbuff[len] = '\0';
    // cout << "receieved: " << recvbuff << endl;
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
            params.x = strtod(recvbuff+i+1, NULL);
            params.y = strtod(recvbuff+sec+1, NULL);                  
            break;
        }
    }
    return true;
}
