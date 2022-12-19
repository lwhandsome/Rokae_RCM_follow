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
    Recver();
    ~Recver();
    bool setup();
    bool getvec(CtrlParam &param);
};

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
    cout << "Setup SUCCESS! Now Listening to port: 31901" << endl;
    return true;
}

bool Recver::getvec(CtrlParam &param)
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
    for (int i = 0; i < len; i++)
    {
        if (recvbuff[i] == ',')
        {
            param.x = strtod(recvbuff, NULL);
            param.y = strtod(recvbuff + i + 1, NULL);
            // cout << "x= " << *x << endl;
            // cout << "y= " << *y << endl;
            break;
        }
    }
    return true;
}
