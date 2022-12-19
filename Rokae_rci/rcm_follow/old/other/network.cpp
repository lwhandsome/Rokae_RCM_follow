#include "network1.h"

int main()
{
    CtrlParam param;
    Recver rec;
    if (!rec.setup())
    {
        cout << "FAIL to setup socket!" << endl;
        return 0;
    }
    while(1)
    {
        if (!rec.getvec(param))
        {
            cout << "FAIL to receieve params!" << endl;
            break;
        }
        else
            cout << "x= " << param.x << " y= " << param.y << endl;
    }
   
        
    return 0;
}