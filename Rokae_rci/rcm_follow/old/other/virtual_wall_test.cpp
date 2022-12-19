/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * 轴空间阻抗运动（s规划）
 */

#include <cmath>
#include <functional>
#include <unistd.h>
#include <vector>
#include <iostream>
#include "robot.h"
#include "ini.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "duration.h"
#include "move.h"

#include "model.h"
//#include "Min_convex.h"       // 虚拟墙头文件
#include <Eigen/Dense>



using namespace Eigen;
using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;


double dis_limit = 0.02;       // 设定安全阈值
double act_len = 0.28;          // 机械臂末端执行器长度

#define M 50
#define my_eps 0.00001
#define inf 0x3f3f3f3f
#define mod 1070000009
#define PI 3.141592653
using namespace std;

// µã
struct node
{
    double x, y, z;      //nodeµÄÔªËØ
    node() {}
    node(double xx, double yy, double zz) :x(xx), y(yy), z(zz) {}

    //ÔËËã·ûÖØÔØ
    node operator +(const node p)	//ÏòÁ¿ŒäÏàŒÓ²Ù×÷
    {
        return node(x + p.x, y + p.y, z + p.z);
    }
    node operator -(const node p)	//ÏòÁ¿ŒäÏàŒõ²Ù×÷
    {
        return node(x - p.x, y - p.y, z - p.z);
    }
    node operator *(const node p)	//ÏòÁ¿Œä²æ³Ë²Ù×÷
    {
        return node(y * p.z - z * p.y, z * p.x - x * p.z, x * p.y - y * p.x);
    }
    node operator *(const double p)	//ÏòÁ¿³ËÒÔÒ»žöÊý
    {
        return node(x * p, y * p, z * p);
    }
    node operator /(const double p)	//ÏòÁ¿³ýÒÔÒ»žöÊý
    {
        return node(x / p, y / p, z / p);
    }
    double operator ^(const node p)	//ÏòÁ¿Œäµã³Ë²Ù×÷
    {
        return x * p.x + y * p.y + z * p.z;
    }
};


// ±íÃæÈýœÇÐÎ
struct face
{
    int a, b, c;  // Í¹°üÒ»žöÃæÉÏµÄÈýžöµãµÄ±àºÅ
    int ok;       // žÃÃæÊÇ·ñÊÇ×îÖÕÍ¹°üÖÐµÄÃæ
    
    face() {}
};


// ¶šÒå±íÊŸÈýÎ¬Í¹°üµÄÀà
// ÔÚÀàÖÐ¶šÒåËùÐèÓÃµœµÄÒ»Ð©±äÁ¿ºÍº¯Êý
class Convex_hull
{   
public:

    Convex_hull() {
        cnt = 0;
        Input_node_num = 0;
        Convex_node_num = 0;
        Convex_face_num = 0;
        cout << "Convex_hull" << endl;
    }

    ~Convex_hull() {}

    
    void Sum_node_number(int num) { Input_node_num = num; }   // ¶ÁÈëInput_node_numÊýÄ¿
  
    void Input_node(int index, node point) {  p[index] = point; }         // ¶ÁÈëµã

    double len(node p)      // ÏòÁ¿µÄ³€¶È¡¢Ä£
    {
        return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    double area(node a, node b, node c)       // ÈýžöµãµÄÃæ»ý*2
    {
        return len((b - a) * (c - a));
    }

    double volume(node a, node b, node c, node d)  // ËÄÃæÌåÌå»ý*6
    {
        return (b - a) * (c - a) ^ (d - a);
    }


    // ¶šÒåptofº¯Êý£¬Íš¹ýÓÐÏÞÌå»ý·šÓÃÀŽÅÐ¶ÏµãÓëÃæµÄ·œÏò
    // ÈôÎªÕý£ºµãÓëÃæÍ¬Ïò
    double ptof(node q, face f);

    // ¶šÒådealºÍdfsº¯Êý£¬ÓÃÀŽžüÐÂÍ¹°ü
    void deal(int q, int a, int b);

    // Î¬»€Í¹°ü£¬ÈôµãqÔÚÍ¹°üÍâÔòžüÐÂÍ¹°ü
    void dfs(int q, int cur);

    // ÅÐ¶ÏÁœžöÈýœÇÐÎÊÇ·ñ¹²Ãæ£¬ÁœžöÈýœÇÐÎ¹²ÃæÔò·µ»Ø1
    int same(int s, int t);

    // ¹¹œš3DÍ¹°ü
    void make();

    // ÅÐ¶ÏÒ»žöµãÊÇ·ñÔÚÍ¹°üÄÚ²¿£¬ÔÚÄÚ²¿·µ»Ø1£¬ÔÚÍâ²¿·µ»Ø0
    int judge(node p);

    // ŒÆËãÒ»žöµãµœÍ¹°üÉÏÒ»žöÆœÃæµÄŸàÀë
    double dis_to_face(node q, face f);

    // ŒÆËãÒ»žöµãµœÍ¹°üÉÏž÷žöÃæµÄ×îÐ¡ŸàÀë
    double min_dis_to_convex(node p);


private:
    node p[M];                 // ¶ÁÈëµÄ³õÊŒµã
    face f[M * 8];             // Í¹°üÈýœÇÐÎ
    vector<int> v;             // µüŽúÓÃ£¬µãµÄÐòºÅ
    int cnt;                   // Í¹°üÈýœÇÐÎÊý£¬ÈýœÇÐÎÃæµÄÐòºÅ
    int to[M][M];              // ±ßiµœjÊÇÊôÓÚÍ¹°üÄÚµÄÄÄžöÃæ

public:
    int Input_node_num;          // ³õÊŒµãÊý(p[M]Êý×éµ±ÖÐµãµÄžöÊý)
    int Convex_node_num;         // Í¹°ü¶¥µãÊýÄ¿
    int Convex_face_num;         // Í¹°ü±íÃæÊýÄ¿

    vector<node> Points_in_Convex;   // žÃÈÝÆ÷ÀïÃæ±£ŽæÍ¹¶àÃæÌåµÄ¶¥µã
    vector<face> Faces_in_Convex;    // žÃÈÝÆ÷ÀïÃæ±£ŽæÍ¹¶àÃæÌåµÄ±íÃæ
};



// ����ptof������ͨ����������������жϵ�����ķ���
// �ú���������Ϊ�����жϵ���ά�����
// �ȵõ����ϵ���������m��n
// ������ά�����������һ��Ϊ����t
// ͨ��m��n�Ĳ�˵õ���ķ��������ٽ���������t����ж����ߵķ����ϵ��
// ����������ص�ֵΪ�����������ķ�����ͬ��
// �����ڴ洢��ʱ����֤��ķ��߷�����͹���ⲿ��
// ��˿���֪���õ���͹���ⲿ --> ��Ϊ�õ��뷨�����н�Ϊ���
// ���Ҵ���ɱ�P�㿴����
// ��Ϊ����������ͬ��
double Convex_hull::ptof(node q, face f)
{
    // ��f��������(a,b,c)Χ��
    // m = b-a
    // n = c-a
    // t = q-a
    // m*nΪ����ķ�����
    node m = p[f.b] - p[f.a];
    node n = p[f.c] - p[f.a];
    node t = q - p[f.a];
    return m * n ^ t;
}

// ����һ���㵽͹����һ��ƽ��ľ���
double  Convex_hull::dis_to_face(node q, face f) {
    // ��f��������(a,b,c)Χ��
    // m = b-a
    // n = c-a
    // t = q-a
    // m*nΪ����ķ�����
    node m = p[f.b] - p[f.a];
    node n = p[f.c] - p[f.a];
    node t = q - p[f.a];

    double len_nor = sqrt((m * n) ^ (m * n));    // ����������
    double len_proj = fabs((m * n) ^ t);         
    double dis = len_proj / len_nor;             // �㵽ƽ�����

    return dis;
}


// ����deal��dfs����
// ��������͹��
// ���жϵõ���q��͹���Ⲣ�ܡ���������curʱ������dfs��������ɾ�����棬Ȼ�����deal������
// ��deal�����У�����Ϊ��ά��q�����ϵ��������㣬�����������Ρ�
// ����ͨ�����ϵ�����a��b��ɵı��ҵ��뵱ǰ�湲�ߵ���һ����f [fa]��
// ��������f [fa]Ϊ͹�����棬�жϵ�q�ܷ񡰿���������
// �����ԣ����ٴε���dfs()����������ɾ����
// �����ɣ����õ�q���������������a��b���һ���µ���������Ϊ�档
// ��Ϊÿ�������εĵ������ǰ�����ʱ���¼�ģ����԰ѱ߷��������Ӧ�ľ�����ab�߹��ߵ���һ����
// ���to[a][b]�ϵļ�Ϊ�뵱ǰ��cur���ߵ���һ����

void Convex_hull::deal(int q, int a, int b)
{
    int fa = to[a][b];      //�뵱ǰ��cnt���ߵ���һ����
    face add;
    if (f[fa].ok)         //��fa��Ŀǰ��͹���ı��������
    {
        if (ptof(p[q], f[fa]) > my_eps)   //����q�ܿ���fa���������fa�������ߣ������µ�͹����
            dfs(q, fa);
        else            //��q����Կ���cur���ͬʱ������a��b���ߵ�fa�棬��p��a��b�����һ���µı���������
        {
            add.a = b;
            add.b = a;
            add.c = q;
            add.ok = 1;
            to[b][a] = to[a][q] = to[q][b] = cnt;
            f[cnt++] = add;
        }
    }
}

//��Ϊÿ�������εĵ������ǰ�����ʱ���¼�ģ����԰ѱ߷��������Ӧ�ľ�����ab�߹��ߵ���һ����
void Convex_hull::dfs(int q, int cur)//ά��͹��������q��͹���������͹��
{
    f[cur].ok = 0;//ɾ����ǰ�棬��Ϊ��ʱ���ڸ����͹���ڲ�

    //����ѱ߷�����(��b,��a)���Ա���deal()���ж��뵱ǰ��(cnt)����(ab)���Ǹ��档
    /*���ж��뵱ͷ��(cnt)���ڵ�3����(�����뵱ǰ��Ĺ����Ƿ���ģ�
        ����ͼ��(1)�ķ��߳���(����ʱ��)����130��312,���ǹ���13����һ��������13,��һ��������31)*/
    deal(q, f[cur].b, f[cur].a);
    deal(q, f[cur].c, f[cur].b);
    deal(q, f[cur].a, f[cur].c);
}


// �ж������������Ƿ���
// ���������ι����򷵻�1
int Convex_hull::same(int s, int t)
{
    // ȡ��һ�������������a,b,c 
    node a = p[f[s].a];
    node b = p[f[s].b];
    node c = p[f[s].c];

    // ��a,b,c����һ������������ֱ�������
    // �����Ϊ0��˵���ĸ��㹲��
    // ���a,b,c����һ�������������(��6����)���棬�����������ι���
    if (fabs(volume(a, b, c, p[f[t].a])) < my_eps
        && fabs(volume(a, b, c, p[f[t].b])) < my_eps
        && fabs(volume(a, b, c, p[f[t].c])) < my_eps)
        return 1;
    return 0;
}

// ����3D͹��
void Convex_hull::make()
{
    /*Part1*/
    // ��ʼ������Ĺ���������Ҫ�ҵ��ĸ�������ĵ㣺
    // ����������ͬ��p[0]��p[1]��
    // Ȼ��Ѱ�������ǲ����ߵĵ�������p[2]��
    // ������ǰ�����㲻����ĵ��ĸ���p[3]��
    // �����ĸ�����Ϊ��ʼ��������ĸ��㡣


    cnt = 0;
    if (Input_node_num < 4)
        return;        // ���жϵ����������ĸ�����������͹����ֱ�ӷ���


    int init_flag = 1;
    for (int i = 1; i < Input_node_num; i++)   // ��֤ǰ�����㲻����,��һ���㲻��
    {
        if (len(p[0] - p[i]) > my_eps)
        {
            swap(p[1], p[i]);    // ����������ĵ����p���������0�ź�1��
            init_flag = 0;              // ȷʵ���������㲻����
            break;
        }
    }
    if (init_flag) return;   //���е���ͬһλ����


    init_flag = 1;                // ������Ϊ1
    for (int i = 2; i < Input_node_num; i++)   // ��֤ǰ�����㲻����
    {
        if (len((p[1] - p[0]) * (p[i] - p[0])) > my_eps)
        {
            swap(p[2], p[i]);    // ���������ߵĵ����p���������0��1��2��
            init_flag = 0;       // ȷʵ���������㲻����
            break;
        }
    }
    if (init_flag) return;

    init_flag = 1;                // ������Ϊ1
    for (int i = 3; i < Input_node_num; i++)   // ��֤ǰ�ĸ��㲻����
    {
        if (fabs(volume(p[0], p[1], p[2], p[i])) > my_eps)
        {
            swap(p[3], p[i]);     // �ĸ���������ĵ����p���������0��1��2��3��
            init_flag = 0;        // ȷʵ�����ĸ��㲻����
            break;
        }
    }
    if (init_flag) return;




    /*Part2*/
    // ���ҵ��ĸ���ʼ�㣨p0��p3��֮�󣬴�����ʼ�����壺
    // ���ĸ�����ѡȡ�����㣬�������ĸ��档
    // ��ѡ��p[1]��p[2]��p[3]�����㣬����������ı��1��2��3������f[0]�С�
    // Ϊ�����жϵ���͹���ڲ������ⲿ���ڴ洢��ʱ����Ҫ��֤����ķ��߷�����͹���ⲿ��
    // �������ȼ��ڣ���һ���߽�㶼Ҫ�����������������ķ���������
    // �жϷ���Ϊ��ͨ��ptof�����ж���һ����p[0]�����ĳ����ϵ��
    // �����߳��ڣ�����Ҫ������f[0]������������ŵ�˳��
    // ͬ���ķ�������f[0]��f[1]��f[2]��f[3]�ĸ��档

    // ע�⣺
    // ����ptof��������Ϊ����������ͬ��
    // ok = 1 --> ����Ϊ͹����һ������

    face add;
    // ������ʼ������(4����Ϊp[0],p[1],[2],p[3])
    // ������Ĳ���Ҫ�����Ĵ�
    for (int i = 0; i < 4; i++)
    {
        add.a = (i + 1) % 4;
        add.b = (i + 2) % 4;
        add.c = (i + 3) % 4;
        add.ok = 1;

        // ptof(p[i], add) > my_eps ����p[i]��add�������
        // ����������Ҫ����add�Ķ���˳��
        if (ptof(p[i], add) > my_eps)   swap(add.c, add.b);    //��֤��ʱ�룬�����������⣬�����µ�ſɿ���

        // to[i][j]��ʾ�����ij������͹���ı�������cnt
        to[add.a][add.b] = to[add.b][add.c] = to[add.c][add.a] = cnt;  //���������߱���  //ÿ�����ϴ�����ı��
        f[cnt++] = add;    // ��͹���м�����add
    }

    /*Part3*/
    /*
    �ñ���������͹����
    ����i��ʾ��ǰ�㣬j��ʾ��ǰ�档
    ͨ��ptof�����жϵ��ܷ񡰿���������棬
    ����õ��͹���������涼���ɼ���
    Ҳ����ptofȫС��0
    ��˵���õ���͹���ڲ����������Ժ���
    ��ʼ������һ������жϡ�
    ������ԣ���˵���������͹��������
    ����dfs��������͹�����棬Ȼ�����һ��������жϣ�
    �����е㶼�ж���ɺ󣬼��ɵõ���Щ��ά�㹹�ɵ���С͹�����塣
    */


    // ����������͹��
    // �˴���i=4 ��ʼ
    for (int i = 4; i < Input_node_num; i++)
    {
        for (int j = 0; j < cnt; j++)               // ��ÿ�����ж��Ƿ��ڵ�ǰ3ά͹���ڻ���(i��ʾ��ǰ��,j��ʾ��ǰ��)
        {
            if (f[j].ok && ptof(p[i], f[j]) > my_eps)  // �Ե�ǰ͹��������жϣ����Ƿ���ܷ񿴵������  // �����ⲿ�ҿ���j������Ҫ����͹��
            {
                dfs(i, j);      //���ܿ�����ǰ�棬����͹������(�ݹ飬���ܲ�ֹ����һ����)����ǰ�������ɺ�break����ѭ��
                break;
            }
        }
    }

    /*Part4*/
    // ���������
    int tmp = cnt; //�Ѳ���͹���ϵ���ɾ����ok=0��

    cout << "tmp = " << tmp << endl;


    cnt = 0;


    for (int i = 0; i < tmp; i++)
        if (f[i].ok)
        {
            f[cnt++] = f[i];
            //��ÿ�����ϵ���������д���
            v.push_back(f[i].a);
            v.push_back(f[i].b);
            v.push_back(f[i].c);


            //��ÿ�������Faces_in_Convex
            Faces_in_Convex.push_back(f[i]);

        }

    //ȥ����ͬ�ĵ㲢��˳������
    sort(v.begin(), v.end());
    v.erase(unique(v.begin(), v.end()), v.end());


    cout << "v.size() = " << v.size() << endl;

    //���
    cout << "---------------------------------" << endl;
    cout << "��ά͹������Ϊ��" << endl;
    cout << "����v" << endl;
    for (int i = 0; i < v.size(); i++)
    {
        cout << v[i] << endl;
        cout << "(" << p[v[i]].x << "," << p[v[i]].y << "," << p[v[i]].z << ")" << endl;

        // ��͹�����㱣�浽Points_in_Convex����
        Points_in_Convex.push_back(p[v[i]]);
    }

    cout << "Points_in_Convex.size() = " << Points_in_Convex.size() << endl;
    for (int i = 0; i < Points_in_Convex.size(); i++)
    {
        cout << "(" << Points_in_Convex[i].x << "," << Points_in_Convex[i].y << "," << Points_in_Convex[i].z << ")" << endl;

    }

    cout << "Faces_in_Convex.size() = " << Faces_in_Convex.size() << endl;
    for (int i = 0; i < Faces_in_Convex.size(); i++)
    {
        cout << "(" << Faces_in_Convex[i].a << "," << Faces_in_Convex[i].b << "," << Faces_in_Convex[i].c << ")" << endl;

    }

   
    Convex_node_num = Points_in_Convex.size();
    Convex_face_num = Faces_in_Convex.size();
    cout << "Convex_node_num = " << Convex_node_num << endl;
    cout << "Convex_face_num = " << Convex_face_num << endl;

}

// �ж�һ�����Ƿ���͹���ڲ�
// ���ڲ�����1
// ���ⲿ����0
int Convex_hull::judge(node p) {
    int index;
    for (index = 0; index < Convex_face_num; index++)
    {
        if (ptof(p, Faces_in_Convex[index]) > my_eps)     // ����
        {
            cout << "超限" << endl;
            return 0;
        }
    }
    if (index == Convex_face_num) {
        cout << "内" << endl;
        return 1;
    }
        
}


// ����һ���㵽͹���ϸ��������С����
// ��ʽ����p��ƽ������һ�����ߵĳ����ڸ�ƽ�淨�����ϵ�ͶӰ����
double  Convex_hull::min_dis_to_convex(node p) {
    double min_dis = 10000.0;

    int index;
    double dis = 0.0;

    for (index = 0; index < Convex_face_num; index++)
    {   
        dis = dis_to_face(p, Faces_in_Convex[index]);
        cout << "index = " << index;
        cout << "  dis = " << dis << endl;

        if (dis < min_dis)     // ����
        {
            min_dis = dis;
        }
    }

    cout << "min_dis = " << min_dis << endl;

    return min_dis;
}


int main(int argc, char *argv[])
{
    std::string ipaddr = "192.168.3.41";
    uint16_t port = 1337;

    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file))
    {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }

    xmate::Robot robot(ipaddr, port,XmateType::XMATE7_PRO);
    //sleep(1);

    robot.setMotorPower(1);
    //const double PI=3.14159;
    std::array<double,7> q_init;
    //std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.1,q_init,q_drag,robot);
    //sleep(1);

    std::cout<<"Start MOVE!!!"<<std::endl;
    
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);

    std::array<double, 7> init_position;
    static bool init = true;
    double time = 0;
    std::cout<<"HERE!!!"<<std::endl;
    VectorXd q_now;  // 当前机械臂位姿与下一步位姿
    XmateModel xmatemodel(&robot, XmateType::XMATE7_PRO);
    JointPositions output;

    /*****************************************************************/
    // 建立虚拟墙
    Convex_hull Virtual_wall;

    int int_num;
    node int_p;
    std::cout << "Input node number: ";
    
    /*
    std::cin >> int_num;                       // 输入点数
    Virtual_wall.Sum_node_number(int_num);

    for (int i = 0; i < Virtual_wall.Input_node_num; i++) {    // 输入这些点的三维坐标
        std::cout << "The " << i << " point: ";
        std::cin >> int_p.x >> int_p.y >> int_p.z;
        Virtual_wall.Input_node(i, int_p);
    }
    */
   
   Virtual_wall.Sum_node_number(8);
   int_p.x = 0.4;
   int_p.y = 0.0;
   int_p.z = 0.0;
   Virtual_wall.Input_node(0, int_p);

   int_p.x = 0.4;
   int_p.y = 0.0;
   int_p.z = 0.25;
   Virtual_wall.Input_node(1, int_p);

   int_p.x = 0.4;
   int_p.y = 0.24;
   int_p.z = 0.0;
   Virtual_wall.Input_node(2, int_p);

   int_p.x = 0.4;
   int_p.y = 0.24;
   int_p.z = 0.25;
   Virtual_wall.Input_node(3, int_p);

   int_p.x = 0.65;
   int_p.y = 0.0;
   int_p.z = 0.0;
   Virtual_wall.Input_node(4, int_p);

   int_p.x = 0.65;
   int_p.y = 0.0;
   int_p.z = 0.25;
   Virtual_wall.Input_node(5, int_p);

   int_p.x = 0.65;
   int_p.y = 0.24;
   int_p.z = 0.0;
   Virtual_wall.Input_node(6, int_p);

   int_p.x = 0.65;
   int_p.y = 0.24;
   int_p.z = 0.25;
   Virtual_wall.Input_node(7, int_p);




    Virtual_wall.make();      // 建立最小凸多面体为Virtual_wall

    std::cout << "Start Action!!"<<std::endl;

    /*******************************************/


    JointControl joint_position_callback;
    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        
        if(init==true){
            init_position = robot_state.q;
            init=false;
        }
        if(robot_state.control_command_success_rate <0.9){
            std::cout<<"通信质量较差："<<robot_state.control_command_success_rate<<std::endl;
        }

        q_now = Map<Matrix<double, 7, 1>>(robot_state.q.data());      // 读取机械臂当前关节角度值  
        std::cout<<"当前关节角"<<q_now[0]<<", "<<q_now[1]<<", "<< q_now[2]<< ", "<< q_now[3]<<", "<<q_now[4]<<", "<<q_now[5]<<", "<< q_now[6]<<std::endl;

        //double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI/4 * time));
        //double delta_angle = 0;
        double delta_angle = M_PI / 40.0 * (1 - std::cos(M_PI/4 * time));

        std::array<double, 7> q_command{ init_position[0] + delta_angle, init_position[1] + delta_angle,
                                                    init_position[2] + delta_angle, init_position[3] - delta_angle,
                                                    init_position[4] + delta_angle, init_position[5] - delta_angle,
                                                    init_position[6] + delta_angle }; 

        // 给输出指令初值
        output = {{q_command[0], q_command[1], q_command[2], q_command[3], q_command[4], q_command[5], q_command[6]}};

        /********************************************************************************************/
        // 机械臂末端法兰空间位姿
        // 正运动学计算q_command对应的末端位置T_judge
        // GetCartPose函数原型为：
        // std::array<double, 16> GetCartPose(std::array<double, 7> &q, SegmentFrame nr = SegmentFrame::kJoint7);
        MatrixXd T_judge = Map<Matrix<double, 4, 4, RowMajor>>(xmatemodel.GetCartPose(q_command).data());

        Vector3d pr_judge = T_judge.block<3, 1>(0, 3);     // T矩阵的第四列，就是pr点的位置坐标

        // T.block<3, 3>(0, 0)是T的旋转矩阵
        // Vector3d(0, 0, 1)是 {0,0,1}列向量
        Vector3d pt_judge = T_judge.block<3, 3>(0, 0) * Vector3d(0, 0, act_len) + pr_judge;
        std::cout<<"末端法兰位置： "<<pr_judge[0]<<", "<< pr_judge[1]<<", "<<pr_judge[2]<< std::endl;
        std::cout<<"末端执行器位置： "<<pt_judge[0]<<", "<< pt_judge[1]<<", "<<pt_judge[2]<< std::endl;

        // 末端执行器位置为待判断位置
        node pt_node(pt_judge[0], pt_judge[1], pt_judge[2]);
       
        // judge函数，点在凸包内部返回1; 在外部返回0


        /*
        if ((!Virtual_wall.judge(pt_node)) || (Virtual_wall.min_dis_to_convex(pt_node) <= dis_limit)) {
            // 不安全，则不移动
            output = {{q_now[0], q_now[1], q_now[2], q_now[3], q_now[4], q_now[5], q_now[6]}};
            std::cout<<"NOT SAFE!!!"<<std::endl;
            
        } else{
            // q_command安全，则按照这个指令移动
            std::cout<<"SAFE MOVE!!!"<<std::endl;
        }
        */
        
        time += 0.001; 
        if(time>45){
            std::cout<<"运动结束："<<std::endl;
            return MotionFinished(output);
        }
        //std::cout << delta_angle << std::endl;
        return output;        
    };

    robot.Control(joint_position_callback);
    return 0;
}
