#pragma once
#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>

#define M 50
#define my_eps 0.00001
#define inf 0x3f3f3f3f
#define mod 1070000009
#define PI 3.141592653
using namespace std;

// 点
struct node
{
    double x, y, z;      //node的元素
    node() {}
    node(double xx, double yy, double zz) :x(xx), y(yy), z(zz) {}

    //运算符重载
    node operator +(const node p)	//向量间相加操作
    {
        return node(x + p.x, y + p.y, z + p.z);
    }
    node operator -(const node p)	//向量间相减操作
    {
        return node(x - p.x, y - p.y, z - p.z);
    }
    node operator *(const node p)	//向量间叉乘操作
    {
        return node(y * p.z - z * p.y, z * p.x - x * p.z, x * p.y - y * p.x);
    }
    node operator *(const double p)	//向量乘以一个数
    {
        return node(x * p, y * p, z * p);
    }
    node operator /(const double p)	//向量除以一个数
    {
        return node(x / p, y / p, z / p);
    }
    double operator ^(const node p)	//向量间点乘操作
    {
        return x * p.x + y * p.y + z * p.z;
    }
};


// 表面三角形
struct face
{
    int a, b, c;  // 凸包一个面上的三个点的编号
    int ok;       // 该面是否是最终凸包中的面
    
    face() {}
};


// 定义表示三维凸包的类
// 在类中定义所需用到的一些变量和函数
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

    
    void Sum_node_number(int num) { Input_node_num = num; }   // 读入Input_node_num数目
  
    void Input_node(int index, node point) {  p[index] = point; }         // 读入点

    double len(node p)      // 向量的长度、模
    {
        return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    double area(node a, node b, node c)       // 三个点的面积*2
    {
        return len((b - a) * (c - a));
    }

    double volume(node a, node b, node c, node d)  // 四面体体积*6
    {
        return (b - a) * (c - a) ^ (d - a);
    }


    // 定义ptof函数，通过有限体积法用来判断点与面的方向
    // 若为正：点与面同向
    double ptof(node q, face f);

    // 定义deal和dfs函数，用来更新凸包
    void deal(int q, int a, int b);

    // 维护凸包，若点q在凸包外则更新凸包
    void dfs(int q, int cur);

    // 判断两个三角形是否共面，两个三角形共面则返回1
    int same(int s, int t);

    // 构建3D凸包
    void make();

    // 判断一个点是否在凸包内部，在内部返回1，在外部返回0
    int judge(node p);

    // 计算一个点到凸包上一个平面的距离
    double dis_to_face(node q, face f);

    // 计算一个点到凸包上各个面的最小距离
    double min_dis_to_convex(node p);


private:
    node p[M];                 // 读入的初始点
    face f[M * 8];             // 凸包三角形
    vector<int> v;             // 迭代用，点的序号
    int cnt;                   // 凸包三角形数，三角形面的序号
    int to[M][M];              // 边i到j是属于凸包内的哪个面

public:
    int Input_node_num;          // 初始点数(p[M]数组当中点的个数)
    int Convex_node_num;         // 凸包顶点数目
    int Convex_face_num;         // 凸包表面数目

    vector<node> Points_in_Convex;   // 该容器里面保存凸多面体的顶点
    vector<face> Faces_in_Convex;    // 该容器里面保存凸多面体的表面
};



