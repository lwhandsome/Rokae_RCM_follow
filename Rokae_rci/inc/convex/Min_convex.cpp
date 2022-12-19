#include "Min_convex.h"
using namespace std;


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
            cout << "�õ���͹���ⲿ" << endl;
            return 0;
        }
    }
    if (index == Convex_face_num) {
        cout << "�õ���͹���ڲ�" << endl;
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