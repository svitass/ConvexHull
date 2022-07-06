// ��ײ���
// ��ͼref: https://blog.csdn.net/weixin_42335313/article/details/105879928
#include <iostream>
#include <gl/glut.h>
#include <math.h>
#include <time.h>

using namespace std;
const int N = 200;  // �������

int M; // num: ��ĸ���  m: ����convex hull��ĸ���

typedef struct Vertex
{
    double x, y, z;  // ���3ά����
}Vertex;

typedef struct Plane
{
    int node[N]; // �����������ʱ��洢��������ʾ��
    int nodeNum; // ����ĸ���
    int flag = 1; // flag=0: ƽ�治����͹����flag=1:ƽ������͹��, flag=2:ƽ����Ϣ��Ҫ���£�����µ��γ��µ�͹ƽ��
}Plane;

typedef struct Line // ������
{
    int v1, v2;
}Line;


int edge[N][N];
int temp_v[N];  //����͹���м��������ƽ��ĵ�

Vertex multiply(Vertex v1, Vertex v2)  // ������� v1 x v2
{
    Vertex vec;
    vec.x = v1.y * v2.z - v1.z * v2.y;
    vec.y = v1.z * v2.x - v1.x * v2.z;
    vec.z = v1.x * v2.y - v1.y * v2.x;
    return vec;
}

double dot(Vertex v1, Vertex v2)  // �������
{
    double res = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    return res;
}

Vertex subduction(Vertex v1, Vertex v2)  // ������� v1-v2
{
    Vertex vec;
    vec.x = v1.x - v2.x;
    vec.y = v1.y - v2.y;
    vec.z = v1.z - v2.z;
    return vec;
}

double getLength(Vertex vec)  // ��ȡ��������
{
    double len = sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
    return len;
}

Line collinear(Vertex* points, int a, int b, int c)  // �����ߵĵ㣬���أ������ȣ����µĵ㸲��
{
    Line line;
    double dis_ab = pow(points[a].x - points[b].x, 2) + pow(points[a].y - points[b].y, 2) + pow(points[a].z - points[b].z, 2);
    double dis_ac = pow(points[a].x - points[c].x, 2) + pow(points[a].y - points[c].y, 2) + pow(points[a].z - points[c].z, 2);
    double dis_bc = pow(points[b].x - points[c].x, 2) + pow(points[b].y - points[c].y, 2) + pow(points[b].z - points[c].z, 2);
    if (dis_ab > dis_ac && dis_ab > dis_bc)  // dis_ab���
    {
        line.v1 = a;
        line.v2 = b;
    }
    else if (dis_ab <= dis_ac && dis_ac >= dis_bc) // dis_ac���
    {
        line.v1 = a;
        line.v2 = c;
    }
    else   // dis_bc���
    {
        line.v1 = b;
        line.v2 = c;
    }
    return line;
}

bool isCollinear(Vertex a, Vertex b, Vertex c) // �ж�3�����Ƿ���
{
    Vertex vec1 = subduction(a, b);
    Vertex vec2 = subduction(a, c);
    double v_cos = abs(dot(vec1, vec2) / (getLength(vec1) * getLength(vec2)));
    if (v_cos == 1) {
        return true;
    }
    return false;
}

int findMaxPoint(Vertex *points, Plane plane)  // ����ƽ���ϼ��˵������
{
    Vertex vmax = points[plane.node[0]];  // �����
    int imax = 0;
    for (int i = 1; i < plane.nodeNum; i++)  // ѡһ�����˵� x_min,y_min,z_min
    {
        if (vmax.x < points[plane.node[i]].x)
        {
            vmax = points[plane.node[i]];
            imax = i;
        }
        else if (vmax.x == points[plane.node[i]].x && vmax.y < points[plane.node[i]].y)
        {
            vmax = points[plane.node[i]];
            imax = i;
        }
        else if (vmax.x == points[plane.node[i]].x && vmax.y == points[plane.node[i]].y && vmax.z < points[plane.node[i]].z)
        {
            vmax = points[plane.node[i]];
            imax = i;
        }
    }
    return imax;
}


Plane coplanar(Vertex* points, Plane plane, int node)  // ������  �Ƚ�͹�ߺͼ���㵽͹���б��
{
    plane.node[plane.nodeNum] = node; // �Ȱ�node����ƽ�棬�ٸ���
    plane.nodeNum = plane.nodeNum + 1;
    int newNodes[N]; // ��Ŵ����µ�node�ڵ�
    Vertex vec1, vec2, vec3;
    int imax = findMaxPoint(points, plane);
    // ����plane�ϵĵ� ��imax��ʼ���Ƚ�����
    int j = 0;
    int i1, i2;  // ��ʾplane.node����
    double v1v2, maxCos = -1;  // ��ʾ���ҽǶ�
    newNodes[j] = plane.node[imax]; // Ӧ�ô�v����
    while (j < plane.nodeNum)
    {
        i1 = imax;
        maxCos = -1;
        if (j == 0)
        {
            vec1 = points[plane.node[i1]];
        }
        else
        {
            vec1 = subduction(points[plane.node[i1]], points[newNodes[j - 1]]);
        }
        i2 = (i1 + 1) % plane.nodeNum;
        if (i2 == i1) break;
        while (i2 != i1)  // ��ֹ�����������е���ѡ������ֵ����
        {
            vec2 = subduction(points[plane.node[i2]], points[plane.node[i1]]);
            v1v2 = dot(vec1, vec2) / (getLength(vec1) * getLength(vec2));  // ��ȡvec1,vec2������ֵ
            if (maxCos < v1v2)
            {
                maxCos = v1v2;
                imax = i2;
            }
            else if (maxCos == v1v2 && maxCos != -1) // 3�㹲��
            {
                Line line = collinear(points, plane.node[i1], plane.node[i2], plane.node[imax]);  
                if (line.v1 == plane.node[i2] || line.v2 == plane.node[i2])
                {
                    imax = i2;
                }
            }
            i2 = (i2 + 1) % plane.nodeNum;
        }
        // ��һ������ĵ�Ϊimax(�н���С�ĵ�)
        j = j + 1;
        newNodes[j] = plane.node[imax];
        if (newNodes[j] == newNodes[0]) break;   // ������һ��Բ
    }
    for (int i = 0; i < j; i++) // ��ֵnewNodes��plane.node
    {
        plane.node[i] = newNodes[i];
    }
    plane.nodeNum = j;
    plane.flag = 1;
    return plane;
}

void checkEdge(int v1, int v2, Plane *planes, int v, int j) // v1,v2:�ߵĶ˵� v:�µĵ� j:�������
{
    if (edge[v1][v2] == 1 && edge[v2][v1] != 1)  // edge==1: ���ƽ��v1v2v
    {
        planes[M].node[0] = v1;
        planes[M].node[1] = v2;
        planes[M].node[2] = v;
        planes[M].nodeNum = 3;
        M = M + 1;
        edge[v1][v2] = 0;  //ȷ��edge����һ��node����ʱ��������
    }
    else if (edge[v1][v2] != 0)  // edge==2: ɾ��͹����ԭ�����б�ab��ƽ��
    {
        planes[j].flag = 0;
        //edge[v1][v2] = 0;
    }
}

Plane inversePlane(Plane plane)  // �ı�ƽ���ϵ�Ĵ洢˳������洢
{
    int j = 0;
    for (int i = plane.nodeNum - 1; i >= 0; i--)
    {
        temp_v[j] = plane.node[i];
        j = j + 1;
    }
    for (int i = 0; i < plane.nodeNum; i++)
    {
        plane.node[i] = temp_v[i];
    }
    return plane;
}

void printPlane(Plane *planes, int &planesNum)
{
    for (int i = 0; i < planesNum; i++)
    {
        int j = 0;
        cout << "(";
        for (j = 0; j < planes[i].nodeNum; j++)
        {
            cout << planes[i].node[j] << " ";
        }
        cout << ")" << endl;
    }
}

int initConvexHull(Vertex* points, int& pointsNum, Plane* planes, int& planesNum)  // ��ʼ��͹������3��������ĵ�
{
    planes[0].node[0] = 0;
    planes[0].node[1] = 1;
    planes[0].nodeNum = 2;
    bool is_line;
    Line line;
    int i = 2;
    while (planes[0].nodeNum <= 2 && i < pointsNum)
    {
        int v1 = planes[0].node[0];
        int v2 = planes[0].node[1];
        is_line = isCollinear(points[v1], points[v2], points[i]);
        if (is_line)
        {
            line = collinear(points, v1, v2, i);
            planes[0].node[0] = line.v1;
            planes[0].node[1] = line.v2;
        }
        else
        {
            planes[0].node[2] = i;
            planes[0].nodeNum += 1;
        }
        i = i + 1;
    }
    planesNum = 1;
    return i;
}

void updateEdge(Plane plane) // plane�ķ������������ڵ���ͬһ�࣬����edge
{
    int a, b;
    for (int k = 0; k < plane.nodeNum; k++)
    {
        a = plane.node[k];
        b = plane.node[(k + 1) % plane.nodeNum];
        edge[a][b] += 1;
    }
}

Vertex getNormalVector(Vertex* points,Plane plane) // ��ȡƽ��plane�ķ�����
{
    int a, b, c;
    Vertex v1, v2, v3, n; // v1,v2,v3��ʾplane�ϲ����ߵĵ�  n��ʾ������
    a = plane.node[0];
    b = plane.node[1];
    c = plane.node[2];
    v1 = points[a];
    v2 = points[b];
    v3 = points[c];
    Vertex vec1 = subduction(v2, v1);
    Vertex vec2 = subduction(v3, v1);
    n = multiply(vec1, vec2);
    return n;
}

void updateEdgeByNode(Vertex *points, Plane *planes, int& planesNum, Vertex node) // ����͹���ϵ�ÿһ�棬������Ϣ��edge
{
    for (int j = 0; j < planesNum; j++)  // ����͹���ϵ���
    {
        if (planes[j].nodeNum == 0 || planes[j].flag == 0) continue; // ��ƽ���Ѵ�͹����ɾ��
        Vertex n = getNormalVector(points,planes[j]);  // ������
        Vertex pnode = subduction(node, points[planes[j].node[0]]);
        double res = dot(n, pnode);
        if (res >= 0) // node��ƽ������, node�������a,b,c����
        {
            updateEdge(planes[j]);
            if (res == 0)  planes[j].flag = 2;  // ��ʶ����ƽ����õ㹲��
        }
        else
        {
            if (planesNum == 1)  // ����ǰ͹����ֻ��һ���棬�������Ĵ洢��˳�򣬱�֤�����������һ��
            {
                planes[j] = inversePlane(planes[j]);
                updateEdge(planes[j]);
            }
        }
    }
}

void updateConvexHull(Vertex *points, Plane *planes, int &planesNum, int i)  // ����͹��  i�¼���Ľڵ�
{
    for (int j = 0; j < planesNum; j++) // ��
    {
        if (planes[j].flag == 2)
        {
            planes[j] = coplanar(points, planes[j], i);
            if (planesNum == 1) continue;  // ͹����ֻ��һ��ƽ���޷���֤ƽ��ķ�����
            Vertex n = getNormalVector(points, planes[j]);  // Vertex* points,Plane plane
            bool check = true;
            for (int k = 0; k <= i; k++)
            {
                Vertex vec = subduction(points[k], points[planes[j].node[0]]);
                if (dot(vec, n) > 0)  // ͹���ϵ����ƽ��ķ�������ͬһ�࣬˵����ƽ�治������
                {
                    check = false;
                    break;
                }
            }
            if (check == false)
            {
                planes[j] = inversePlane(planes[j]);
            }
            continue;
        }
        for (int k = 0; k < planes[j].nodeNum; k++)
        {
            int a = planes[j].node[k];
            int b = planes[j].node[(k + 1) % planes[j].nodeNum];
            checkEdge(a, b, planes, i, j);
        }
        if (planes[j].flag == 0) planes[j].nodeNum = 0;
    }
}


void getConvexHull(Vertex *points, int &pointsNum, Plane *planes, int &planesNum)  // �㼯����ĸ�����ƽ�棬ƽ����� 
{
    int v_index = initConvexHull(points, pointsNum, planes, planesNum);
    // ����͹��
    bool flag = true; // ���ڸı�һ�ε�һ��ƽ���˳��
    M = planesNum;
    //cout << "��ʼ��͹��" << endl;
    //printPlane(planes, planesNum);
    for (int i = v_index; i < pointsNum; i++)  // ����㵽convex hull
    {
        updateEdgeByNode(points,planes,planesNum,points[i]);
        // �鿴ÿ�����ÿ���ߣ���edge=1�ı߼����µ��� ���ֱ�ԭ�е�˳�����򣩣�node��Ϊ���Ľڵ����
        updateConvexHull(points, planes, planesNum, i); // �㼯��ƽ�漯��ƽ�������������
        planesNum = M;
        if (planesNum > 1 && flag) // �ı�p[0]�ĵ��˳��ʹ������
        {
            planes[0] = inversePlane(planes[0]);
            flag = false;
        }
        // ����edge
        for (int j = 0; j <= i; j++)
        {
            for (int k = 0; k <= i; k++)
            {
                edge[j][k] = 0;
            }
        }
        //cout << "��" << i << "����: (" << points[i].x << "," << points[i].y << "," << points[i].z << ")" << endl;
        //printPlane(planes, planesNum);
    }

}

void axis(double length)  // ����������
{
    glColor3f(0.0f, 0.0f, 0.0f); // ���û�ͼ��ɫ
    /*
    ��������Ҫ���е����ţ�ƽ�ƵȲ�������glPushMatrix��glPopMatrix֮��
    glPushMatrix()��glPopMatrix()�����ʹ���ܹ�������һ�εı任�Ա��α仯��Ӱ�죬ʹ���α任������������ϵ��ԭ��Ϊ�ο������
    ԭ��
    (1) OPenGL�е�modelview����任Ĭ�ϱ��α任���ϴα任�����������ϴ�modelview�任����������������ϵ�µ�λ���Ǳ���modelview�任�����
    (2) openGL���彨ģʵ�����Ƿ������ߵġ���һ��������������ϵ��ԭ��λ�û��Ƴ������壻�ڶ�����ͨ��modelview�任�������������ϵԭ�㴦��������з���任�����������ƶ�����������ϵ��Ŀ��λ�ô�
    (3) ��modelview�任����glPushMatrix��glPopMatrix֮���ܹ�ʹ���α任���ϴα任����
    */
    glPushMatrix();
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0, 0.0, length);
    glEnd();
    // ����ǰ�������Ƶ�ָ��λ��
    glTranslated(0, 0.0, length - 0.2); // ����һ��ƽ�ƾ���ʹ�þ����뵱ǰ������ˣ�ʹ������ͼ�ν���ƽ�Ʊ任����Ե�ǰ���õ�����ԭ�� x,y,z: ƽ�Ʒ���
    glColor3f(1.0, 0.0, 0.0);
    glutWireCone(0.04, 0.3, 8, 8); // �����������ͷ
    glPopMatrix();

}


void display(Vertex *points,int pointsNum,Plane *planes, int planesNum)
{
    glClear(GL_COLOR_BUFFER_BIT);  // �����ɫ����
    glMatrixMode(GL_PROJECTION); //��ͶӰ����Ӧ�����ľ������
    glLoadIdentity(); //����ǰ����ϵ��ԭ���Ƶ�����Ļ���ģ�������һ����λ����

    glOrtho(-10.0, 10.0, -10.0, 10.0, -10, 10); // ����ǰ�Ŀ��ӿռ�����Ϊ��ͶӰ�ռ� glOrtho(left,right,bottom,top,near,far)
    glPointSize(1);  //ָ��դ�񻯵��ֱ��
    glMatrixMode(GL_MODELVIEW); // ��ģ���Ӿ������ջӦ�����ľ������
    glLoadIdentity();
    // gluLookAt(eye,center,up) z: centerָ��eye(�����߷����෴)
    gluLookAt(1.3, 1.6, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    //gluLookAt(0, 1.3, 1.6, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // ������ϵ
    axis(5); // z��

    glPushMatrix();
    glRotated(90.0, 0.0, 1.0, 0);  //   x��      glrotate:��ת���ԭ�����
    axis(5);
    glPopMatrix();

    glPushMatrix();
    glRotated(-90.0, 1.0, 0.0, 0.0); //    y��
    axis(5);
    glPopMatrix();

    // ��͹��
    for (int i = 0; i < pointsNum; i++) //�������еĶ���
    {
        glColor3f(0.0, 1.0, 1.0); // ������ɫ
        glPointSize(4);
        glBegin(GL_POINTS);
        glVertex3f(points[i].x, points[i].y, points[i].z);
        glEnd();
    }
    for (int i = 0; i < planesNum; i++)  // ����͹������
    {
        Plane temp_p = planes[i];
        glColor3f(1.0, 0.0, 0.0); // ���ʺ�ɫ
        glPointSize(6);
        glBegin(GL_LINE_LOOP);
        for (int j = 0; j < temp_p.nodeNum; j++)
        {
            glVertex3f(points[temp_p.node[j]].x, points[temp_p.node[j]].y, points[temp_p.node[j]].z);
        }
        glEnd();
    }


    //glColor3f(0.0, 0.0, 0.0); // ���û�ͼ��ɫ��Ĭ��
    //glLineWidth(1.0f); //�����߿�Ĭ��1.0f
    //axis(2); 
    /**
    OpenGLҪ��ָ�������������������glBegin����֮��,glEnd֮ǰ,����ָ���Ķ��㽫������
    ��glBegin��ָ�����ʹ����Щ�㣺 GL_POLYGON��ʾ������Σ��ɵ����ӳɶ���Σ�
    OpenGLָ���㣺glVertex*, glVertex2i,glVertex2f,glVertex3f,glVertex3fv  ���ֱ�ʾ�����ĸ���,v��ʾ���ݵļ���������ʹ��ָ��ķ�ʽ
    glBegin����������
    ��1��GL_POINTS: ���Ƶ�  ��2��GL_LINES:�����߶Σ������������ϣ�������
    ��3��GL_LINE_STRIP: ���ƴӵ�һ�����㵽���һ����������������һ���߶Σ���n��n+1�����㶨�����߶�n������n-1���߶�
    ��4��GL_LINE_LOOP: ���ƴӵ�һ�����㵽���һ����������������һ���߶Σ����һ������͵�һ��������������n��n+1�����㶨�����߶�n, ����n���߶�
    ��5��GL_TRIANGLES: ��ÿ��������Ϊһ��������������
    ��6��GL_QUADS: ������4��������ɵ�һ�鵥�����ı���
    ��7��GL_POLYGON: ������һ��͹�����
    */
    //glPopMatrix(); //����������y����ת
    glRotated(90.0, 0.0, 1.0, 0);
    glutSwapBuffers();

    //glFlush(); // ǿ��ˢ�»��壬��֤��ͼ�����ִ�У������������ڻ������еȴ���
}

//��ײ���  ����������ά�㼯�����㼯���ɵ�͹���Ƿ�����ײ
bool isCollision(Vertex* points1, int pointsNum1, Vertex* points2, int pointsNum2)
{
    // �Ȼ�ȡ��һ���㼯��͹��
    Plane plane1[N];
    int planeNum1;
    getConvexHull(points1, pointsNum1, plane1, planeNum1);
    bool flag = true;  // ��ʾv��͹���ڲ�   
    // ����points�еĵ㣬�����Ƿ���plane1��ײ
    for (int i = 0; i < pointsNum2; i++)
    {
        flag = true;
        Vertex v = points2[i];
        for (int j = 0; j < planeNum1; j++) // �ȵõ�ƽ��ķ�������Ȼ���жϵ�v��ƽ���λ��
        {
            Plane p = plane1[j];
            Vertex n = getNormalVector(points1, p);  // ����ƽ��ķ�������ָ��͹����ࡣv������ƽ�淨���������<0,˵��v��͹���ڲ�����ײ
            Vertex pv = subduction(v, points1[p.node[0]]);
            double dotRes = dot(pv, n);
            if (dotRes > 0) // v�ڸ�ƽ����࣬����͹���ڲ�
            {
                flag = false;
                break;
            }
            else if (dotRes == 0) // v��ƽ�湲��
            {
                for (int k = 1; k <= p.nodeNum; k++)  // �ȽϽǶȴ�С, ��v��ƽ���ڣ�v��ƽ������߽Ƕ�һֱ����
                {
                    int v1_index = p.node[k - 1];
                    int v2_index = p.node[k % p.nodeNum];
                    int v3_index = p.node[(k + 1) % p.nodeNum];
                    Vertex v1 = points1[v1_index];
                    Vertex v2 = points1[v2_index];
                    Vertex v3 = points1[v3_index];
                    Vertex vec1 = subduction(v2, v1);
                    Vertex vec2 = subduction(v3, v2);
                    double angle1Cos = dot(vec1, vec2) / (getLength(vec1) * getLength(vec2));
                    Vertex vec3 = subduction(v, v2);
                    double angle2Cos = dot(vec1, vec3) / (getLength(vec1) * getLength(vec3));
                    if (angle2Cos > angle1Cos)  // angle2<angle1,����ƽ����
                    {
                        flag = false;
                        break;
                    }
                    else if (angle2Cos == angle1Cos) // v2,v3,v����
                    {
                        points1[pointsNum1] = v;
                        Line line = collinear(points1, v2_index, v2_index, pointsNum1); // �ú����᷵��3��������������˵���ɱߣ�����߳���ȣ�����pointsNum��������һ����
                        if (line.v1 != pointsNum1 && line.v2 != pointsNum1)
                        {
                            flag = false;
                            break;
                        }

                     
                    }
                }

            }
        }
        if (flag == true) break;   // ˵����v��͹���� �� ƽ���� �� ���ߣ� �ҵ���һ��points1�ĵ���points2��ײ�����ñ��������ĵ���
    }
    return flag;
}


int main(int argc, char** argv)
{
    // ��ʼ������͹���Ķ����ƽ��
    Vertex points1[N], points2[N];  // ���ڱ��������㼯�е����Ϣ
    Plane planes1[N], planes2[N];  // ���ڱ�������͹����ƽ�����Ϣ

    int pointsNum1, pointsNum2;   // ����㼯�е�ĸ���
    int planeNum1, planeNum2;   // ����͹����ƽ��ĸ���
    // �������ݵ�
    cout << "����㼯1����ĸ����͵��3ά���꣩��" << endl;
    cin >> pointsNum1;
    for (int i = 0; i < pointsNum1; i++)
    {
        cin >> points1[i].x >> points1[i].y >> points1[i].z;
    }
    cout << "����㼯2����ĸ����͵��3ά���꣩��" << endl;
    cin >> pointsNum2;
    for (int i = 0; i < pointsNum2; i++)
    {
        cin >> points2[i].x >> points2[i].y >> points2[i].z;
    }

    long startTime, endTime;
    double totalTime;
    startTime = clock();
    bool res = isCollision(points1, pointsNum1, points2, pointsNum2);
    endTime = clock();
    totalTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
    cout << "��ײ����ʱ" << totalTime << "��" << endl;
    
    if (res == true)
    {
        cout << "����͹����ײ" << endl;
    }
    else 
    {
        cout << "����͹��û����ײ" << endl;
    }

    // ����͹��
    getConvexHull(points1, pointsNum1, planes1, planeNum1);
    getConvexHull(points2, pointsNum2, planes2, planeNum2);
    // ʹ��opengl��ͼ     
    glutInit(&argc, argv);  //��ʼ��GLUT��
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE); // ����ͼ����ʾģʽ
    glutInitWindowPosition(0, 0); //���ô���λ�ã����Ͻ�Ϊԭ��
    glutInitWindowSize(640, 500); //���ô��ڴ�С
    glutCreateWindow("Convex Hull"); //�������ڣ��ַ���Ϊ��������
    glClearColor(1.0, 1.0, 1.0, 0); //���ñ�����ɫ

    // ͼ�λ���
    //glutDisplayFunc(&display);  // ����ͼ��   // �ݲ������ε����вκ���
    //  ###############################################����͹��start############################################################
    glClear(GL_COLOR_BUFFER_BIT);  // �����ɫ����
    glMatrixMode(GL_PROJECTION); //��ͶӰ����Ӧ�����ľ������
    glLoadIdentity(); //����ǰ����ϵ��ԭ���Ƶ�����Ļ���ģ�������һ����λ����

    glOrtho(-10.0, 10.0, -10.0, 10.0, -10, 10); // ����ǰ�Ŀ��ӿռ�����Ϊ��ͶӰ�ռ� glOrtho(left,right,bottom,top,near,far)
    glPointSize(1);  //ָ��դ�񻯵��ֱ��
    glMatrixMode(GL_MODELVIEW); // ��ģ���Ӿ������ջӦ�����ľ������
    glLoadIdentity();
    // gluLookAt(eye,center,up) z: centerָ��eye(�����߷����෴)
    gluLookAt(1.3, 1.6, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    //gluLookAt(0, 1.3, 1.6, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // ������ϵ
    axis(5); // z��

    glPushMatrix();
    glRotated(90.0, 0.0, 1.0, 0);  //   x��      glrotate:��ת���ԭ�����
    axis(5);
    glPopMatrix();

    glPushMatrix();
    glRotated(-90.0, 1.0, 0.0, 0.0); //    y��
    axis(5);
    glPopMatrix();

    // ��͹��1
    for (int i = 0; i < pointsNum1; i++) //�������еĶ���
    {
        glColor3f(0.0, 1.0, 1.0); // ������ɫ
        glPointSize(4);
        glBegin(GL_POINTS);
        glVertex3f(points1[i].x, points1[i].y, points1[i].z);
        glEnd();
    }
    for (int i = 0; i < planeNum1; i++)  // ����͹������
    {
        Plane temp_p = planes1[i];
        glColor3f(1.0, 0.0, 0.0); // ���ʺ�ɫ
        glPointSize(6);
        glBegin(GL_LINE_LOOP);
        for (int j = 0; j < temp_p.nodeNum; j++)
        {
            glVertex3f(points1[temp_p.node[j]].x, points1[temp_p.node[j]].y, points1[temp_p.node[j]].z);
        }
        glEnd();
    }

    // ��͹��2
    for (int i = 0; i < pointsNum2; i++) //�������еĶ���
    {
        glColor3f(0.0, 1.0, 1.0); // ������ɫ
        glPointSize(4);
        glBegin(GL_POINTS);
        glVertex3f(points2[i].x, points2[i].y, points2[i].z);
        glEnd();
    }
    for (int i = 0; i < planeNum2; i++)  // ����͹������
    {
        Plane temp_p = planes2[i];
        glColor3f(1.0, 0.0, 0.0); // ���ʺ�ɫ
        glPointSize(6);
        glBegin(GL_LINE_LOOP);
        for (int j = 0; j < temp_p.nodeNum; j++)
        {
            glVertex3f(points2[temp_p.node[j]].x, points2[temp_p.node[j]].y, points2[temp_p.node[j]].z);
        }
        glEnd();
    }



    glRotated(90.0, 0.0, 1.0, 0);
    glutSwapBuffers();
    // ##############################################################͹�����ƽ���###################################################################
    glutMainLoop(); //glut�¼�����ѭ��������ͼ�λ��ƣ����̣���������

    return 0;

    //cout << "Hello World!\n";
}




