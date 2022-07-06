// 碰撞检测
// 画图ref: https://blog.csdn.net/weixin_42335313/article/details/105879928
#include <iostream>
#include <gl/glut.h>
#include <math.h>
#include <time.h>

using namespace std;
const int N = 200;  // 点的上限

int M; // num: 点的个数  m: 现有convex hull面的个数

typedef struct Vertex
{
    double x, y, z;  // 点的3维坐标
}Vertex;

typedef struct Plane
{
    int node[N]; // 点的索引，逆时针存储，用来表示面
    int nodeNum; // 顶点的个数
    int flag = 1; // flag=0: 平面不属于凸包，flag=1:平面属于凸包, flag=2:平面信息需要更新，添加新点形成新的凸平面
}Plane;

typedef struct Line // 定义线
{
    int v1, v2;
}Line;


int edge[N][N];
int temp_v[N];  //保存凸包中即将组成新平面的点

Vertex multiply(Vertex v1, Vertex v2)  // 向量叉乘 v1 x v2
{
    Vertex vec;
    vec.x = v1.y * v2.z - v1.z * v2.y;
    vec.y = v1.z * v2.x - v1.x * v2.z;
    vec.z = v1.x * v2.y - v1.y * v2.x;
    return vec;
}

double dot(Vertex v1, Vertex v2)  // 向量点乘
{
    double res = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    return res;
}

Vertex subduction(Vertex v1, Vertex v2)  // 向量相减 v1-v2
{
    Vertex vec;
    vec.x = v1.x - v2.x;
    vec.y = v1.y - v2.y;
    vec.z = v1.z - v2.z;
    return vec;
}

double getLength(Vertex vec)  // 获取向量长度
{
    double len = sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
    return len;
}

Line collinear(Vertex* points, int a, int b, int c)  // 处理共线的点，返回，如果相等，用新的点覆盖
{
    Line line;
    double dis_ab = pow(points[a].x - points[b].x, 2) + pow(points[a].y - points[b].y, 2) + pow(points[a].z - points[b].z, 2);
    double dis_ac = pow(points[a].x - points[c].x, 2) + pow(points[a].y - points[c].y, 2) + pow(points[a].z - points[c].z, 2);
    double dis_bc = pow(points[b].x - points[c].x, 2) + pow(points[b].y - points[c].y, 2) + pow(points[b].z - points[c].z, 2);
    if (dis_ab > dis_ac && dis_ab > dis_bc)  // dis_ab最大
    {
        line.v1 = a;
        line.v2 = b;
    }
    else if (dis_ab <= dis_ac && dis_ac >= dis_bc) // dis_ac最大
    {
        line.v1 = a;
        line.v2 = c;
    }
    else   // dis_bc最大
    {
        line.v1 = b;
        line.v2 = c;
    }
    return line;
}

bool isCollinear(Vertex a, Vertex b, Vertex c) // 判断3个点是否共线
{
    Vertex vec1 = subduction(a, b);
    Vertex vec2 = subduction(a, c);
    double v_cos = abs(dot(vec1, vec2) / (getLength(vec1) * getLength(vec2)));
    if (v_cos == 1) {
        return true;
    }
    return false;
}

int findMaxPoint(Vertex *points, Plane plane)  // 查找平面上极端点的索引
{
    Vertex vmax = points[plane.node[0]];  // 极大点
    int imax = 0;
    for (int i = 1; i < plane.nodeNum; i++)  // 选一个极端点 x_min,y_min,z_min
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


Plane coplanar(Vertex* points, Plane plane, int node)  // 处理共面  比较凸边和加入点到凸点的斜率
{
    plane.node[plane.nodeNum] = node; // 先把node加入平面，再更新
    plane.nodeNum = plane.nodeNum + 1;
    int newNodes[N]; // 存放待更新的node节点
    Vertex vec1, vec2, vec3;
    int imax = findMaxPoint(points, plane);
    // 更新plane上的点 从imax开始，比较余弦
    int j = 0;
    int i1, i2;  // 表示plane.node索引
    double v1v2, maxCos = -1;  // 表示余弦角度
    newNodes[j] = plane.node[imax]; // 应该存v索引
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
        while (i2 != i1)  // 终止条件：在所有点中选出余弦值最大的
        {
            vec2 = subduction(points[plane.node[i2]], points[plane.node[i1]]);
            v1v2 = dot(vec1, vec2) / (getLength(vec1) * getLength(vec2));  // 获取vec1,vec2的余弦值
            if (maxCos < v1v2)
            {
                maxCos = v1v2;
                imax = i2;
            }
            else if (maxCos == v1v2 && maxCos != -1) // 3点共线
            {
                Line line = collinear(points, plane.node[i1], plane.node[i2], plane.node[imax]);  
                if (line.v1 == plane.node[i2] || line.v2 == plane.node[i2])
                {
                    imax = i2;
                }
            }
            i2 = (i2 + 1) % plane.nodeNum;
        }
        // 下一个加入的点为imax(夹角最小的点)
        j = j + 1;
        newNodes[j] = plane.node[imax];
        if (newNodes[j] == newNodes[0]) break;   // 加入了一个圆
    }
    for (int i = 0; i < j; i++) // 赋值newNodes到plane.node
    {
        plane.node[i] = newNodes[i];
    }
    plane.nodeNum = j;
    plane.flag = 1;
    return plane;
}

void checkEdge(int v1, int v2, Plane *planes, int v, int j) // v1,v2:边的端点 v:新的点 j:面的索引
{
    if (edge[v1][v2] == 1 && edge[v2][v1] != 1)  // edge==1: 添加平面v1v2v
    {
        planes[M].node[0] = v1;
        planes[M].node[1] = v2;
        planes[M].node[2] = v;
        planes[M].nodeNum = 3;
        M = M + 1;
        edge[v1][v2] = 0;  //确保edge在下一个node遍历时正常工作
    }
    else if (edge[v1][v2] != 0)  // edge==2: 删除凸包中原来含有边ab的平面
    {
        planes[j].flag = 0;
        //edge[v1][v2] = 0;
    }
}

Plane inversePlane(Plane plane)  // 改变平面上点的存储顺序，逆序存储
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

int initConvexHull(Vertex* points, int& pointsNum, Plane* planes, int& planesNum)  // 初始化凸包，找3个不共面的点
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

void updateEdge(Plane plane) // plane的法向量跟新增节点在同一侧，更新edge
{
    int a, b;
    for (int k = 0; k < plane.nodeNum; k++)
    {
        a = plane.node[k];
        b = plane.node[(k + 1) % plane.nodeNum];
        edge[a][b] += 1;
    }
}

Vertex getNormalVector(Vertex* points,Plane plane) // 获取平面plane的法向量
{
    int a, b, c;
    Vertex v1, v2, v3, n; // v1,v2,v3表示plane上不共线的点  n表示法向量
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

void updateEdgeByNode(Vertex *points, Plane *planes, int& planesNum, Vertex node) // 遍历凸包上的每一面，保存信息至edge
{
    for (int j = 0; j < planesNum; j++)  // 遍历凸包上的面
    {
        if (planes[j].nodeNum == 0 || planes[j].flag == 0) continue; // 该平面已从凸包上删除
        Vertex n = getNormalVector(points,planes[j]);  // 法向量
        Vertex pnode = subduction(node, points[planes[j].node[0]]);
        double res = dot(n, pnode);
        if (res >= 0) // node在平面的外侧, node点可能与a,b,c相连
        {
            updateEdge(planes[j]);
            if (res == 0)  planes[j].flag = 2;  // 标识出该平面与该点共面
        }
        else
        {
            if (planesNum == 1)  // 若当前凸包中只有一个面，需调整点的存储的顺序，保证点跟法向量在一侧
            {
                planes[j] = inversePlane(planes[j]);
                updateEdge(planes[j]);
            }
        }
    }
}

void updateConvexHull(Vertex *points, Plane *planes, int &planesNum, int i)  // 更新凸包  i新加入的节点
{
    for (int j = 0; j < planesNum; j++) // 面
    {
        if (planes[j].flag == 2)
        {
            planes[j] = coplanar(points, planes[j], i);
            if (planesNum == 1) continue;  // 凸包上只有一个平面无法验证平面的法向量
            Vertex n = getNormalVector(points, planes[j]);  // Vertex* points,Plane plane
            bool check = true;
            for (int k = 0; k <= i; k++)
            {
                Vertex vec = subduction(points[k], points[planes[j].node[0]]);
                if (dot(vec, n) > 0)  // 凸包上点跟该平面的法向量在同一侧，说明该平面不是逆序
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


void getConvexHull(Vertex *points, int &pointsNum, Plane *planes, int &planesNum)  // 点集，点的个数，平面，平面个数 
{
    int v_index = initConvexHull(points, pointsNum, planes, planesNum);
    // 更新凸包
    bool flag = true; // 用于改变一次第一个平面的顺序
    M = planesNum;
    //cout << "初始化凸包" << endl;
    //printPlane(planes, planesNum);
    for (int i = v_index; i < pointsNum; i++)  // 加入点到convex hull
    {
        updateEdgeByNode(points,planes,planesNum,points[i]);
        // 查看每个面的每条边，将edge=1的边加入新的面 保持边原有的顺序（逆序），node作为最后的节点加入
        updateConvexHull(points, planes, planesNum, i); // 点集，平面集，平面数，点的索引
        planesNum = M;
        if (planesNum > 1 && flag) // 改变p[0]的点的顺序，使其逆序
        {
            planes[0] = inversePlane(planes[0]);
            flag = false;
        }
        // 更新edge
        for (int j = 0; j <= i; j++)
        {
            for (int k = 0; k <= i; k++)
            {
                edge[j][k] = 0;
            }
        }
        //cout << "第" << i << "个点: (" << points[i].x << "," << points[i].y << "," << points[i].z << ")" << endl;
        //printPlane(planes, planesNum);
    }

}

void axis(double length)  // 绘制坐标轴
{
    glColor3f(0.0f, 0.0f, 0.0f); // 设置绘图颜色
    /*
    将本次需要进行的缩放，平移等操作放在glPushMatrix和glPopMatrix之间
    glPushMatrix()和glPopMatrix()的配对使用能够消除上一次的变换对本次变化的影响，使本次变换是以世界坐标系的原点为参考点进行
    原理：
    (1) OPenGL中的modelview矩阵变换默认本次变换和上次变换不独立，即上次modelview变换后物体在世界坐标系下的位置是本次modelview变换的起点
    (2) openGL物体建模实际上是分两步走的。第一步，在世界坐标系的原点位置绘制出该物体；第二步，通过modelview变换矩阵对世界坐标系原点处的物体进行仿射变换，将该物体移动到世界坐标系的目标位置处
    (3) 将modelview变换放在glPushMatrix和glPopMatrix之间能够使本次变换和上次变换独立
    */
    glPushMatrix();
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0, 0.0, length);
    glEnd();
    // 将当前操作点移到指定位置
    glTranslated(0, 0.0, length - 0.2); // 定义一个平移矩阵，使该矩阵与当前矩阵相乘，使后续的图形进行平移变换，相对当前设置的坐标原点 x,y,z: 平移分量
    glColor3f(1.0, 0.0, 0.0);
    glutWireCone(0.04, 0.3, 8, 8); // 生成坐标轴箭头
    glPopMatrix();

}


void display(Vertex *points,int pointsNum,Plane *planes, int planesNum)
{
    glClear(GL_COLOR_BUFFER_BIT);  // 清楚颜色缓冲
    glMatrixMode(GL_PROJECTION); //对投影矩阵应用随后的矩阵操作
    glLoadIdentity(); //将当前坐标系的原点移到了屏幕中心：类似于一个复位操作

    glOrtho(-10.0, 10.0, -10.0, 10.0, -10, 10); // 将当前的可视空间设置为正投影空间 glOrtho(left,right,bottom,top,near,far)
    glPointSize(1);  //指定栅格化点的直径
    glMatrixMode(GL_MODELVIEW); // 对模型视景矩阵堆栈应用随后的矩阵操作
    glLoadIdentity();
    // gluLookAt(eye,center,up) z: center指向eye(和视线方向相反)
    gluLookAt(1.3, 1.6, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    //gluLookAt(0, 1.3, 1.6, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // 画坐标系
    axis(5); // z轴

    glPushMatrix();
    glRotated(90.0, 0.0, 1.0, 0);  //   x轴      glrotate:旋转轴从原点出发
    axis(5);
    glPopMatrix();

    glPushMatrix();
    glRotated(-90.0, 1.0, 0.0, 0.0); //    y轴
    axis(5);
    glPopMatrix();

    // 画凸包
    for (int i = 0; i < pointsNum; i++) //画出所有的顶点
    {
        glColor3f(0.0, 1.0, 1.0); // 画笔蓝色
        glPointSize(4);
        glBegin(GL_POINTS);
        glVertex3f(points[i].x, points[i].y, points[i].z);
        glEnd();
    }
    for (int i = 0; i < planesNum; i++)  // 画出凸包的面
    {
        Plane temp_p = planes[i];
        glColor3f(1.0, 0.0, 0.0); // 画笔红色
        glPointSize(6);
        glBegin(GL_LINE_LOOP);
        for (int j = 0; j < temp_p.nodeNum; j++)
        {
            glVertex3f(points[temp_p.node[j]].x, points[temp_p.node[j]].y, points[temp_p.node[j]].z);
        }
        glEnd();
    }


    //glColor3f(0.0, 0.0, 0.0); // 设置绘图颜色，默认
    //glLineWidth(1.0f); //设置线宽，默认1.0f
    //axis(2); 
    /**
    OpenGL要求：指定顶点的命令必须包含在glBegin函数之后,glEnd之前,否则指定的顶点将被忽略
    由glBegin来指明如何使用这些点： GL_POLYGON表示画多边形（由点连接成多边形）
    OpenGL指定点：glVertex*, glVertex2i,glVertex2f,glVertex3f,glVertex3fv  数字表示参数的个数,v表示传递的几个参数将使用指针的方式
    glBegin参数解析：
    （1）GL_POINTS: 绘制点  （2）GL_LINES:绘制线段（顶点的排列组合？？？）
    （3）GL_LINE_STRIP: 绘制从第一个顶点到最后一个顶点依次相连的一组线段，第n和n+1个顶点定义了线段n，绘制n-1条线段
    （4）GL_LINE_LOOP: 绘制从第一个顶点到最后一个顶点依次相连的一组线段，最后一个顶点和第一个顶点相连，第n和n+1个顶点定义了线段n, 绘制n条线段
    （5）GL_TRIANGLES: 把每个顶点作为一个独立的三角形
    （6）GL_QUADS: 绘制由4个顶点组成的一组单独的四边形
    （7）GL_POLYGON: 绘制了一个凸多边形
    */
    //glPopMatrix(); //用来整体绕y轴旋转
    glRotated(90.0, 0.0, 1.0, 0);
    glutSwapBuffers();

    //glFlush(); // 强制刷新缓冲，保证绘图命令将被执行（而不是让其在缓冲区中等待）
}

//碰撞检测  给定两个三维点集，检查点集构成的凸包是否发生碰撞
bool isCollision(Vertex* points1, int pointsNum1, Vertex* points2, int pointsNum2)
{
    // 先获取第一个点集的凸包
    Plane plane1[N];
    int planeNum1;
    getConvexHull(points1, pointsNum1, plane1, planeNum1);
    bool flag = true;  // 表示v在凸包内部   
    // 遍历points中的点，检查点是否与plane1相撞
    for (int i = 0; i < pointsNum2; i++)
    {
        flag = true;
        Vertex v = points2[i];
        for (int j = 0; j < planeNum1; j++) // 先得到平面的法向量，然后判断点v与平面的位置
        {
            Plane p = plane1[j];
            Vertex n = getNormalVector(points1, p);  // 所有平面的法向量均指向凸包外侧。v与所有平面法向量点积都<0,说明v在凸包内部。相撞
            Vertex pv = subduction(v, points1[p.node[0]]);
            double dotRes = dot(pv, n);
            if (dotRes > 0) // v在该平面外侧，不在凸包内部
            {
                flag = false;
                break;
            }
            else if (dotRes == 0) // v与平面共面
            {
                for (int k = 1; k <= p.nodeNum; k++)  // 比较角度大小, 若v在平面内，v与平面的切线角度一直大于
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
                    if (angle2Cos > angle1Cos)  // angle2<angle1,点在平面外
                    {
                        flag = false;
                        break;
                    }
                    else if (angle2Cos == angle1Cos) // v2,v3,v共线
                    {
                        points1[pointsNum1] = v;
                        Line line = collinear(points1, v2_index, v2_index, pointsNum1); // 该函数会返回3个点中最长的两个端点组成边，如果边长相等，会用pointsNum覆盖其中一个点
                        if (line.v1 != pointsNum1 && line.v2 != pointsNum1)
                        {
                            flag = false;
                            break;
                        }

                     
                    }
                }

            }
        }
        if (flag == true) break;   // 说明点v在凸包内 或 平面内 或 共线， 找到了一个points1的点与points2相撞，不用遍历其他的点了
    }
    return flag;
}


int main(int argc, char** argv)
{
    // 初始化两个凸包的顶点和平面
    Vertex points1[N], points2[N];  // 用于保存两个点集中点的信息
    Plane planes1[N], planes2[N];  // 用于保存两个凸包中平面的信息

    int pointsNum1, pointsNum2;   // 保存点集中点的个数
    int planeNum1, planeNum2;   // 保存凸包中平面的个数
    // 输入数据点
    cout << "输入点集1（点的个数和点的3维坐标）：" << endl;
    cin >> pointsNum1;
    for (int i = 0; i < pointsNum1; i++)
    {
        cin >> points1[i].x >> points1[i].y >> points1[i].z;
    }
    cout << "输入点集2（点的个数和点的3维坐标）：" << endl;
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
    cout << "碰撞检测耗时" << totalTime << "秒" << endl;
    
    if (res == true)
    {
        cout << "两个凸包相撞" << endl;
    }
    else 
    {
        cout << "两个凸包没有碰撞" << endl;
    }

    // 计算凸包
    getConvexHull(points1, pointsNum1, planes1, planeNum1);
    getConvexHull(points2, pointsNum2, planes2, planeNum2);
    // 使用opengl绘图     
    glutInit(&argc, argv);  //初始化GLUT库
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE); // 设置图形显示模式
    glutInitWindowPosition(0, 0); //设置窗口位置，左上角为原点
    glutInitWindowSize(640, 500); //设置窗口大小
    glutCreateWindow("Convex Hull"); //创建窗口，字符串为窗口名称
    glClearColor(1.0, 1.0, 1.0, 0); //设置背景颜色

    // 图形绘制
    //glutDisplayFunc(&display);  // 绘制图形   // 暂不清楚如何调用有参函数
    //  ###############################################绘制凸包start############################################################
    glClear(GL_COLOR_BUFFER_BIT);  // 清楚颜色缓冲
    glMatrixMode(GL_PROJECTION); //对投影矩阵应用随后的矩阵操作
    glLoadIdentity(); //将当前坐标系的原点移到了屏幕中心：类似于一个复位操作

    glOrtho(-10.0, 10.0, -10.0, 10.0, -10, 10); // 将当前的可视空间设置为正投影空间 glOrtho(left,right,bottom,top,near,far)
    glPointSize(1);  //指定栅格化点的直径
    glMatrixMode(GL_MODELVIEW); // 对模型视景矩阵堆栈应用随后的矩阵操作
    glLoadIdentity();
    // gluLookAt(eye,center,up) z: center指向eye(和视线方向相反)
    gluLookAt(1.3, 1.6, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    //gluLookAt(0, 1.3, 1.6, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // 画坐标系
    axis(5); // z轴

    glPushMatrix();
    glRotated(90.0, 0.0, 1.0, 0);  //   x轴      glrotate:旋转轴从原点出发
    axis(5);
    glPopMatrix();

    glPushMatrix();
    glRotated(-90.0, 1.0, 0.0, 0.0); //    y轴
    axis(5);
    glPopMatrix();

    // 画凸包1
    for (int i = 0; i < pointsNum1; i++) //画出所有的顶点
    {
        glColor3f(0.0, 1.0, 1.0); // 画笔蓝色
        glPointSize(4);
        glBegin(GL_POINTS);
        glVertex3f(points1[i].x, points1[i].y, points1[i].z);
        glEnd();
    }
    for (int i = 0; i < planeNum1; i++)  // 画出凸包的面
    {
        Plane temp_p = planes1[i];
        glColor3f(1.0, 0.0, 0.0); // 画笔红色
        glPointSize(6);
        glBegin(GL_LINE_LOOP);
        for (int j = 0; j < temp_p.nodeNum; j++)
        {
            glVertex3f(points1[temp_p.node[j]].x, points1[temp_p.node[j]].y, points1[temp_p.node[j]].z);
        }
        glEnd();
    }

    // 画凸包2
    for (int i = 0; i < pointsNum2; i++) //画出所有的顶点
    {
        glColor3f(0.0, 1.0, 1.0); // 画笔蓝色
        glPointSize(4);
        glBegin(GL_POINTS);
        glVertex3f(points2[i].x, points2[i].y, points2[i].z);
        glEnd();
    }
    for (int i = 0; i < planeNum2; i++)  // 画出凸包的面
    {
        Plane temp_p = planes2[i];
        glColor3f(1.0, 0.0, 0.0); // 画笔红色
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
    // ##############################################################凸包绘制结束###################################################################
    glutMainLoop(); //glut事件处理循环，包括图形绘制，键盘，鼠标输入等

    return 0;

    //cout << "Hello World!\n";
}




