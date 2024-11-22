#ifndef CONVEXHULL2D_H
#define CONVEXHULL2D_H

#include "math/vector3.h"

#ifndef DOUBLE_ZERO
#define DOUBLE_ZERO 0.00000001
#endif

using namespace std;

namespace ARGOS_ROS_SWARM{

class CConvexHull2D {
// Graham Scan 算法求凸包
public :

CConvexHull2D() {points.clear();};
CConvexHull2D(const vector<CVector3>& points);
vector<CVector3>& GetPoints() {return points;}

//求一堆点的凸包，在构造函数里调用了
static vector<CVector3> grahamScan(const vector<CVector3>& input_points);
//求最小外接矩形
vector<CVector3> rotatingCalipers();
//求面积
double areaSize() const;

//求最长轴，返回两个点的索引
void findConvexhullMaxDiameter(uint32_t &indexI, uint32_t &indexJ) const;
CVector3 findConvexhullMaxDiameter() const;
//延horizontal方向,求span
double findArbritraryDiameterLength(const CVector3& horizontal) const;
//延horizontal方向切m刀 返回分割线
vector< vector<CVector3> > slice(const CVector3& horizontal, uint32_t m) const;
//延horizontal方向, 先走start距离切一刀，再每走width距离再切一刀
vector<CConvexHull2D> slice(const CVector3& horizontal, double start, double width) const;
//延horizontal方向切m刀 返回分割线
vector<CConvexHull2D> sliceBySize(const CVector3& horizontal, uint32_t m) const;

private:

vector<CVector3> points;

}; // end of class

} // end of namespace

#endif
