#include "math/ConvexHull2D.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#include "math/vector3.h"
#include "ConvexHull2D.h"

using namespace std;

namespace ARGOS_ROS_SWARM {

// 比较函数，用于寻找最低且最左的点
bool comparePoints(const CVector3& a, const CVector3& b) {
	return a.GetY() < b.GetY() || (a.GetY() == b.GetY() && a.GetX() < b.GetX());
}

// 计算两点之间的极角（用于排序）
double polarAngle(const CVector3& a, const CVector3& b) {
	// atan2 returns (-pi, pi], if two points are the same, set it -pi so that it will be the first one in sort
	if ((b.GetY() - a.GetY() == 0) && (b.GetX() - a.GetX() == 0))
		return -M_PI;
	return atan2(b.GetY() - a.GetY(), b.GetX() - a.GetX());
}

// 计算点 O, A, B 的叉积（判断 A->B 是否为左转）
double checkAntiClockWise(const CVector3& O, const CVector3& A, const CVector3& B) {
	return (A.GetX() - O.GetX()) * (B.GetY() - O.GetY()) - (A.GetY() - O.GetY()) * (B.GetX() - O.GetX());
}

// 计算两个点之间的欧氏距离
double dist2D(const CVector3& A, const CVector3& B) {
	return sqrt((A.GetX() - B.GetX()) * (A.GetX() - B.GetX()) + (A.GetY() - B.GetY()) * (A.GetY() - B.GetY()));
}

// 计算两个向量的点积
double dot2D(const CVector3& A, const CVector3& B) {
	return A.GetX() * B.GetX() + A.GetY() * B.GetY();
}

// 计算点 O, A, B 的叉积（判断 A->B 是否为左转）
double crossProduct2D(const CVector3& A, const CVector3& B) {
	return A.GetX() * B.GetY() - A.GetY() * B.GetX();
}

CVector3 clockwise90(const CVector3& A) {
	return CVector3(A.GetY(), -A.GetX(), 0);
}

CVector3 anticlockwise90(const CVector3& A) {
	return CVector3(-A.GetY(), A.GetX(), 0);
}

// 计算向量的单位法向量
CVector3 normalize2D(const CVector3& A, const CVector3& B) {
	double dx = B.GetX() - A.GetX();
	double dy = B.GetY() - A.GetY();
	double length = sqrt(dx * dx + dy * dy);
	return CVector3(dx / length, dy / length, 0);
}

bool raySegmentIntersection2D(const CVector3& O,
                              const CVector3& D,
                              const CVector3& A,
                              const CVector3& B,
                              CVector3& intersection) {
	CVector3 OA = A - O;
	CVector3 OB = B - O;
	CVector3 AB = B - A;

	double D_cross_AB = crossProduct2D(D, AB);
	double OA_cross_D = crossProduct2D(OA, D);

	// 射线与AB平行，没有交点
	if (fabs(D_cross_AB) < 1e-9) return false;

	// chatgpt 这么说的，没看为什么
	double t = crossProduct2D(OA, AB) / D_cross_AB;  // 射线方向参数
    double u = OA_cross_D / D_cross_AB;     // 线段方向参数

    // 检查交点是否在射线上 (t >= 0) 且在线段 AB 上 (0 <= u <= 1)
    if (t >= 0 && u >= 0 && u <= 1) {
        intersection.SetX(O.GetX() + t * D.GetX());
        intersection.SetY(O.GetY() + t * D.GetY());
        intersection.SetZ(0);
        return true;
    }
    return false;
}

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
CConvexHull2D::CConvexHull2D(const vector<CVector3> &input_points)
{
	points = grahamScan(input_points);
}

// Graham Scan 算法求凸包
vector<CVector3> CConvexHull2D::grahamScan(const vector<CVector3>& _points) {
	vector<CVector3> points = _points; //sort will change the order of points
	// 1. 找到起点（y 最小，x 最小的点）
	CVector3 start = *min_element(points.begin(), points.end(), comparePoints);

	// 2. 按与起点的极角排序，如果极角相同，则按距离排序
	sort(points.begin(), points.end(), [&](const CVector3& a, const CVector3& b) {
		double angleA = polarAngle(start, a);
		double angleB = polarAngle(start, b);
		if (angleA == angleB) {
			return hypot(a.GetX() - start.GetX(), a.GetY() - start.GetY()) < hypot(b.GetX() - start.GetX(), b.GetY() - start.GetY());
		}
		return angleA < angleB;
	});

	// 3. 使用栈构建凸包
	vector<CVector3> hull;
	for (const auto& p : points) {
		// 如果不是左转，就退栈
		while (hull.size() > 1 && checkAntiClockWise(hull[hull.size() - 2], hull.back(), p) <= 0) {
			hull.pop_back();
		}
		hull.push_back(p);
	}

	return hull;
}

// 旋转卡壳算法：计算凸包的最小外接矩形并返回矩形的四个顶点
vector<CVector3> CConvexHull2D::rotatingCalipers() {
	uint32_t n = points.size();
	if (n < 3) return {};  // 如果点数少于3个，则无法形成矩形

	// 初始化最小面积
	double minArea = numeric_limits<double>::max();
	vector<CVector3> minRectangle;  // 存储最小外接矩形的四个顶点

	// 遍历凸包的每一条边
	for (uint32_t i = 0; i < n; ++i) {
		double left = 0, right = 0, top = 0, bottom = 0;  // 初始化旋转卡壳四个指针

		CVector3 A = CVector3(points[i]);
		CVector3 B = CVector3(points[(i + 1) % n]);  // 当前边AB

		// 当前边的方向向量
		CVector3 HorizontalUnit = normalize2D(A, B);
		CVector3 VerticalUnit = CVector3(-HorizontalUnit.GetY(), HorizontalUnit.GetX(), 0);

		for (uint32_t j = 0; j < n; j++) {
			double horizontalShadow = dot2D(points[j] - A, HorizontalUnit);
			double verticalShadow = dot2D(points[j] - A, VerticalUnit);
			if (horizontalShadow < left) left = horizontalShadow;
			if (horizontalShadow > right) right = horizontalShadow;
			if (verticalShadow < bottom) bottom = verticalShadow;
			if (verticalShadow > top) top = verticalShadow;
		}

		// 计算当前矩形的四个顶点
		CVector3 LeftBottom  = A + HorizontalUnit * left  + VerticalUnit * bottom;
		CVector3 RightBottom = A + HorizontalUnit * right + VerticalUnit * bottom;
		CVector3 RightTop    = A + HorizontalUnit * right + VerticalUnit * top;
		CVector3 LeftTop     = A + HorizontalUnit * left  + VerticalUnit * top;

		// 如果当前面积更小，记录矩形四个顶点
		double area = (right - left) * (top - bottom);
		if (area < minArea) {
			minArea = area;
			minRectangle = {LeftBottom, RightBottom, RightTop, LeftTop};
		}
	}

	return minRectangle;  // 返回最小外接矩形的四个顶点
}


double CConvexHull2D::areaSize() const
{
	uint32_t n = points.size();
	if (n <= 2) {return 0;}  // 如果点数少于3个，直接返回2两个点的连线

	double size = 0;
	for (uint32_t i = 1; i < n - 1; i++)
		size += crossProduct2D(points[i] - points[0], points[i+1] - points[0]) * 0.5;

	return size;
}

CVector3 CConvexHull2D::findConvexhullMaxDiameter() const
{
	uint32_t i, j;
	findConvexhullMaxDiameter(i, j);
	return points[j] - points[i];
}

void CConvexHull2D::findConvexhullMaxDiameter(uint32_t &indexI, uint32_t &indexJ) const
{
    uint32_t n = points.size();
	if (n < 2) {indexI = 0; indexJ = 0;}  // 如果点数少于3个，直接返回2两个点的连线
	if (n == 2) {indexI = 0; indexJ = 1;}  // 如果点数少于3个，直接返回2两个点的连线

	double maxDiameter = 0;
	uint32_t maxI = 0;
	uint32_t maxJ = 1;
	for (uint32_t i = 0; i < n; ++i) {
		double maxFocalDiameter = 0;
		uint32_t maxFocalJ = i + 1;
		for (uint32_t j = (i + 1) % n; j != i; j = (j + 1) % n) {
			double focalDiameter = (points[j] - points[i]).Length();
			if ( focalDiameter > maxFocalDiameter) {
				maxFocalJ = j;
				maxFocalDiameter = focalDiameter;
			}
			else
				break;
		}
		if (maxFocalDiameter > maxDiameter) {
			maxI = i; maxJ = maxFocalJ;
			maxDiameter = maxFocalDiameter;
		}
	}

	indexI = maxI;
	indexJ = maxJ;
}

double ARGOS_ROS_SWARM::CConvexHull2D::findArbritraryDiameterLength(const CVector3 &horizontal) const
{
	uint32_t n = points.size();
	CVector3 dir = CVector3(horizontal).Normalize();

	// find left and right
	double min = 0;
	double max = 0;
	uint32_t minI = 0;
	uint32_t maxI = 0;
	for (uint32_t i = 1; i < n; i++) {
		double shadow = dot2D(points[i] - points[0], dir);
		if (shadow < min) { min = shadow; minI = i; }
		if (shadow > max) { max = shadow; maxI = i; }
	}

	double span = max - min;

	return span;
}

vector<vector<CVector3>> CConvexHull2D::slice(const CVector3& horizontal, uint32_t m) const
{
	CVector3 dir = CVector3(horizontal).Normalize();
	CVector3 dirAnti90 = anticlockwise90(dir);
	uint32_t n = points.size();
	// find left and right
	double min = 0;
	double max = 0;
	uint32_t minI = 0;
	uint32_t maxI = 0;
	for (uint32_t i = 1; i < n; i++) {
		double shadow = dot2D(points[i] - points[0], dir);
		if (shadow < min) { min = shadow; minI = i; }
		if (shadow > max) { max = shadow; maxI = i; }
	}

	double span = max - min;
	double step = span / (m+1);

	// 以minI点为原点，dir为X轴，给所有点作坐标转换
	vector<CVector3> P = {CVector3(0,0,0)};
	for (uint32_t i = (minI+1)%n; i != minI; i = (i+1)%n )
		P.emplace_back(
			dot2D(points[i] - points[minI], dir),
			dot2D(points[i] - points[minI], dirAnti90),
			0
		);

	vector<vector<CVector3>> result;
	uint32_t downIndex = 1, upIndex = n-1;
	for (uint32_t i = 0; i < m; i++) {
		double currentX = step * (i+1);
		while (P[downIndex].GetX() < currentX) { downIndex = (downIndex+1)%n; }
		while (P[upIndex].GetX() < currentX) { upIndex = (upIndex+n-1)%n; }

		//down segment:
		CVector3 downSegmentDir = P[downIndex] - P[(downIndex+n-1)%n];
		double downK = downSegmentDir.GetY() / downSegmentDir.GetX();
		double downB = P[downIndex].GetY() - downK * P[downIndex].GetX();
		double downY = downK * currentX + downB;

		//up segment:
		CVector3 upSegmentDir = P[upIndex] - P[(upIndex+1)%n];
		double upK = upSegmentDir.GetY() / upSegmentDir.GetX();
		double upB = P[upIndex].GetY() - upK * P[upIndex].GetX();
		double upY = upK * currentX + upB;

		vector<CVector3> pair;
		pair.push_back(points[minI] + dir * currentX + dirAnti90 * downY);
		pair.push_back(points[minI] + dir * currentX + dirAnti90 * upY);
		result.push_back(pair);
	}

	return result;
}

vector<CConvexHull2D> CConvexHull2D::slice(const CVector3& horizontal, double start, double width) const {
	CVector3 dir = CVector3(horizontal).Normalize();
	CVector3 dirAnti90 = anticlockwise90(dir);
	uint32_t n = points.size();
	// find left and right
	double min = 0;
	double max = 0;
	uint32_t minI = 0;
	uint32_t maxI = 0;
	for (uint32_t i = 1; i < n; i++) {
		double shadow = dot2D(points[i] - points[0], dir);
		if (shadow < min) { min = shadow; minI = i; }
		if (shadow > max) { max = shadow; maxI = i; }
	}

	double span = max - min;

	// 以minI点为原点，dir为X轴，给所有点作坐标转换
	vector<CVector3> P = {CVector3(0,0,0)};
	for (uint32_t i = (minI+1)%n; i != minI; i = (i+1)%n )
		P.emplace_back(
			dot2D(points[i] - points[minI], dir),
			dot2D(points[i] - points[minI], dirAnti90),
			0
		);

	vector<CConvexHull2D> result;
	uint32_t downIndex = 1, upIndex = n-1;
	// create the first small convexhull
	result.emplace_back();
	// push the first point of the first small convexhull
	result.back().GetPoints().push_back(points[minI]);
	double currentX = start;
	while (currentX < span) {
		while (P[downIndex].GetX() < currentX) {
			if (P[downIndex].GetX() < currentX - DOUBLE_ZERO)
				result.back().GetPoints().push_back(points[minI] + dir * P[downIndex].GetX() + dirAnti90 * P[downIndex].GetY());
			downIndex = (downIndex+1)%n;
		}
		vector<CVector3> upChain;
		while (P[upIndex].GetX() < currentX) {
			if (P[downIndex].GetX() < currentX - DOUBLE_ZERO)
				upChain.push_back(points[minI] + dir * P[upIndex].GetX() + dirAnti90 * P[upIndex].GetY());
			upIndex = (upIndex+n-1)%n;
		}

		//down segment:
		CVector3 downSegmentDir = P[downIndex] - P[(downIndex+n-1)%n];
		double downK = downSegmentDir.GetY() / downSegmentDir.GetX();
		double downB = P[downIndex].GetY() - downK * P[downIndex].GetX();
		double downY = downK * currentX + downB;
		CVector3 downPoint = points[minI] + dir * currentX + dirAnti90 * downY;

		//up segment:
		CVector3 upSegmentDir = P[upIndex] - P[(upIndex+1)%n];
		double upK = upSegmentDir.GetY() / upSegmentDir.GetX();
		double upB = P[upIndex].GetY() - upK * P[upIndex].GetX();
		double upY = upK * currentX + upB;
		CVector3 upPoint = points[minI] + dir * currentX + dirAnti90 * upY;

		result.back().GetPoints().push_back(downPoint);
		result.back().GetPoints().push_back(upPoint);

		for (int32_t i = upChain.size() - 1; i >= 0; i--) // use int32_t rather than uint, because if size is 0, uint32 -1 would be larger than 0
			result.back().GetPoints().push_back(upChain[i]);

		result.emplace_back();
		result.back().GetPoints().push_back(upPoint);
		result.back().GetPoints().push_back(downPoint);

		currentX += width;
	}

	// push the last one or two points
	for (uint32_t i = downIndex; i <= upIndex; i++)
		result.back().GetPoints().push_back(points[minI] + dir * P[i].GetX() + dirAnti90 * P[i].GetY());

	return result;
}

vector<CConvexHull2D> CConvexHull2D::sliceBySize(const CVector3 &horizontal, uint32_t m) const
{
	double sliceAreaSize = areaSize() / (m + 1);

	CVector3 dir = CVector3(horizontal).Normalize();
	CVector3 dirAnti90 = anticlockwise90(dir);
	uint32_t n = points.size();
	// find left and right
	double min = 0;
	double max = 0;
	uint32_t minI = 0;
	uint32_t maxI = 0;
	for (uint32_t i = 1; i < n; i++) {
		double shadow = dot2D(points[i] - points[0], dir);
		if (shadow < min) { min = shadow; minI = i; }
		if (shadow > max) { max = shadow; maxI = i; }
	}

	double span = max - min;

	// 以minI点为原点，dir为X轴，给所有点作坐标转换
	vector<CVector3> P = {CVector3(0,0,0)};
	for (uint32_t i = (minI+1)%n; i != minI; i = (i+1)%n )
		P.emplace_back(
			dot2D(points[i] - points[minI], dir),
			dot2D(points[i] - points[minI], dirAnti90),
			0
		);

	vector<CConvexHull2D> result;
	uint32_t downIndex = 1, upIndex = n-1;
	// create the first small convexhull
	result.emplace_back();
	// push the first point of the first small convexhull
	result.back().GetPoints().push_back(points[minI]);

	double topLength = 0;
	double lastX = 0;
	double accumulateSize = 0;
	vector<CVector3> upChain;
	uint32_t iSlice = 0;
	while (iSlice < m) {
		// find the nearest X from both up and down
		bool UporDown_UpisTrue = false; // true down
		double currentX = P[downIndex].GetX();
		if (P[upIndex].GetX() < currentX) { currentX = P[upIndex].GetX(); UporDown_UpisTrue = true;}
		//down segment:
		CVector3 downSegmentDir = P[downIndex] - P[(downIndex+n-1)%n];
		double downK = downSegmentDir.GetY() / downSegmentDir.GetX();
		double downB = P[downIndex].GetY() - downK * P[downIndex].GetX();
		double downY = downK * currentX + downB;
		if (abs(downSegmentDir.GetX()) < DOUBLE_ZERO) downY = P[downIndex].GetY();
		//up segment:
		CVector3 upSegmentDir = P[upIndex] - P[(upIndex+1)%n];
		double upK = upSegmentDir.GetY() / upSegmentDir.GetX();
		double upB = P[upIndex].GetY() - upK * P[upIndex].GetX();
		double upY = upK * currentX + upB;
		if (abs(upSegmentDir.GetX()) < DOUBLE_ZERO) upY = P[upIndex].GetY();

		double bottomLength = upY - downY;
		double smallNarrowAreaSize = (topLength + bottomLength) * (currentX - lastX) * 0.5;

		if (accumulateSize + smallNarrowAreaSize < sliceAreaSize) {
			if (UporDown_UpisTrue) {
				upChain.push_back(points[minI] + dir * P[upIndex].GetX() + dirAnti90 * P[upIndex].GetY());
				upIndex = (upIndex+n-1)%n;
			}
			else {
				result.back().GetPoints().push_back(points[minI] + dir * P[downIndex].GetX() + dirAnti90 * P[downIndex].GetY());
				downIndex = (downIndex+1)%n;
			}
			lastX = currentX;
			topLength = bottomLength;
			accumulateSize += smallNarrowAreaSize;
		}
		else {
			// target size
			double S = sliceAreaSize - accumulateSize;
			double A = topLength;
			double B = bottomLength;
			double H = currentX - lastX;
			double X = H / (B-A) * (-A + sqrt(A*A + 2*S*(B-A)/H) );

			cout << "X     = " << X << endl;
			cout << "lastX = " << lastX << endl;
			currentX = X + lastX;
			cout << "currentX     = " << currentX << endl;

			//down segment:
			CVector3 downSegmentDir = P[downIndex] - P[(downIndex+n-1)%n];
			double downK = downSegmentDir.GetY() / downSegmentDir.GetX();
			double downB = P[downIndex].GetY() - downK * P[downIndex].GetX();
			double downY = downK * currentX + downB;
			CVector3 downPoint = points[minI] + dir * currentX + dirAnti90 * downY;
			//up segment:
			CVector3 upSegmentDir = P[upIndex] - P[(upIndex+1)%n];
			double upK = upSegmentDir.GetY() / upSegmentDir.GetX();
			double upB = P[upIndex].GetY() - upK * P[upIndex].GetX();
			double upY = upK * currentX + upB;
			CVector3 upPoint = points[minI] + dir * currentX + dirAnti90 * upY;

			result.back().GetPoints().push_back(downPoint);
			result.back().GetPoints().push_back(upPoint);
			for (int32_t i = upChain.size() - 1; i >= 0; i--) // use int32_t rather than uint, because if size is 0, uint32 -1 would be larger than 0
				result.back().GetPoints().push_back(upChain[i]);

			result.emplace_back();
			result.back().GetPoints().push_back(upPoint);
			result.back().GetPoints().push_back(downPoint);

			topLength = upY - downY;
			lastX = currentX;
			accumulateSize = 0;
			upChain.clear();

			iSlice++;
		}
	}

	// push the last one or two points
	for (uint32_t i = downIndex; i <= upIndex; i++)
		result.back().GetPoints().push_back(points[minI] + dir * P[i].GetX() + dirAnti90 * P[i].GetY());

	return result;
}

} // end namespace
