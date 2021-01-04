#ifndef MAP_H
#define MAP_H
#include "../utils/point.h"
#include <assert.h>
#include "harray2d.h"

namespace gmapping
{
//PointAccumulator表示地图中一个cell（栅格）包括的内容
/*
acc：   栅格累计被击中位置
n：     栅格被击中次数
visits：栅格被访问的次数
*/
//PointAccumulator的一个对象，就是一个栅格，gmapping中其他类模板的cell就是这个

struct PointAccumulator
{
    //float类型的point
    typedef point<float> FloatPoint;
    //构造函数
    PointAccumulator() : acc(0, 0), n(0), visits(0) {}
    PointAccumulator(int i) : acc(0, 0), n(0), visits(0) { assert(i == -1); }
    //计算栅格被击中坐标累计值的平均值
    inline Point mean() const { return 1. / n * Point(acc.x, acc.y); }
    //返回该栅格被占用的概率，范围是 -1（没有访问过） 、[0,1]
    inline operator double() const { return visits ? (double)n * 1 / (double)visits : -1; }
    //更新该栅格成员变量
    inline void update(bool value, const Point &p = Point(0, 0));
    //该栅格被击中的位置累计，最后取累计值的均值
    FloatPoint acc;
    //n表示该栅格被击中的次数，visits表示该栅格被访问的次数
    int n, visits;
};

//更新该栅格成员变量，value表示该栅格是否被击中，击中n++，未击中仅visits++;
void PointAccumulator::update(bool value, const Point &p)
{
    if (value)
    {
        acc.x += static_cast<float>(p.x);
        acc.y += static_cast<float>(p.y);
        n++;
        visits += 1;
    }
    else
        visits++;
}

//在gmapping使用的过程中，cell为PointAccumulator Storage为HierarchicalArray2D<PointAccumulator>
/*
Point   m_center;									//地图的中心
double  m_worldSizeX, m_worldSizeY, m_delta;		//世界坐标系的大小，以及地图的分辨率
Storage m_storage;								    //地图数据存储的地方
int     m_mapSizeX, m_mapSizeY;						//地图的大小
int     m_sizeX2, m_sizeY2;                         //表示地图大小的一半
static const Cell m_unknown;                        //表示栅格的状态，表示整个地图栅格的初始状态
*/
//Map的一个对象，就是一张地图，大小尺寸、分辨率、地图中心

template <class Cell, class Storage, const bool isClass = true>
class Map
{
public:
    //构造函数 设置地图大小以及地图的分辨率
    Map(const Point &center, double worldSizeX, double worldSizeY, double delta);
    Map(const Point &center, double xmin, double ymin, double xmax, double ymax, double delta);

    //重新设置地图大小
    void resize(double xmin, double ymin, double xmax, double ymax);

    //物理坐标和栅格坐标的转换,后面加const，避免修改类的成员
    inline IntPoint world2map(const Point &p) const;
    inline IntPoint world2map(double x, double y) const { return world2map(Point(x, y)); }
    inline Point map2world(const IntPoint &p) const;
    inline Point map2world(int x, int y) const { return map2world(IntPoint(x, y)); }

    //返回地图的一些参数
    inline int getMapSizeX() const { return m_mapSizeX; }
    inline int getMapSizeY() const { return m_mapSizeY; }
    inline double getDelta() const { return m_delta; }

    //通过物理坐标或者栅格坐标返回单一栅格对象
    inline Cell &cell(const IntPoint &p);
    inline const Cell &cell(const IntPoint &p) const;
    inline Cell &cell(const Point &p);
    inline const Cell &cell(const Point &p) const;

    inline Cell &cell(int x, int y) { return cell(IntPoint(x, y)); }
    inline const Cell &cell(int x, int y) const { return cell(IntPoint(x, y)); }
    inline Cell &cell(double x, double y) { return cell(Point(x, y)); }
    inline const Cell &cell(double x, double y) const { return cell(Point(x, y)); }

    //判断某一个物理位置或者栅格坐标是否在地图里面 引用的是模板类m_storage(实际上是HierarchicalArray2D)中的函数
    inline bool isInside(int x, int y) const { return m_storage.cellState(IntPoint(x, y)) & Inside; }
    inline bool isInside(const IntPoint &p) const { return m_storage.cellState(p) & Inside; }
    inline bool isInside(double x, double y) const { return m_storage.cellState(world2map(x, y)) & Inside; }
    inline bool isInside(const Point &p) const { return m_storage.cellState(world2map(p)) & Inside; }

    //返回m_storage
    inline Storage &storage() { return m_storage; }
    inline const Storage &storage() const { return m_storage; }

protected:
    Point m_center;                             //地图的中心
    double m_worldSizeX, m_worldSizeY, m_delta; //世界坐标系的大小，以及地图的分辨率
    Storage m_storage;                          //地图数据存储的地方
    int m_mapSizeX, m_mapSizeY;                 //地图的大小
    int m_sizeX2, m_sizeY2;                     //表示地图大小的一半
    static const Cell m_unknown;                //一个cell的默认值-1
};

//此处初始化m_unknown，调用Cell的构造函数
template <class Cell, class Storage, const bool isClass>
const Cell Map<Cell, Storage, isClass>::m_unknown = Cell(-1);

//构造函数
template <class Cell, class Storage, const bool isClass>
Map<Cell, Storage, isClass>::Map(const Point &center, double worldSizeX, double worldSizeY, double delta) : m_storage((int)ceil(worldSizeX / delta), (int)ceil(worldSizeY / delta))
{
    m_center = center;
    m_worldSizeX = worldSizeX;
    m_worldSizeY = worldSizeY;
    m_delta = delta;
    m_mapSizeX = m_storage.getXSize() << m_storage.getPatchSize();
    m_mapSizeY = m_storage.getYSize() << m_storage.getPatchSize();
    m_sizeX2 = m_mapSizeX >> 1;
    m_sizeY2 = m_mapSizeY >> 1;
}

//构造函数
template <class Cell, class Storage, const bool isClass>
Map<Cell, Storage, isClass>::Map(const Point &center, double xmin, double ymin, double xmax, double ymax, double delta) : m_storage((int)ceil((xmax - xmin) / delta), (int)ceil((ymax - ymin) / delta))
{
    m_center = center;
    m_worldSizeX = xmax - xmin;
    m_worldSizeY = ymax - ymin;
    m_delta = delta;
    m_mapSizeX = m_storage.getXSize() << m_storage.getPatchSize();
    m_mapSizeY = m_storage.getYSize() << m_storage.getPatchSize();
    m_sizeX2 = (int)round((m_center.x - xmin) / m_delta);
    m_sizeY2 = (int)round((m_center.y - ymin) / m_delta);
}

//重新设置地图的大小，ceil向上取整、floor向下取整
template <class Cell, class Storage, const bool isClass>
void Map<Cell, Storage, isClass>::resize(double xmin, double ymin, double xmax, double ymax)
{
    IntPoint imin = world2map(xmin, ymin);
    IntPoint imax = world2map(xmax, ymax);

    int pxmin, pymin, pxmax, pymax;
    pxmin = (int)floor((float)imin.x / (1 << m_storage.getPatchMagnitude()));
    pxmax = (int)ceil((float)imax.x / (1 << m_storage.getPatchMagnitude()));
    pymin = (int)floor((float)imin.y / (1 << m_storage.getPatchMagnitude()));
    pymax = (int)ceil((float)imax.y / (1 << m_storage.getPatchMagnitude()));
    m_storage.resize(pxmin, pymin, pxmax, pymax);

    m_mapSizeX = m_storage.getXSize() << m_storage.getPatchSize();
    m_mapSizeY = m_storage.getYSize() << m_storage.getPatchSize();

    m_worldSizeX = xmax - xmin;
    m_worldSizeY = ymax - ymin;

    m_sizeX2 -= pxmin * (1 << m_storage.getPatchMagnitude());
    m_sizeY2 -= pymin * (1 << m_storage.getPatchMagnitude());
}

//物理和栅格坐标转换，round（）四舍五入
template <class Cell, class Storage, const bool isClass>
IntPoint Map<Cell, Storage, isClass>::world2map(const Point &p) const
{
    return IntPoint((int)round((p.x - m_center.x) / m_delta) + m_sizeX2, (int)round((p.y - m_center.y) / m_delta) + m_sizeY2);
}

template <class Cell, class Storage, const bool isClass>
Point Map<Cell, Storage, isClass>::map2world(const IntPoint &p) const
{
    return Point((p.x - m_sizeX2) * m_delta,
                 (p.y - m_sizeY2) * m_delta) +
           m_center;
}

//通过物理坐标或者栅格坐标返回栅格
template <class Cell, class Storage, const bool isClass>
Cell &Map<Cell, Storage, isClass>::cell(const IntPoint &p)
{
    AccessibilityState s = m_storage.cellState(p);
    if (!(s & Inside))
        assert(0);
    return m_storage.cell(p);
}

template <class Cell, class Storage, const bool isClass>
Cell &Map<Cell, Storage, isClass>::cell(const Point &p)
{
    IntPoint ip = world2map(p);
    AccessibilityState s = m_storage.cellState(ip);
    if (!(s & Inside))
        assert(0);
    return m_storage.cell(ip);
}

template <class Cell, class Storage, const bool isClass>
const Cell &Map<Cell, Storage, isClass>::cell(const IntPoint &p) const
{
    AccessibilityState s = m_storage.cellState(p);
    if (s & Allocated)
        return m_storage.cell(p);
    return m_unknown;
}

template <class Cell, class Storage, const bool isClass>
const Cell &Map<Cell, Storage, isClass>::cell(const Point &p) const
{
    IntPoint ip = world2map(p);
    AccessibilityState s = m_storage.cellState(ip);
    if (s & Allocated)
        return m_storage.cell(ip);
    return m_unknown;
}

//最终的地图类
typedef Map<PointAccumulator, HierarchicalArray2D<PointAccumulator>> ScanMatcherMap; //一张地图的类

}; // namespace GMapping

#endif