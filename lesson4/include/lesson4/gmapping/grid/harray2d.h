#ifndef HARRAY2D_H
#define HARRAY2D_H
#include <iostream>
#include <memory>
#include <set>
//必须说明的是set关联式容器,在set中每个元素的值都唯一，而且系统能根据元素的值自动进行排序
//map和set的插入删除效率比用其他序列容器高
#include "../utils/point.h"
#include "array2d.h"

namespace gmapping
{
//枚举类型
enum AccessibilityState{Outside=0x0, Inside=0x1, Allocated=0x2};

//这里的cell在gmapping里面指的是PointAccumulator
/*
	HierarchicalArray2D 类继承自 Array2D ，成员变量存在一个指向地图块的二级指针，Cell ** m_cells;
	所以，该类是一个二维patch数组，一个patch就是一个地图块，一个二级指针，指向一块地图
	一个patch包含2^m_patchMagnitude * 2^m_patchMagnitude个栅格
	也就是说地图实际上是分为两层的，第一层的元素为Patch（每一块地图块） 第二层的元素为cell（每一个栅格）

	patch的大小等级由 m_patchMagnitude决定
	patch的实际大小等于 m_patchSize = 1<<m_patchMagnitude = 2^m_patchMagnitude
	patch的实际大小表示一个patch里面包含有m_patchSize*m_patchSize的cell
	在gmapping源码中 m_patchMagnitude的默认大小为5 也就是说patch的默认大小为32*32
*/
//HierarchicalArray2D 的一个对象就是一个 存储“地图补丁”的二维数组（栅格地图）

template <class Cell>
class HierarchicalArray2D: public Array2D<std::shared_ptr< Array2D<Cell> > >
{
	public:
		typedef std::set< point<int>, pointcomparator<int> > PointSet;
		//构造函数
		HierarchicalArray2D(int xsize, int ysize, int patchMagnitude=5);
		//析构函数
		virtual ~HierarchicalArray2D(){}
		//调整存储 “地图补丁” 的二维数组的大小
		void resize(int ixmin, int iymin, int ixmax, int iymax);
		//输入 “栅格” 的栅格坐标做入口参数，返回 “地图补丁” 的 “栅格坐标”，也就是一个栅格属于哪个地图补丁
		inline IntPoint patchIndexes(int x, int y) const;
		inline IntPoint patchIndexes(const IntPoint& p) const { return patchIndexes(p.x,p.y);}
		//输入 “栅格” 栅格坐标做入口参数，返回该 “地图补丁” 的内存是否已经分配
		inline bool isAllocated(int x, int y) const;
		inline bool isAllocated(const IntPoint& p) const { return isAllocated(p.x,p.y);}
		//输入 “栅格” 栅格坐标做入口参数，返回一个栅格对象
		inline Cell& cell(int x, int y);
		inline Cell& cell(const IntPoint& p) { return cell(p.x,p.y); }
		inline const Cell& cell(int x, int y) const;
		inline const Cell& cell(const IntPoint& p) const { return cell(p.x,p.y); }
		//输入 “栅格” 栅格坐标做入口参数，返回“地图补丁”的状态
		inline AccessibilityState cellState(int x, int y) const ;
		inline AccessibilityState cellState(const IntPoint& p) const { return cellState(p.x,p.y); }	
		//设置地图的有效区域
		inline void setActiveArea(const PointSet&, bool patchCoords=false);
		//给有效区域(被激光扫过的区域)分配内存
		inline void allocActiveArea();
		
		//基本函数
		inline int getPatchSize() const {return m_patchMagnitude;}
		inline int getPatchMagnitude() const {return m_patchMagnitude;}
		const PointSet& getActiveArea() const {return m_activeArea; }		
	protected:
		//为一个地图补丁中的小栅格申请内存
		virtual Array2D<Cell> * createPatch() const;

		PointSet m_activeArea;  //存储地图中使用到的Cell的坐标
		int m_patchMagnitude;   //patch的大小等级  
		int m_patchSize;		//patch的实际大小 
};

//输入栅格地图大小，和地图patch的等级，初始化一个将装满地图补丁的二维数组，每一个元素都是一个patch
template <class Cell>
HierarchicalArray2D<Cell>::HierarchicalArray2D(int xsize, int ysize, int patchMagnitude) 
  :Array2D<std::shared_ptr< Array2D<Cell> > >::Array2D((xsize>>patchMagnitude), (ysize>>patchMagnitude))
{
	m_patchMagnitude = patchMagnitude;//地图补丁的大小等级
	m_patchSize = 1<<m_patchMagnitude;//每块地图补丁的边大小，而非每块地图补丁中的栅格数目
}

//调整存储 “地图补丁” 的二维数组的大小，也就是个数变化
template <class Cell>
void HierarchicalArray2D<Cell>::resize(int xmin, int ymin, int xmax, int ymax)
{
	//新地图补丁数组的尺寸
	int xsize=xmax-xmin;
	int ysize=ymax-ymin;
	//为新的“地图补丁”二维数组申请内存
	std::shared_ptr< Array2D<Cell> > ** newcells(new std::shared_ptr< Array2D<Cell> > *[xsize]);
	for (int x=0; x<xsize; x++)
	{
		newcells[x] = new std::shared_ptr< Array2D<Cell> >[ysize];
		for (int y=0; y<ysize; y++)
		{
			newcells[x][y] = std::shared_ptr< Array2D<Cell> >(0);//指向为NULL
		}
	}
	//xmin、ymin最小值为0，因为地图坐标没有负值的缘故
	int dx= xmin < 0 ? 0 : xmin;
	int dy= ymin < 0 ? 0 : ymin;
	//此方法可以合理的遍历内存，不越界也不做无用的访问，因为只取原来的值
	int Dx=xmax<this->m_xsize?xmax:this->m_xsize;
	int Dy=ymax<this->m_ysize?ymax:this->m_ysize;
	//把原来的地图中的栅格数据赋值到新的内存中，然后销毁原来的指针
	for (int x=dx; x<Dx; x++)
	{
		for (int y=dy; y<Dy; y++)
		{
			newcells[x-xmin][y-ymin]=this->m_cells[x][y];
		}
		delete [] this->m_cells[x];//销毁每一列，从小向大销毁
	}
	delete [] this->m_cells;
	this->m_cells=newcells;//新的二级指针指向老的二级指针
	this->m_xsize=xsize;   //新的存储“地图补丁”的二维数组大小
	this->m_ysize=ysize; 
}

//输入一个小栅格，寻找它所属的地图补丁大栅格patch的坐标
template <class Cell>
IntPoint HierarchicalArray2D<Cell>::patchIndexes(int x, int y) const
{
	if (x>=0 && y>=0)
		return IntPoint(x>>m_patchMagnitude, y>>m_patchMagnitude);
	return IntPoint(-1, -1);
}

//为一个地图补丁中的小栅格申请内存
template <class Cell>
Array2D<Cell>* HierarchicalArray2D<Cell>::createPatch() const
{
	return new Array2D<Cell>(1<<m_patchMagnitude, 1<<m_patchMagnitude);
}

//输入小栅格的栅格坐标做入口参数，判断该所属地图补丁是否已分配内存
template <class Cell>
bool HierarchicalArray2D<Cell>::isAllocated(int x, int y) const
{
	IntPoint c=patchIndexes(x,y);//转换到大栅格地图补丁patch的坐标
	std::shared_ptr< Array2D<Cell> >& ptr=this->m_cells[c.x][c.y];
	return (ptr != 0);
}

//输入小栅格的在地图中的栅格坐标，返回一个在地图补丁中的一个小栅格对象
template <class Cell>
Cell &HierarchicalArray2D<Cell>::cell(int x, int y)
{
    IntPoint c = patchIndexes(x, y);
    assert(this->isInside(c.x, c.y)); //“地图补丁” 的 “栅格坐标” 是否在整个存储“地图补丁”的二维数组中
    if (!this->m_cells[c.x][c.y])     //若指向为空，说明还未申请则为其申请内存空间
    {
        Array2D<Cell> *patch = createPatch();                            //为每一块地图补丁patch申请内存
        this->m_cells[c.x][c.y] = std::shared_ptr<Array2D<Cell>>(patch); //该地图补丁指向了此时的patch指针，不再为空了
    }
    std::shared_ptr<Array2D<Cell>> &ptr = this->m_cells[c.x][c.y];
    return (*ptr).cell(IntPoint(x - (c.x << m_patchMagnitude), y - (c.y << m_patchMagnitude)));
}

template <class Cell>
const Cell &HierarchicalArray2D<Cell>::cell(int x, int y) const
{
    assert(isAllocated(x, y));
    IntPoint c = patchIndexes(x, y);
    const std::shared_ptr<Array2D<Cell>> &ptr = this->m_cells[c.x][c.y];
    return (*ptr).cell(IntPoint(x - (c.x << m_patchMagnitude), y - (c.y << m_patchMagnitude)));
}

//输入 “栅格” 栅格坐标做入口参数，返回“地图补丁”的状态，Outside=0x0, Inside=0x1, Allocated=0x2
template <class Cell>
AccessibilityState  HierarchicalArray2D<Cell>::cellState(int x, int y) const 
{
	if (this->isInside(patchIndexes(x,y))) //首先输入的地图中的栅格坐标，是否存在
	{
		if(isAllocated(x,y))               //是否已经分配内存
			return (AccessibilityState)((int)Inside|(int)Allocated);
		else
			return Inside;
	}
	return Outside;
}

/*
设置地图的有效区域
注意：这里只是把点存储到对应的队列中，并没有进行内存的分配，真正的内存分配在后面的allocActiveArea()函数
*/
template <class Cell>
void HierarchicalArray2D<Cell>::setActiveArea(const typename HierarchicalArray2D<Cell>::PointSet& aa, bool patchCoords)
{
	m_activeArea.clear();
	for (PointSet::const_iterator it= aa.begin(); it!=aa.end(); ++it) 
	{
		IntPoint p;
		if (patchCoords)//是否是patch的坐标
			p=*it;
		else
			p=patchIndexes(*it);
		m_activeArea.insert(p);//将一个个地图补丁大栅格坐标插入m_activeArea中，元素唯一，自动排序
	}
}

/*
给有效区域(被激光扫过的区域)分配内存
给setActiveArea()函数插入的patch进行内存的分配。
*/
template <class Cell>
void HierarchicalArray2D<Cell>::allocActiveArea()
{
	for (PointSet::const_iterator it= m_activeArea.begin(); it!=m_activeArea.end(); ++it)//遍历地图补丁大栅格
	{
		const std::shared_ptr< Array2D<Cell> >& ptr=this->m_cells[it->x][it->y];
		Array2D<Cell>* patch = 0;
        //如果对应的active没有被分配内存 则进行内存分配
		//一个patch的内存没有分配的话，是没有内存存储栅格的，也就是没有array2D的对象的二级指针
		if (!ptr)
		{
			patch=createPatch();//分配指向小栅格的内存
		} 
		else
		{	
			patch= new Array2D<Cell>(*ptr);
		}
		this->m_cells[it->x][it->y]=std::shared_ptr< Array2D<Cell> >(patch);//每一个地铺补丁大栅格都指向了一个分配了的地图补丁，存储每一个小栅格
	}
}

};

#endif