#ifndef ARRAY2D_H
#define ARRAY2D_H

#include <assert.h>
#include "../utils/point.h"
#include <iostream>

namespace gmapping {

//Array2D类 可以理解为二维的数组，数组的模板元素为Cell,在gmapping的使用过程中cell类的实例是PointAccumulator
/*
	Cell ** m_cells;       指向每个栅格的二级指针，可通过m_cells[i][j]访问每个栅格的内容
	int m_xsize, m_ysize;  栅格地图的大小
*/
//Array2D的一个对象就是一个二维数组（栅格地图）

template<class Cell, const bool debug=false> 
class Array2D
{
	public:
		//构造函数，参数栅格地图大小，为每个栅格申请内存
		Array2D(int xsize=0, int ysize=0);
		//重载的构造函数，根据Array2D对象作为参数，为每个栅格申请内存
		Array2D(const Array2D<Cell,debug> &);
		//析构函数，销毁整个地图每个栅格的内存
		~Array2D();

		//判断某个栅格是否在该栅格地图内
		inline bool isInside(int x, int y) const;
		inline bool isInside(const IntPoint& p) const { return isInside(p.x, p.y);}

		//输入栅格坐标（x，y）返回对应的栅格对象
		inline Cell& cell(int x, int y);
		inline Cell& cell(const IntPoint& p) {return cell(p.x,p.y);}
		inline const Cell& cell(int x, int y) const;
		inline const Cell& cell(const IntPoint& p) const {return cell(p.x,p.y);}

		//获取二维栅格的XY尺寸
		inline int getXSize() const {return m_xsize;}
		inline int getYSize() const {return m_ysize;}

		//每个地图指向所有栅格的二级指针的成员
		Cell ** m_cells;
	protected:
		//二维栅格的的尺寸
		int m_xsize, m_ysize;
};

//参数栅格地图大小，为整个栅格地图的每个栅格申请内存
template <class Cell, const bool debug>
Array2D<Cell,debug>::Array2D(int xsize, int ysize)
{
	m_xsize=xsize;
	m_ysize=ysize;
	if (m_xsize>0 && m_ysize>0)
	{
		m_cells=new Cell*[m_xsize];
		for (int i=0; i<m_xsize; i++)
			m_cells[i]=new Cell[m_ysize];
	}
	else
	{
		m_xsize=m_ysize=0;
		m_cells=0;
	}
}

//根据Array2D对象作为参数，为整个栅格地图的每个栅格申请内存
template <class Cell, const bool debug>
Array2D<Cell,debug>::Array2D(const Array2D<Cell,debug> & g)
{
	m_xsize=g.m_xsize;
	m_ysize=g.m_ysize;
	m_cells=new Cell*[m_xsize];
	for (int x=0; x<m_xsize; x++)
	{
		m_cells[x]=new Cell[m_ysize];
		for (int y=0; y<m_ysize; y++)
			m_cells[x][y]=g.m_cells[x][y];
	}
}

//销毁整个地图每个栅格的内存
template <class Cell, const bool debug>
Array2D<Cell,debug>::~Array2D()
{
  for (int i=0; i<m_xsize; i++)
  {
    delete [] m_cells[i];
    m_cells[i]=0;
  }
  delete [] m_cells;
  m_cells=0;//指向为空
}

//判断某个栅格是否在该栅格地图内
template <class Cell, const bool debug>
inline bool Array2D<Cell,debug>::isInside(int x, int y) const
{
	return x>=0 && y>=0 && x<m_xsize && y<m_ysize; 
}

//输入栅格坐标（x，y）返回cell对象
template <class Cell, const bool debug>
inline Cell& Array2D<Cell,debug>::cell(int x, int y)
{
	assert(isInside(x,y));
	return m_cells[x][y];
}

template <class Cell, const bool debug>
inline const Cell& Array2D<Cell,debug>::cell(int x, int y) const
{
	assert(isInside(x,y));
	return m_cells[x][y];
}

};

#endif