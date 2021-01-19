#ifndef GRIDLINETRAVERSAL_H
#define GRIDLINETRAVERSAL_H

#include <cstdlib>
#include <vector>
#include "lesson4/gmapping/utils/point.h"

namespace gmapping
{

/*
这个类主要功能是用bresemham算法来求得两个点之间的连线通过的一些点
*/

typedef struct
{
    int num_points;
    std::vector<IntPoint> points;
} GridLineTraversalLine;

struct GridLineTraversal
{
    inline static void gridLine(IntPoint start, IntPoint end, GridLineTraversalLine *line);
    inline static void gridLineCore(IntPoint start, IntPoint end, GridLineTraversalLine *line);
};

void GridLineTraversal::gridLineCore(IntPoint start, IntPoint end, GridLineTraversalLine *line)
{
    int dx, dy;             // 横纵坐标间距
    int incr1, incr2;       // P_m增量
    int d;                  // P_m
    int x, y, xend, yend;   // 直线增长的首末端点坐标
    int xdirflag, ydirflag; // 横纵坐标增长方向
    int cnt = 0;            // 直线过点的点的序号

    dx = abs(end.x - start.x);
    dy = abs(end.y - start.y);

    // 斜率绝对值小于等于1的情况，优先增加x
    if (dy <= dx)
    {
        d = 2 * dy - dx;       // 初始点P_m0值
        incr1 = 2 * dy;        // 情况（1）
        incr2 = 2 * (dy - dx); // 情况（2）

        // 将增长起点设置为横坐标小的点处，将 x的增长方向 设置为 向右侧增长
        if (start.x > end.x)
        {
            // 起点横坐标比终点横坐标大，ydirflag = -1（负号可以理解为增长方向与直线始终点方向相反）
            x = end.x; 
            y = end.y;
            ydirflag = (-1);
            xend = start.x; // 设置增长终点横坐标
        }
        else
        {
            x = start.x;
            y = start.y;
            ydirflag = 1;
            xend = end.x;
        }

        //加入起点坐标
        line->points.push_back(IntPoint(x, y));
        // line->points[cnt].x = x;
        // line->points[cnt].y = y;
        cnt++;

        // 向 右上 方向增长
        if (((end.y - start.y) * ydirflag) > 0)
        {
            // start.y > end.y
            while (x < xend)
            {
                x++;
                if (d < 0)
                {
                    d += incr1;
                }
                else
                {
                    y++;
                    d += incr2; // 纵坐标向正方向增长
                }
                line->points.push_back(IntPoint(x, y));
                // line->points[cnt].x = x;
                // line->points[cnt].y = y;
                cnt++;
            }
        }
        // 向 右下 方向增长
        else
        {
            while (x < xend)
            {
                x++;
                if (d < 0)
                {
                    d += incr1;
                }
                else
                {
                    y--;
                    d += incr2; // 纵坐标向负方向增长
                }
                line->points.push_back(IntPoint(x, y));
                // line->points[cnt].x = x;
                // line->points[cnt].y = y;
                cnt++;
            }
        }
    }
    // 斜率绝对值大于1的情况，优先增加y
    else
    {
        // dy > dx，当斜率k的绝对值|k|>1时，在y方向进行单位步进
        d = 2 * dx - dy; 
        incr1 = 2 * dx;  
        incr2 = 2 * (dx - dy);

        // 将增长起点设置为纵坐标小的点处，将 y的增长方向 设置为 向上侧增长
        if (start.y > end.y)
        {
            y = end.y; // 取最小的纵坐标作为起点
            x = end.x;
            yend = start.y;
            xdirflag = (-1); 
        }
        else
        {
            y = start.y;
            x = start.x;
            yend = end.y;
            xdirflag = 1;
        }
        // 添加起点
        line->points.push_back(IntPoint(x, y));
        // line->points[cnt].x = x;
        // line->points[cnt].y = y;
        cnt++;
        // 向 上右 增长
        if (((end.x - start.x) * xdirflag) > 0)
        {
            while (y < yend)
            {
                y++;
                if (d < 0)
                {
                    d += incr1;
                }
                else
                {
                    x++;
                    d += incr2; // 横坐标向正方向增长
                }
                // 添加新的点
                line->points.push_back(IntPoint(x, y));
                // line->points[cnt].x = x;
                // line->points[cnt].y = y;
                cnt++;
            }
        }
        // 向 上左 增长
        else
        {
            while (y < yend)
            {
                y++;
                if (d < 0)
                {
                    d += incr1;
                }
                else
                {
                    x--;
                    d += incr2; //横坐标向负方向增长
                }
                line->points.push_back(IntPoint(x, y));
                // line->points[cnt].x = x;
                // line->points[cnt].y = y;
                // 记录添加所有点的数目
                cnt++;
            }
        }
    }
    line->num_points = cnt;
}

// 最终在外面被使用的bresenham画线算法
void GridLineTraversal::gridLine(IntPoint start, IntPoint end, GridLineTraversalLine *line)
{
    int i, j;
    int half;
    IntPoint v;
    gridLineCore(start, end, line);
    if (start.x != line->points[0].x ||
        start.y != line->points[0].y)
    {
        half = line->num_points / 2;
        for (i = 0, j = line->num_points - 1; i < half; i++, j--)
        {
            v = line->points[i];
            line->points[i] = line->points[j];
            line->points[j] = v;
        }
    }
}
}
; // namespace GMapping

#endif
