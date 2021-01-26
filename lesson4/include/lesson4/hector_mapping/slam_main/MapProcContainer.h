//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef _hectormapproccontainer_h__
#define _hectormapproccontainer_h__

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../matcher/ScanMatcher.h"
#include "../util/MapLockerInterface.h"

class GridMap;
class ConcreteOccGridMapUtil;
class DataContainer;

namespace hectorslam
{

/**
 * 图层对象,包含多个图层及其对应处理工具、匹配方法、互斥锁等。
 */
class MapProcContainer
{
public:
    MapProcContainer(GridMap *gridMapIn, OccGridMapUtilConfig<GridMap> *gridMapUtilIn, ScanMatcher<OccGridMapUtilConfig<GridMap>> *scanMatcherIn)
        : gridMap(gridMapIn), gridMapUtil(gridMapUtilIn), scanMatcher(scanMatcherIn), mapMutex(0)
    {
    }

    virtual ~MapProcContainer()
    {
    }

    void cleanup()
    {
        delete gridMap;
        delete gridMapUtil;
        delete scanMatcher;

        if (mapMutex)
        {
            delete mapMutex;
        }
    }

    // 重置地图
    void reset()
    {
        gridMap->reset();
        gridMapUtil->resetCachedData();
    }

    // 重置cache中的数据
    void resetCachedData()
    {
        gridMapUtil->resetCachedData();
    }

    //// 获取本图层的尺度 scale = 1.0 / map_resolution.
    float getScaleToMap() const { return gridMap->getScaleToMap(); };

    const GridMap &getGridMap() const { return *gridMap; };
    GridMap &getGridMap() { return *gridMap; };

    // 添加地图锁
    void addMapMutex(MapLockerInterface *mapMutexIn)
    {
        if (mapMutex)
        {
            delete mapMutex;
        }

        mapMutex = mapMutexIn;
    }

    /// 获取当前地图的互斥锁
    MapLockerInterface *getMapMutex()
    {
        return mapMutex;
    }

    /**
   * 给定位姿初值，估计当前scan在当前图层中的位姿 ---- 位姿为世界系下的pose 、  dataContainer应与当前图层尺度匹配
   * @param beginEstimateWorld 世界系下的位姿
   * @param dataContainer       激光数据
   * @param covMatrix           scan-match的不确定性 -- 协方差矩阵
   * @param maxIterations       最大的迭代次数
   * @return
   */
    Eigen::Vector3f matchData(const Eigen::Vector3f &beginEstimateWorld, const DataContainer &dataContainer, Eigen::Matrix3f &covMatrix, int maxIterations)
    {
        return scanMatcher->matchData(beginEstimateWorld, *gridMapUtil, dataContainer, covMatrix, maxIterations);
    }

    /**
   * 有Scan数据更新地图
   * @param dataContainer   当前scan激光数据
   * @param robotPoseWorld  当前scan世界系下位姿
   */
    void updateByScan(const DataContainer &dataContainer, const Eigen::Vector3f &robotPoseWorld)
    {
        if (mapMutex)
        {
            mapMutex->lockMap();
        } //加锁，禁止其他线程竞争地图资源

        /// 更新地图
        gridMap->updateByScan(dataContainer, robotPoseWorld);

        if (mapMutex)
        {
            mapMutex->unlockMap();
        } //地图解锁
    }

    GridMap *gridMap;                                        // 地图网格
    OccGridMapUtilConfig<GridMap> *gridMapUtil;              // 网格工具、设置
    ScanMatcher<OccGridMapUtilConfig<GridMap>> *scanMatcher; // 地图对应匹配模块
    MapLockerInterface *mapMutex;                            // 地图锁，修改地图时，需要加锁避免多线程资源访问冲突。
};
}

#endif
