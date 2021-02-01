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

#ifndef _hectormaprepmultimap_h__
#define _hectormaprepmultimap_h__

#include "MapRepresentationInterface.h"
#include "MapProcContainer.h"

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../matcher/ScanMatcher.h"

namespace hectorslam
{

/**
 * 金字塔（多分辨率）地图对象。
 */
class MapRepMultiMap : public MapRepresentationInterface
{

public:
    /**
     * 构建金字塔地图，第一层地图格子代表的物理尺寸最小，格子数最多，地图精度高；层数越高，格子代表的物理尺寸越大，格子数越少，精度越低。
     * @param mapResolution 首层地图分辨率，每个格子代表的物理尺寸边长
     * @param mapSizeX
     * @param mapSizeY
     * @param numDepth
     * @param startCoords
     */
    MapRepMultiMap(float mapResolution, int mapSizeX, int mapSizeY,
                   unsigned int numDepth,
                   const Eigen::Vector2f &startCoords)
    {
        Eigen::Vector2i resolution(mapSizeX, mapSizeY); // 第一层地图大小

        float totalMapSizeX = mapResolution * static_cast<float>(mapSizeX); // 实际物理尺寸范围
        float mid_offset_x = totalMapSizeX * startCoords.x();              

        float totalMapSizeY = mapResolution * static_cast<float>(mapSizeY);
        float mid_offset_y = totalMapSizeY * startCoords.y();

        for (unsigned int i = 0; i < numDepth; ++i)
        {
            std::cout << "HectorSM map lvl " << i << ": cellLength: " << mapResolution << " res x:" << resolution.x() << " res y: " << resolution.y() << "\n";

            /** 创建网格地图 **/
            GridMap *gridMap = new hectorslam::GridMap(mapResolution, resolution, Eigen::Vector2f(mid_offset_x, mid_offset_y));
            /** 处理上面网格地图的一些工具 **/
            OccGridMapUtilConfig<GridMap> *gridMapUtil = new OccGridMapUtilConfig<GridMap>(gridMap);
            /** 匹配工具 **/
            ScanMatcher<OccGridMapUtilConfig<GridMap>> *scanMatcher = new hectorslam::ScanMatcher<OccGridMapUtilConfig<GridMap>>();

            /** 上述三个内容统一放在MapProContainer容器中 **/
            mapContainer.push_back(MapProcContainer(gridMap, gridMapUtil, scanMatcher));

            resolution /= 2;       // 地图格子行、列格子数减半
            mapResolution *= 2.0f; // 地图精度减半
        }

        // 输入的激光数据即对应首层地图,第一层的激光数据无需记录到dataContainers中，因此数据容器大小 = 金字塔层数 - 1 = 地图容器大小 - 1 。
        // dataContainers[i]对应mapContainer[i+1], 输入的数据dataContainer 对应 mapContainer[0]
        dataContainers.resize(numDepth - 1);
    }

    virtual ~MapRepMultiMap()
    {
        unsigned int size = mapContainer.size();

        for (unsigned int i = 0; i < size; ++i)
        {
            mapContainer[i].cleanup();
        } /// 析构函数，需释放使用的动态内存
    }

    virtual void reset()
    {
        unsigned int size = mapContainer.size();

        for (unsigned int i = 0; i < size; ++i)
        {
            mapContainer[i].reset(); // 重置地图
        }
    }

    virtual float getScaleToMap() const { return mapContainer[0].getScaleToMap(); }; // 获取地图尺度 scale = 1.0 / map_resolution.

    virtual int getMapLevels() const { return mapContainer.size(); };                                      // 获取地图总层数
    virtual const GridMap &getGridMap(int mapLevel) const { return mapContainer[mapLevel].getGridMap(); }; // 获取指定层的地图

    virtual void addMapMutex(int i, MapLockerInterface *mapMutex)
    {
        mapContainer[i].addMapMutex(mapMutex); /// 给每层地图添加互斥锁
    }

    MapLockerInterface *getMapMutex(int i)
    {
        return mapContainer[i].getMapMutex(); /// 获取指定层的地图锁
    }

    virtual void onMapUpdated() /// 提示地图已经得到更新，cache中的临时数据无效
    {
        unsigned int size = mapContainer.size();

        for (unsigned int i = 0; i < size; ++i)
        {
            mapContainer[i].resetCachedData();
        }
    }

    /**
     * 地图匹配，通过多分辨率地图求解当前激光帧的pose。
     * @param beginEstimateWorld
     * @param dataContainer
     * @param covMatrix
     * @return
     */
    virtual Eigen::Vector3f matchData(const Eigen::Vector3f &beginEstimateWorld, const DataContainer &dataContainer, Eigen::Matrix3f &covMatrix)
    {
        size_t size = mapContainer.size();

        Eigen::Vector3f tmp(beginEstimateWorld);

        /// coarse to fine 的pose求精过程，i层的求解结果作为i-1层的求解初始值。
        for (int index = size - 1; index >= 0; --index)
        {
            //std::cout << " m " << i;
            if (index == 0)
            {
                tmp = (mapContainer[index].matchData(tmp, dataContainer, covMatrix, 5)); /// 输入数据dataContainer 对应 mapContainer[0]
            }
            else
            {
                // 根据地图层数对原始激光数据坐标进行缩小，得到对应图层尺寸的激光数据
                dataContainers[index - 1].setFrom(dataContainer, static_cast<float>(1.0 / pow(2.0, static_cast<double>(index))));
                tmp = (mapContainer[index].matchData(tmp, dataContainers[index - 1], covMatrix, 3));
                /// dataContainers[i]对应mapContainer[i+1]
            }
        }
        return tmp;
    }

    /**
     * 每层地图由当前scan与计算的位姿进行更新
     * @param dataContainer    第一层激光数据，其他层激光数据存在 dataContainers 中
     * @param robotPoseWorld   当前帧的世界系下位姿
     */
    virtual void updateByScan(const DataContainer &dataContainer, const Eigen::Vector3f &robotPoseWorld)
    {
        unsigned int size = mapContainer.size();

        for (unsigned int i = 0; i < size; ++i)
        {
            //std::cout << " u " <<  i;
            if (i == 0)
            {
                mapContainer[i].updateByScan(dataContainer, robotPoseWorld);
            }
            else
            {
                mapContainer[i].updateByScan(dataContainers[i - 1], robotPoseWorld);
            }
        }
        //std::cout << "\n";
    }

    /** 设置网格为free状态的概率 **/
    virtual void setUpdateFactorFree(float free_factor)
    {
        size_t size = mapContainer.size();

        for (unsigned int i = 0; i < size; ++i)
        {
            GridMap &map = mapContainer[i].getGridMap();
            map.setUpdateFreeFactor(free_factor);
        }
    }
    /** 设置网格为occupied状态的概率 **/
    virtual void setUpdateFactorOccupied(float occupied_factor)
    {
        size_t size = mapContainer.size();

        for (unsigned int i = 0; i < size; ++i)
        {
            GridMap &map = mapContainer[i].getGridMap();
            map.setUpdateOccupiedFactor(occupied_factor);
        }
    }

protected:
    std::vector<MapProcContainer> mapContainer; /// 不同图层的地图对象
    std::vector<DataContainer> dataContainers;  /// 不同图层对应的激光数据
};
} // namespace hectorslam

#endif
