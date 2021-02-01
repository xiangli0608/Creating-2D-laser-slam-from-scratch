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

#ifndef _hectorslamprocessor_h__
#define _hectorslamprocessor_h__

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../matcher/ScanMatcher.h"
#include "../scan/DataPointContainer.h"

#include "../util/UtilFunctions.h"
#include "../util/MapLockerInterface.h"

#include "MapRepresentationInterface.h"
#include "MapRepMultiMap.h"

#include <float.h>

namespace hectorslam
{

/**
* Hector 系统处理核，是Hector Slam的接口。其包含三个部分的函数：
* 1. 关键的 update() 函数，用于处理新的激光数据并更新slam状态，reset()用于重置hector系统；
* 2. get**()函数，获取hector的系统状态，包括pose、map及其他参数；
* 3. set 函数，用于设置地图部分参数。
*/
class HectorSlamProcessor
{
public:
    HectorSlamProcessor(float mapResolution, int mapSizeX, int mapSizeY,
                        const Eigen::Vector2f &startCoords, int multi_res_size)
    {
        /* 构建初始地图 */
        mapRep = new MapRepMultiMap(mapResolution, mapSizeX, mapSizeY, multi_res_size, startCoords);

        this->reset();

        /* 设置进行地图更新的位姿变化阈值 **/
        this->setMapUpdateMinDistDiff(0.4f * 1.0f);
        this->setMapUpdateMinAngleDiff(0.13f * 1.0f);
    }

    ~HectorSlamProcessor()
    {
        delete mapRep;
    }

    /**
    * 对每一帧的激光数据进行处理
    * @param dataContainer  激光数据存储容器，坐标已转换成地图尺度，为地图中激光系的坐标
    * @param poseHintWorld  激光系在地图中的初始pose
    * @param map_without_matching 是否进行匹配
    */
    void update(const DataContainer &dataContainer, const Eigen::Vector3f &poseHintWorld, bool map_without_matching = false)
    {
        //std::cout << "\nph:\n" << poseHintWorld << "\n";

        /** 1. 位姿匹配 **/
        Eigen::Vector3f newPoseEstimateWorld;

        if (!map_without_matching)
        {
            // 进行 scan to map 的地方
            newPoseEstimateWorld = (mapRep->matchData(poseHintWorld, dataContainer, lastScanMatchCov));
        }
        else
        {
            newPoseEstimateWorld = poseHintWorld;
        }

        lastScanMatchPose = newPoseEstimateWorld;

        /** 2.地图更新 **/
        if (util::poseDifferenceLargerThan(newPoseEstimateWorld, lastMapUpdatePose, paramMinDistanceDiffForMapUpdate, paramMinAngleDiffForMapUpdate) || map_without_matching)
        { 
            // 仅在位姿变化大于阈值 或者 map_without_matching为真 的时候进行地图更新
            mapRep->updateByScan(dataContainer, newPoseEstimateWorld);
            mapRep->onMapUpdated();
            lastMapUpdatePose = newPoseEstimateWorld;
        }
    }

    /** slam系统重置 **/
    void reset()
    {
        lastMapUpdatePose = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
        lastScanMatchPose = Eigen::Vector3f::Zero();
        //重置地图
        mapRep->reset();
    }

    // 上一次匹配到的位姿
    const Eigen::Vector3f &getLastScanMatchPose() const { return lastScanMatchPose; };
    // 上一次匹配的协方差
    const Eigen::Matrix3f &getLastScanMatchCovariance() const { return lastScanMatchCov; };
    float getScaleToMap() const { return mapRep->getScaleToMap(); };

    // 获取地图层数
    int getMapLevels() const { return mapRep->getMapLevels(); };
    // 获取指定图层地图的常量引用
    const GridMap &getGridMap(int mapLevel = 0) const { return mapRep->getGridMap(mapLevel); };
    // 给指定图层添加互斥锁
    void addMapMutex(int i, MapLockerInterface *mapMutex) { mapRep->addMapMutex(i, mapMutex); };
    // 获取指定图层的锁
    MapLockerInterface *getMapMutex(int i) { return mapRep->getMapMutex(i); };

    // 设置概率、距离阈值参数
    void setUpdateFactorFree(float free_factor) { mapRep->setUpdateFactorFree(free_factor); };
    void setUpdateFactorOccupied(float occupied_factor) { mapRep->setUpdateFactorOccupied(occupied_factor); };
    void setMapUpdateMinDistDiff(float minDist) { paramMinDistanceDiffForMapUpdate = minDist; };
    void setMapUpdateMinAngleDiff(float angleChange) { paramMinAngleDiffForMapUpdate = angleChange; };

protected:
    MapRepresentationInterface *mapRep; // 地图接口对象--纯虚类

    Eigen::Vector3f lastMapUpdatePose;
    Eigen::Vector3f lastScanMatchPose;
    Eigen::Matrix3f lastScanMatchCov;

    float paramMinDistanceDiffForMapUpdate;
    float paramMinAngleDiffForMapUpdate;
};
}

#endif
