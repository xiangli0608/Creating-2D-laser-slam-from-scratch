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

#ifndef __OccGridMapUtil_h_
#define __OccGridMapUtil_h_

#include <cmath>

#include "../scan/DataPointContainer.h"
#include "../util/UtilFunctions.h"

namespace hectorslam
{

/**
*
* @tparam ConcreteOccGridMap     -----> 就是MapContainer中 的  GridMap
* @tparam ConcreteCacheMethod    -----> 就是 GridMapCacheArray
*/
template <typename ConcreteOccGridMap, typename ConcreteCacheMethod>
class OccGridMapUtil
{
public:
    OccGridMapUtil(const ConcreteOccGridMap *gridMap)
        : concreteGridMap(gridMap), size(0)
    {
        mapObstacleThreshold = gridMap->getObstacleThreshold();
        cacheMethod.setMapSize(gridMap->getMapDimensions());
    }

    ~OccGridMapUtil()
    {
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** map中的Pose转换到世界系下的Pose  --- 仿射变换**/
    inline Eigen::Vector3f getWorldCoordsPose(const Eigen::Vector3f &mapPose) const { return concreteGridMap->getWorldCoordsPose(mapPose); };
    /** 世界系下的Pose转换到地图系上的Pose **/
    inline Eigen::Vector3f getMapCoordsPose(const Eigen::Vector3f &worldPose) const { return concreteGridMap->getMapCoordsPose(worldPose); };
    /** 地图点坐标转换到世界系坐标 **/
    inline Eigen::Vector2f getWorldCoordsPoint(const Eigen::Vector2f &mapPoint) const { return concreteGridMap->getWorldCoords(mapPoint); };

    /**
     * 使用当前pose投影dataPoints到地图，计算出 H 矩阵 b列向量， 理论部分详见Hector论文： 《A Flexible and Scalable SLAM System with Full 3D Motion Estimation》.
     * @param pose    地图系上的位姿
     * @param dataPoints  已转换为地图尺度的激光点数据
     * @param H   需要计算的 H矩阵
     * @param dTr  需要计算的 g列向量
     */
    void getCompleteHessianDerivs(const Eigen::Vector3f &pose,
                                  const DataContainer &dataPoints,
                                  Eigen::Matrix3f &H,
                                  Eigen::Vector3f &dTr)
    {
        int size = dataPoints.getSize();

        // 获取变换矩阵
        Eigen::Affine2f transform(getTransformForState(pose));

        float sinRot = sin(pose[2]);
        float cosRot = cos(pose[2]);

        H = Eigen::Matrix3f::Zero();
        dTr = Eigen::Vector3f::Zero();

        // 按照公式计算H、b
        for (int i = 0; i < size; ++i)
        {
            // 地图尺度下的激光坐标系下的激光点坐标
            const Eigen::Vector2f &currPoint(dataPoints.getVecEntry(i));

            // 将激光点坐标转换到地图系上, 通过双线性插值计算栅格概率
            // transformedPointData[0]--通过插值得到的栅格值
            // transformedPointData[1]--栅格值x方向的梯度
            // transformedPointData[2]--栅格值y方向的梯度
            Eigen::Vector3f transformedPointData(interpMapValueWithDerivatives(transform * currPoint));

            // 目标函数f(x)  (1-M(Pm))
            float funVal = 1.0f - transformedPointData[0];

            // 计算g列向量的 x 与 y 方向的值
            dTr[0] += transformedPointData[1] * funVal;
            dTr[1] += transformedPointData[2] * funVal;

            // 根据公式计算
            float rotDeriv = ((-sinRot * currPoint.x() - cosRot * currPoint.y()) * transformedPointData[1] +
                              (cosRot * currPoint.x() - sinRot * currPoint.y()) * transformedPointData[2]);
            // 计算g列向量的 角度 的值
            dTr[2] += rotDeriv * funVal;

            // 计算 hessian 矩阵
            H(0, 0) += util::sqr(transformedPointData[1]);
            H(1, 1) += util::sqr(transformedPointData[2]);
            H(2, 2) += util::sqr(rotDeriv);

            H(0, 1) += transformedPointData[1] * transformedPointData[2];
            H(0, 2) += transformedPointData[1] * rotDeriv;
            H(1, 2) += transformedPointData[2] * rotDeriv;
        }

        // H是对称矩阵，只算上三角就行， 减少计算量。
        H(1, 0) = H(0, 1);
        H(2, 0) = H(0, 2);
        H(2, 1) = H(1, 2);
    }

    /**
     * 双线性插值计算网格中任一点的得分（占据概率）以及该点处的梯度
     * @param coords  激光点地图坐标
     * @return ret(0) 是网格值 ， ret(1) 是栅格值在x方向的导数 ， ret(2)是栅格值在y方向的导数
     */
    Eigen::Vector3f interpMapValueWithDerivatives(const Eigen::Vector2f &coords)
    {
        // 检查coords坐标是否是在地图坐标范围内
        if (concreteGridMap->pointOutOfMapBounds(coords))
        {
            return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        }

        // 对坐标进行向下取整，即得到坐标(x0,y0)
        Eigen::Vector2i indMin(coords.cast<int>());

        // 得到双线性插值的因子
        // | x |   | x0 |   | x-x0 |
        // |   | - |    | = |      |
        // | y |   | y0 |   | y-y0 |
        Eigen::Vector2f factors(coords - indMin.cast<float>());

        // 获得地图的X方向最大边界
        int sizeX = concreteGridMap->getSizeX();

        // 计算(x0, y0)点的网格索引值
        int index = indMin[1] * sizeX + indMin[0]; 

        // 下边这取4个点的栅格值，感觉就是导致hector大地图后计算变慢的原因

        /** 首先判断cache中该地图点在本次scan中是否被访问过，若有则直接取值；没有则立马计算概率值并更新到cache **/
        /** 这个cache的作用是，避免单次scan重复访问同一网格时带来的重复概率计算。地图更新后，网格logocc改变，cache数据就会无效。 **/
        /** 但是这种方式内存开销太大..相当于同时维护两份地图，使用 hash map 是不是会更合适些 **/
        if (!cacheMethod.containsCachedData(index, intensities[0]))
        {
            intensities[0] = getUnfilteredGridPoint(index); // 得到M(P00),P00(x0,y0)
            cacheMethod.cacheData(index, intensities[0]);
        }

        ++index;
        if (!cacheMethod.containsCachedData(index, intensities[1]))
        {
            intensities[1] = getUnfilteredGridPoint(index);
            cacheMethod.cacheData(index, intensities[1]);
        }

        index += sizeX - 1;
        if (!cacheMethod.containsCachedData(index, intensities[2]))
        {
            intensities[2] = getUnfilteredGridPoint(index);
            cacheMethod.cacheData(index, intensities[2]);
        }

        ++index;
        if (!cacheMethod.containsCachedData(index, intensities[3]))
        {
            intensities[3] = getUnfilteredGridPoint(index);
            cacheMethod.cacheData(index, intensities[3]);
        }

        float dx1 = intensities[0] - intensities[1]; // 求得(M(P00) - M(P10))的值
        float dx2 = intensities[2] - intensities[3]; // 求得(M(P01) - M(P11))的值

        float dy1 = intensities[0] - intensities[2]; // 求得(M(P00) - M(P01))的值
        float dy2 = intensities[1] - intensities[3]; // 求得(M(P10) - M(P11))的值

        // 得到双线性插值的因子,注意x0+1=x1,y0+1=y1,则
        //     | x-x0 |   | 1-x+x0 |   | x1-x |
        // 1 - |      | = |        | = |      |
        //     | y-y0 |   | 1-y+y0 |   | y1-x |
        float xFacInv = (1.0f - factors[0]); // 求得(x1-x)的值
        float yFacInv = (1.0f - factors[1]); // 求得(y1-y)的值

        //         y-y0 | x-x0            x1-x        |    y1-y | x-x0            x1-x        |
        // M(Pm) = ------|------ M(P11) + ------ M(P01)| + ------|------ M(P10) + ------ M(P00)|
        //         y1-y0| x1-x0           x1-x0       |    y1-y0| x1-x0           x1-x0       |
        // 注意：此处y1-y0=x1-x0=1,那么对应函数返回值，可以写成
        // M(Pm) = (M(P00) * (x1-x) + M(P10) * (x-x0)) * (y1-y) + (M(P01) * (x1-x) + M(P11) * (x-x0)) * (y-y0)

        // d(M(Pm))     y-y0 |                |    y1-y |                |
        // ---------- = ------| M(P11) - M(P01)| + ------| M(P10) - M(P00)|
        //    dx        y1-y0|                |    y1-y0|                |
        // 同理，化简可得 d(M(Pm))/dx = -((M(P00) - M(P10)) * (y1-y) + (M(P01) - M(P11)) * (y-y0))
        // 同样地，也有   d(M(Pm))/dy = -((M(P00) - M(P01)) * (x1-x) + (M(P10) - M(P11)) * (x-x0))

        // 计算网格值，计算梯度 --- 原版这里的dx、dy的计算有错误，已经改过来了
        return Eigen::Vector3f(
            ((intensities[0] * xFacInv + intensities[1] * factors[0]) * (yFacInv)) +
                ((intensities[2] * xFacInv + intensities[3] * factors[0]) * (factors[1])),
            -((dx1 * yFacInv) + (dx2 * factors[1])),
            -((dy1 * xFacInv) + (dy2 * factors[0]))
            // -((dx1 * xFacInv) + (dx2 * factors[0])), // 应该为： -((dx1 * yFacInv) + (dx2 * factors[1]))
            // -((dy1 * yFacInv) + (dy2 * factors[1]))  // 应该为： -((dy1 * xFacInv) + (dy2 * factors[0]))
        );
    }

    float getUnfilteredGridPoint(int index) const
    {
        return (concreteGridMap->getGridProbabilityMap(index));
    }

    float getUnfilteredGridPoint(Eigen::Vector2i &gridCoords) const
    {
        return (concreteGridMap->getGridProbabilityMap(gridCoords.x(), gridCoords.y()));
    }

    /**
     * 通过计算出的 pose 和激光数据，估计出协方差矩阵的sampling的方式（个人猜测是这种方式），见论文中的参考文献 [20]
     * 论文中这样说的：Use a sampling based covariance estimate,sampling different pose estimates close to
     * scan matching pose and constructing a covariance from those.
     * 具体实现未研究，感兴趣的可看看论文中公式（15）对应的那一段。
     * @param mapPose     估计出的地图系下的pose
     * @param dataPoints  地图尺度下的激光数据
     * @return
     */
    Eigen::Matrix3f getCovarianceForPose(const Eigen::Vector3f &mapPose, const DataContainer &dataPoints)
    {

        float deltaTransX = 1.5f;
        float deltaTransY = 1.5f;
        float deltaAng = 0.05f;

        float x = mapPose[0];
        float y = mapPose[1];
        float ang = mapPose[2];

        Eigen::Matrix<float, 3, 7> sigmaPoints;

        sigmaPoints.block<3, 1>(0, 0) = Eigen::Vector3f(x + deltaTransX, y, ang);
        sigmaPoints.block<3, 1>(0, 1) = Eigen::Vector3f(x - deltaTransX, y, ang);
        sigmaPoints.block<3, 1>(0, 2) = Eigen::Vector3f(x, y + deltaTransY, ang);
        sigmaPoints.block<3, 1>(0, 3) = Eigen::Vector3f(x, y - deltaTransY, ang);
        sigmaPoints.block<3, 1>(0, 4) = Eigen::Vector3f(x, y, ang + deltaAng);
        sigmaPoints.block<3, 1>(0, 5) = Eigen::Vector3f(x, y, ang - deltaAng);
        sigmaPoints.block<3, 1>(0, 6) = mapPose;

        Eigen::Matrix<float, 7, 1> likelihoods;

        likelihoods[0] = getLikelihoodForState(Eigen::Vector3f(x + deltaTransX, y, ang), dataPoints);
        likelihoods[1] = getLikelihoodForState(Eigen::Vector3f(x - deltaTransX, y, ang), dataPoints);
        likelihoods[2] = getLikelihoodForState(Eigen::Vector3f(x, y + deltaTransY, ang), dataPoints);
        likelihoods[3] = getLikelihoodForState(Eigen::Vector3f(x, y - deltaTransY, ang), dataPoints);
        likelihoods[4] = getLikelihoodForState(Eigen::Vector3f(x, y, ang + deltaAng), dataPoints);
        likelihoods[5] = getLikelihoodForState(Eigen::Vector3f(x, y, ang - deltaAng), dataPoints);
        likelihoods[6] = getLikelihoodForState(Eigen::Vector3f(x, y, ang), dataPoints);

        float invLhNormalizer = 1 / likelihoods.sum();

        std::cout << "\n lhs:\n"
                  << likelihoods;

        Eigen::Vector3f mean(Eigen::Vector3f::Zero());

        for (int i = 0; i < 7; ++i)
        {
            mean += (sigmaPoints.block<3, 1>(0, i) * likelihoods[i]);
        }

        mean *= invLhNormalizer;

        Eigen::Matrix3f covMatrixMap(Eigen::Matrix3f::Zero());

        for (int i = 0; i < 7; ++i)
        {
            Eigen::Vector3f sigPointMinusMean(sigmaPoints.block<3, 1>(0, i) - mean);
            covMatrixMap += (likelihoods[i] * invLhNormalizer) * (sigPointMinusMean * (sigPointMinusMean.transpose()));
        }

        return covMatrixMap;

        //covMatrix.cwise() * invLhNormalizer;
        //transform = getTransformForState(Eigen::Vector3f(x-deltaTrans, y, ang);
    }

    /**
     * 世界坐标系下的协方差矩阵 --- 两个坐标系变量上 角度尺度相同，但是位移x、y相差一个尺度。
     * 所以 x^theta y^theta 协方差相差尺度 ， theta^theta协方差不差尺度、其他协方差相差尺度的平方
     * @param covMatMap
     * @return
     */
    Eigen::Matrix3f getCovMatrixWorldCoords(const Eigen::Matrix3f &covMatMap)
    {

        //std::cout << "\nCovMap:\n" << covMatMap;

        Eigen::Matrix3f covMatWorld;

        float scaleTrans = concreteGridMap->getCellLength();
        float scaleTransSq = util::sqr(scaleTrans);

        covMatWorld(0, 0) = covMatMap(0, 0) * scaleTransSq;
        covMatWorld(1, 1) = covMatMap(1, 1) * scaleTransSq;

        covMatWorld(1, 0) = covMatMap(1, 0) * scaleTransSq;
        covMatWorld(0, 1) = covMatWorld(1, 0);

        covMatWorld(2, 0) = covMatMap(2, 0) * scaleTrans;
        covMatWorld(0, 2) = covMatWorld(2, 0);

        covMatWorld(2, 1) = covMatMap(2, 1) * scaleTrans;
        covMatWorld(1, 2) = covMatWorld(2, 1);

        covMatWorld(2, 2) = covMatMap(2, 2);

        return covMatWorld;
    }

    /**  下面三个函数为估计协方差矩阵的方式，具体没研究...看上面提到的论文吧 **/
    float getLikelihoodForState(const Eigen::Vector3f &state, const DataContainer &dataPoints)
    {
        float resid = getResidualForState(state, dataPoints);

        return getLikelihoodForResidual(resid, dataPoints.getSize());
    }

    float getLikelihoodForResidual(float residual, int numDataPoints)
    {
        float numDataPointsA = static_cast<int>(numDataPoints);
        float sizef = static_cast<float>(numDataPointsA);

        return 1 - (residual / sizef);
    }

    float getResidualForState(const Eigen::Vector3f &state, const DataContainer &dataPoints)
    {
        int size = dataPoints.getSize();

        int stepSize = 1;
        float residual = 0.0f;

        Eigen::Affine2f transform(getTransformForState(state));

        for (int i = 0; i < size; i += stepSize)
        {

            float funval = 1.0f - interpMapValue(transform * dataPoints.getVecEntry(i));
            residual += funval;
        }

        return residual;
    }

    /**
     * 仅双线性插值计算网格内任意一点的值，详细注释见下一个函数interpMapValueWithDerivatives()
     */
    float interpMapValue(const Eigen::Vector2f &coords)
    {
        //check if coords are within map limits.
        if (concreteGridMap->pointOutOfMapBounds(coords))
        {
            return 0.0f;
        }

        //map coords are alway positive, floor them by casting to int
        Eigen::Vector2i indMin(coords.cast<int>());

        //get factors for bilinear interpolation
        Eigen::Vector2f factors(coords - indMin.cast<float>());

        int sizeX = concreteGridMap->getSizeX();

        int index = indMin[1] * sizeX + indMin[0];

        // get grid values for the 4 grid points surrounding the current coords. Check cached data first, if not contained
        // filter gridPoint with gaussian and store in cache.
        if (!cacheMethod.containsCachedData(index, intensities[0]))
        {
            intensities[0] = getUnfilteredGridPoint(index);
            cacheMethod.cacheData(index, intensities[0]);
        }

        ++index;

        if (!cacheMethod.containsCachedData(index, intensities[1]))
        {
            intensities[1] = getUnfilteredGridPoint(index);
            cacheMethod.cacheData(index, intensities[1]);
        }

        index += sizeX - 1;

        if (!cacheMethod.containsCachedData(index, intensities[2]))
        {
            intensities[2] = getUnfilteredGridPoint(index);
            cacheMethod.cacheData(index, intensities[2]);
        }

        ++index;

        if (!cacheMethod.containsCachedData(index, intensities[3]))
        {
            intensities[3] = getUnfilteredGridPoint(index);
            cacheMethod.cacheData(index, intensities[3]);
        }

        float xFacInv = (1.0f - factors[0]);
        float yFacInv = (1.0f - factors[1]);

        return ((intensities[0] * xFacInv + intensities[1] * factors[0]) * (yFacInv)) +
               ((intensities[2] * xFacInv + intensities[3] * factors[0]) * (factors[1]));
    }

    /** 计算变换矩阵  这里的仿射矩阵只有旋转和平移，因此实际为欧氏变换矩阵 T ，参考Eigen::Affine或者《十四讲》3.5节**/
    Eigen::Affine2f getTransformForState(const Eigen::Vector3f &transVector) const
    {
        return Eigen::Translation2f(transVector[0], transVector[1]) * Eigen::Rotation2Df(transVector[2]);
    }

    Eigen::Translation2f getTranslationForState(const Eigen::Vector3f &transVector) const
    {
        return Eigen::Translation2f(transVector[0], transVector[1]);
    }

    /** 重置cache，添加cache中的计数，使cache内的数据无效 **/
    void resetCachedData()
    {
        cacheMethod.resetCache();
    }

    /** 貌似没用额.. **/
    void resetSamplePoints()
    {
        samplePoints.clear();
    }

    const std::vector<Eigen::Vector3f> &getSamplePoints() const
    {
        return samplePoints;
    }

protected:
    Eigen::Vector4f intensities; /// 记录附近的四个格点的占据概率值

    ConcreteCacheMethod cacheMethod; /// 对应 GridMapCacheArray

    const ConcreteOccGridMap *concreteGridMap; /// 对应 GridMap ---> OccGridMapBase

    std::vector<Eigen::Vector3f> samplePoints; // 貌似没用

    int size; // 未用?

    float mapObstacleThreshold; //// 作用是啥??????
};
} // namespace hectorslam

#endif
