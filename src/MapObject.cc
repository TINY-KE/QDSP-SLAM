/**
* This file is part of https://github.com/JingwenWang95/DSP-SLAM
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include "MapObject.h"
#include "Converter.h"
#include <Eigen/Dense>

namespace ORB_SLAM2
{

int MapObject::nNextId = 0;

MapObject::MapObject(const Eigen::Matrix4f &T, const Eigen::Matrix<float, 64, 1> &vCode, KeyFrame *pRefKF, Map *pMap, int class_id) :
        mpRefKF(pRefKF), mpNewestKF(pRefKF), mnBALocalForKF(0), mnAssoRefID(0), mnFirstKFid(pRefKF->mnId),
        mnCorrectedByKF(0), mnCorrectedReference(0), mnLoopObjectForKF(0), mnBAGlobalForKF(0),
        w(1.), h(1.), l(1.), mbBad(false), mbDynamic(false), mpMap(pMap), nObs(0), mRenderId(-1)
{
    assert(false && "Debug: 用了到MapObject的默认构造函数，说明物体的位姿可以来自于初始化的时候!");

    // Transformation Matrix in Sim3
    Sim3Two = T;
    Sim3Tow = Sim3Two.inverse();

    // Decompose T into Rotation, translation and scale
    Rwo = T.topLeftCorner<3, 3>();
    // scale is fixed once the object is initialized
    scale = pow(Rwo.determinant(), 1./3.);
    invScale = 1. / scale;
    Rwo /= scale;
    two = T.topRightCorner<3, 1>();

    // Transformation Matrix in SE3
    SE3Two = Eigen::Matrix4f::Identity();
    SE3Two.topLeftCorner<3, 3>() = Rwo;
    SE3Two.topRightCorner<3, 1>() = two;
    SE3Tow = SE3Two.inverse();

    vShapeCode = vCode;
    velocity = Eigen::Vector3f::Zero();
    mnId = nNextId++;

    label = class_id;

    // ellipsoid-version
    mpEllipsold = new g2o::ellipsoid();
    // 椭球体相关flag
    mbValidEllipsoldFlag = false;
    mbValidDepthPointCloudFlag = false;
}

MapObject::MapObject(KeyFrame *pRefKF, Map *pMap, int class_id) :
        mpRefKF(pRefKF), mpNewestKF(pRefKF), mnBALocalForKF(0), mnAssoRefID(0), mnFirstKFid(pRefKF->mnId),
        mnCorrectedByKF(0), mnCorrectedReference(0), mnLoopObjectForKF(0), mnBAGlobalForKF(0),
        reconstructed(false), w(1.), h(1.), l(1.), mbBad(false), mbDynamic(false), mpMap(pMap), nObs(0), mRenderId(-1)
{
    mnId = nNextId++;
    scale = 1.;
    invScale = 1.;
    vShapeCode = Eigen::Matrix<float, 64, 1>::Zero();
    label = class_id;
    // ellipsoid-version
    mpEllipsold = new g2o::ellipsoid();
    // 椭球体相关flag
    mbValidEllipsoldFlag = false;
    mbValidDepthPointCloudFlag = false;
}

void MapObject::AddObjectObservation(KeyFrame *pKF, int idx)
{
    unique_lock<mutex> lock(mMutexObject);

    if(pKF->GetObjectDetections()[idx] == NULL){
        std::cerr << "[debug] 没有提取出椭球体，无法用于合法物体观测" << std::endl;
        std::exit(EXIT_FAILURE);  // 退出程序，返回非 0 状态（失败）
    }

    if(!mObservations.count(pKF))
        nObs++;
    mObservations[pKF]=idx;
    mpNewestKF = pKF;
}

std::map<KeyFrame*, size_t> MapObject::GetObservations()
{
    unique_lock<mutex> lock(mMutexObject);
    return mObservations;
}

void MapObject::EraseObservation(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexObject);
    if(mObservations.count(pKF))
    {
        nObs--;
        mObservations.erase(pKF);

        if(mpRefKF==pKF)
            mpRefKF=mObservations.begin()->first;

        if(mpNewestKF == pKF)
        {
            int mnLargestKFId = 0;
            KeyFrame *pNewestKF = static_cast<KeyFrame*>(nullptr);
            for(std::map<KeyFrame *, size_t>::iterator mit = mObservations.begin(), mend = mObservations.end(); mit != mend; mit++)
            {
                KeyFrame* plKF = mit->first;
                if (plKF->mnId > mnLargestKFId)
                {
                    mnLargestKFId = plKF->mnId;
                    pNewestKF = plKF;
                }
            }
            mpNewestKF = pNewestKF;
        }

    }
}

int MapObject::Observations()
{
    unique_lock<mutex> lock(mMutexObject);
    return nObs;
}

void MapObject::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock(mMutexObject);
        mbBad = true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapObjectMatch(mit->second);
    }
}

bool MapObject::isBad()
{
    unique_lock<mutex> lock(mMutexObject);
    return mbBad;
}

int MapObject::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexObject);
    if (mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapObject::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexObject);
    return (mObservations.count(pKF));
}

void MapObject::Replace(MapObject *pMO)
{
    if(pMO->mnId==this->mnId)
        return;

    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexObject);
        obs = mObservations;
        mObservations.clear();
        mbBad = true;
        mpReplaced = pMO;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMO->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapObjectMatch(mit->second, pMO);
            pMO->AddObjectObservation(pKF, mit->second);
        }
        else
        {
            pKF->EraseMapObjectMatch(mit->second);
        }
    }

    this->SetBadFlag();
}

KeyFrame* MapObject::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexObject);
    return mpRefKF;
}


void MapObject::SetObjectPoseSim3(const Eigen::Matrix4f &Two)
{
    unique_lock<mutex> lock(mMutexObject);
    Sim3Two = Two;
    Sim3Tow = Sim3Two.inverse();

    // Decompose T into Rotation, translation and scale
    Rwo = Two.topLeftCorner<3, 3>();
    // scale is fixed once the object is initialized
    // 缩放因子
    scale = pow(Rwo.determinant(), 1./3.);
    // 旋转矩阵除以scale，从而将物体缩小到单位球内
    invScale = 1. / scale;
    Rwo /= scale;
    two = Two.topRightCorner<3, 1>();

    // Transformation Matrix in SE3
    SE3Two = Eigen::Matrix4f::Identity();
    SE3Two.topLeftCorner<3, 3>() = Rwo;
    SE3Two.topRightCorner<3, 1>() = two;
    SE3Tow = SE3Two.inverse();
}

void MapObject::SetObjectPoseSim3(const Eigen::Matrix4f &Two, double s)
{
    unique_lock<mutex> lock(mMutexObject);
    Sim3Two = Two;
    Sim3Tow = Sim3Two.inverse();

    // Decompose T into Rotation, translation and scale
    Rwo = Two.topLeftCorner<3, 3>();
    // scale is fixed once the object is initialized
    // 缩放因子
    scale = s;
    // 旋转矩阵除以scale，从而将物体缩小到单位球内
    invScale = 1. / scale;
    Rwo /= scale;
    two = Two.topRightCorner<3, 1>();

    // Transformation Matrix in SE3
    SE3Two = Eigen::Matrix4f::Identity();
    SE3Two.topLeftCorner<3, 3>() = Rwo;
    SE3Two.topRightCorner<3, 1>() = two;
    SE3Tow = SE3Two.inverse();
}

void MapObject::SetObjectPoseSE3(const Eigen::Matrix4f &Two)
{
    unique_lock<mutex> lock(mMutexObject);
    SE3Two = Two;
    SE3Tow = SE3Two.inverse();
    Rwo = SE3Two.topLeftCorner<3, 3>();
    two = SE3Two.topRightCorner<3, 1>();
    Sim3Two.topLeftCorner<3, 3>() = Rwo * scale;
    Sim3Two.topRightCorner<3, 1>() = two;
    Sim3Tow = Sim3Two.inverse();
}

void MapObject::SetShapeCode(const Eigen::Matrix<float, 64, 1> &code)
{
    unique_lock<mutex> lock(mMutexObject);
    vShapeCode = code;
}

void MapObject::UpdateReconstruction(const Eigen::Matrix4f &T, const Eigen::Matrix<float, 64, 1> &vCode)
{
    SetObjectPoseSim3(T);
    SetShapeCode(vCode);
}

std::vector<MapPoint*> MapObject::GetMapPointsOnObject()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return vector<MapPoint *>(map_points.begin(), map_points.end());
}

void MapObject::RemoveOutliersSimple()
{
    // First pass, remove all the outliers marked by ORB-SLAM
    int n_pts = 0;
    Eigen::Vector3f x3D_mean = Eigen::Vector3f::Zero();
    for (auto pMP : GetMapPointsOnObject())
    {
        if (!pMP)
            continue;
        if (pMP->isBad())
            this->EraseMapPoint(pMP);
        else
        {
            x3D_mean += Converter::toVector3f(pMP->GetWorldPos());
            n_pts++;
        }
    }

    if (n_pts == 0)
    {
        this->SetBadFlag();
        return;
    }

    // Second pass, remove obvious outliers
    x3D_mean /= n_pts;
    for (auto pMP : GetMapPointsOnObject())
    {
        Eigen::Vector3f x3Dw = Converter::toVector3f(pMP->GetWorldPos());
        if ((x3Dw - x3D_mean).norm() > 4.0)
        {
            this->EraseMapPoint(pMP);
        }
    }
}


// 移除或标记掉在三维空间中偏离模型边界的离群点（Outliers）。
// 它使用了模型的边界范围（x/y/z 最小最大值）来判断点是否处于合理范围内，如果超出一定比例，则认为是离群点。
void MapObject::RemoveOutliersModel()
{
    // sanity check: too few number of vertices
    if (vertices.rows() <= 10)
        return;

    // .col(*).minCoeff();是使用 Eigen 库 中的函数，来求取一个矩阵中某一列的最小值。
    float xmin = vertices.col(0).minCoeff();
    float xmax = vertices.col(0).maxCoeff();
    float ymin = vertices.col(1).minCoeff();
    float ymax = vertices.col(1).maxCoeff();
    float zmin = vertices.col(2).minCoeff();
    float zmax = vertices.col(2).maxCoeff();

    w = (xmax - xmin) * scale;
    h = (ymax - ymin) * scale;
    l = (zmax - zmin) * scale;

    // 定义三个缩放因子 sx、sy 和 sz，用于判断点是否在边界之外。
    float sx = 1.2;
    float sy = 1.5;
    float sz = 1.2;

    auto mvpMapPoints = GetMapPointsOnObject();
    for (auto pMP : mvpMapPoints)
    {
        if (!pMP)
            continue;

        if (pMP->isBad())
        {
            this->EraseMapPoint(pMP);
        }
        else
        {
            // 判断是否为离群点：
            auto x3Dw = Converter::toVector3f(pMP->GetWorldPos());
            // 把点从世界坐标系转换到物体坐标系下
            auto x3Do = invScale * Rwo.inverse() * x3Dw - invScale * Rwo.inverse() * two;
            if (x3Do(0) > sx * xmax || x3Do(0) < sx * xmin ||
                x3Do(1) > sy * ymax || x3Do(1) < sy * ymin ||
                x3Do(2) > sz * zmax || x3Do(2) < sz * zmin)
            {
                pMP->SetOutlierFlag();
            }
        }
    }
}

// void MapObject::ComputeCuboidPCA(bool updatePose)
// {
//     // 1: 移除异常点
//     RemoveOutliersSimple();
//     auto mvpMapPoints = GetMapPointsOnObject();
//     int N = mvpMapPoints.size();

//     if (N == 0)
//     {
//         this->SetBadFlag();
//         return;
//     }

//     //  2: 计算点云的均值和协方差矩阵
//     Eigen::Vector3f x3D_mean = Eigen::Vector3f::Zero();
//     Eigen::MatrixXf Xpts = Eigen::MatrixXf::Zero(N, 3);
//     Eigen::MatrixXf Xpts_shifted = Eigen::MatrixXf::Zero(N, 3);
//     for (int i = 0; i < N; i++)
//     {
//         auto pMP = mvpMapPoints[i];
//         cv::Mat x3Dw = pMP->GetWorldPos();
//         Xpts(i, 0) = x3Dw.at<float>(0);
//         Xpts(i, 1) = x3Dw.at<float>(1);
//         Xpts(i, 2) = x3Dw.at<float>(2);
//         x3D_mean += Converter::toVector3f(pMP->GetWorldPos());
//     }

//     x3D_mean /= N;
//     for (int i = 0; i < N; i++)
//     {
//         Xpts_shifted.row(i) = Xpts.row(i) - x3D_mean.transpose();
//     }

//     // 3：执行主成分分析 (PCA)
//     auto covX = Xpts_shifted.transpose() * Xpts_shifted;
//     // cout << covX << endl;
//     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covX);

//     auto eigenvectors = eigensolver.eigenvectors();

//     // 4: 生成旋转矩阵
//     // Get rotation matrix, following ShapeNet definition
//     // x : right, y: up, z: back
//     Eigen::Matrix3f R;
//     // Assume the order of principal axis: y, x, -z
//     R.col(0) = eigenvectors.col(1);
//     R.col(1) = eigenvectors.col(0);
//     R.col(2) = -eigenvectors.col(2);

//     Eigen::Vector3f ground(0, 0, 1);  // Ground direction
//     R.col(1) = ground;                // Set Y axis to target direction
//     R.col(0) = R.col(2).cross(R.col(1));  // Recompute X axis as the cross product of Z and Y
//     R.col(0).normalize();               // Normalize X axis
//     R.col(2) = R.col(1).cross(R.col(0));  // Recompute Z axis as the cross product of Y and X
//     R.col(2).normalize();               // Normalize Z axis

//     // Check if det(R) = -1
//     if (R.determinant() < 0)
//         R.col(0) = -R.col(0);

//     // Check if y direction is pointing upward by comparing its angle between camera
//     auto neg_y = Eigen::Vector3f(0.f, -1.f, 0.f);
//     if (neg_y.dot(R.col(1)) < 0)
//     {
//         R.col(0) = -R.col(0);
//         R.col(1) = -R.col(1);
//     }

//     // 5: 计算包围盒尺寸
//     int lo = int (0.05 * N);  // percentile threshold
//     int hi = int (0.95 * N);
//     auto Xpts_o = R.inverse() * Xpts.transpose(); // 3 x N
//     Eigen::VectorXf x, y, z;
//     x = Xpts_o.row(0);  // x corresponds to w
//     y = Xpts_o.row(1);  // y corresponds to h
//     z = Xpts_o.row(2);  // z corresponds to l
//     // Sort the vectors
//     std::sort(x.data(),x.data() + x.size());
//     std::sort(y.data(),y.data() + y.size());
//     std::sort(z.data(),z.data() + z.size());

//     // PCA box dims
//     w = (x(hi) - x(lo));
//     h = (y(hi) - y(lo));
//     l = (z(hi) - z(lo));
//     Eigen::Vector3f cuboid_centre_o((x(hi) + x(lo)) / 2., (y(hi) + y(lo)) / 2., (z(hi) + z(lo)) / 2.);
//     Eigen::Vector3f cuboid_centre_w = R * cuboid_centre_o;

//     //  6: 移除异常点
//     // // Remove outliers using computed PCA box
//     // int num_outliers = 0;
//     // float s = 1.2;
//     // for (auto pMP : mvpMapPoints)
//     // {
//     //     if (!pMP)
//     //         continue;

//     //     if (pMP->isBad())
//     //     {
//     //         this->EraseMapPoint(pMP);
//     //     }
//     //     else
//     //     {
//     //         auto x3Dw = Converter::toVector3f(pMP->GetWorldPos());
//     //         auto x3Do = R.inverse() * x3Dw - R.inverse() * cuboid_centre_w;
//     //         if (x3Do(0) > s * w / 2 || x3Do(0) < -s * w / 2 ||
//     //             x3Do(1) > s * h / 2 || x3Do(1) < -s * h / 2 ||
//     //             x3Do(2) > s * l / 2 || x3Do(2) < -s * l / 2)
//     //         {
//     //             pMP->SetOutlierFlag();
//     //             num_outliers++;
//     //         }
//     //     }
//     // }

//     // 7: 更新物体位姿
//     // Update object pose with pose computed by PCA, only for the very first few frames
//     if (updatePose)
//     {
//         Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
//         T.topLeftCorner(3, 3) = 0.40 * l * R;
//         // cout << R.determinant() << " " << endl;
//         // cout << pow(T.topLeftCorner(3, 3).determinant(), 1./3) << endl;
//         T.topRightCorner(3, 1) = cuboid_centre_w;
//         SetObjectPoseSim3(T);
//     }
// }

void MapObject::ComputeCuboidPCA_ellipsoid(bool updatePose)
{
    // 1: 移除异常点
    RemoveOutliersSimple();
    auto mvpMapPoints = GetMapPointsOnObject();
    int N = mvpMapPoints.size();

    if (N == 0)
    {
        this->SetBadFlag();
        return;
    }

    //  2: 计算点云的均值和协方差矩阵
    Eigen::Vector3f x3D_mean = Eigen::Vector3f::Zero();
    Eigen::MatrixXf Xpts = Eigen::MatrixXf::Zero(N, 3);
    Eigen::MatrixXf Xpts_shifted = Eigen::MatrixXf::Zero(N, 3);
    for (int i = 0; i < N; i++)
    {
        auto pMP = mvpMapPoints[i];
        cv::Mat x3Dw = pMP->GetWorldPos();
        Xpts(i, 0) = x3Dw.at<float>(0);
        Xpts(i, 1) = x3Dw.at<float>(1);
        Xpts(i, 2) = x3Dw.at<float>(2);
        x3D_mean += Converter::toVector3f(pMP->GetWorldPos());
    }

    x3D_mean /= N;
    for (int i = 0; i < N; i++)
    {
        Xpts_shifted.row(i) = Xpts.row(i) - x3D_mean.transpose();
    }

    // 3：执行主成分分析 (PCA)
    auto covX = Xpts_shifted.transpose() * Xpts_shifted;
    // cout << covX << endl;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covX);

    auto eigenvectors = eigensolver.eigenvectors();

    // 4: 生成旋转矩阵
    // Get rotation matrix, following ShapeNet definition
    // x : right, y: up, z: back
    Eigen::Matrix3f R;
    // Assume the order of principal axis: y, x, -z
    R.col(0) = eigenvectors.col(1);
    R.col(1) = eigenvectors.col(0);
    R.col(2) = -eigenvectors.col(2);

    // Check if det(R) = -1
    if (R.determinant() < 0)
        R.col(0) = -R.col(0);

    // Check if y direction is pointing upward by comparing its angle between camera
    auto neg_y = Eigen::Vector3f(0.f, -1.f, 0.f);
    if (neg_y.dot(R.col(1)) < 0)
    {
        R.col(0) = -R.col(0);
        R.col(1) = -R.col(1);
    }

    // 5: 计算包围盒尺寸
    int lo = int (0.05 * N);  // percentile threshold
    int hi = int (0.95 * N);
    auto Xpts_o = R.inverse() * Xpts.transpose(); // 3 x N
    Eigen::VectorXf x, y, z;
    x = Xpts_o.row(0);  // x corresponds to w
    y = Xpts_o.row(1);  // y corresponds to h
    z = Xpts_o.row(2);  // z corresponds to l
    // Sort the vectors
    std::sort(x.data(),x.data() + x.size());
    std::sort(y.data(),y.data() + y.size());
    std::sort(z.data(),z.data() + z.size());

    // PCA box dims
    w = (x(hi) - x(lo));
    h = (y(hi) - y(lo));
    l = (z(hi) - z(lo));
    Eigen::Vector3f cuboid_centre_o((x(hi) + x(lo)) / 2., (y(hi) + y(lo)) / 2., (z(hi) + z(lo)) / 2.);
    Eigen::Vector3f cuboid_centre_w = R * cuboid_centre_o;

    //  6: 移除异常点
    // Remove outliers using computed PCA box
    int num_outliers = 0;
    float s = 1.2;
    for (auto pMP : mvpMapPoints)
    {
        if (!pMP)
            continue;

        if (pMP->isBad())
        {
            this->EraseMapPoint(pMP);
        }
        else
        {
            auto x3Dw = Converter::toVector3f(pMP->GetWorldPos());
            auto x3Do = R.inverse() * x3Dw - R.inverse() * cuboid_centre_w;
            if (x3Do(0) > s * w / 2 || x3Do(0) < -s * w / 2 ||
                x3Do(1) > s * h / 2 || x3Do(1) < -s * h / 2 ||
                x3Do(2) > s * l / 2 || x3Do(2) < -s * l / 2)
            {
                pMP->SetOutlierFlag();
                num_outliers++;
            }
        }
    }

    // 7: 更新物体位姿
    // Update object pose with pose computed by PCA, only for the very first few frames
    if (updatePose)
    {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.topLeftCorner(3, 3) = 0.40 * l * R;
        // cout << R.determinant() << " " << endl;
        // cout << pow(T.topLeftCorner(3, 3).determinant(), 1./3) << endl;
        T.topRightCorner(3, 1) = cuboid_centre_w;
        SetObjectPoseSim3(T);

        
        Eigen::Vector3f tt = T.block<3,1>(0,3);  // 最后一列前3行
        Eigen::Matrix3f RR = T.block<3,3>(0,0);
        Eigen::Matrix3f Ron = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f(1,0,0)).matrix()
                * Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f(0,1,0)).matrix();
        RR = RR *  Ron.inverse();  
        Eigen::Vector3f euler = RR.eulerAngles(2, 1, 0);  // ZYX 顺序
        Eigen::Matrix<double, 9, 1> MinimalVector;  // 或 Eigen::VectorXd vec(9);
        MinimalVector << tt[0],tt[1],tt[2],    euler[0],euler[1],euler[2],    w,h,l;
        mpEllipsold->fromMinimalVector( MinimalVector );
    }
}

void MapObject::ComputeCuboidPCA_manhattan(bool updatePose)
{
    // 1: 移除异常点
    RemoveOutliersSimple();
    auto mvpMapPoints = GetMapPointsOnObject();
    int N = mvpMapPoints.size();

    if (N == 0)
    {
        this->SetBadFlag();
        return;
    }

    //  2: 计算点云的均值和协方差矩阵
    Eigen::Vector3f x3D_mean = Eigen::Vector3f::Zero();
    Eigen::MatrixXf Xpts = Eigen::MatrixXf::Zero(N, 3);
    Eigen::MatrixXf Xpts_shifted = Eigen::MatrixXf::Zero(N, 3);
    for (int i = 0; i < N; i++)
    {
        auto pMP = mvpMapPoints[i];
        cv::Mat x3Dw = pMP->GetWorldPos();
        Xpts(i, 0) = x3Dw.at<float>(0);
        Xpts(i, 1) = x3Dw.at<float>(1);
        Xpts(i, 2) = x3Dw.at<float>(2);
        x3D_mean += Converter::toVector3f(pMP->GetWorldPos());
    }

    x3D_mean /= N;
    for (int i = 0; i < N; i++)
    {
        Xpts_shifted.row(i) = Xpts.row(i) - x3D_mean.transpose();
    }



    // 4: 生成旋转矩阵
    // Get rotation matrix, following ShapeNet definition
    // x : right, y: up, z: back
    Eigen::Matrix3f R;

    // Eigen::Vector3f world_x(1, 0, 0);  // Ground direction
    // R.col(2) = world_x;                // Set Y axis to target direction
    // Eigen::Vector3f world_y(0, 1, 0);  // Ground direction
    // R.col(0) = world_y;                // Set Y axis to target direction
    // Eigen::Vector3f world_z(0, 0, 1);  // Ground direction
    // R.col(1) = world_z;                // Set Y axis to target direction
    if(label == 59)   //57是沙发  59是床
    {
        Eigen::Vector3f world_x(1, 0, 0);  // Ground direction
        Eigen::Vector3f world_y(0, 1, 0);  // Ground direction
        Eigen::Vector3f world_z(0, 0, 1);  // Ground direction

        // 床的背面（主朝向）与y轴相反
        // R.col(0) = world_x;                // Set Y axis to target direction
        // R.col(2) = -1*world_y;                // Set Y axis to target direction
        // R.col(1) = world_z;                // Set Y axis to target direction
            
        // 床的背面（主朝向）与x轴一致
        // R.col(0) = world_y;                // Set Y axis to target direction
        // R.col(2) = world_x;                // Set Y axis to target direction
        // R.col(1) = world_z;                // Set Y axis to target direction

        // 床的背面（主朝向）与y轴一致
        R.col(0) = -1*world_x; 
        R.col(2) = world_y;  
        R.col(1) = world_z; 
    }
    // else if(label == 57)   //57是沙发  59是床
    // {
    //     Eigen::Vector3f world_x(1, 0, 0);  // Ground direction
    //     Eigen::Vector3f world_y(0, 1, 0);  // Ground direction
    //     Eigen::Vector3f world_z(0, 0, 1);  // Ground direction
    //     R.col(0) = -1*world_y;                // Set Y axis to target direction
    //     R.col(2) = -1*world_x;                // Set Y axis to target direction
    //     R.col(1) = world_z;                // Set Y axis to target direction
    // }
    else  //其他物体与世界坐标系一致
    {
        Eigen::Vector3f world_x(1, 0, 0);  // Ground direction
        R.col(0) = -1*world_x;                // Set Y axis to target direction
        Eigen::Vector3f world_y(0, 1, 0);  // Ground direction
        R.col(2) = world_y;                // Set Y axis to target direction
        Eigen::Vector3f world_z(0, 0, 1);  // Ground direction
        R.col(1) = world_z;                // Set Y axis to target direction
    }
    // Check if det(R) = -1
    if (R.determinant() < 0)
        R.col(0) = -R.col(0);

    // Check if y direction is pointing upward by comparing its angle between camera
    auto neg_y = Eigen::Vector3f(0.f, -1.f, 0.f);
    if (neg_y.dot(R.col(1)) < 0)
    {
        R.col(0) = -R.col(0);
        R.col(1) = -R.col(1);
    }

    // 5: 计算包围盒尺寸
    int lo = int (0.05 * N);  // percentile threshold
    int hi = int (0.95 * N);
    auto Xpts_o = R.inverse() * Xpts.transpose(); // 3 x N
    Eigen::VectorXf x, y, z;
    x = Xpts_o.row(0);  // x corresponds to w
    y = Xpts_o.row(1);  // y corresponds to h
    z = Xpts_o.row(2);  // z corresponds to l
    // Sort the vectors
    std::sort(x.data(),x.data() + x.size());
    std::sort(y.data(),y.data() + y.size());
    std::sort(z.data(),z.data() + z.size());

    // PCA box dims
    w = (x(hi) - x(lo));
    h = (y(hi) - 0);
    l = (z(hi) - z(lo));
    Eigen::Vector3f cuboid_centre_o((x(hi) + x(lo)) / 2., (y(hi) + 0) / 2., (z(hi) + z(lo)) / 2.);
    Eigen::Vector3f cuboid_centre_w = R * cuboid_centre_o;

    //  6: 加上地面的点
    vector<MapPoint*> vpMP = mpMap->GetAllMapPoints();
    for (auto pMP : vpMP) {
        if (!pMP)
            continue;
        if (pMP->isBad())
            continue;

        cv::Mat pw_mat = pMP->GetWorldPos();
        Eigen::Vector3f pw(pw_mat.at<float>(0), pw_mat.at<float>(1), pw_mat.at<float>(2));
        auto po = R.inverse() * pw - cuboid_centre_w;

        if (po(0) > -1*w/2.0 && po(0) < w/2.0 && po(1) > -1*h/2.0 && po(1) < h/2.0 && po(2) > -1*l/2.0 && po(2) < l/2.0) {
            this->AddMapPoints(pMP);
        }
    }

    // 7:更新物体点云的包络框
    compute_corner(); 
    
    
    // 8:更新物体位姿
    // Update object pose with pose computed by PCA, only for the very first few frames
    if (updatePose)
    {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        // 计算w l h的立方根
        float s = sqrt(w*w + h*h + l*l);
        T.topLeftCorner(3, 3) = 0.5* s* R;
        // cout << R.determinant() << " " << endl;
        // cout << pow(T.topLeftCorner(3, 3).determinant(), 1./3) << endl;
        T.topRightCorner(3, 1) = cuboid_centre_w;
        SetObjectPoseSim3(T);
    }
}


void MapObject::ComputeCuboidPCA(bool updatePose)
{
    // 1: 移除异常点
    RemoveOutliersSimple();
    auto mvpMapPoints = GetMapPointsOnObject();
    int N = mvpMapPoints.size();

    if (N == 0)
    {
        this->SetBadFlag();
        return;
    }

    //  2: 计算点云的均值和协方差矩阵
    Eigen::Vector3f x3D_mean = Eigen::Vector3f::Zero();
    Eigen::MatrixXf Xpts = Eigen::MatrixXf::Zero(N, 3);
    Eigen::MatrixXf Xpts_shifted = Eigen::MatrixXf::Zero(N, 3);
    for (int i = 0; i < N; i++)
    {
        auto pMP = mvpMapPoints[i];
        cv::Mat x3Dw = pMP->GetWorldPos();
        Xpts(i, 0) = x3Dw.at<float>(0);
        Xpts(i, 1) = x3Dw.at<float>(1);
        Xpts(i, 2) = x3Dw.at<float>(2);
        x3D_mean += Converter::toVector3f(pMP->GetWorldPos());
    }

    x3D_mean /= N;
    for (int i = 0; i < N; i++)
    {
        Xpts_shifted.row(i) = Xpts.row(i) - x3D_mean.transpose();
    }

    // 3：执行主成分分析 (PCA)
    auto covX = Xpts_shifted.transpose() * Xpts_shifted;
    // cout << covX << endl;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covX);

    auto eigenvectors = eigensolver.eigenvectors();

    // 4: 生成旋转矩阵
    // Get rotation matrix, following ShapeNet definition
    // x : right, y: up, z: back
    Eigen::Matrix3f R;
    // Assume the order of principal axis: y, x, -z
    R.col(0) = eigenvectors.col(1);
    R.col(1) = eigenvectors.col(0);
    R.col(2) = -eigenvectors.col(2);

    // Check if det(R) = -1
    if (R.determinant() < 0)
        R.col(0) = -R.col(0);

    // Check if y direction is pointing upward by comparing its angle between camera
    auto neg_y = Eigen::Vector3f(0.f, -1.f, 0.f);
    if (neg_y.dot(R.col(1)) < 0)
    {
        R.col(0) = -R.col(0);
        R.col(1) = -R.col(1);
    }

    // 5: 计算包围盒尺寸
    int lo = int (0.05 * N);  // percentile threshold
    int hi = int (0.95 * N);
    auto Xpts_o = R.inverse() * Xpts.transpose(); // 3 x N
    Eigen::VectorXf x, y, z;
    x = Xpts_o.row(0);  // x corresponds to w
    y = Xpts_o.row(1);  // y corresponds to h
    z = Xpts_o.row(2);  // z corresponds to l
    // Sort the vectors
    std::sort(x.data(),x.data() + x.size());
    std::sort(y.data(),y.data() + y.size());
    std::sort(z.data(),z.data() + z.size());

    // PCA box dims
    w = (x(hi) - x(lo));
    h = (y(hi) - y(lo));
    l = (z(hi) - z(lo));
    Eigen::Vector3f cuboid_centre_o((x(hi) + x(lo)) / 2., (y(hi) + y(lo)) / 2., (z(hi) + z(lo)) / 2.);
    Eigen::Vector3f cuboid_centre_w = R * cuboid_centre_o;

    //  6: 移除异常点
    // Remove outliers using computed PCA box
    int num_outliers = 0;
    float s = 1.2;
    for (auto pMP : mvpMapPoints)
    {
        if (!pMP)
            continue;

        if (pMP->isBad())
        {
            this->EraseMapPoint(pMP);
        }
        else
        {
            auto x3Dw = Converter::toVector3f(pMP->GetWorldPos());
            auto x3Do = R.inverse() * x3Dw - R.inverse() * cuboid_centre_w;
            if (x3Do(0) > s * w / 2 || x3Do(0) < -s * w / 2 ||
                x3Do(1) > s * h / 2 || x3Do(1) < -s * h / 2 ||
                x3Do(2) > s * l / 2 || x3Do(2) < -s * l / 2)
            {
                pMP->SetOutlierFlag();
                num_outliers++;
            }
        }
    }

    // 7: 更新物体位姿
    // Update object pose with pose computed by PCA, only for the very first few frames
    if (updatePose)
    {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.topLeftCorner(3, 3) = 0.40 * l * R;
        // cout << R.determinant() << " " << endl;
        // cout << pow(T.topLeftCorner(3, 3).determinant(), 1./3) << endl;
        T.topRightCorner(3, 1) = cuboid_centre_w;
        SetObjectPoseSim3(T);
    }
}


void MapObject::AddMapPoints(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexFeatures);
    map_points.insert(pMP);
}

void MapObject::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexFeatures);
    map_points.erase(pMP);
    pMP->SetBadFlag();
}

void MapObject::AddEllipsoidVertices(Eigen::Vector3f pMV)
{
    unique_lock<mutex> lock(mMutexEllipsoidVertices);
    ellipsoid_vertices.push_back(pMV);
}

void MapObject::EraseEllipsoidVertices()
{
    unique_lock<mutex> lock(mMutexEllipsoidVertices);
    ellipsoid_vertices.clear();
}

std::vector<Eigen::Vector3f> MapObject::GetEllipsoidVertices()
{
    unique_lock<mutex> lock(mMutexEllipsoidVertices);
    // return vector<Eigen::Vector3f>(ellipsoid_vertices.begin(), ellipsoid_vertices.end());
    return ellipsoid_vertices;  // 自动调用拷贝构造函数
}

void MapObject::SetVelocity(const Eigen::Vector3f &v)
{
    unique_lock<mutex> lock(mMutexObject);
    velocity = v;
}

Eigen::Matrix4f MapObject::GetPoseSim3()
{
    unique_lock<mutex> lock(mMutexObject);
    return Sim3Two;
}

Eigen::Matrix4f MapObject::GetPoseSE3()
{
    unique_lock<mutex> lock(mMutexObject);
    return SE3Two;
}

Eigen::Matrix<float, 64, 1> MapObject::GetShapeCode()
{
    unique_lock<mutex> lock(mMutexObject);
    return vShapeCode;
}

int MapObject::GetRenderId()
{
    unique_lock<mutex> lock(mMutexObject);
    return mRenderId;
}

void MapObject::SetRenderId(int id)
{
    unique_lock<mutex> lock(mMutexObject);
    mRenderId = id;
}

void MapObject::SetDynamicFlag()
{
    unique_lock<mutex> lock(mMutexObject);
    mbDynamic = true;
}

bool MapObject::isDynamic()
{
    unique_lock<mutex> lock(mMutexObject);
    return mbDynamic;
}



void MapObject::compute_corner() {

        //     8------7
        //    /|     /|
        //   / |    / |
        //  5------6  |
        //  |  4---|--3
        //  | /    | /
        //  1------2
        // lenth ：corner_2[0] - corner_1[0]
        // width ：corner_2[1] - corner_3[1]
        // height：corner_2[2] - corner_6[2]
        
        float x_min_obj = (-0.5)*this->w;
        float x_max_obj = (0.5)*this->w;
        float y_min_obj = (-0.5)*this->h;
        float y_max_obj = (0.5)*this->h;
        float z_min_obj = (-0.5)*this->l;
        float z_max_obj = (0.5)*this->l;
  
        this->corner_1 = (Sim3Two * Eigen::Vector4f(x_min_obj, y_min_obj, z_min_obj, 1) ).head<3>();
        this->corner_2 = (Sim3Two * Eigen::Vector4f(x_max_obj, y_min_obj, z_min_obj, 1) ).head<3>();
        this->corner_3 = (Sim3Two * Eigen::Vector4f(x_max_obj, y_max_obj, z_min_obj, 1) ).head<3>();
        this->corner_4 = (Sim3Two * Eigen::Vector4f(x_min_obj, y_max_obj, z_min_obj, 1) ).head<3>();
        this->corner_5 = (Sim3Two * Eigen::Vector4f(x_min_obj, y_min_obj, z_max_obj, 1) ).head<3>();
        this->corner_6 = (Sim3Two * Eigen::Vector4f(x_max_obj, y_min_obj, z_max_obj, 1) ).head<3>();
        this->corner_7 = (Sim3Two * Eigen::Vector4f(x_max_obj, y_max_obj, z_max_obj, 1) ).head<3>();
        this->corner_8 = (Sim3Two * Eigen::Vector4f(x_min_obj, y_max_obj, z_max_obj, 1) ).head<3>();

}

void MapObject::SetEllipsoid(g2o::ellipsoid e){
    // 为ellipsoid赋值
    unique_lock<mutex> lock(mMutexObject);
    double MinEllipsoidSize = Config::Get<double>("EllipsoidExtraction.MinEllipsoidSize");
    if(e.scale(0) <= MinEllipsoidSize || e.scale(1) <= MinEllipsoidSize || e.scale(2) <= MinEllipsoidSize){
        std::cerr << "[debug] SetEllipsoid() 遇到 输入椭球体 无效, 种类："<< e.miLabel <<", scale:"<< e.scale.transpose() << endl;
        std::exit(EXIT_FAILURE);  // 或者：std::abort();
    } else {
        (*mpEllipsold) = e;
        mpEllipsold->setColor(Vector3d(128.0/255.0,0.0,128.0/255));
    }
}

void MapObject::SetPoseByEllipsoid(g2o::ellipsoid* e, double scale_manual)
{
    Eigen::Matrix4f Two;
    {
    if(mpEllipsold == NULL) {
        {
            // 这里遇到了一个死锁的问题
            unique_lock<mutex> lock(mMutexObject);
            mpEllipsold = new g2o::ellipsoid(*(e));
        }
    }
    // 这里遇到了一个死锁的问题
    unique_lock<mutex> lock(mMutexObject);

    mbValidEllipsoldFlag = true;
    cout << "[debug] MapObject::SetPoseByEllipsoid, Object_id = " << mnId << ", mpEllipsold->prob = " << mpEllipsold->prob << ", scale = " << mpEllipsold->scale.transpose() << std::endl;

    // SE3Quat pose;  // rigid body transformation, object in world coordinate
    // Vector3d scale; // a,b,c : half length of axis x,y,z

    // world -> object
    Two = Converter::toMatrix4f(e->pose);


    Vector3d& scale = e->scale;
    float s = scale.norm() * 2;

    // Rx(90)*Ry(-90) 原本x轴从物体正面朝外，z轴朝上；变化后，z轴从物体背面朝外，y轴朝上
    Eigen::Matrix3f Ron = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f(1,0,0)).matrix()
        * Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f(0,1,0)).matrix();
    Two.topLeftCorner(3, 3) = Two.topLeftCorner(3, 3) * Ron;

    cout << "[deub] SetPoseByEllipsoid: 椭球体对角线长度：" << s << "0.5倍对角线长度：" << 0.50 * s << endl;

    Two.topLeftCorner(3, 3) = 0.50 * s * scale_manual * Two.topLeftCorner(3, 3);


    // 如果 Two和SE3Two的z轴方向相反（允许一定误差），则将Two绕着y轴旋转180度
    Eigen::Vector3f TwoZAxis = Two.block<3, 1>(0, 2);      // Two 的 z 轴方向
    Eigen::Vector3f SE3TwoZAxis = SE3Two.block<3, 1>(0, 2); // SE3Two 的 z 轴方向
    TwoZAxis.normalize();
    SE3TwoZAxis.normalize();
    float dot_product = TwoZAxis.dot(SE3TwoZAxis);
    const float tolerance = 0.01f;  // 允许的误差范围
    if (dot_product <= -0.5f) {  // 点积小于等于 -0.5 表示角度在 [120°, 240°]
        cout << "[debug] Two 和 SE3Two 的 z 轴方向相反，旋转 Two 180 度" << std::endl;
        // 绕 y 轴旋转 180 度的旋转矩阵
        Eigen::Matrix3f Ry180 = Eigen::AngleAxisf(M_PI, Eigen::Vector3f(0, 1, 0)).matrix();
        // 应用旋转到 Two
        Two.block<3, 3>(0, 0) = Two.block<3, 3>(0, 0) * Ry180;
    }


    w = e->scale(1) * 2;  // x
    h = e->scale(2) * 2;  // y
    l = e->scale(0) * 2;  // z

    } //这个括号是为了让上面的lock先释放，万万不可删

    // double s = std::sqrt(w * w + h * h + l * l)/2;
    // SetObjectPoseSim3(Two, s); // Two
    SetObjectPoseSim3(Two); // Two

    SetEllipsoid(*e);
}


bool MapObject::hasValidDepthPointCloud()
{
    unique_lock<mutex> lock(mMutexPointCloud);
    return mbValidDepthPointCloudFlag;
}



void MapObject::AddDepthPointCloudFromObjectDetection(pcl::PointCloud<PointType>::Ptr new_pcd_ptr)
{
    // std::cout << "AddDepthPointCloudFromObjectDetection" << std::endl;
    // // 这里可能还需要一次降采样操作
    unique_lock<mutex> lock(mMutexPointCloud);

    // std::cout << "mnId = " << mnId << std::endl;
    if (new_pcd_ptr == nullptr) {
        std::cout << " [debug] AddDepthPointCloudFromObjectDetection: 1 new_pcd_ptr = nullptr" << std::endl;
        return;
    }
    
    if (mpPcdCloudPtr == nullptr) 
    {
        std::cout << " [debug] AddDepthPointCloudFromObjectDetection: 2 mpPcdCloudPtr = nullptr" << std::endl;
        mpPcdCloudPtr = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
        // std::cout << "error in 1" << std::endl;
        *mpPcdCloudPtr = *(new_pcd_ptr);
        mbValidDepthPointCloudFlag = true;
    }
    else{
        std::cout << " [debug] AddDepthPointCloudFromObjectDetection: 3 Merging .. " << std::endl;
        pcl::PointCloud<PointType>::Ptr mergedCloud(new pcl::PointCloud<PointType>);
        pcl::concatenate(*(new_pcd_ptr), *mpPcdCloudPtr, *mergedCloud);
        mpPcdCloudPtr->clear();
        new_pcd_ptr->clear();
        *mpPcdCloudPtr = *mergedCloud;
        mbValidDepthPointCloudFlag = true;
    }

    // std::cout << "error in 2" << std::endl;
    
    // 打印合并后的点云的大小
    // std::cout << "Debug: Merged Cloud Size: " << pcd_ptr->size() << std::endl;
    // std::cout << "mnId = " << mnId << ", Merged Cloud Size: " << std::endl;

    // 进行一次将采样
    double grid_size = Config::Get<double>("Mapping.PcdCloudVoxelSize");
    int PcdCloudVoxelType = Config::Get<int>("Mapping.PcdCloudVoxelType");
    if(PcdCloudVoxelType == 1){
        static pcl::VoxelGrid<PointType> voxel;
        double gridsize = grid_size;
        voxel.setLeafSize( gridsize, gridsize, gridsize );
        voxel.setInputCloud( mpPcdCloudPtr );
        pcl::PointCloud<PointType>::Ptr tmp( new pcl::PointCloud<PointType>() );
        voxel.filter( *tmp );
        mpPcdCloudPtr.reset(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*tmp, *mpPcdCloudPtr);
        tmp.reset();
    }
    else if(PcdCloudVoxelType == 2){
        static pcl::ApproximateVoxelGrid<PointType> voxel;  // 修改类名
        double gridsize = grid_size;
        // 设置体素大小
        voxel.setLeafSize(gridsize, gridsize, gridsize);
        // 设置输入点云
        voxel.setInputCloud(mpPcdCloudPtr);
        // 输出到临时点云
        pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>());
        voxel.filter(*tmp);
        // 重新赋值回原始点云指针
        mpPcdCloudPtr.reset(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*tmp, *mpPcdCloudPtr);
        tmp.reset();
    } 


    // std::cout << "Debug: Undersampled Cloud Size: " << pcd_ptr->size() << std::endl;

    // TODO: 这里判定点云有效的参数有待写入参数文件
    if (mpPcdCloudPtr->size() > 5) {
        mbValidDepthPointCloudFlag = true;
    }

    mPcdCloudPoints = std::make_shared<PointCloud>(pclXYZToQuadricPointCloud(mpPcdCloudPtr));

    // return true;
}


g2o::ellipsoid* MapObject::GetEllipsold()
{
    unique_lock<mutex> lock(mMutexObject);
    if (mpEllipsold == NULL) {
        std::cerr << "[debug] GetEllipsold() 遇到 mpEllipsold == NULL" << endl;
        std::exit(EXIT_FAILURE);  // 或者：std::abort();
        return NULL;
    }
    else{
        return mpEllipsold;
    }
}

pcl::PointCloud<PointType>::Ptr MapObject::GetDepthPointCloudPCL()
{
    return mpPcdCloudPtr;
}

std::shared_ptr<PointCloud> MapObject::GetPointCloud()
{
    unique_lock<mutex> lock(mMutexPointCloud);
    return mPcdCloudPoints;
}

}

