// Single-frame ellipsoid extraction from RGB-D data

#ifndef ELLIPSOIDSLAM_ELLIPSOIDEXTRACTOR_H
#define ELLIPSOIDSLAM_ELLIPSOIDEXTRACTOR_H

#include <opencv2/opencv.hpp>

#include <ellipsoid-version/Ellipsoid.h>
#include <ellipsoid-version/Geometry.h>
#include <Map.h>
#include <ellipsoid-version/BasicEllipsoidEdges.h>

#include <src/symmetry/PointCloudFilter.h>
#include <src/symmetry/Symmetry.h>
#include <src/symmetry/SymmetrySolver.h>

#include <Eigen/Core>

#include <iostream>
// #include <pcl/io/io.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>		  

typedef pcl::PointXYZ PointType;

typedef pcl::Normal NormalType;

namespace ORB_SLAM2
{

class Map;

struct PCAResult
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    bool result;
    Eigen::Vector3d center;
    Eigen::Matrix3d rotMat;
    Eigen::Vector3d covariance; // unused yet.
    Eigen::Vector3d scale;
};

enum ExtractionMethod
{
    PCA_EXTRACTION = 1,
    ATLAS_EXTRACTION = 2
};

std::vector<g2o::ConstrainPlane*> GenerateConstrainPlanesOfBbox(Vector4d& bbox, Matrix3d& calib, int rows, int cols);
void VisualizeConstrainPlanes(g2o::ellipsoid& e_local, g2o::SE3Quat& Twc, ORB_SLAM2::Map* pMap);

class EllipsoidExtractor
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EllipsoidExtractor();   
    EllipsoidExtractor(Map* pMap);


    // open symmetry plane estimation to finish point cloud completion
    void OpenSymmetry();

    // The supporting plane is used to segment object points and estimate the object orientation
    void SetSupportingPlane(g2o::plane* pPlane, bool local);
  
    // API: estimate a 3d ellipsoid from RGB-D data and a bounding box, the ellipsoid is in global coordinate
    g2o::ellipsoid EstimateLocalEllipsoid(cv::Mat& depth, Eigen::Vector4d& bbox, int label, double prob, Eigen::VectorXd &pose, camera_intrinsic& camera);   

    // API2: [new] estimate using multi-planes
    g2o::ellipsoid EstimateLocalEllipsoidUsingMultiPlanes(cv::Mat& depth, Eigen::Vector4d& bbox, int label, double prob, Eigen::VectorXd &pose, camera_intrinsic& camera, pcl::PointCloud<PointType>::Ptr& pcd_ptr);   
    g2o::ellipsoid EstimateLocalEllipsoidUsingNormalVoters(cv::Mat& depth, Eigen::Vector4d& bbox, cv::Mat& mask, int label, double prob, Eigen::VectorXd &pose, camera_intrinsic& camera, pcl::PointCloud<PointType>::Ptr& pcd_ptr);   
    
    
    // API2.1: given a supporting plane(local coordinate)
    g2o::ellipsoid EstimateLocalEllipsoidWithSupportingPlane(cv::Mat& depth, Eigen::Vector4d& bbox, int label, double prob, Eigen::VectorXd &pose, camera_intrinsic& camera, g2o::plane* pSupPlane);

    g2o::ellipsoid EstimateEllipsoidFromPCDCloud(pcl::PointCloud<PointType>::Ptr& pcd_ptr, g2o::plane* ground, int ManualDirection = 0);

    void OpenVisualization(Map* pMap);   // if opened, the pointcloud during the process will be visualized 
    void ClearPointCloudList(); // clear the visualized point cloud 


    bool GetResult();   // get extraction result.
    SymmetryOutputData GetSymmetryOutputData();  // get the detail of symmetry estimation
    ORB_SLAM2::PointCloud* GetPointCloudInProcess();   // get the object point cloud
    ORB_SLAM2::PointCloud* GetPointCloudDebug();   // get the debug point cloud before Eucliden filter

    void SetExtractionMethod(int method);
    void OpenManhattanPlanesFilter(const std::vector<g2o::plane*> vpPlanes);
    void CloseManhattanPlanes();   // close the manhattan plane filter

private:
    void LoadSymmetryPrior();  // define object symmetry prior

    pcl::PointCloud<PointType>::Ptr ExtractPointCloud(cv::Mat& depth, Eigen::Vector4d& bbox, Eigen::VectorXd &pose, camera_intrinsic& camera);    // extract point cloud from depth image.
    pcl::PointCloud<PointType>::Ptr ExtractPointCloud(cv::Mat& depth, Eigen::Vector4d& bbox, cv::Mat& mask_cv, Eigen::VectorXd &pose, camera_intrinsic& camera);    // extract point cloud from depth image.
    PCAResult ProcessPCA(pcl::PointCloud<PointType>::Ptr &pCloudPCL);   // apply principal component analysis 
    g2o::ellipsoid ConstructEllipsoid(PCAResult &data);   // generate a sparse ellipsoid estimation from PCA result.

    void ApplyGravityPrior(PCAResult &data);    // add the supporting groundplane prior to calibrate the rotation matrix

    ORB_SLAM2::PointCloud* ApplyEuclideanFilter(ORB_SLAM2::PointCloud* pCloud, Vector3d &center);   // apply euclidean filter to get the object points
    ORB_SLAM2::PointCloud* ApplyPlaneFilter(ORB_SLAM2::PointCloud* pCloud, double z);    // filter points lying under the supporting plane

    ORB_SLAM2::PointCloud* ApplySupportingPlaneFilter(ORB_SLAM2::PointCloud* pCloud);   

    bool GetCenter(cv::Mat& depth, Eigen::Vector4d& bbox, Eigen::VectorXd &pose, camera_intrinsic& camera, Vector3d& center); // get a coarse 3d center of the object
    double getDistanceFromPointToCloud(Vector3d& point, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);  // get the minimum distance between a point and a pointcloud

    // make sure the rotation matrix is right-handed
    void AdjustChirality(PCAResult &data);

    // adjust the axis order to make z axis near the normal of the ground plane
    void AlignZAxisToGravity(PCAResult &data);
    // this function will be called if groundplane is set when aligning Z axis
    Eigen::Matrix3d calibRotMatAccordingToGroundPlane( Matrix3d& rotMat, const Vector3d& normal);

    void VisualizePointCloud(const string& name, ORB_SLAM2::PointCloud* pCloud, const Vector3d &color = Vector3d(-1,-1,-1), int point_size = 2);
    // void VisualizeEllipsoid(const string& name, g2o::ellipsoid* pObj);

    PCAResult ProcessPCANormalized(ORB_SLAM2::PointCloud* pObject);

    g2o::ellipsoid GetEllipsoidFromNomalizedPointCloud(ORB_SLAM2::PointCloud* pCloud);

    PCAResult PCA(pcl::PointCloud<PointType>::Ptr& pCloudPCL);
    double NormalVoter(pcl::PointCloud<PointType>::Ptr& pCloudPCL);

public:
    g2o::ellipsoid OptimizeEllipsoidUsingPlanes(g2o::ellipsoid &e_in, MatrixXd& mPlanesParam);
    g2o::ellipsoid OptimizeEllipsoidWithBboxPlanesAndMHPlanes(const g2o::ellipsoid &init_guess, std::vector<g2o::plane> &BboxPlanes, double Bbox_Weight, 
                                                                                g2o::plane &SupprotingPlane, double Supproting_Weight,
                                                                                g2o::plane &BackingPlane, double Backing_Weight);
    g2o::ellipsoid OptimizeEllipsoidWithBboxPlanesAndMHPlanes(const g2o::ellipsoid &init_guess, std::vector<g2o::plane> &BboxPlanes, double Bbox_Weight, 
                                                                                g2o::plane &SupprotingPlane, double Supproting_Weight);
    g2o::ellipsoid OptimizeEllipsoidWithMHPlanes(const g2o::ellipsoid &init_guess,  int num_opt, 
                                                            g2o::plane &SupprotingPlane, double Supproting_Weight,
                                                            g2o::plane &BackingPlane, double Backing_Weight);
    g2o::ellipsoid OptimizeEllipsoidWithSupportingPlanes(const g2o::ellipsoid &init_guess,  int num_opt,  g2o::plane &SupprotingPlane, double Supproting_Weight);                                                                                
private:
    ORB_SLAM2::PointCloud* ApplyMHPlanesFilter(ORB_SLAM2::PointCloud* pCloud, std::vector<g2o::plane*>& vpPlanes);

    g2o::SE3Quat GenerateGravityCoordinate(const Vector3d& center, const Vector3d& gravity_normal);
private:
    bool mResult;  // estimation result.

    int miEuclideanFilterState; // Euclidean filter result:  1 no clusters 2: not the biggest cluster 3: fail to find valid cluster 0: success
    int miSystemState;  // 0: normal 1: no depth value for center point 2: fail to filter. 3: no point left after downsample
    ORB_SLAM2::PointCloud* mpPoints;    // store object points
    ORB_SLAM2::PointCloud* mpPointsDebug;    // store points for debugging ( points before Euclidean filter)

    // supporting plane
    bool mbSetPlane;    
    g2o::plane* mpDefaultSupportingPlane;

    // symmetry prior
    std::map<int,int> mmLabelSymmetry;

    int miMethod;
    
public: 
    // data generated in the process
    Vector3d mDebugCenter;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> mDebugEuclideanFilterClouds;
    PointCloudPCL::Ptr mDebugCenterCloud;

private:
    SymmetrySolver* mpSymSolver;

    SymmetryOutputData mSymmetryOutputData;

    int mbOpenVisualizeDepthPoints = true;
    Map* mpMap;
    int miExtractCount;

    bool mbOpenSymmetry;

    bool mbOpenMHPlanesFilter;
    std::vector<g2o::plane*> mvpHomeDominantMHPlanes;

    bool mbLocalSupportingPlane_nouse;
};


}

#endif // ELLIPSOIDSLAM_ELLIPSOIDEXTRACTOR_H