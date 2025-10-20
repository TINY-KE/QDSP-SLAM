// 2020-5-13, by lzw
// 该文件用于处理平面与物体的“支撑关系”， 包括判断、约束、关联等内容。

#ifndef RELATIONSHIP_H
#define RELATIONSHIP_H

#include <iostream>
#include <vector>

#include <ellipsoid-version/Ellipsoid.h>
#include <ellipsoid-version/Plane.h>

#include <Eigen/Core>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/kdtree/kdtree_flann.h>
namespace ORB_SLAM2
{
    enum RELATION_TYPE
    {
        INVALID = 0,
        SUPPORTING = 1,
        BACKING = 2
    };

    // 该类在局部坐标系下计算, 匹配局部物体与局部平面之间的潜在约束关系。
    // 提取结果保存为  obj_id -> plane_id 的映射. 以及结构体.
    class Frame;
    class KeyFrame;
    class Relation
    {

    public:
        int obj_id; // 索引: 与localEllipsoids, measurements同索引.
        int plane_instance_id;  // 关联 SupportingPlanes 的 id
        int plane_id;
        g2o::plane* pPlane;
        g2o::ellipsoid* pEllipsoid;
        Frame* pFrame;
        RELATION_TYPE type; // 无效关系0, 支撑关系 1, 倚靠关系 2.

        Eigen::VectorXd SaveToVec();
        void LoadFromVec(const Eigen::VectorXd& vec);
        bool InitRelation(const Eigen::VectorXd& vec, ORB_SLAM2::Frame* pFrameIn);
    
    private:
        int GetDataNum();
    };
    typedef std::vector<Relation> Relations;

    class RelationExtractor
    {
    public:
        Relations ExtractRelations(std::vector<g2o::ellipsoid *> &vpEllips, std::vector<g2o::plane *> &vpPlanes, KeyFrame* pKF, std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& vpPlanesPoints);

        Relations ExtractSupporttingRelations(std::vector<g2o::ellipsoid *> &vpEllips, std::vector<g2o::plane *> &vpPlanes, Frame* pFrame, int model = 1);

        bool TooCloseOfPlaneToCorners(g2o::plane* plane, Eigen::Matrix3Xd& mCorners, Eigen::Matrix3Xd& mIds);
    };

} // namespace ORB_SLAM2

#endif
