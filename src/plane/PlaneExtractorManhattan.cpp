// 本文件基于曼哈顿假设提取平面

#include "PlaneExtractorManhattan.h"
#include <src/config/Config.h>

namespace ORB_SLAM2
{

void PlaneExtractorManhattan::SetGroundPlane(g2o::plane* gplane)
{
    mpGroundplane = gplane;  //world坐标系下的地面
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr samplePointsOnPlane(const Eigen::Vector4d& plane_param, int M) {
    double A = plane_param[0];
    double B = plane_param[1];
    double C = plane_param[2];
    double D = plane_param[3];

    // 单位法向量
    Eigen::Vector3d normal(A, B, C);
    normal.normalize();

    // 找平面上的一个点：令 x = y = 0, 计算 z
    Eigen::Vector3d P0;
    if (std::abs(C) > 1e-6) {
        P0 = Eigen::Vector3d(0, 0, -D / C);
    } else if (std::abs(B) > 1e-6) {
        P0 = Eigen::Vector3d(0, -D / B, 0);
    } else {
        P0 = Eigen::Vector3d(-D / A, 0, 0);
    }

    // 构造两个在平面上的正交向量 u 和 v（用 Gram-Schmidt）
    Eigen::Vector3d u = normal.unitOrthogonal();         // 与 normal 垂直
    Eigen::Vector3d v = normal.cross(u).normalized();    // 与 normal 和 u 同时垂直

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 采样范围 [-r, r]
    const double r = 50.0; // 控制平面大小
    int N = std::sqrt(M);
    if (N < 1) N = 1;

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            double alpha = -r + 2 * r * i / (N - 1);
            double beta  = -r + 2 * r * j / (N - 1);
            Eigen::Vector3d point = P0 + alpha * u + beta * v;

            pcl::PointXYZRGB pcl_point;
            pcl_point.x = point.x();
            pcl_point.y = point.y();
            pcl_point.z = point.z();

            pcl_point.r = 0;
            pcl_point.g = 255;
            pcl_point.b = 0;

            cloud->points.push_back(pcl_point);
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

bool PlaneExtractorManhattan::extractManhattanPlanes(const cv::Mat &depth, Eigen::Vector3d& local_gt, g2o::SE3Quat &Twc)
{
    // ************************
    // INIT
    // ************************
    mParam.RangeOpen = false;
    mvPotentialGroundPlanePoints.clear();

    mvAllMHPlanesPoints.clear();
    mvpAllMHPlanes.clear();   // 当前提取的所有曼哈顿平面 ("仅满足垂直平行约束")
    // 把地面和地面上随机生成的点云放进去
    g2o::plane* pGroundplane_local = new g2o::plane(*mpGroundplane);
    pGroundplane_local->transform(Twc.inverse());
    mvpAllMHPlanes.push_back(pGroundplane_local);
    PointCloudPCL::Ptr pGroundplanePoints_local  = samplePointsOnPlane(pGroundplane_local->param, 100);
    pGroundplane_local->miMHType = g2o::MANHATTAN_PLANE_TYPE::GROUND; // 地面是平行的
    mvAllMHPlanesPoints.push_back(*pGroundplanePoints_local); 

    mvpStructuralMHPlanes_bigenough.clear();     // 当前提取的潜在曼哈顿结构平面 ("经过大小过滤")
    mbResult = false;

    // 1. 提取初步平面
    // 注意: 该部分结果完全可以与地平面共用
    extractPlanes(depth);
    if( mvPlaneCoefficients.size() < 1)     // there should be more than 1 potential planes 
    {
        std::cout << "[debug] Extracting Manhattan Planes Failed, 未提取到曼哈顿平面" << std::endl;
        return false;   // 没有平面
    }    

    // *******************************************
    //     筛选曼哈顿！  需要参考平面: 地平面(重力方向)
    // the Manhattan should meet the criteria:
    // vertical to or parrel to the gravity direction.
    // *******************************************
    std::vector<g2o::plane*> vpPlanes;
    std::vector<std::pair<g2o::plane*, int>> mapPlaneSize;

    // 构造曼哈顿平面筛选条件
    // config_angle_delta：容忍角度误差为 5 度，控制“平行”、“垂直”的判断；
    // MH_plane_size：平面点数的最小阈值（用于判断是否“足够大”），防止误识别小平面。
    double config_angle_delta = 5 / 180.0 * M_PI;   // 5 deg.
    int MH_points_in_plane_size = depth.rows * depth.cols * mf_MH_points_in_plane_size_scale;   // 要求具有屏幕 1/4的点!

    // 遍历所有初步提取的平面，判断是否是曼哈顿平面
    for( int i=0; i<mvPlaneCoefficients.size(); i++ )
    {
        // 提取该平面法向量：
        Eigen::Vector4d vec;
        auto& coeff = mvPlaneCoefficients[i];
        vec << double(coeff.at<float>(0)), double(coeff.at<float>(1)), double(coeff.at<float>(2)), double(coeff.at<float>(3));

        // 判断法向与重力方向夹角
        Eigen::Vector3d axis_gravity = local_gt;       
        Eigen::Vector3d axisNorm = vec.head(3);
        double cos_theta = axisNorm.transpose() * axis_gravity;
        cos_theta = cos_theta / axisNorm.norm() / axis_gravity.norm();
        double angle = acos( cos_theta );      // acos : [0,pi]

        // 角度接近 0° 或 180°：水平平面（如地面、桌面）；
        // 角度接近 90°：垂直平面（如墙面）；
        // 否则不是曼哈顿平面，忽略。
        g2o::MANHATTAN_PLANE_TYPE iMHType = g2o::MANHATTAN_PLANE_TYPE::OTHERS;
        if( std::abs(angle - 0)<config_angle_delta || 
                std::abs(angle- M_PI) < config_angle_delta ) 
        {
            iMHType = g2o::MANHATTAN_PLANE_TYPE::HORIZONTAL;    // parallel
        }
        // ---- DEBUG: 暂时取消垂直倚靠关系. 只考虑普遍存在的支撑关系
        else if( std::abs(angle - M_PI/2.0)<config_angle_delta )
        {
            iMHType = g2o::MANHATTAN_PLANE_TYPE::VERTICAL; // Vertical
        }

        // 若是曼哈顿平面（与地面平行或垂直），则保存进 mvpAllMHPlanes 中
        if( iMHType != g2o::MANHATTAN_PLANE_TYPE::OTHERS )
        {
            g2o::plane* pPlane = new g2o::plane();
            pPlane->param= vec;
            pPlane->miMHType = iMHType;

            mvpAllMHPlanes.push_back(pPlane);   // 满足了曼哈顿假设的都放进去
            mvAllMHPlanesPoints.push_back(mvPlanePoints[i]);

            // 如果满足曼哈顿平面大小要求，则加入到 mvpStructuralMHPlanes_bigenough 中
            int num_size = mvPlanePoints[i].size();
            if( num_size > MH_points_in_plane_size){
                vpPlanes.push_back(pPlane);
                mvPotentialGroundPlanePoints.push_back(mvPlanePoints[i]);
            }
            else
            {
                // 若属于支撑平面, 大小要求放宽!
                if( iMHType == 1 && num_size > 200)
                {
                    vpPlanes.push_back(pPlane);
                    mvPotentialGroundPlanePoints.push_back(mvPlanePoints[i]);
                }
            }
            
        }  
    }

    // std::cout<< "[debug] Extracting Manhattan Planes, 提取到Potential Structural MHPlanes 数量: " << vpPlanes.size() << std::endl;
    if( vpPlanes.size() < 1) {      // there should be more than 1 valid planes 
        return false;
    }

    // 保存结果！
    mvpStructuralMHPlanes_bigenough = vpPlanes;

    // 针对此次结果，更新房间的主导曼哈顿平面
    UpdateHomeDominantStructuralMHPlanes(Twc);

    return true;
}

void PlaneExtractorManhattan::AddNewDominantMHPlane(g2o::plane* vP)
{
    mvpHomeDominantStructuralMHPlanes.push_back(vP);
    std::cout << "[Manhattan] Add new MH Plane. Total : " << mvpHomeDominantStructuralMHPlanes.size() << std::endl;

    if(mvpHomeDominantStructuralMHPlanes.size() == 5)
        mbDominantResult = true;
}

std::vector<g2o::plane*> PlaneExtractorManhattan::GetHomeDominantStructuralMHPlanes()
{
    return mvpHomeDominantStructuralMHPlanes;
}


PlaneExtractorManhattan::PlaneExtractorManhattan():mbResult(false),mbDominantResult(false),mpGroundplane(NULL)
{}

PlaneExtractorManhattan::PlaneExtractorManhattan(PlaneExtractorParam& param, g2o::plane* gplane):mbResult(false),mbDominantResult(false),mpGroundplane(NULL)
{
    SetParam(param);
    SetGroundPlane(gplane);
    mf_MH_points_in_plane_size_scale = Config::Get<double>("Plane.MH_points_in_plane_size_scale");
}

// zhjd: 从当前帧提取出的结构性曼哈顿平面中筛选出“主导曼哈顿平面”（Dominant Manhattan Planes）并进行全局更新。
// 在不重复已有平面的前提下，筛选出新的、方向独立且不重叠的主导平面，维护一个最多5个的主导曼哈顿平面集合。
void PlaneExtractorManhattan::UpdateHomeDominantStructuralMHPlanes(g2o::SE3Quat &Twc)
{
    // 若mbDominantResult为 false，说明尚未更新主导曼哈顿平面，继续处理。
    if(!mbDominantResult)
    {
        // 一一判断 mvpStructuralMHPlanes_bigenough 是否为新的曼哈顿帧, 条件:
        // 0). 平面之大小: 已经过滤完毕.
        // 1). 不能与已有的距离太近 ( 2m )
        // 2). 与已有平面法向量相同的, 不能超过2个 
        for( auto& vP:mvpStructuralMHPlanes_bigenough)
        {
            // 最多只保留5个主导曼哈顿平面，超过则不再添加。   
            if( mvpHomeDominantStructuralMHPlanes.size() >= 5) break; // 已经满了


            // 对每个检查所有已确定 MH Plane
            int state = 0;
            int parallel_count = 0;
            int vertical_count = 0;

            // 将当前结构平面转换到世界坐标系
            g2o::plane* pPlanesGlobal = new g2o::plane(*vP); pPlanesGlobal->transform(Twc);
            Vector4d param_plane = pPlanesGlobal->param; 
            Eigen::Vector3d norm_plane = param_plane.head(3);

            // 遍历已有主导平面（加上地面）,对每个已有平面判断角度关系, 判断角度关系 + 距离关系
            std::vector<g2o::plane*> MHPlanes = mvpHomeDominantStructuralMHPlanes;
            MHPlanes.push_back(mpGroundplane);   // 加入地平面
            for( auto& vMHP : MHPlanes ) 
            {
                Vector4d param_MHplane = vMHP->param; 
                Eigen::Vector3d norm_MHplane = param_MHplane.head(3);

                // 判断法向量角度
                double cos_theta = norm_plane.transpose() * norm_MHplane;
                cos_theta = cos_theta / norm_plane.norm() / norm_MHplane.norm();
                double angle = acos( cos_theta );      // acos : [0,pi]
                
                double angle_tolerance = 5 * M_PI / 180.0;
                if( std::abs(angle-0) < angle_tolerance || std::abs(angle-M_PI) < angle_tolerance )
                {
                    // 与该平面平行
                    parallel_count++;

                    double distance = pPlanesGlobal->distanceToPlane(*vMHP);
                    if( distance < 3.0) // TODO: 输出调试这个值.
                    {
                        state = 1;      // 1) 发现距离太近平面
                        // std::cout << "[debug] UpdateHomeDominantStructuralMHPlanes.distance : " << distance << std::endl;
                    }
                }
                else if (std::abs(angle-M_PI/2) < angle_tolerance )
                {
                    // 与该平面垂直
                    vertical_count++;
                }
                else 
                {
                    // 与已有平面，不垂直也不平行； 不满足曼哈顿假设.
                    state = 3;
                }
            }

            // 平行的平面数量大于2
            if(parallel_count >= 2)
                state = 2;                    
            

            if(state == 0){
                // 与地面平行，且距离其他平面的距离足够远
                Eigen::Vector3d center_wc = Twc.translation();
                if (pPlanesGlobal->distanceToPoint(center_wc, true) < 0)
                    pPlanesGlobal->param = - pPlanesGlobal->param;

                // 可视化
                pPlanesGlobal->InitFinitePlane(center_wc, 10);

                // 通过检查, 添加进 DMHP
                AddNewDominantMHPlane(pPlanesGlobal);
                mbResult = true;    // 添加了新的，本次成功.

                // std::cout<< "[debug] 房间 Dominant Manhattan Planes添加成功, 已有平面数量: " << mvpHomeDominantStructuralMHPlanes.size() << std::endl;
            }
            else {
                switch (state) {
                    case 0:
                        // std::cout << "[debug] 房间 Dominant Manhattan Planes添加成功, 与地面平行，且距离已有平面足够远（<3米）" << std::endl;
                        break;
                    case 1:
                        // std::cout << "[debug] 房间 Dominant Manhattan Planes添加失败, 虽然平行，但是距离已有平面太近（<3米）" << std::endl;
                        break;
                    case 2:
                        // std::cout << "[debug] 房间 Dominant Manhattan Planes添加失败, 平行的平面数量大于2" << std::endl;
                        break;
                    case 3:
                        // std::cout << "[debug] 房间 Dominant Manhattan Planes添加失败, 与已有平面既不平行也不垂直" << std::endl;
                        break;
                    default:
                        // std::cout << "[debug] 房间 Dominant Manhattan Planes添加失败, 未知原因" << state << std::endl;
                        break;
                }
            }

        }

    }
}


std::vector<g2o::plane*> PlaneExtractorManhattan::GetAllMHPlanes()
{
    return mvpAllMHPlanes;
}


bool PlaneExtractorManhattan::GetMHResult()
{
    return mbResult;
}

std::vector<PointCloudPCL> PlaneExtractorManhattan::GetAllMHPlanesPoints()
{
    return mvAllMHPlanesPoints;
}

}