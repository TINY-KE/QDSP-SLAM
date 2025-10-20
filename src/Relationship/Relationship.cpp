#include "Relationship.h"
// #include "include/ellipsoid-version/Frame.h"
#include "Frame.h"
#include "include/ellipsoid-version/ConstrainPlane.h"

using namespace std;
namespace ORB_SLAM2
{
    bool sort_plane_dis(pair<double,g2o::plane*>&p1, pair<double,g2o::plane*>&p2)
    {
        return p1.first < p2.first;
    }

    bool RelationExtractor::TooCloseOfPlaneToCorners(g2o::plane* plane, Matrix3Xd& mCorners, Matrix3Xd& mIds){

        // TODO:

        return true;
    }


    Relations RelationExtractor::ExtractRelations(std::vector<g2o::ellipsoid *> &vpEllips, std::vector<g2o::plane *> &vpPlanes, KeyFrame* pKF, std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& vpPlanesPoints)
    {
        auto mvpKeyframeGlobalEllipsolds = pKF->GetEllipsoldsGlobal();
        Relations relations_return;
        int obj_num = vpEllips.size();
        // std::cout<< "[debug] RelationExtractor::ExtractRelations, obj_num: " << obj_num << ", plane_num: " << vpPlanes.size() << std::endl;
        
        double backing_distance_max = Config::Get<double>("EllipsoidExtractor.backing_distance_max"); 
        for (int obj_id = 0; obj_id < obj_num; obj_id++)
        {
            g2o::ellipsoid *pEllip = vpEllips[obj_id];
            if(pEllip == NULL ) {
                // std::cout << "[Relation] NULL ellipsoid." << std::endl;
                continue;
            }

            // std::cout<<"debug: vpEllips[obj_id]->miLabel: ";
            // std::cout<< vpEllips[obj_id]->miLabel <<std::endl;
            int object_label = vpEllips[obj_id]->miLabel;  // 假设标签值
            bool is_on_ground = false;
            std::vector<int> Objects_on_ground_Labels = {56, 57/* 椅子，沙发 */ ,13 /* 板凳 */, 58 /* 盆栽植物 */, 59 /* 床 */, 60 /* 餐桌 */, 72 /* 冰箱 */};
            if (std::find(Objects_on_ground_Labels.begin(), Objects_on_ground_Labels.end(), object_label) != Objects_on_ground_Labels.end()) {
                is_on_ground = true;
            } else {
                is_on_ground = false;
            }

            std::vector<int> Objects_dont_use_backing = {56, 57/* 椅子，沙发 */ , 58 /* 盆栽植物 */, 60 /* 餐桌 */};
            bool dont_use_backing = false;
            if (std::find(Objects_dont_use_backing.begin(), Objects_dont_use_backing.end(), object_label) != Objects_dont_use_backing.end()) {
                dont_use_backing = true;
            } else {
                dont_use_backing = false;
            }

            // std::cout<< "[debug] RelationExtractor::ExtractRelations, obj_id: " << obj_id << std::endl;
            if(pEllip->mbBackingPlaneDefined || pEllip->mbSupportingPlaneDefined ) {
                std::cerr << "[Error]: 已提前有MHP! "<< pEllip->mbSupportingPlaneDefined << ", "<< pEllip->mbBackingPlaneDefined << std::endl;
                exit(-1);
            }
            // 获取物体的六个面
            Matrix3Xd mCorners;  mCorners.resize(3,8);
            Matrix3Xd mIds; mIds.resize(3, 6);
            // 分别对应 地面、顶面、四个侧面
            mIds << 1, 5, 1, 3, 6, 8,
                    2, 8, 5, 7, 7, 5,
                    3, 7, 6, 8, 3, 1;
            
            // 物体的六个平面，平面法向量均指向物体外侧
            std::vector<g2o::plane*> obj_planes = pEllip->GetCubePlanesGlobal(mCorners);  // 椭球体所在的坐标系
            g2o::plane* pObj_bottom_plane = obj_planes[0];  //物体的底面

            // 寻找最佳支撑平面
            g2o::plane* pSupportingPlane_best = NULL;
            int sup_plane_id=-1;
            // 地面中心的坐标
            Vector3d bottomplane_centor = (mCorners.col(mIds(0,0)-1) + mCorners.col(mIds(2,0)-1))/2;
            std::vector<std::pair<double, g2o::plane*>> supprortingPlaneDisVec;
            for (int plane_id = 0; plane_id < vpPlanes.size(); plane_id++)
            {
                g2o::plane *pPlane = vpPlanes[plane_id];
                // typedef pcl::PointXYZRGB PointT;
                // typedef pcl::PointCloud<PointT> PointCloudPCL;
                PointCloudPCL PlanePoints = vpPlanesPoints[plane_id];

                // 判断水平面是否是支撑平面
                if(pPlane->miMHType==g2o::MANHATTAN_PLANE_TYPE::HORIZONTAL){
                    double z_dis = pObj_bottom_plane->distanceToPlane(*pPlane);
                    
                    pcl::KdTreeFLANN<PointT> kdtree;
                    kdtree.setInputCloud(PlanePoints.makeShared());  // 构建 KD-Tree

                    PointT searchPoint;  //构造查询点
                    searchPoint.x = bottomplane_centor(0);
                    searchPoint.y = bottomplane_centor(1);
                    searchPoint.z = bottomplane_centor(2);

                    std::vector<int> nearest_indices(1);
                    std::vector<float> nearest_sqr_distances(1);

                    // 计算nearest_point与bottomplane_centor之间的距离
                    double xy_distance = 1000;
                    double min_xyz_distance;
                    // 物体的半轴长度
                    double object_length = 0;
                    if (kdtree.nearestKSearch(searchPoint, 1, nearest_indices, nearest_sqr_distances) > 0) {
                        int index = nearest_indices[0];
                        min_xyz_distance = std::sqrt(nearest_sqr_distances[0]);
                        // 你可以通过索引获取点
                        const PointT& nearest_point = PlanePoints[index];
                        xy_distance = std::sqrt(std::pow(nearest_point.x - bottomplane_centor(0), 2) + std::pow(nearest_point.y - bottomplane_centor(1), 2));
                        object_length = std::sqrt(std::pow(pEllip->scale(0) - bottomplane_centor(0), 2) + std::pow(pEllip->scale(0) - bottomplane_centor(1), 2));
                    }
                    // if(z_dis>0 && xy_distance<object_length*2 )   // 即平面不能在物体底面之上; 平面不能离地面中心太远
                    //     supprortingPlaneDisVec.push_back(make_pair(z_dis, pPlane));

                    if(is_on_ground){
                        g2o::plane* pPlaneGlobal = new g2o::plane(*pPlane);
                        pPlaneGlobal->transform(pKF->cam_pose_Twc);
                        double height = -1 * pPlaneGlobal->param[3] / pPlaneGlobal->param[2];
                        if(height<0.1) {
                            supprortingPlaneDisVec.push_back(make_pair(min_xyz_distance, pPlane));
                        }
                    }
                    else{
                        if(z_dis>0 && z_dis<0.2) {  // 即平面不能在物体底面之上; 平面不能离地面中心太远
                            supprortingPlaneDisVec.push_back(make_pair(min_xyz_distance, pPlane));
                            std::cout << "  最近xyz距离为: " << min_xyz_distance << std::endl;
                            std::cout << "  物体长度 object_length*1.5: " << object_length*1.5 << std::endl;
                        }
                    }
                }
            }

            if(supprortingPlaneDisVec.size()!=0 ) {
                sort(supprortingPlaneDisVec.begin(), supprortingPlaneDisVec.end(), sort_plane_dis);
                double dis_min = supprortingPlaneDisVec[0].first;
                pSupportingPlane_best = supprortingPlaneDisVec[0].second;
                {
                    // 寻找 plane_id
                    for(int i=0;i<vpPlanes.size();i++)
                    {
                        if(vpPlanes[i] == pSupportingPlane_best)
                            sup_plane_id = i;
                    }
                    
                    // 保存该组关系
                    Relation rl;
                    rl.obj_id = obj_id;
                    rl.plane_id = sup_plane_id;
                    rl.type = RELATION_TYPE::SUPPORTING;
                    rl.pPlane = pSupportingPlane_best;
                    rl.pEllipsoid = pEllip;
                    // rl.pFrame = pFrame;
                    relations_return.push_back(rl);
                    
                    // 存入到frame中的local椭球体中
                    g2o::ConstrainPlane* mhp = new g2o::ConstrainPlane(pSupportingPlane_best);
                    mhp->type = CONSTRAINPLANE_STATE::SUPPORTING; 
                    pEllip->mpSupportingPlane = mhp;
                    pEllip->mbSupportingPlaneDefined = true;

                    // 存入到KeyFrame的global椭球体中
                    g2o::plane* p_global = new g2o::plane(*pSupportingPlane_best);
                    p_global->transform(pKF->cam_pose_Twc);
                    g2o::ConstrainPlane* mhp_global = new g2o::ConstrainPlane(p_global);
                    mhp_global->type = CONSTRAINPLANE_STATE::SUPPORTING; 
                    // std::cout<<"[deubg] pSupportingPlane_best:"<< p_global->param.transpose() << std::endl;
                    mvpKeyframeGlobalEllipsolds[obj_id]->mpSupportingPlane = mhp_global;
                    mvpKeyframeGlobalEllipsolds[obj_id]->mbSupportingPlaneDefined = true;
                }
            }
            
            // 寻找最佳倚靠平面
            int back_plane_id=-1;
            g2o::plane* pBackingPlane_best = NULL;
            std::vector<std::pair<double, g2o::plane*>> backingPlaneAreaVec;
            // std::cout<< "[debug] RelationExtractor::ExtractRelations, obj_id: " << obj_id << ", plane_num: " << vpPlanes.size() << std::endl;
            for (int plane_id = 0; plane_id < vpPlanes.size(); plane_id++)
            {
                g2o::plane *pPlane = vpPlanes[plane_id];
                PointCloudPCL PlanePoints = vpPlanesPoints[plane_id];

                if(pPlane->miMHType==g2o::MANHATTAN_PLANE_TYPE::VERTICAL){

                    for(int j=2; j<6; j+=1){

                        g2o::plane* pObj_side_plane = obj_planes[j];
                        // 判断是否平行
                        double angle_diff = pObj_side_plane->angleToPlane(*pPlane);
                        if (std::abs(angle_diff) < M_PI / 180.0 * 30  ||  std::abs(angle_diff-M_PI) < M_PI / 180.0 * 30 )  // 容忍 30 度
                        {
                            g2o::plane plane_align = *pPlane;
                            if(std::abs(angle_diff-M_PI) < M_PI / 180.0 * 30)
                                plane_align.param = -plane_align.param; // 调转方向
                            
                            Vector3d sideplane_centor = (mCorners.col(mIds(0,j)-1) + mCorners.col(mIds(2,j)-1))/2;

                            double dis = plane_align.distanceToPoint(sideplane_centor, true);

                            if ( dis < 0.1 &&  dis > -1*backing_distance_max)  // 平面最多进入物体内部10cm
                            {
                                // 平面中点的数量
                                backingPlaneAreaVec.push_back(make_pair(PlanePoints.size(), pPlane));
                                // std::cout << "  [success] plane_id: " << plane_id << ", area: " << PlanePoints.size() << ", angle: " << angle_diff << ", dis: " << dis << std::endl;
                            }
                            else{
                                // std::cout << "  [fail]    plane_id: " << plane_id << ", area: " << PlanePoints.size() << ", angle: " << angle_diff << ", dis: " << dis << std::endl;
                            }
                        }
                        else{
                                // std::cout << "  [fail]    plane_id: " << plane_id << ", area: " << PlanePoints.size() << ", angle: " << angle_diff << std::endl;
                        }
                    }         
                }
            }

            // std::cout<<

            if(backingPlaneAreaVec.size()!=0 && !dont_use_backing ) {
                std::sort(backingPlaneAreaVec.begin(), backingPlaneAreaVec.end(), 
                    [](const auto& a, const auto& b) {
                        return a.first > b.first; 
                    });
                double area_max = backingPlaneAreaVec[0].first;
                pBackingPlane_best = backingPlaneAreaVec[0].second;
                // if(area_max>640*480/180)
                {
                    // 寻找 plane_id
                    for(int i=0;i<vpPlanes.size();i++)
                    {
                        if(vpPlanes[i] == pBackingPlane_best)
                            back_plane_id = i;
                    }
                    
                    // 保存该组关系
                    Relation rl;
                    rl.obj_id = obj_id;
                    rl.plane_id = back_plane_id;
                    rl.type = RELATION_TYPE::BACKING;
                    rl.pPlane = pBackingPlane_best;
                    rl.pEllipsoid = pEllip;
                    // rl.pFrame = pFrame;
                    relations_return.push_back(rl);
                    // pEllip->mRelations.push_back(rl);

                    // 存入到frame中的local椭球体中
                    g2o::ConstrainPlane* mhp = new g2o::ConstrainPlane(pBackingPlane_best);
                    mhp->type = CONSTRAINPLANE_STATE::BACKING; 
                    pEllip->mpBackingPlane = mhp;
                    pEllip->mbBackingPlaneDefined = true;

                    // 存入到KeyFrame的global椭球体中
                    g2o::plane* p_global = new g2o::plane(*pBackingPlane_best);
                    p_global->transform(pKF->cam_pose_Twc);
                    g2o::ConstrainPlane* mhp_global = new g2o::ConstrainPlane(p_global);
                    mhp_global->type = CONSTRAINPLANE_STATE::BACKING; 
                    mvpKeyframeGlobalEllipsolds[obj_id]->mpBackingPlane = mhp_global;
                    mvpKeyframeGlobalEllipsolds[obj_id]->mbBackingPlaneDefined = true;
                }
            }
            
        }
        
        return relations_return;        
    }


    
    // 新函数，仅仅提取支撑关系
    // 要求传入平面: 满足曼哈顿假设, 且与地平面平行.
    Relations RelationExtractor::ExtractSupporttingRelations(std::vector<g2o::ellipsoid *> &vpEllips, std::vector<g2o::plane *> &vpPlanes, Frame* pFrame, int model)
    {
        Relations relations;
        int obj_num = vpEllips.size();
        for (int obj_id = 0; obj_id < obj_num; obj_id++)
        {
            g2o::ellipsoid *pEllip = vpEllips[obj_id];
            if(pEllip == NULL ) {
                // std::cout << "[Relation] NULL ellipsoid." << std::endl;
                continue;
            }

            bool success = false;
            int sup_plane_id=-1;
            g2o::plane* pPlane_best = NULL;
            if(model == 1){
                Matrix3Xd mCorners;  mCorners.resize(3,8);
                Matrix3Xd mIds; mIds.resize(3, 6);
                std::vector<g2o::plane*> obj_planes = pEllip->GetCubePlanesGlobal(mCorners);  // 椭球体所在的坐标系
                g2o::plane* pObj_bottom_plane = obj_planes[0];

                int plane_num = vpPlanes.size();
                
                // 计算所有平面与某物体距离, 取最近小于阈值平面.
                std::vector<std::pair<double, g2o::plane*>> planeDisVec;
                for (int plane_id = 0; plane_id < plane_num; plane_id++)
                {
                    g2o::plane *pPlane = vpPlanes[plane_id];
                    if(pPlane->miMHType==g2o::MANHATTAN_PLANE_TYPE::HORIZONTAL){
                        double dis = pObj_bottom_plane->distanceToPlane(*pPlane);
                        if(dis>0)   // 即平面不能在物体底面之上
                            planeDisVec.push_back(make_pair(dis, pPlane));
                    }
                }
                if(planeDisVec.size()<1) continue;
                // 获得最近距离平面.
                sort(planeDisVec.begin(), planeDisVec.end(), sort_plane_dis);
                
                double dis_min = planeDisVec[0].first;
                pPlane_best = planeDisVec[0].second;
                
                if( dis_min < 0.3 ) success = true;
            }
            else if( model == 0 )
            {
                // ***************** 尚未完成 *****************
                // ******************************************
                //        获得到中心点最近的平面, 且要求沿着重力方向. //
                // 配置
                double config_pointmodel_dis_thresh = 2;    // 暂时设2m, 形同虚设; 其实应该跟物体体积有关.

                // 若属于点模型，使用中点判断距离
                int plane_num = vpPlanes.size();
                Eigen::Vector3d center = pEllip->pose.translation();
                
                // 计算所有平面与某物体[中心]距离 (带方向), 取最近小于阈值平面.
                std::vector<std::pair<double, g2o::plane*>> planeDisVec;
                for (int plane_id = 0; plane_id < plane_num; plane_id++)
                {
                    g2o::plane *pPlane = vpPlanes[plane_id];
                    if(pPlane->miMHType==g2o::MANHATTAN_PLANE_TYPE::HORIZONTAL){
                        double dis = pPlane->distanceToPoint(center, true);
                        if(dis>0 && dis < config_pointmodel_dis_thresh)   // 位于平面上方
                            planeDisVec.push_back(make_pair(dis, pPlane));
                    }
                }
                if(planeDisVec.size()<1) continue;
                // 获得最近距离平面.
                sort(planeDisVec.begin(), planeDisVec.end(), sort_plane_dis);
                success = true;
                
                double dis_min = planeDisVec[0].first;
                pPlane_best = planeDisVec[0].second;
            }

            if(success)
            {
                // 寻找 plane_id
                for(int i=0;i<vpPlanes.size();i++)
                {
                    if(vpPlanes[i] == pPlane_best)
                        sup_plane_id = i;
                }
                
                // 保存该组关系
                Relation rl;
                rl.obj_id = obj_id;
                rl.plane_id = sup_plane_id;
                rl.type = RELATION_TYPE::SUPPORTING;     // Type id : {0: no Relation ;  1 : Supporting ;  2 : LeanOn}
                rl.pPlane = pPlane_best;
                rl.pEllipsoid = pEllip;
                rl.pFrame = pFrame;
                relations.push_back(rl);
            }
        }
        return relations;        
    }

        // int obj_id; // 索引: 与localEllipsoids, measurements同索引.
        // int plane_instance_id;  // 关联 SupportingPlanes 的 id
        // int plane_id;
        // g2o::plane* pPlane;
        // g2o::ellipsoid* pEllipsoid;
        // Frame* pFrame;
        // int type; // 无效关系0, 支撑关系 1, 倚靠关系 2.

    Eigen::VectorXd Relation::SaveToVec()
    {
        Eigen::VectorXd vec;
        vec.resize(GetDataNum());

        vec << double(obj_id), double(plane_instance_id), double(plane_id), double(type), 
                pPlane->param;

        // 保存平面!

        return vec;
    }

    bool Relation::InitRelation(const Eigen::VectorXd& vec, ORB_SLAM2::Frame* pFrameIn)
    {
        // 根据 obj_id, plane_id 初始化 pPlane, pEllipsoid 的值.

        this->pFrame = pFrameIn;
        LoadFromVec(vec);

        if(pFrameIn->mpLocalObjects.size() <= obj_id)
        {
            std::cout << "Wrong size of mpLocalObjects in pFrameIn." << std::endl;
            std::cout << "Objects in frame : " << pFrameIn->mpLocalObjects.size() << std::endl;
            std::cout << "obj_id : " << obj_id << std::endl;
            std::cout << "vec : " << vec.transpose() << std::endl;
            std::cout << "timestamp of frame : " << std::to_string(pFrameIn->mTimeStamp) << std::endl;
            return false;
        }

        this->pEllipsoid = pFrameIn->mpLocalObjects[obj_id];

        return true;
    }

    void Relation::LoadFromVec(const Eigen::VectorXd& vec)
    {
        // if(vec.size()!=GetDataNum()){
        //     std::cerr << "Wrong size of vec to load." << std::endl;
        // }
        // obj_id = round(vec[0]);
        // plane_instance_id = round(vec[1]);
        // plane_id = round(vec[2]);
        // type = round(vec[3]);

        // Vector4d planeVec = vec.head(8).tail(4);    // 4-8
        // g2o::plane* pPlaneConstruct = new g2o::plane(planeVec);
        // pPlane = pPlaneConstruct;

        return;
    }

    int Relation::GetDataNum()
    {
        // planevec : 4
        return 4 + 4;
    }

} // namespace ORB_SLAM2