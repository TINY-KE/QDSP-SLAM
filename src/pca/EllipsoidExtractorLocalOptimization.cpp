#include "EllipsoidExtractor.h"
#include "EllipsoidExtractorEdges.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"


namespace ORB_SLAM2
{

g2o::ellipsoid EllipsoidExtractor::OptimizeEllipsoidUsingPlanes(g2o::ellipsoid &e_in, MatrixXd& mPlanesParam)
{
    // 此处使用图优化, 与各个平面做优化.
    // 迭代过程中，固定其中一个转角轴? 即垂直方向解耦合.?
    // 先不管细节优化，先实现功能. 即，重力方向约束+各平面的联合优化.
    // 立方体也直接转为平面. - > 部分平面是选择性的: 遮挡部分去除.
    
    // initialize graph optimization.
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);
    graph.setVerbose(false);        // Set output.

    // Add objects vertices
    g2o::VertexEllipsoidXYZABC *vEllipsoid = new g2o::VertexEllipsoidXYZABC();
    vEllipsoid->setEstimate(e_in);
    vEllipsoid->setId(graph.vertices().size());
    vEllipsoid->setFixed(false);
    graph.addVertex(vEllipsoid);    

    // // Add gravity prior
    // if(mbGroundPlaneSet && mbSetGravityPrior ){
    //     g2o::EdgeEllipsoidGravityPlanePrior *vGravityPriorEdge = new g2o::EdgeEllipsoidGravityPlanePrior;
    //     vGravityPriorEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));

    //     vGravityPriorEdge->setMeasurement(mGroundPlaneNormal);  
    //     Matrix<double,1,1> inv_sigma;
    //     inv_sigma << 1 * dGravityPriorScale;
    //     MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
    //     vGravityPriorEdge->setInformation(info);
        
    //     graph.addEdge(vGravityPriorEdge);
    //     edgesEllipsoidGravityPlanePrior.push_back(vGravityPriorEdge);
        
    // }

    // Add edges from planes

    // 需要创建新的边. 即 3d 约束 plane-ellipsoid
    int plane_num = mPlanesParam.rows();
    for( int i=0;i<plane_num;i++)
    {
        Vector4d planeVec = mPlanesParam.row(i).head(4);
        g2o::EdgeEllipsoidPlane* pEdge = new g2o::EdgeEllipsoidPlane;
        pEdge->setId(graph.edges().size());
        pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
        pEdge->setMeasurement(planeVec);

        Matrix<double,1,1> inv_sigma;
        inv_sigma << 1;
        MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
        pEdge->setInformation(info);

        graph.addEdge(pEdge);
    }

    // graph.initializeOptimization();
    // graph.optimize( 10 );  //optimization step
    // DEBUG: CLOSE IT!!!!
    std::cout << "[ATTENTION LOCALOPTIMIZATION] CLOSED." << std::endl;

    g2o::ellipsoid e_optimized = vEllipsoid->estimate();

    return e_optimized;
}


g2o::ellipsoid EllipsoidExtractor::OptimizeEllipsoidWithBboxPlanesAndMHPlanes(const g2o::ellipsoid &init_guess, 
                                                                                std::vector<g2o::plane> &BboxPlanes, double Bbox_Weight, 
                                                                                g2o::plane &SupprotingPlane, double Supproting_Weight,
                                                                                g2o::plane &BackingPlane, double Backing_Weight)
{
    // 基本参数的读取
    double config_plane_angle_sigma = Config::Get<double>("EllipsoidExtractor.Optimizer.PlaneNormalAngle");
    bool bUseGroundPlaneWeight = true;  // 请将地平面放在平面约束的第一个！

    // initialize graph optimization.
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);
    graph.setVerbose(false); // Set output.

    // 添加椭球体节点
    // Add objects vertices
    g2o::VertexEllipsoidXYZABCYaw *vEllipsoid;
    // 注意单目版本和非单目版本区别
    // if(bMonocularVersion)
    //     vEllipsoid = new g2o::VertexEllipsoidXYABHeightYaw();
    // else 
    //     vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
    vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
    vEllipsoid->setEstimate(init_guess);
    vEllipsoid->setId(graph.vertices().size());
    vEllipsoid->setFixed(false);
    graph.addVertex(vEllipsoid);

    // 添加相机位姿节点（直接设置为Identity，且不参与优化）
    // 这里的平面与椭球体已经位于同一个坐标系，所以创建一个单位变换作为SE3
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setId(graph.vertices().size());
    vSE3->setEstimate(g2o::SE3Quat()); // Identity
    vSE3->setFixed(true);
    graph.addVertex(vSE3);


    // 等会儿，是否对朝向做估计？ 要不只估计 x,y,z,a,b,c



    // 1) 无normal 边约束
    for (int i = 0; i < BboxPlanes.size(); i++)
    {
        g2o::EdgeSE3EllipsoidPlane *pEdge = new g2o::EdgeSE3EllipsoidPlane;
        pEdge->setId(graph.edges().size());
        pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vSE3));
        pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vEllipsoid));
        pEdge->setMeasurement(BboxPlanes[i].param);

        pEdge->setNormalDirection(true);   // 要求被约束的椭球体在平面的法向量方向，防止其出现在地面下方或相机后方。

        double pl_weight = 1;
        pl_weight = Bbox_Weight;
        Matrix<double, 1, 1> inv_sigma;
        inv_sigma << 1 * pl_weight;
        MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
        pEdge->setInformation(info);
        pEdge->setRobustKernel(new g2o::RobustKernelHuber());

        graph.addEdge(pEdge);
    }
    



    // 2) 支撑面 normal 边 ( 约束其朝向 ). 
    int flag_valid_angle = 1;  // 关闭边约束的朝向!

    g2o::EdgeSE3EllipsoidPlaneWithNormal* pEdge_sup = new g2o::EdgeSE3EllipsoidPlaneWithNormal;
    pEdge_sup->setId(graph.edges().size());
    pEdge_sup->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
    pEdge_sup->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
    pEdge_sup->setMeasurement(SupprotingPlane.param);

    Matrix<double,2,1> inv_sigma;
    inv_sigma << 1, 1/(config_plane_angle_sigma * 1 / 180.0 * M_PI) * flag_valid_angle;   // 距离, 角度标准差 ; 暂时不管

    inv_sigma = inv_sigma * Supproting_Weight;
    MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
    pEdge_sup->setInformation(info);
    pEdge_sup->setRobustKernel( new g2o::RobustKernelHuber() );

    graph.addEdge(pEdge_sup);




    // 3) 倚靠面 normal 边 ( 约束其朝向 ). 
    int flag_valid_angle_back = 1;  // 关闭边约束的朝向!

    g2o::EdgeSE3EllipsoidPlaneWithNormal* pEdge_back = new g2o::EdgeSE3EllipsoidPlaneWithNormal;
    pEdge_back->setId(graph.edges().size());
    pEdge_back->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
    pEdge_back->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
    pEdge_back->setMeasurement(BackingPlane.param);

    Matrix<double,2,1> inv_sigma_back;
    inv_sigma_back << 1, 1/(config_plane_angle_sigma * 1 / 180.0 * M_PI) * flag_valid_angle_back;   // 距离, 角度标准差 ; 暂时不管

    inv_sigma_back = inv_sigma_back * Backing_Weight;
    MatrixXd info_back = inv_sigma_back.cwiseProduct(inv_sigma_back).asDiagonal();
    pEdge_back->setInformation(info_back);
    pEdge_back->setRobustKernel( new g2o::RobustKernelHuber() );

    graph.addEdge(pEdge_back);




    
    // 开始优化
    int num_opt = 10;
    // std::cout << "Begin Optimization of ellipsoid with prior... x " << num_opt << std::endl;
    graph.initializeOptimization();
    graph.optimize( num_opt );  //optimization step
    // std::cout << "Optimization done." << std::endl;

    // // 保存最终 cost 
    // mdCost = graph.chi2();

    return vEllipsoid->estimate();
}



g2o::ellipsoid EllipsoidExtractor::OptimizeEllipsoidWithBboxPlanesAndMHPlanes(const g2o::ellipsoid &init_guess, 
                                                                                std::vector<g2o::plane> &BboxPlanes, double Bbox_Weight, 
                                                                                g2o::plane &SupprotingPlane, double Supproting_Weight)
{
    // 基本参数的读取
    double config_plane_angle_sigma = Config::Get<double>("EllipsoidExtractor.Optimizer.PlaneNormalAngle");
    bool bUseGroundPlaneWeight = true;  // 请将地平面放在平面约束的第一个！

    // initialize graph optimization.
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);
    graph.setVerbose(false); // Set output.

    // 添加椭球体节点
    // Add objects vertices
    g2o::VertexEllipsoidXYZABCYaw *vEllipsoid;
    // 注意单目版本和非单目版本区别
    // if(bMonocularVersion)
    //     vEllipsoid = new g2o::VertexEllipsoidXYABHeightYaw();
    // else 
    //     vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
    vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
    vEllipsoid->setEstimate(init_guess);
    vEllipsoid->setId(graph.vertices().size());
    vEllipsoid->setFixed(false);
    graph.addVertex(vEllipsoid);

    // 添加相机位姿节点（直接设置为Identity，且不参与优化）
    // 这里的平面与椭球体已经位于同一个坐标系，所以创建一个单位变换作为SE3
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setId(graph.vertices().size());
    vSE3->setEstimate(g2o::SE3Quat()); // Identity
    vSE3->setFixed(true);
    graph.addVertex(vSE3);


    // 等会儿，是否对朝向做估计？ 要不只估计 x,y,z,a,b,c



    // 1) 无normal 边约束
    for (int i = 0; i < BboxPlanes.size(); i++)
    {
        g2o::EdgeSE3EllipsoidPlane *pEdge = new g2o::EdgeSE3EllipsoidPlane;
        pEdge->setId(graph.edges().size());
        pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vSE3));
        pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vEllipsoid));
        pEdge->setMeasurement(BboxPlanes[i].param);

        pEdge->setNormalDirection(true);   // 要求被约束的椭球体在平面的法向量方向，防止其出现在地面下方或相机后方。

        double pl_weight = 1;
        pl_weight = Bbox_Weight;
        Matrix<double, 1, 1> inv_sigma;
        inv_sigma << 1 * pl_weight;
        MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
        pEdge->setInformation(info);
        pEdge->setRobustKernel(new g2o::RobustKernelHuber());

        graph.addEdge(pEdge);
    }
    



    // 2) 支撑面 normal 边 ( 约束其朝向 ). 
    int flag_valid_angle = 1;  // 关闭边约束的朝向!

    g2o::EdgeSE3EllipsoidPlaneWithNormal* pEdge_sup = new g2o::EdgeSE3EllipsoidPlaneWithNormal;
    pEdge_sup->setId(graph.edges().size());
    pEdge_sup->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
    pEdge_sup->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
    pEdge_sup->setMeasurement(SupprotingPlane.param);

    Matrix<double,2,1> inv_sigma;
    inv_sigma << 1, 1/(config_plane_angle_sigma * 1 / 180.0 * M_PI) * flag_valid_angle;   // 距离, 角度标准差 ; 暂时不管

    inv_sigma = inv_sigma * Supproting_Weight;
    MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
    pEdge_sup->setInformation(info);
    pEdge_sup->setRobustKernel( new g2o::RobustKernelHuber() );

    graph.addEdge(pEdge_sup);




    
    // 开始优化
    int num_opt = 10;
    // std::cout << "Begin Optimization of ellipsoid with prior... x " << num_opt << std::endl;
    graph.initializeOptimization();
    graph.optimize( num_opt );  //optimization step
    // std::cout << "Optimization done." << std::endl;

    // // 保存最终 cost 
    // mdCost = graph.chi2();

    return vEllipsoid->estimate();
}

g2o::ellipsoid EllipsoidExtractor::OptimizeEllipsoidWithMHPlanes(const g2o::ellipsoid &init_guess, int num_opt, 
                                                                                g2o::plane &SupprotingPlane, double Supproting_Weight,
                                                                                g2o::plane &BackingPlane, double Backing_Weight)
{
    // 基本参数的读取
    double config_plane_angle_sigma = Config::Get<double>("EllipsoidExtractor.Optimizer.PlaneNormalAngle");
    bool bUseGroundPlaneWeight = true;  // 请将地平面放在平面约束的第一个！

    // initialize graph optimization.
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);
    graph.setVerbose(false); // Set output.

    // 添加椭球体节点
    // Add objects vertices
    g2o::VertexEllipsoidXYZABCYaw *vEllipsoid;
    // 注意单目版本和非单目版本区别
    // if(bMonocularVersion)
    //     vEllipsoid = new g2o::VertexEllipsoidXYABHeightYaw();
    // else 
    //     vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
    vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
    vEllipsoid->setEstimate(init_guess);
    vEllipsoid->setId(graph.vertices().size());
    vEllipsoid->setFixed(false);
    graph.addVertex(vEllipsoid);

    // 添加相机位姿节点（直接设置为Identity，且不参与优化）
    // 这里的平面与椭球体已经位于同一个坐标系，所以创建一个单位变换作为SE3
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setId(graph.vertices().size());
    vSE3->setEstimate(g2o::SE3Quat()); // Identity
    vSE3->setFixed(true);
    graph.addVertex(vSE3);


    // 等会儿，是否对朝向做估计？ 要不只估计 x,y,z,a,b,c


    



    // 2) 支撑面 normal 边 ( 约束其朝向 ). 
    int flag_valid_angle = 1;  // 关闭边约束的朝向!

    g2o::EdgeSE3EllipsoidPlaneWithNormal* pEdge_sup = new g2o::EdgeSE3EllipsoidPlaneWithNormal;
    pEdge_sup->setId(graph.edges().size());
    pEdge_sup->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
    pEdge_sup->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
    pEdge_sup->setMeasurement(SupprotingPlane.param);

    Matrix<double,2,1> inv_sigma;
    inv_sigma << 1, 1/(config_plane_angle_sigma * 1 / 180.0 * M_PI) * flag_valid_angle;   // 距离, 角度标准差 ; 暂时不管

    inv_sigma = inv_sigma * Supproting_Weight;
    MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
    pEdge_sup->setInformation(info);
    pEdge_sup->setRobustKernel( new g2o::RobustKernelHuber() );

    graph.addEdge(pEdge_sup);




    // 3) 倚靠面 normal 边 ( 约束其朝向 ). 
    int flag_valid_angle_back = 1;  // 关闭边约束的朝向!

    g2o::EdgeSE3EllipsoidPlaneWithNormal* pEdge_back = new g2o::EdgeSE3EllipsoidPlaneWithNormal;
    pEdge_back->setId(graph.edges().size());
    pEdge_back->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
    pEdge_back->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
    pEdge_back->setMeasurement(BackingPlane.param);

    Matrix<double,2,1> inv_sigma_back;
    inv_sigma_back << 1, 1/(config_plane_angle_sigma * 1 / 180.0 * M_PI) * flag_valid_angle_back;   // 距离, 角度标准差 ; 暂时不管

    inv_sigma_back = inv_sigma_back * Backing_Weight;
    MatrixXd info_back = inv_sigma_back.cwiseProduct(inv_sigma_back).asDiagonal();
    pEdge_back->setInformation(info_back);
    pEdge_back->setRobustKernel( new g2o::RobustKernelHuber() );

    graph.addEdge(pEdge_back);




    
    // 开始优化
    // std::cout << "Begin Optimization of ellipsoid with prior... x " << num_opt << std::endl;
    graph.initializeOptimization();
    graph.optimize( num_opt );  //optimization step
    // std::cout << "Optimization done." << std::endl;

    // // 保存最终 cost 
    // mdCost = graph.chi2();

    return vEllipsoid->estimate();
}

g2o::ellipsoid EllipsoidExtractor::OptimizeEllipsoidWithSupportingPlanes(const g2o::ellipsoid &init_guess,   int num_opt,
                                                                                g2o::plane &SupprotingPlane, double Supproting_Weight)
{
    // 基本参数的读取
    double config_plane_angle_sigma = Config::Get<double>("EllipsoidExtractor.Optimizer.PlaneNormalAngle");
    bool bUseGroundPlaneWeight = true;  // 请将地平面放在平面约束的第一个！

    // initialize graph optimization.
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);
    graph.setVerbose(false); // Set output.

    // 添加椭球体节点
    // Add objects vertices
    g2o::VertexEllipsoidXYZABCYaw *vEllipsoid;
    // 注意单目版本和非单目版本区别
    // if(bMonocularVersion)
    //     vEllipsoid = new g2o::VertexEllipsoidXYABHeightYaw();
    // else 
    //     vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
    vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
    vEllipsoid->setEstimate(init_guess);
    vEllipsoid->setId(graph.vertices().size());
    vEllipsoid->setFixed(false);
    graph.addVertex(vEllipsoid);

    // 添加相机位姿节点（直接设置为Identity，且不参与优化）
    // 这里的平面与椭球体已经位于同一个坐标系，所以创建一个单位变换作为SE3
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setId(graph.vertices().size());
    vSE3->setEstimate(g2o::SE3Quat()); // Identity
    vSE3->setFixed(true);
    graph.addVertex(vSE3);


    // 等会儿，是否对朝向做估计？ 要不只估计 x,y,z,a,b,c


    



    // 2) 支撑面 normal 边 ( 约束其朝向 ). 
    int flag_valid_angle = 1;  // 关闭边约束的朝向!

    g2o::EdgeSE3EllipsoidPlaneWithNormal* pEdge_sup = new g2o::EdgeSE3EllipsoidPlaneWithNormal;
    pEdge_sup->setId(graph.edges().size());
    pEdge_sup->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
    pEdge_sup->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
    pEdge_sup->setMeasurement(SupprotingPlane.param);

    Matrix<double,2,1> inv_sigma;
    inv_sigma << 1, 1/(config_plane_angle_sigma * 1 / 180.0 * M_PI) * flag_valid_angle;   // 距离, 角度标准差 ; 暂时不管

    inv_sigma = inv_sigma * Supproting_Weight;
    MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
    pEdge_sup->setInformation(info);
    pEdge_sup->setRobustKernel( new g2o::RobustKernelHuber() );

    graph.addEdge(pEdge_sup);





    
    // 开始优化
    // std::cout << "Begin Optimization of ellipsoid with prior... x " << num_opt << std::endl;
    graph.initializeOptimization();
    graph.optimize( num_opt );  //optimization step
    // std::cout << "Optimization done." << std::endl;

    // // 保存最终 cost 
    // mdCost = graph.chi2();

    return vEllipsoid->estimate();
}

}