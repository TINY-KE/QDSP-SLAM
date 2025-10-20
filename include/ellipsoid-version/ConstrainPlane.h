// 约束平面由bbox和Depth矩形框产生, 属于 plane 的装饰对象
#ifndef ELLIPSOIDSLAM_CONSTRAINPLANE_H
#define ELLIPSOIDSLAM_CONSTRAINPLANE_H

#include <Eigen/Core>

namespace g2o
{
    enum CONSTRAINPLANE_STATE
    {
        INVALID = -1,
        BBOXPLANE = 0,
        CUBOIDS = 1,
        SUPPORTING = 2,
        BACKING = 3
    };

    class plane;
    class ConstrainPlane
    {
    public:
        ConstrainPlane(plane* ppl);
        // 特有属性
        bool valid; // 最终flag : 若属于边界, 则无效; 不再参与关联与优化
        bool image_border;       // 是否位于图像边缘
        bool association_border; // 是否在关联时被判断在边缘
        int state;  // invalid 时, 两种state: 1, 内部  2, 外部

        CONSTRAINPLANE_STATE type;  

        plane* pPlane;

        // load and save
        Eigen::VectorXd toVector();
        void fromVector(Eigen::VectorXd& vec);
        static int vectorSize();

    };

} // namespace ORB_SLAM2
#endif