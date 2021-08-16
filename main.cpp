#include <iostream>
#include "tools/load_cdv.h"
#include "tools/UWBtools.h"
#include "modules/LocVertex.h"
#include "modules/MpcVertex.h"
#include "modules/LocUnaryEdge.h"
#include "modules/MpcBinaryEdge.h"
#include "modules/LocMoveBinaryEdge.h"
#include "Eigen/Dense"
#include "g2o//core/block_solver.h"
#include "cs.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <chrono>

using namespace std;
using namespace Eigen;

int main() {
    MatrixXd result = load_csv<MatrixXd>("/home/DataBase/20210413_indoor/move_02.csv");
    int numDatas = result.rows();
    int windowSize = 100;
    double sigma = 0.5;
    double los_sigma = 1;
    double mpc_sigma = 1;
    double move_sigma = 0.1;


/* 开始定义图模型 */
    typedef g2o::BlockSolverX Block;
    std::unique_ptr<Block::LinearSolverType> linearSolver (new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
    g2o::OptimizationAlgorithmDogleg * solver = new g2o::OptimizationAlgorithmDogleg(std::move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);


/* 图模型初始化 */

    /* 添加位置节点 */
    LocVertex* LocVerLists[numDatas];
    LocVertex* LocVerLists_back[numDatas];
    for(int i = 0;i<windowSize;i++)
    {
        LocVerLists[i] = new LocVertex();
        LocVerLists[i]->setEstimate(Eigen::Vector4d(result(i,0)*cos(result(i,1)),result(i,0)*sin(result(i,1)),0,0));
        LocVerLists[i]->setId(i);
        optimizer.addVertex(LocVerLists[i]);
    }

    /* 添加多径节点 */
    MpcVertex* MpcVer = new MpcVertex();
    MpcVer->setEstimate(Eigen::Vector2d(2,1));
    MpcVer->setId(numDatas);
    optimizer.addVertex(MpcVer);

    /* 添加位置观测一元边 */

    LocUnaryEdge* LocEdgeLists[numDatas];
    for (int i = 0;i<windowSize;i++)
    {
        LocEdgeLists[i] = new LocUnaryEdge();
        LocEdgeLists[i]->setId(i);    //设置 ID
        LocEdgeLists[i]->setVertex(0,LocVerLists[i]);
        Eigen::Vector2d thisLocMeasurement = result.block<1,2>(i,0);
        LocEdgeLists[i]->setMeasurement(result.block<1,2>(i,0));
        LocEdgeLists[i]->setInformation(Eigen::Matrix<double,2,2>::Identity()*1/(los_sigma * los_sigma));
        optimizer.addEdge(LocEdgeLists[i]);
    }

    /* 添加多径观测二元边 */
    MpcBinaryEdge* MpcEdgeLists[numDatas];
    for(int i = 0;i<windowSize; i++)
    {
        int IdIndex = i + numDatas;
        MpcEdgeLists[i] = new MpcBinaryEdge();
        MpcEdgeLists[i]->setId(IdIndex);
        MpcEdgeLists[i]->setVertex(0,LocVerLists[i]);
        MpcEdgeLists[i]->setVertex(1,MpcVer);
        Eigen::Vector2d thisMpcMeasurement = result.block<1,2>(i,2);
        MpcEdgeLists[i]->setMeasurement(result.block<1,2>(i,2));
        MpcEdgeLists[i]->setInformation(Eigen::Matrix<double,2,2>::Identity()*1/(mpc_sigma *mpc_sigma));
        optimizer.addEdge(MpcEdgeLists[i]);
    }
/* 添加姿态观测二元边 */
    LocMoveBinaryEdge* LocMoveEdgesLists[numDatas - 1];
    for(int i = 0;i<(windowSize-1);i++)
    {
        int IDIndex = i + 2* numDatas;
        LocMoveEdgesLists[i] = new LocMoveBinaryEdge();
        LocMoveEdgesLists[i]->setId(IDIndex);
        LocMoveEdgesLists[i]->setVertex(0,LocVerLists[i]);
        LocMoveEdgesLists[i]->setVertex(1,LocVerLists[i+1]);
        LocMoveEdgesLists[i]->setMeasurement(Eigen::Vector4d::Zero());
        LocMoveEdgesLists[i]->setInformation(Eigen::Matrix<double,4,4>::Identity() * 1/(move_sigma * move_sigma));
        optimizer.addEdge(LocMoveEdgesLists[i]);
    }

/* 执行第一次优化 */
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    for (int i = 0;i<windowSize;i++)
    {
        LocVerLists_back[i] = new LocVertex();
        LocVerLists_back[i]->setEstimate(LocVerLists[i]->estimate());
    }
/* 进行下一次更新迭代 */
    for (int i = windowSize;i<numDatas;i++)
    {
        int removeIndex = i - windowSize;
        optimizer.removeVertex(LocVerLists[removeIndex], true);
        //这里不知道是否需要排除相应的边，因此暂且不排除
        LocVerLists[i] = new LocVertex();
        LocVerLists[i]->setEstimate(Eigen::Vector4d(result(i,0)*cos(result(i,1)),result(i,0)*sin(result(i,1)),0,0));
        LocVerLists[i]->setId(i);
        optimizer.addVertex(LocVerLists[i]);

        //开始增加位置单向边
        LocEdgeLists[i] = new LocUnaryEdge();
        LocEdgeLists[i]->setId(i);
        LocEdgeLists[i]->setVertex(0,LocVerLists[i]);
        LocEdgeLists[i]->setMeasurement(result.block<1,2>(i,0));
        LocEdgeLists[i]->setInformation(Eigen::Matrix<double,2,2>::Identity()*1/(los_sigma * los_sigma));
        optimizer.addEdge(LocEdgeLists[i]);

        //开始增加多径观测双向边
        int mpcIdIndex = i + numDatas;
        MpcEdgeLists[i] = new MpcBinaryEdge();
        MpcEdgeLists[i]->setId(mpcIdIndex);
        MpcEdgeLists[i]->setVertex(0,LocVerLists[i]);
        MpcEdgeLists[i]->setVertex(1,MpcVer);
        MpcEdgeLists[i]->setMeasurement(result.block<1,2>(i,2));
        MpcEdgeLists[i]->setInformation(Eigen::Matrix<double,2,2>::Identity()*1/(mpc_sigma *mpc_sigma));
        optimizer.addEdge(MpcEdgeLists[i]);

        //开始增加运动姿态边
        int moveIdIndex = i + 2 * numDatas;
        LocMoveEdgesLists[i] = new LocMoveBinaryEdge();
        LocMoveEdgesLists[i]->setId(moveIdIndex);
        LocMoveEdgesLists[i]->setVertex(0,LocVerLists[i-1]);
        LocMoveEdgesLists[i]->setVertex(1,LocVerLists[i]);
        LocMoveEdgesLists[i]->setMeasurement(Eigen::Vector4d::Zero());
        LocMoveEdgesLists[i]->setInformation(Eigen::Matrix<double,4,4>::Identity() * 1/(move_sigma * move_sigma));
        optimizer.addEdge(LocMoveEdgesLists[i]);

        optimizer.initializeOptimization();
        optimizer.optimize(20);
        LocVerLists_back[i] = new LocVertex();
        LocVerLists_back[i]->setEstimate(LocVerLists[i]->estimate());
        cout<<"now Vertex Index is"<<i<<endl;
    }


/* 将优化结果转存在一个矩阵中 */
    Eigen::MatrixXd afterResult(numDatas,2);
    for (int i = 0;i<numDatas;i++)
    {
        afterResult.row(i) = LocVerLists_back[i]->estimate();
    }

    MatToCSV(afterResult, "move_02.csv");

    cout<<"mpc information "<<MpcVer->estimate()<<endl;
    return 0;
}
