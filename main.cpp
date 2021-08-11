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
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"


using namespace std;
using namespace Eigen;

int main() {
    MatrixXd result = load_csv<MatrixXd>("/home/DataBase/20210413_indoor/move_02.csv");
    int numDatas = result.rows();
    double sigma = 0.5;
    double los_sigma = 0.8;
    double mpc_sigma = 0.3;
    double move_sigma = 0.1;


/* 开始定义图模型 */
    typedef g2o::BlockSolverX Block;
    std::unique_ptr<Block::LinearSolverType> linearSolver (new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
    g2o::OptimizationAlgorithmDogleg * solver = new g2o::OptimizationAlgorithmDogleg(std::move(solver_ptr));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

/* 添加位置节点 */
    LocVertex* LocVerLists[numDatas];
    for(int i = 0;i<numDatas;i++)
    {
        LocVerLists[i] = new LocVertex();
        LocVerLists[i]->setEstimate(Eigen::Vector4d(result(i,0)*cos(result(i,1)),result(i,0)*sin(result(i,1)),0,0));
        LocVerLists[i]->setId(i);
        optimizer.addVertex(LocVerLists[i]);
    }

/* 添加多径节点 */
    MpcVertex* MpcVer = new MpcVertex();
    MpcVer->setEstimate(Eigen::Vector2d(4,0));
    MpcVer->setId(numDatas);
    optimizer.addVertex(MpcVer);

/* 添加位置观测一元边 */

    LocUnaryEdge* LocEdgeLists[numDatas];
    for (int i = 0;i<numDatas;i++)
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
    for(int i = 0;i<numDatas; i++)
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
    for(int i = 0;i<(numDatas-1);i++)
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



/* 执行优化 */
    optimizer.initializeOptimization();
    optimizer.optimize(100);


/* 将优化结果转存在一个矩阵中 */
    Eigen::MatrixXd afterResult(numDatas,2);
    for (int i = 0;i<numDatas;i++)
    {
        afterResult.row(i) = LocVerLists[i]->estimate();
    }

    MatToCSV(afterResult, "move_02.csv");
    return 0;

}
