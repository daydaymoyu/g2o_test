//
// Created by xiangqian on 18-5-27.
//
#include <iostream>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"

G2O_USE_TYPE_GROUP(slam3d);

#define MAXITERATION 50

using namespace std;
using namespace g2o;

int main() {

    cout << "-----------g2o_test---------" << endl;

    BlockSolverX::LinearSolverType *linearSolver = new LinearSolverCSparse<BlockSolverX::PoseMatrixType>();
    BlockSolverX *blockSolver = new BlockSolverX(linearSolver);

    OptimizationAlgorithmLevenberg *optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);

    SparseOptimizer optimizer;

    if (!optimizer.load("../sphere_bignoise_vertex3.g2o")) {
        cout << "error loading graph" << endl;
        return -1;
    } else {
        cout << optimizer.vertices().size() << " vertices" << endl;
        cout << optimizer.edges().size() << " edges" << endl;
    }

    VertexSE3* firstPose = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
    firstPose->setFixed(true);

    optimizer.setAlgorithm(optimizationAlgorithm);
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    cerr<<"optimizing..."<<endl;
    optimizer.optimize(MAXITERATION);
    cerr<<"done."<<endl;

    optimizer.save("../sphere_optimized.g2o");

    return 0;
}