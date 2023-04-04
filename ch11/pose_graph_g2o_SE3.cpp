#include <iostream>
#include <fstream>
#include <string>

#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/slam3d/types_slam3d.h>

using namespace std;

int main(int argc, char** argv)
{
    if ( argc != 2)
    {
        cout << "Usage: pose_graph_g2o_SE3 sphere.g2o" << endl;
        return 1;
    }
    ifstream fin( argv[1]);
    if ( !fin)
    {
        cout << "file " << argv[1] << " does not exits." << endl;
        return 1;
    }

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> BlockSolver_6_6;
    std::unique_ptr<BlockSolver_6_6::LinearSolverType> linearSolver;
    linearSolver = std::make_unique<g2o::LinearSolverCholmod<BlockSolver_6_6::PoseMatrixType>>();
    // BlockSolver_6_6::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<BlockSolver_6_6::PoseMatrixType>();
    // BlockSolver_6_6* solver_ptr = new BlockSolver_6_6(linearSolver);
    // choose from GM, LM, DOGLEG
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolver_6_6>(std::move(linearSolver)));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver);

    int vertexCnt = 0, edgeCnt = 0;
    while ( !fin.eof())
    {
        string name;
        fin >> name;
        if ( name == "VERTEX_SE3:QUAT")
        {
            g2o::VertexSE3* v = new g2o::VertexSE3();
            int index = 0;
            fin >> index;
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            if(index == 0)
                v->setFixed(true);
        }
        else if( name == "EDGE_SE3:QUAT")
        {
            g2o::EdgeSE3* e = new g2o::EdgeSE3();
            int idx1, idx2;
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
        }
        if( !fin.good()) break;        
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "prepare optimizing ..." << endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    cout << "calling optimizing ..." << endl;
    optimizer.optimize(30);

    cout << "saving optimization result ..." << endl;
    optimizer.save("result.g2o");

    return 0;    
    
}