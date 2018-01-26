#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
//#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include <opencv2/core/core.hpp>
#include <Eigen/StdVector>

#include "full_ba.h"
#include "dep_ba.h"

using namespace std;


int fullBundleAdjustment(Eigen::Ref<Eigen::MatrixXd> keyframes, Eigen::Ref<Eigen::MatrixXd> mapPoints, Eigen::Ref<Eigen::MatrixXd> pointsRelation )  {

    int primaryKeyframeId = keyframes.row(0)(0) ;

       //step 1 setup keyframes
    std::vector<KeyFrame> vpKFs;
    for(int n = 0; n < keyframes.rows(); n++) {
        Eigen::MatrixXd currentFrame(4, 4);
        currentFrame << keyFrameRowToMatrix(keyframes.row(n));
        // cout << currentFrame << std::endl;
        // cout << keyframes.row(n)(0) << endl;
        KeyFrame frame = std::make_pair( std::make_pair(n,keyframes.row(n)(0)) , currentFrame);
        vpKFs.push_back(frame);
    }
    
    //step 2 setup mappoints
    std::vector<MapPoint> vpMP;
    for(int n = 0; n < mapPoints.rows(); n++) {
        Eigen::MatrixXd currentPoint(1, 3);
        // cout << mapPoints.row(n) << endl;
        currentPoint <<  mappointRowToMatrix(mapPoints.row(n));
        // cout << currentPoint(0) << endl;
        MapPoint point = std::make_pair( std::make_pair(n, mapPoints.row(n)(0) ) , currentPoint);
        vpMP.push_back(point );
    }
    
    //mappoints to skip
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());
    
    //Step 3 Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    
    
    
    unsigned long maxKFid = 0;
    
    //Step 4 Set KeyFrame vertices
    for(int i = 0; i < vpKFs.size(); i++) {
        KeyFrame pKFi = vpKFs[i];
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(toSE3QuatFromMatrix(pKFi.second));
        vSE3->setId(pKFi.first.second);
        if (pKFi.first.second == primaryKeyframeId ) {
            vSE3->setFixed(true);
        } else {
            vSE3->setFixed(false);
        }
        optimizer.addVertex(vSE3);
        if(pKFi.first.second>maxKFid)
        maxKFid=pKFi.first.second;
    }
    
    
    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);
    
    //optimizer check
    int optimizerCheck = 0;
    
    for(int i = 0; i < vpMP.size(); i++) {
        MapPoint pMP = vpMP[i];
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate( toVector3d( pMP.second ));
        int id = pMP.first.second+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        
        
        // cout << pMP.second << endl;
        const std::vector<std::pair<KeyFrame,int>> observations = getObservationsWithRelation(pMP, vpKFs, mapPoints, pointsRelation);
        
        
        //SET EDGES
        int nEdges = 0;
        for(std::pair<KeyFrame,int> mit :  observations) {
            nEdges++;
            KeyFrame pKFi = mit.first;
            
            //keypoint of mappoint in the frame
            Eigen::Matrix<double,1,2> kpUn;
            Eigen::MatrixXd currentPoint(1, pointsRelation.cols());
            currentPoint << pointsRelation.row(mit.second);
            kpUn  << currentPoint(2),  currentPoint(3);
            
            // Monocular observation
            //            if(pKFi->mvuRight[mit->second]<0)
            //            {
            Eigen::Matrix<double,2,1> obs;
            obs << kpUn(0), kpUn(1);
            
            
            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
            
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi.first.second)));
            e->setMeasurement(obs);
            //                const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity());
            
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);
            
            e->fx = CameraFx;
            e->fy = CameraFy;
            e->cx = CameraCx;
            e->cy = CameraCy;
            
            optimizerCheck++;
            optimizer.addEdge(e);
            
            
        }
        
        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }
    
    
    if (optimizerCheck < 3 ) {
        return  0;
    }
    
    
    // Optimize!
    int nIterations = 5;
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);
    
    
    // Recover optimized data
    
    //Keyframes
    //TODO: nLoopKF check base code
    for(int i=0; i<vpKFs.size(); i++) {
        KeyFrame pKF = vpKFs[i];
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF.first.second));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF.second = toEigenBundel(SE3quat);
        // cout<< pKF.second << endl;
        keyframes(pKF.first.first, 2) = pKF.second(0, 0);
        keyframes(pKF.first.first, 3) = pKF.second(0, 1);
        keyframes(pKF.first.first, 4) = pKF.second(0, 2);
        keyframes(pKF.first.first, 5) = pKF.second(0, 3);
        keyframes(pKF.first.first, 6) = pKF.second(1, 0);
        keyframes(pKF.first.first, 7) = pKF.second(1, 1);
        keyframes(pKF.first.first, 8) = pKF.second(1, 2);
        keyframes(pKF.first.first, 9) = pKF.second(1, 3);
        keyframes(pKF.first.first, 10) = pKF.second(2, 0);
        keyframes(pKF.first.first, 11) = pKF.second(2, 1);
        keyframes(pKF.first.first, 12) = pKF.second(2, 2);
        keyframes(pKF.first.first, 13) = pKF.second(2, 3);
        keyframes(pKF.first.first, 14) = pKF.second(3, 0);
        keyframes(pKF.first.first, 15) = pKF.second(3, 1);
        keyframes(pKF.first.first, 16) = pKF.second(3, 2);
        keyframes(pKF.first.first, 17) = pKF.second(3, 3);
    }
    
    //Points
    //TODO: nLoopKF check base code
    for(int i=0; i<vpMP.size(); i++) {
        if(vbNotIncludedMP[i])
        continue;
        MapPoint pMP = vpMP[i];
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP.first.second+maxKFid+1));
        pMP.second = toEigenVector(vPoint->estimate()) ;
        //        pMP->UpdateNormalAndDepth();

        mapPoints(pMP.first.first, 1) = pMP.second(0, 0);
        mapPoints(pMP.first.first, 2) = pMP.second(0, 1);
        mapPoints(pMP.first.first, 3) = pMP.second(0, 2);
    }
    cout << "done" << endl;
    return 1;
}