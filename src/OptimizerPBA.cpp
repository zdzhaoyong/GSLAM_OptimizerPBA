#include <GSLAM/core/Optimizer.h>
#include <pba/pba.h>
#include <pba/DataInterface.h>
#include <pba/pba_util.h>

using namespace GSLAM;

class OptimizerPBA: public Optimizer
{
public:
    OptimizerPBA(OptimzeConfig config=OptimzeConfig()):Optimizer(config){}

    virtual ~OptimizerPBA(){}
    // MAPPING: Do bundle adjust with auto calibration or not: BUNDLEADJUST, INVDEPTH_BUNDLE, POSEGRAPH
    virtual bool optimize(BundleGraph& graph){
        if(graph.invDepthObserves.size()||graph.se3Graph.size()||graph.sim3Graph.size()||graph.gpsGraph.size())
        {
            LOG(ERROR)<<"OptimizerPBA can only handle mappoint bundle adjust problem!";
            return false;
        }
        using namespace pba;

        // CameraT, Point3D, Point2D are defined in pba/DataInterface.h
        vector<CameraT>         camera_data;    //camera (input/ouput)
        vector<PBAPoint3>       point_data;     //3D point(iput/output)
        vector<PBAPoint2f>      measurements;   //measurment/projection vector
        vector<int>             camidx,ptidx;  //index of camera/point for each projection

        // Get Calibration Parameters for later projection
        pi::Point2d cxcy(0.,0.);
        pi::Point2d fxfy(1.,1.);
        bool        cameraValid=graph.camera.isValid();

        const float fx = fxfy.x;
        const float fy = fxfy.y;
        const float cx = cxcy.x;
        const float cy = cxcy.y;

        float   f,
                d[2],
                q[9], c[3];
        f =fx;
        float fx_fy=f/fy;
        d[0]=d[1]=0;

        /// 1.insert cameras
        int id=0;
        CameraT cameraT;
        cameraT.SetFocalLength(f);
        cameraT.SetNormalizedMeasurementDistortion(d[0]);    // FIXME: why d[0]?
        cameraT.SetFixedIntrinsic();
        camera_data.resize(graph.keyframes.size(),cameraT);
        for(int i=0,iend=graph.keyframes.size();i<iend;i++)
        {
            GSLAM::KeyFrameEstimzation& keyframe=graph.keyframes[i];
            pi::SE3f se3=keyframe.estimation.get_se3().inverse();
            se3.get_rotation().getMatrix(q);

            pi::Point3f t=se3.get_translation();
            c[0] = t[0]; c[1] = t[1];c[2] = t[2];

            camera_data[i].SetMatrixRotation(q);
            camera_data[i].SetTranslation(c);
            camera_data[i].SetFocalLength(f);
            camera_data[i].SetNormalizedMeasurementDistortion(d[0]);    // FIXME: why d[0]?
            //            camera_data[id].SetFixedIntrinsic();

            if(keyframe.dof==GSLAM::UPDATE_KF_SE3)
                camera_data[i].SetVariableCamera();
        }

        ///2.insert points
        id=0;
        point_data.resize(graph.mappoints.size());
        for(int i=0,iend=graph.mappoints.size();i<iend;i++)
        {
            pi::Point3d& pt=graph.mappoints[i].first;
            point_data[i].SetPoint(pt.x,pt.y,pt.z);
        }

        ///3.insert observes
        measurements.resize(graph.mappointObserves.size());
        camidx.resize(graph.mappointObserves.size());
        ptidx.resize(graph.mappointObserves.size());
        for(int i=0,iend=graph.mappointObserves.size();i<iend;i++)
        {
            GSLAM::BundleEdge&  edge=graph.mappointObserves[i];
            GSLAM::CameraAnchor obs=edge.measurement;
            if(cameraValid) obs=graph.camera.UnProject(obs.x,obs.y);
            measurements[i]=PBAPoint2f(obs.x,obs.y);
            camidx[i]      =edge.frameId;
            ptidx[i]       =edge.pointId;
        }

        int N=camera_data.size();
        int M=point_data.size();
        int K=measurements.size();

        //printf("\nadjustBundle_pba: \n");
        //printf("  N (cams) = %d, M (points) = %d, K (measurements) = %d\n", N, M, K);

        //////////////////////////////////////////////////////////
        /// begin PBA
        //////////////////////////////////////////////////////////
        string inputFile=svar.GetString("PBA.InputFile","NoFile");
        if(inputFile!="NoFile")
        {
            vector<int>    ptc;  //point color
            vector<string> names;//keyframe names
            SaveNVM(inputFile.c_str(),camera_data,point_data,measurements,
                    ptidx, camidx,
                    names,
                    ptc);
        }

        ParallelBA::DeviceT device = ParallelBA::PBA_CUDA_DEVICE_DEFAULT;

        ParallelBA pba(device);

        pba.SetFixedIntrinsics(true);

        pba.SetCameraData(camera_data.size(),  &camera_data[0]);    //set camera parameters
        pba.SetPointData(point_data.size(),    &point_data[0]);     //set 3D point data
        pba.SetProjection(measurements.size(), &measurements[0],    //set the projections
                            &ptidx[0], &camidx[0]);

        pba.GetInternalConfig()->__verbose_level=0;
    //    pba.GetInternalConfig()->__lm_delta_threshold=1e-5;
        pba.RunBundleAdjustment();

        string outputFile=svar.GetString("PBA.OutputFile","NoFile");
        if(outputFile!="NoFile")
        {
            vector<int>    ptc;  //index of camera/point for each projection
            vector<string> names;
            SaveNVM(outputFile.c_str(),camera_data,point_data,measurements,ptidx,camidx,names,ptc);
        }

        //////////////////////////////////////////////////////////
        /// copy data back
        //////////////////////////////////////////////////////////
        ///
        for(int j=0; j<M; j++)
        {
            Point3Type& point=graph.mappoints[j].first;
            point[0]=point_data[j].xyz[0];
            point[1]=point_data[j].xyz[1];
            point[2]=point_data[j].xyz[2];
        }

        pi::SE3f se3;
        for(int i=0;i<N;i++)
        {
            camera_data[i].GetMatrixRotation(q);
            camera_data[i].GetTranslation(c);
            se3.get_rotation().fromMatrix(q);
            se3.get_translation()=Point3Type(c[0],c[1],c[2]);
            graph.keyframes[i].estimation.get_se3()=se3.inverse();
        }

        return true;
    }

};


USE_OPTIMIZER_PLUGIN(OptimizerPBA);
