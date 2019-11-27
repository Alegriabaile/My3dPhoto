//
// Created by ale on 19-11-26.
//

#ifndef MY3DPHOTO_5DEPTHDEFORMER_H
#define MY3DPHOTO_5DEPTHDEFORMER_H

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "0m3d.h"
#include "1Logger.h"

namespace m3d
{

    template <size_t gridw, size_t gridh>
    class DepthDeformer
    {
        class Match6
        {
        public:
            double paras[6];//src xyd, dst xyd.
        };
    public:
        const double grid_w;
        const double grid_h;

    private:
        int num_cameras_;
        int num_matches_;

        int* camera_i_;
        int* camera_j_;
        double* matches_;

        //mutable data, (variables to be optimized)
        double* parameters_;//camera extrisics
        double* scales_;
        double* offsets_;

        //initial
        std::vector<m3d::Frame> & frames;
        m3d::Graph & graph;


    public:
        DepthDeformer(std::vector<m3d::Frame> & frames, m3d::Graph & graph, char *argv0);
        virtual ~DepthDeformer();

    private:
        int initParameters();
        int updateParameters();
        int solveProblem();

        bool resize(int num_matches_, int num_cameras_);
        int insert(const int k, const int i, const int j, const double * const matches);

        //get the position of w0 in the 1dimension vector of the logic grid, and get the 4 bilinear weight.
        //w0 w2
        //w1 w3
        size_t bilinearWeight(double w, double h, double cols, double rows, double& w0, double&w1, double& w2, double& w3);
        size_t bilinearWeight(double w, double h, double cols, double rows, double *ws);
        //transform point0 to point1 with transform rvec,tvec.
        int transformPoint3d(const cv::Vec3f& point0, const cv::Vec3f& rvec, const cv::Vec3f& tvec, cv::Vec3f& point1);

        //from grids of scales and offsets to final deformed depth.
        int deformDepthMap(const double *const scales, const double *const offsets, cv::Mat & depth);


        //visit the inner data outsize the class.
        int camera_i_index(int k) const { return camera_i_[k];}
        int camera_j_index(int k) const { return camera_j_[k];}
        int num_matches() const { return num_matches_; }
        int num_cameras() const { return num_cameras_; }
        const double * const const_matches() const { return matches_; }

        double* mutable_cameras() { return parameters_; }
        double* mutable_scales(int k){ return scales_ + gridw*gridh*k;}
        double* mutable_offsets(int k){ return offsets_ + gridw*gridh*k;}

        double* mutable_camera_i(int k){ return mutable_cameras() + camera_i_[k] * 6;}
        double* mutable_camera_j(int k){ return mutable_cameras() + camera_j_[k] * 6;}
    };


    ///////////////////////////////////////////////////////////////////////////////////////////////////
    template <size_t gridw, size_t gridh>
    DepthDeformer<gridw, gridh>::DepthDeformer(std::vector<m3d::Frame> &frames_, m3d::Graph &graph_, char* argv0)
    : frames(frames_), graph(graph_)
    , num_matches_(0), num_cameras_(0)
    , camera_i_(nullptr), camera_j_(nullptr)
    , matches_(nullptr), parameters_(nullptr)
    , scales_(nullptr), offsets_(nullptr)
    , grid_w(gridw), grid_h(gridh)
    {
        google::InitGoogleLogging(argv0);
        initParameters();
        solveProblem();
        updateParameters();
    }

    template <size_t gridw, size_t gridh>
    DepthDeformer<gridw, gridh>::~DepthDeformer()
    {
        if(camera_i_ != nullptr)
            delete[] camera_i_;
        if(camera_j_ != nullptr)
            delete[] camera_j_;
        if(matches_ != nullptr)
            delete[] matches_;
        if(parameters_ != nullptr)
            delete[] parameters_;
        if(scales_ != nullptr)
            delete[] scales_;
        if(offsets_ != nullptr)
            delete[] offsets_;
        camera_i_ = camera_j_ = nullptr;
        matches_ = parameters_ = scales_ = offsets_ = nullptr;
    }

    template <size_t gridw, size_t gridh>
    bool DepthDeformer<gridw, gridh>::resize(int num_matches_, int num_cameras_)
    {
        this->num_matches_  = num_matches_;
        this->num_cameras_ = num_cameras_;

        if(camera_i_ != nullptr)
            delete[] camera_i_;
        if(camera_j_ != nullptr)
            delete[] camera_j_;
        if(matches_ != nullptr)
            delete[] matches_;
        if(parameters_ != nullptr)
            delete[] parameters_;
        if(scales_ != nullptr)
            delete[] scales_;
        if(offsets_ != nullptr)
            delete[] offsets_;
        camera_i_ = camera_j_ = nullptr;
        matches_ = parameters_ = scales_ = offsets_ = nullptr;

        camera_i_ = new int[num_matches_]; camera_j_ = new int[num_matches_];
        matches_  = new double[num_matches_*6];//a_xyd, b_xyd
        parameters_ = new double[num_cameras_*6];//rotate_angles, translation
        scales_ = new double[num_cameras_*gridw*gridh];
        offsets_ = new double[num_cameras_*gridw*gridh];
        return true;
    }

    template <size_t gridw, size_t gridh>
    int DepthDeformer<gridw, gridh>::insert(const int k, const int i, const int j, const double * const matches)
    {
        if(k<0 || k>=num_matches_)
            return -1;
        if(i<0 || i>num_cameras_ || j<0 || j>num_cameras_)
            return -2;

        for(int index = 0; index<6; index++)
            *(matches_+k*6+index) = matches[index];

        camera_i_[k] = i;
        camera_j_[k] = j;

        return 0;
    }


    template <size_t gridw, size_t gridh>
    int DepthDeformer<gridw, gridh>::initParameters()
    {
        const std::vector<bool> &activatedFrames = graph.activatedFrames;
        const std::vector<bool> &activatedEdges = graph.activatedEdges;

        size_t vsz = 0;
        size_t esz = 0;

        for(size_t i = 0; i < activatedEdges.size(); ++i)
            if(activatedEdges[i])
                ++esz;
        for(size_t i = 0; i < activatedFrames.size(); ++i)
            if(activatedFrames[i])
                ++vsz;
        if(vsz<2)
        {
//            m3d::LOG("DepthDeformer<gridw, gridh>::initParameters(): ", "vsz < 2");
            std::cout<<"DepthDeformer<gridw, gridh>::initParameters(): \t\t vsz < 2"<<std::endl;
            exit(-1);
        }

        std::vector<size_t> camera_i, camera_j;
        std::vector<Match6> matches;
        for(size_t i=0; i<graph.edges.size(); ++i)
        {
            size_t src = graph.edges[i].src;
            size_t dst = graph.edges[i].dst;

            Match6 match;
            double xyd[3];
            const m3d::Edge & edge = graph.edges[i];

            for(size_t k=0; k<edge.srcPts.size(); ++k)
            {
                const cv::Mat &srcDepth = frames[src].depth;
                const cv::Mat &dstDepth = frames[dst].depth;

                const double srcColorCols = frames[src].image.cols;
                const double srcColorRows = frames[src].image.rows;
                const double dstColorCols = frames[dst].image.cols;
                const double dstColorRows = frames[dst].image.rows;

                xyd[0] = edge.srcPts[k].pt.x*(double)srcDepth.cols/srcColorCols;//x
                xyd[1] = edge.srcPts[k].pt.y*(double)srcDepth.rows/srcColorRows;//y
                xyd[2] = srcDepth.at<float>(std::floor(xyd[1]), std::floor(xyd[0]));//d

                for(size_t index = 0; index < 3; ++index)
                    match.paras[index] = xyd[index];

                xyd[0] = edge.dstPts[k].pt.x*(double)dstDepth.cols/dstColorCols;//x
                xyd[1] = edge.dstPts[k].pt.y*(double)dstDepth.rows/dstColorRows;//y
                xyd[2] = dstDepth.at<float>(std::floor(xyd[1]), std::floor(xyd[0]));//d

                for(size_t index = 0; index < 3; ++index)
                    match.paras[index + 3] = xyd[index];

                matches.push_back(match);
                camera_i.push_back(src);
                camera_j.push_back(dst);
            }
        }

        this->resize(matches.size(), frames.size());
        for(int k = 0; k<matches.size(); ++k)
            this->insert(k, camera_i[k], camera_j[k], matches[k].paras);

        for(int k=0; k<frames.size(); ++k)
        {
            double *cameras = this->mutable_cameras();
            for(size_t i = 0; i < 6; ++i)
                cameras[k*6 + i] = frames[k].extrinsicD.rts[i];
        }
        for(int k=0; k<frames.size(); ++k)
        {
            double *scales = this->mutable_scales(k);
            double *offsets = this->mutable_offsets(k);
            for(int i=0; i<gridw*gridh; ++i)
            {
                scales[i] = 0.1;
                offsets[i] = 0.0;
            }
        }

        return 0;
    }

    template <size_t gridw, size_t gridh>
    size_t DepthDeformer<gridw, gridh>::bilinearWeight(double w, double h, double cols, double rows, double& w0, double&w1, double& w2, double& w3)
    {
        size_t k0 = std::floor(w/cols*(grid_w-1))*grid_h + std::floor(h/rows*(grid_h-1));
        //求双线性插值权重
        double x1 = cols/(grid_w-1) * std::floor(w/cols*(grid_w-1));
        double y1 = rows/(grid_h-1) * std::floor(h/rows*(grid_h-1));
        double x2 = x1 + cols/(grid_w-1);
        double y2 = y1 + rows/(grid_h-1);

        w0 = (x2-w)*(y2-h)/( (x2-x1)*(y2-y1));
        w1 = (x2-w)*(h-y1)/( (x2-x1)*(y2-y1));
        w2 = (w-x1)*(y2-h)/( (x2-x1)*(y2-y1));
        w3 = (w-x1)*(h-y1)/( (x2-x1)*(y2-y1));

        return k0;
    }

    template <size_t gridw, size_t gridh>
    size_t DepthDeformer<gridw, gridh>::bilinearWeight(double w, double h, double cols, double rows, double *ws)
    {
        size_t k0 = std::floor(w/cols*(grid_w-1))*grid_h + std::floor(h/rows*(grid_h-1));
        //求双线性插值权重
        double x1 = cols/(grid_w-1) * std::floor(w/cols*(grid_w-1));
        double y1 = rows/(grid_h-1) * std::floor(h/rows*(grid_h-1));
        double x2 = x1 + cols/(grid_w-1);
        double y2 = y1 + rows/(grid_h-1);

        ws[0] = (x2-w)*(y2-h)/( (x2-x1)*(y2-y1));
        ws[1] = (x2-w)*(h-y1)/( (x2-x1)*(y2-y1));
        ws[2] = (w-x1)*(y2-h)/( (x2-x1)*(y2-y1));
        ws[3] = (w-x1)*(h-y1)/( (x2-x1)*(y2-y1));

        return k0;
    }

    template <size_t gridw, size_t gridh>
    int DepthDeformer<gridw, gridh>::transformPoint3d(const cv::Vec3f& point0, const cv::Vec3f& rvec, const cv::Vec3f& tvec, cv::Vec3f& point1)
    {
        //Rotate and Translate matrix.
        //matrix: |Point1; 1| = |R, t|*|point0; 1|
        cv::Mat R = (cv::Mat_<double>(3,3));
        cv::Mat rv = (cv::Mat_<double>(3,1) << rvec[0], rvec[1], rvec[2]);
        cv::Rodrigues( rv, R );
        double x = point0[0];
        double y = point0[1];
        double z = point0[2];

        point1[0] = R.at<double>(0, 0) * x + R.at<double>(0, 1) * y + R.at<double>(0, 2) * z + tvec[0];
        point1[1] = R.at<double>(1, 0) * x + R.at<double>(1, 1) * y + R.at<double>(1, 2) * z + tvec[1];
        point1[2] = R.at<double>(2, 0) * x + R.at<double>(2, 1) * y + R.at<double>(2, 2) * z + tvec[2];

        //cout<<R<<point1<<endl;
        return 0;
    }

#define DEPTH_TRANSFER_DEBUG
    template <size_t gridw, size_t gridh>
    int DepthDeformer<gridw, gridh>::deformDepthMap(const double * const scales, const double * const offsets, cv::Mat& depth)
    {
#ifdef DEPTH_TRANSFER_DEBUG
        using namespace std;
        for(size_t j=0; j<gridw*gridh; ++j)
        {
            cout<<scales[j]<<" ";//"("<<std::atan(scales[j])/(3.141592654)+(0.5)<<")"<<" ";
        }
        cout<<endl;
        for(size_t j=0; j<gridw*gridh; ++j)
        {
            cout<<offsets[j]<<" ";//"("<<std::atan(offsets[j])/(3.141592654)+(0.5)<<")"<<" ";
        }
        cout<<endl;

        double maxVal, minVal;
        minMaxLoc(depth, &minVal, &maxVal);
        cout<<"original min, max: "<<minVal<<", "<<maxVal<<endl;
#endif
        size_t rows = depth.rows;
        size_t cols = depth.cols;
        for(size_t h = 0; h < rows; ++h)
            for(size_t w = 0; w<cols; ++w)
            {

                double w0, w2;
                double w1, w3;
                int k0 = bilinearWeight(w, h, cols, rows, w0, w1, w2, w3);
                double scale = scales[k0]*w0 + scales[k0+1]*w1 + scales[k0+gridh]*w2 + scales[k0+gridh+1]*w3;
                double offset = offsets[k0]*w0 + offsets[k0+1]*w1 + offsets[k0+gridh]*w2 + offsets[k0+gridh+1]*w3;

                double d = depth.at<float>(h, w);
                double d_new = d/(scale+offset*d);
                depth.at<float>(h, w) = d_new;
            }
#ifdef DEPTH_TRANSFER_DEBUG
        minMaxLoc(depth, &minVal, &maxVal);
        cout<<"deformed min, max: "<<minVal<<", "<<maxVal<<endl;
#endif
        return 0;
    }
#undef DEPTH_TRANSFER_DEBUG

    template <size_t gridw, size_t gridh>
    int DepthDeformer<gridw, gridh>::updateParameters()
    {
        const double * parameters = this->mutable_cameras();
        for(size_t i=0; i<frames.size(); ++i)
            for(size_t j = 0; j < 6; ++j)
                frames[i].extrinsicD.rts[j] = parameters[i*6 + j];

        for(size_t i=0; i<frames.size(); ++i)
        {
            if(!graph.activatedFrames[i])
                continue;
            double * scales = this->mutable_scales(i);
            double * offsets = this->mutable_offsets(i);
            this->deformDepthMap(scales, offsets, frames[i].depth);
        }
        //**instant 3d photography**
        //we compute a center of projection for the panorama
        //by tracing the camera front vectors backwards and
        //finding the 3D point that minimizes the distance to all of them

        //实现：找到中心点尽量使[中心点，相机点]的向量与对应相机点的向量平行，即叉积为0
        //令方向归一化，则使得所有的相机，下式：sin(theta)
        // = |n X vector(center - camera_original)|/(|n|*|vector(center - camera_original)|)
        //最小，为使计算简单，去掉|n|*|vector(center - camera_original)|项，即只要优化
        //|n X vector(center - camera_original)|使其最小即可。

        /*
        const std::vector<bool> &activatedFrames = graph.activatedFrames;

        uint ksize = frames.size();
        std::vector<cv::Mat> N(ksize);
        std::vector<cv::Mat> NNT(ksize);
        std::vector<cv::Mat> NNTo(ksize);
        cv::Mat A = cv::Mat::zeros(3,3,CV_32FC1);
        cv::Mat b = cv::Mat(3,1,CV_32FC1, cv::Scalar(0));
        for(int i=0; i<frames.size(); ++i)
        {
            cv::Vec3f point0(0,0,100);
            cv::Vec3f orig0(0,0,0);
            cv::Vec3f point1, orig1;
            cv::Vec3f rvec, tvec;

            rvec[0] = parameters[i*6]; rvec[1] = parameters[i*6+1]; rvec[2] = parameters[i*6+2];
            tvec[0] = parameters[i*6+3]; tvec[1] = parameters[i*6+4]; tvec[2] = parameters[i*6+5];

            transformPoint3d(point0, rvec, tvec, point1);
            transformPoint3d(orig0, rvec, tvec, orig1);

            cv::Vec3f n(normalize(point1-orig1));
            N[i] = (cv::Mat_<float>(3,3) <<
                                         0, -n[2], n[1], n[2], 0, -n[0], -n[1], n[0], 0 );
            cv::Mat o = (cv::Mat_<float>(3,1) <<  orig1[0], orig1[1], orig1[2]);
            NNT[i] = N[i]*(-N[i]); NNTo[i] = NNT[i]*o;

            A += NNT[i]; b += NNTo[i];
            //cout<<orig1<<", "<<point1<<", "<<n<<";"<<endl;
        }

        cv::Mat A_inverse;
        invert(A, A_inverse);
        cv::Mat center = A_inverse*b;
        */


        //todo
        //how to get the rotation center...
//        for(size_t i=0; i<frames.size(); ++i)
//        {
//            if(!graph.activatedFrames[i])
//                continue;
////            for(size_t j = 0; j < 3; ++j)
////                frames[i].extrinsicD.rts[j] = parameters[i*6 + j];
//            for(size_t j = 0; j < 3; ++j)
//                frames[i].extrinsicD.rts[j+3] -= center.at<float>(j);
//        }


        return 0;
    }





////////////about ceres functor and solver////////////////////////////
    class CostFunctor1
    {
    private:

        double matchesSrc[3];//matches of x,y,d from depth(src, dst)
        double matchesDst[3];
        double ws[4];//w0, w1, w2, w3 of biliear weight of deformable grids.
        double intrinsics[4];//intrinsic of depth, fx, fy, cx, cy.

        //alias of matches, ws and intrinsics.
        double &a_x, &a_y, &a_d;
        double &b_x, &b_y, &b_d;
        double &w0, &w2;
        double &w1, &w3;
        double &fx, &fy, &cx, &cy;

    public:
        //const values.

        CostFunctor1(const double * const matchesSrc_, const double * const matchesDst_, const double * const ws_, const double * const intrinsics_)
        : a_x(matchesSrc[0]), a_y(matchesSrc[1]), a_d(matchesSrc[2])
        , b_x(matchesDst[0]), b_y(matchesDst[1]), b_d(matchesDst[2])
        , w0(ws[0]), w1(ws[1]), w2(ws[2]), w3(ws[3])
        , fx(intrinsics[0]), fy(intrinsics[1]), cx(intrinsics[2]), cy(intrinsics[3])
        {
            for(size_t i = 0; i < 3; ++i)
                matchesSrc[i] = matchesSrc_[i];
            for(size_t i = 0; i < 3; ++i)
                matchesDst[i] = matchesDst_[i];
            for(size_t i = 0; i < 4; ++i)
                ws[i] = ws_[i];
            for(size_t i = 0; i < 4; ++i)
                intrinsics[i] = intrinsics_[i];
        }

        // camera[0,1,2] are the angle-axis rotation.
        // camera[3,4,5] are the translation.

        //bilinear grid:
        //0, 2
        //1, 3
        template <typename T>
        bool operator()(const T* const camera_a, const T* const camera_b,
                        const T* const scales_a0, const T* const scales_a1, const T* const scales_a2, const T* const scales_a3,
                        const T* const offsets_a0, const T* const offsets_a1, const T* const offsets_a2, const T* const offsets_a3,
                        T* residuals) const
        {
            //a_x, a_y, b_x, b_y, based on depth image size
            //Bilinear Interpolation
            T scale_weight = scales_a0[0]*w0 + scales_a1[0]*w1 + scales_a2[0]*w2 + scales_a3[0]*w3;
            T offset_weight = offsets_a0[0]*w0 + offsets_a1[0]*w1 + offsets_a2[0]*w2 + offsets_a3[0]*w3;
            //T a_d_ = T(1.0) / (scale_weight * (T(1.0) /T(a_d)) + offset_weight );
            T a_d_ = T(a_d) / (scale_weight + offset_weight*T(a_d) );

            //i_point's 3d position in camera_a's coordinate.
            T point[3];
            point[0] = T(a_d_*(a_x-cx)/fx );
            point[1] = T(a_d_*(a_y-cy)/fy );
            point[2] = T(a_d_ );

            //i_point's position in global coordinate.
            T p[3];
            //RodriguesRotate<T>(camera_a_, point, p);
            ceres::AngleAxisRotatePoint(camera_a, point, p);
            //T a_matrix[3][3];
            //ceres::AngleAxisToRotationMatrix(camera_a_, a_matrix);
            p[0] += camera_a[3];
            p[1] += camera_a[4];
            p[2] += camera_a[5];

            //i_point's 3d position in camera_b's coordinate.
            p[0] -= camera_b[3];
            p[1] -= camera_b[4];
            p[2] -= camera_b[5];
            //RodriguesRotate<T>(camera_b, p, point);
            T camera_b_[3];
            camera_b_[0] = -camera_b[0];
            camera_b_[1] = -camera_b[1];
            camera_b_[2] = -camera_b[2];
            ceres::AngleAxisRotatePoint(camera_b_, p, point);
            //i_point's 2d position from camera_a to camera_b.
            const T x=point[0]/point[2];
            const T y=point[1]/point[2];
            T a_u = T(fx)*x+T(cx);// u = (f*x + cx*z)/d
            T a_v = T(fy)*y+T(cy);// v = (f*y + cy*z)/d

            // The error is the difference between the 2d points aligned in camera_b, difference(Pa->b(i_point), j_point) ).
            T b_u = T(b_x);
            T b_v = T(b_y);

            residuals[0] = a_u - b_u;
            residuals[1] = a_v - b_v;
            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const double * const matchesSrc_, const double * const matchesDst_, const double *const ws_, const double *const intrinsics_)
        {
            return (new ceres::AutoDiffCostFunction<CostFunctor1, 2, 6, 6, 1, 1, 1, 1, 1, 1, 1, 1>(
                    new CostFunctor1(matchesSrc_, matchesDst_, ws_, intrinsics_)));
        }

    };

//E_smoothness
    struct CostFunctor2{
        CostFunctor2(double l1_): l1(l1_){}

        //在一个加了const限定符的成员函数中，不能够调用 非const成员函数
        template <typename T> bool operator()(const T* const scales1, const T* const scales2,
                                              const T* const offsets1, const T* const offsets2, T* residual) const {
            residual[0] = T(l1) * (scales1[0] - scales2[0]);
            residual[1] = T(l1) * (offsets1[0] - offsets2[0]);
            return true;
        }

        static ceres::CostFunction* Create(const double l1_=1e3)
        {
            return ( new ceres::AutoDiffCostFunction < CostFunctor2, 2, 1,1,1,1> ( new CostFunctor2(l1_)));
        }
    private:
        double l1;
    };


//E_scale
    struct CostFunctor3 {
        CostFunctor3(double l2_): l2(l2_) {}
        template <typename T>
        bool operator()(const T* const scales, T* residual) const {
            residual[0] = T(l2) * ceres::sqrt(T(1.0) / scales[0]);
            return true;
        }

        static ceres::CostFunction* Create(const double l2=1e-2) {
            return (new ceres::AutoDiffCostFunction<CostFunctor3, 1, 1>(new CostFunctor3(l2)));
        }

    private:
        double l2;
    };

    template <size_t gridw, size_t gridh>
    int DepthDeformer<gridw, gridh>::solveProblem()
    {
        const double* const matches = this->const_matches();
        const int num_matches = this->num_matches();
        const int num_cameras = this->num_cameras();

        // Create residuals for each observation in the bundle adjustment problem. The
        // parameters for cameras and points are added automatically.
        ceres::Problem problem;

        size_t k0;
        double ws[4];
        //Eprojection
        for (int i = 0; i < num_matches; ++i)
        {
            size_t src = this->camera_i_index(i);
            size_t dst = this->camera_j_index(i);

            const double * const matchesSrc = matches + 6*i;
            const double * const matchesDst = matches + 6*i+3;

            double * srcIntrinsicsD = frames[src].intrinsicD.intrinsic_d();
            double srcDepthCols = frames[src].depth.cols;
            double srcDepthRows = frames[src].depth.rows;

            k0 = this->bilinearWeight(matchesSrc[0], matchesSrc[1], srcDepthCols, srcDepthRows, ws);
            ceres::CostFunction* costFunctor1_0 =
                    CostFunctor1::Create(matchesSrc, matchesDst, ws, srcIntrinsicsD);

            double * scales = this->mutable_scales(this->camera_i_index(i));
            double * offsets = this->mutable_offsets(this->camera_i_index(i));
            problem.AddResidualBlock(costFunctor1_0,
                                     new ceres::CauchyLoss(0.5) /* squared loss */,
                                     this->mutable_camera_i(i),
                                     this->mutable_camera_j(i),
                                     scales+k0, scales+k0+1, scales+k0+gridh, scales+k0+gridh+1,
                                     offsets+k0, offsets+k0+1, offsets+k0+gridh, offsets+k0+gridh+1);

            //主要为了求解B图像的scales、offsets因子
            double * dstIntrinsicsD = frames[dst].intrinsicD.intrinsic_d();
            double dstDepthCols = frames[dst].depth.cols;
            double dstDepthRows = frames[dst].depth.rows;
            k0 = this->bilinearWeight(matchesDst[0], matchesDst[1], dstDepthCols, dstDepthRows, ws);
            ceres::CostFunction* costFunctor1_1 =
                    CostFunctor1::Create(matchesDst, matchesSrc, ws, dstIntrinsicsD);

            scales = this->mutable_scales(this->camera_j_index(i));
            offsets = this->mutable_offsets(this->camera_j_index(i));
            problem.AddResidualBlock(costFunctor1_1,
                                     new ceres::CauchyLoss(0.5) ,
                                     this->mutable_camera_j(i),
                                     this->mutable_camera_i(i),
                                     scales+k0, scales+k0+1, scales+k0+gridh, scales+k0+gridh+1,
                                     offsets+k0, offsets+k0+1, offsets+k0+gridh, offsets+k0+gridh+1);
        }

        //l1, l2最终会经过平方
        //目前只适用于kinect深度
        const double ConstK = 1.0f;
        std::cout<<"num_matches : " << num_matches<<std::endl;
        for(int c = 0; c<num_cameras; ++c)
        {
            double * scales = this->mutable_scales(c);
            double * offsets = this->mutable_offsets(c);
            //Esmothness
            double l1 = 1e3*ConstK;//
            for(int w=0; w<gridw; ++w)
            {
                for (int h=0; h<gridh-1; ++h)
                {

                    int k = w * gridh + h;
                    ceres::CostFunction *costFunctor2_h =
                            CostFunctor2::Create(l1);
                    problem.AddResidualBlock(costFunctor2_h, NULL,
                                             scales+k, scales+k+1,
                                             offsets+k, offsets+k+1);
                }
            }
            for(int w=0; w<gridw-1; ++w)
            {
                for (int h=0; h<gridh; ++h)
                {
                    int k = w * gridh + h;
                    ceres::CostFunction *costFunctor2_w =
                            CostFunctor2::Create(l1);
                    problem.AddResidualBlock(costFunctor2_w, NULL,
                                             scales+k, scales+k+gridh,
                                             offsets+k, offsets+k+gridh);
                }
            }

            //Escale
            double l2 = 1e-2*ConstK;
            for(int k=0; k<gridw*gridh; ++k)
            {
                ceres::CostFunction *costFunctor3 =
                        CostFunctor3::Create(l2);
                problem.AddResidualBlock(costFunctor3, NULL, scales+k);
            }
        }

        // Make Ceres automatically detect the bundle structure. Note that the
        // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
        // for standard bundle adjustment problems.
        ceres::Solver::Options options;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.linear_solver_type = ceres::DENSE_QR;//ceres::DENSE_SCHUR; //ceres::LEVENBERG_MARQUARDT;

        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 50;//100;
        options.num_threads = 4;
        options.num_linear_solver_threads = 4;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";
        return 0;
    }
}

#endif //MY3DPHOTO_5DEPTHDEFORMER_H
