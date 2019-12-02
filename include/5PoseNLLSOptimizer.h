//
// Created by ale on 19-11-27.
//

#ifndef MY3DPHOTO_5POSENLLSOPTIMIZER_H
#define MY3DPHOTO_5POSENLLSOPTIMIZER_H

#include "0m3d.h"
#include "1Logger.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace m3d {

    class RigidProblem {
        class Match6 {
        public:
            double paras[6];//src xyd, dst xyd.
        };

    public:
        int num_cameras_;
        int num_matches_;

        int *camera_i_;
        int *camera_j_;
        double *matches_;

        //mutable data, (variables to be optimized)
        double *parameters_;//camera extrisics

        //initial
        std::vector<m3d::Frame> &frames;
        m3d::Graph &graph;

        RigidProblem(std::vector<m3d::Frame> &frames, m3d::Graph &graph, char *argv0);
        ~RigidProblem();

    private:
        int initParameters();

        int updateParameters();

        int solveProblem();

        bool resize(int num_matches_, int num_cameras_);

        int insert(const int k, const int i, const int j, const double *const matches);


        int camera_i_index(int k) const { return camera_i_[k]; }

        int camera_j_index(int k) const { return camera_j_[k]; }

        int num_matches() const { return num_matches_; }

        int num_cameras() const { return num_cameras_; }

        const double *const const_matches() const { return matches_; }

        double *mutable_cameras() { return parameters_; }

        double *mutable_camera_i(int k) { return mutable_cameras() + camera_i_[k] * 6; }

        double *mutable_camera_j(int k) { return mutable_cameras() + camera_j_[k] * 6; }

//    int point_transfer(const cv::Vec3f& point0, const cv::Vec3f& rvec, const cv::Vec3f& tvec, cv::Vec3f& point1);
//    int depth_transfer(const double *const scales, const double *const offsets, cv::Mat & depth);
    };


//////////////////////////模板代码不能放到cpp文件中，因为其模板参数必须在编译时给出，而cpp文件在编译时已经被编译为源代码////////////////////
    RigidProblem::RigidProblem(std::vector<m3d::Frame> &frames_, m3d::Graph &graph_, char *argv0)
            : frames(frames_), graph(graph_), num_matches_(0), num_cameras_(0), camera_i_(nullptr), camera_j_(nullptr),
              matches_(nullptr), parameters_(nullptr) {
        google::InitGoogleLogging(argv0);
        initParameters();
        solveProblem();
        updateParameters();
    }

    RigidProblem::~RigidProblem() {
        if (camera_i_ != nullptr)
            delete[] camera_i_;
        if (camera_j_ != nullptr)
            delete[] camera_j_;
        if (matches_ != nullptr)
            delete[] matches_;
        if (parameters_ != nullptr)
            delete[] parameters_;

        camera_i_ = camera_j_ = nullptr;
        matches_ = parameters_ = nullptr;
    }

    bool RigidProblem::resize(int num_matches_, int num_cameras_) {
        this->num_matches_ = num_matches_;
        this->num_cameras_ = num_cameras_;

        if (camera_i_ != nullptr)
            delete[] camera_i_;
        if (camera_j_ != nullptr)
            delete[] camera_j_;
        if (matches_ != nullptr)
            delete[] matches_;
        if (parameters_ != nullptr)
            delete[] parameters_;

        camera_i_ = camera_j_ = nullptr;
        matches_ = parameters_ = nullptr;

        camera_i_ = new int[num_matches_];
        camera_j_ = new int[num_matches_];
        matches_ = new double[num_matches_ * 6];//a_xyd, b_xyd
        parameters_ = new double[num_cameras_ * 6];//rotate_angles, translation

        return true;
    }

    int RigidProblem::insert(const int k, const int i, const int j, const double *const matches) {
        if (k < 0 || k >= num_matches_)
            return -1;
        if (i < 0 || i > num_cameras_ || j < 0 || j > num_cameras_)
            return -2;

        for (int index = 0; index < 6; index++)
            *(matches_ + k * 6 + index) = matches[index];

        camera_i_[k] = i;
        camera_j_[k] = j;

        return 0;
    }

    int RigidProblem::initParameters() {
        const std::vector<bool> &activatedFrames = graph.activatedFrames;
        const std::vector<bool> &activatedEdges = graph.activatedEdges;

        size_t vsz = 0;
        size_t esz = 0;

        for (size_t i = 0; i < activatedEdges.size(); ++i)
            if (activatedEdges[i])
                ++esz;
        for (size_t i = 0; i < activatedFrames.size(); ++i)
            if (activatedFrames[i])
                ++vsz;
        if (vsz < 2) {
//            m3d::LOG("DepthDeformer<gridw, gridh>::initParameters(): ", "vsz < 2");
            std::cout << "DepthDeformer<gridw, gridh>::initParameters(): \t\t vsz < 2" << std::endl;
            exit(-1);
        }

        std::vector<size_t> camera_i, camera_j;
        std::vector<Match6> matches;
        for (size_t i = 0; i < graph.edges.size(); ++i) {
            size_t src = graph.edges[i].src;
            size_t dst = graph.edges[i].dst;

            Match6 match;
            double xyd[3];
            const m3d::Edge &edge = graph.edges[i];

            for (size_t k = 0; k < edge.srcPts.size(); ++k) {
                const cv::Mat &srcDepth = frames[src].depth;
                const cv::Mat &dstDepth = frames[dst].depth;

                const double srcColorCols = frames[src].image.cols;
                const double srcColorRows = frames[src].image.rows;
                const double dstColorCols = frames[dst].image.cols;
                const double dstColorRows = frames[dst].image.rows;

                xyd[0] = edge.srcPts[k].pt.x * (double) srcDepth.cols / srcColorCols;//x
                xyd[1] = edge.srcPts[k].pt.y * (double) srcDepth.rows / srcColorRows;//y
                xyd[2] = srcDepth.at<float>(std::floor(xyd[1]), std::floor(xyd[0]));//d

                for (size_t index = 0; index < 3; ++index)
                    match.paras[index] = xyd[index];

                xyd[0] = edge.dstPts[k].pt.x * (double) dstDepth.cols / dstColorCols;//x
                xyd[1] = edge.dstPts[k].pt.y * (double) dstDepth.rows / dstColorRows;//y
                xyd[2] = dstDepth.at<float>(std::floor(xyd[1]), std::floor(xyd[0]));//d

                for (size_t index = 0; index < 3; ++index)
                    match.paras[index + 3] = xyd[index];

                matches.push_back(match);
                camera_i.push_back(src);
                camera_j.push_back(dst);
            }
        }

        this->resize(matches.size(), frames.size());
        for (int k = 0; k < matches.size(); ++k)
            this->insert(k, camera_i[k], camera_j[k], matches[k].paras);

        for (int k = 0; k < frames.size(); ++k) {
            double *cameras = this->mutable_cameras();
            for (size_t i = 0; i < 6; ++i)
                cameras[k * 6 + i] = frames[k].extrinsicD.rts[i];
        }

        return 0;
    }
    int RigidProblem::updateParameters()
    {
        const double *parameters = this->mutable_cameras();
        for (size_t i = 0; i < frames.size(); ++i)
            for (size_t j = 0; j < 6; ++j)
                frames[i].extrinsicD.rts[j] = parameters[i * 6 + j];

        return 0;
    }

////////////about ceres functor and solver////////////////////////////
    class RigidCostFunctor1 {
    private:

        double matchesSrc[3];//matches of x,y,d from depth(src, dst)
        double matchesDst[3];
        double ws[4];//w0, w1, w2, w3 of biliear weight of deformable grids.
        double intrinsics[4];//intrinsic of depth, fx, fy, cx, cy.

        //alias of matches, ws and intrinsics.
        double &a_x, &a_y, &a_d;
        double &b_x, &b_y, &b_d;
        double &fx, &fy, &cx, &cy;

    public:
        //const values.

        RigidCostFunctor1(const double *const matchesSrc_, const double *const matchesDst_,
                          const double *const intrinsics_)
                : a_x(matchesSrc[0]), a_y(matchesSrc[1]), a_d(matchesSrc[2]), b_x(matchesDst[0]), b_y(matchesDst[1]),
                  b_d(matchesDst[2]), fx(intrinsics[0]), fy(intrinsics[1]), cx(intrinsics[2]), cy(intrinsics[3]) {
            for (size_t i = 0; i < 3; ++i)
                matchesSrc[i] = matchesSrc_[i];
            for (size_t i = 0; i < 3; ++i)
                matchesDst[i] = matchesDst_[i];
            for (size_t i = 0; i < 4; ++i)
                intrinsics[i] = intrinsics_[i];
        }

        // camera[0,1,2] are the angle-axis rotation.
        // camera[3,4,5] are the translation.
        template<typename T>
        bool operator()(const T *const camera_a, const T *const camera_b, T *residuals) const {
            //i_point's 3d position in camera_a's coordinate.
            T point[3];
            point[0] = T(a_d * (a_x - cx) / fx);
            point[1] = T(a_d * (a_y - cy) / fy);
            point[2] = T(a_d);

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
            const T x = point[0] / point[2];
            const T y = point[1] / point[2];
            T a_u = fx * x + cx;// u = (f*x + cx*z)/d
            T a_v = fy * y + cy;// v = (f*y + cy*z)/d

            // The error is the difference between the 2d points aligned in camera_b, difference(Pa->b(i_point), j_point) ).
            T b_u = T(b_x);
            T b_v = T(b_y);
            residuals[0] = a_u - b_u;
            residuals[1] = a_v - b_v;
            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction *
        Create(const double *const matchesSrc_, const double *const matchesDst_, const double *const intrinsics_) {
            return (new ceres::AutoDiffCostFunction<RigidCostFunctor1, 2, 6, 6>(
                    new RigidCostFunctor1(matchesSrc_, matchesDst_, intrinsics_)));
        }

    };

    int RigidProblem::solveProblem() {
        const double *const matches = this->const_matches();
        const int num_matches = this->num_matches();
        const int num_cameras = this->num_cameras();

        // Create residuals for each observation in the bundle adjustment problem. The
        // parameters for cameras and points are added automatically.
        ceres::Problem problem;

        for (int k = 0; k < num_matches; ++k) {
            // Each Residual block takes a point and a camera as input and outputs a 2
            // dimensional residual. Internally, the cost function stores the observed
            // image location and compares the reprojection against the observation.

            const double *const matchesSrc = matches + 6 * k;
            const double *const matchesDst = matches + 6 * k + 3;

            size_t src = this->camera_i_index(k);
            double *srcIntrinsicsD = frames[src].intrinsicD.intrinsic_d();

            ceres::CostFunction *cost_function =
                    RigidCostFunctor1::Create(matchesSrc, matchesDst, srcIntrinsicsD);
            problem.AddResidualBlock(cost_function,
                                     new ceres::CauchyLoss(0.5) /* squared loss */,
                                     this->mutable_camera_i(k),
                                     this->mutable_camera_j(k));
        }

        // Make Ceres automatically detect the bundle structure. Note that the
        // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
        // for standard bundle adjustment problems.
        ceres::Solver::Options options;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.linear_solver_type = ceres::DENSE_SCHUR;//ceres::DENSE_SCHUR; //ceres::LEVENBERG_MARQUARDT;

        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 200;//100;
        options.num_threads = 4;
        options.num_linear_solver_threads = 4;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";
        return 0;
    }
}

#endif //MY3DPHOTO_5POSENLLSOPTIMIZER_H
