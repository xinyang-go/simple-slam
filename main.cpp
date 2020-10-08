#include <iostream>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/core/eigen.hpp>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <boost/timer/timer.hpp>
#include "include/obj_manager.hpp"
#include "MindVision/MindVision.hpp"


struct FramePoint;

struct Frame;

struct MapPoint;

struct Map;

struct MapPoint {
    Eigen::Vector3d pt;
    int look_cnt{1};
    g2o::VertexSBAPointXYZ *v_xyz{nullptr};
};

struct Map {

};

struct FramePoint {
    cv::Point2d pt;
    ObjManager<MapPoint>::Ptr map_point;
};

struct Frame {
    cv::Mat image;
    Sophus::SE3d Tcw;
    std::vector<ObjManager<FramePoint>::Ptr> frame_points;
    g2o::VertexSE3Expmap *v_map{nullptr};
};

class Slam {
public:
    explicit Slam(std::string_view config) {
        cv::FileStorage fs(config.begin(), cv::FileStorage::READ);
        fs["max_history_frame_size"] >> reinterpret_cast<int &>(max_history_frame_size);
        fs["max_frame_point_size"] >> reinterpret_cast<int &>(max_frame_point_size);
        fs["point_world_look_cnt"] >> reinterpret_cast<int &>(point_world_look_cnt);
        fs["K"] >> K;
        fs["C"] >> C;
        cx = K.at<float>(0, 2);
        cy = K.at<float>(1, 2);
        fx = K.at<float>(0, 0);
        fy = K.at<float>(1, 1);
    }

    ObjManager<Frame>::Ptr operator()(const cv::Mat &src) {
        current_frame = ObjManager<Frame>::create();
        current_frame->image = src;

        // track old key points from last frame
        if (!history_frames.empty())
            trackKeyPoints();
        // find some new key points in new frame
        if (current_frame->frame_points.size() < max_frame_point_size)
            detectKeyPoints();
        // estimate pose
        if (!history_frames.empty())
            g2oEstimate();

        // add new frame
        history_frames.emplace_back(current_frame);
        // remove old frame
        while (history_frames.size() >= max_history_frame_size) {
            history_frames.pop_front();
        }
        return current_frame;
    }

private:
    void trackKeyPoints() {
        // prepare key points in last frame
        std::vector<cv::Point2f> last_kps, curr_kps;
        for (const auto &fp: history_frames.back()->frame_points) {
            last_kps.emplace_back(fp->pt);
        }

        // track key points
        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(history_frames.back()->image, current_frame->image, last_kps, curr_kps,
                                 status, error, {11, 11}, 3,
                                 {cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01});

        // save tracked key points
        for (int i = 0; i < curr_kps.size(); i++) {
            if (status[i]) {
                auto fp = ObjManager<FramePoint>::create();
                fp->pt = curr_kps[i];
                fp->map_point = history_frames.back()->frame_points[i]->map_point;
                fp->map_point->look_cnt++;
                current_frame->frame_points.emplace_back(fp);
            }
        }
    }

    void detectKeyPoints() {
        // get the mask
        cv::Mat mask(current_frame->image.size(), CV_8U, 255);
        cv::Point2d range(10, 10);
        for (const auto &fp: current_frame->frame_points) {
            cv::rectangle(mask, fp->pt - range, fp->pt + range, 0, cv::FILLED);
        }
        // detect new key points
        cv::Ptr<cv::GFTTDetector> gftt = cv::GFTTDetector::create(
                max_frame_point_size - current_frame->frame_points.size(), 0.03, 20, 3);
        std::vector<cv::KeyPoint> kps;
        gftt->detect(current_frame->image, kps, mask);
        // save new key points
        for (const auto &kp: kps) {
            auto fp = ObjManager<FramePoint>::create();
            fp->pt = kp.pt;
            fp->map_point = ObjManager<MapPoint>::create();
            fp->map_point->pt = {(kp.pt.x - cx) / fx, (kp.pt.y - cy) / fy, 1};
            current_frame->frame_points.emplace_back(fp);
        }
    }

    void g2oEstimate() {
        // 构建图优化，先设定g2o
        typedef g2o::BlockSolverX BlockSolverType;  // pose is 6, landmark is 3
        typedef g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
        // 梯度下降方法，可以从GN, LM, DogLeg 中选
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;     // 图模型
        optimizer.setAlgorithm(solver);   // 设置求解器

        // 添加节点
        int vertex_id = 0;
        // 历史位姿节点
        for (const auto &frame: history_frames) {
            auto *v = new g2o::VertexSE3Expmap();
            v->setId(vertex_id++);
            v->setFixed(true);
            v->setEstimate({frame->Tcw.rotationMatrix(), frame->Tcw.translation()});
            frame->v_map = v;
            optimizer.addVertex(v);
        }
        // 待估计位姿节点
        {
            auto *v = new g2o::VertexSE3Expmap();
            current_frame->Tcw = history_frames.back()->Tcw;
            v->setId(vertex_id++);
            v->setEstimate({current_frame->Tcw.rotationMatrix(), current_frame->Tcw.translation()});
            current_frame->v_map = v;
            optimizer.addVertex(v);
        }
        // 特征世界坐标节点
        for (const auto &[id, p_ptw]: ObjManager<MapPoint>::set()) {
            auto &ptw = *p_ptw.lock();
            if (ptw.look_cnt < 2) continue;
            auto *v = new g2o::VertexSBAPointXYZ();
            v->setId(vertex_id++);
            v->setEstimate(ptw.pt);
            if (ptw.look_cnt > point_world_look_cnt) {
                v->setFixed(true);
            } else {
                v->setMarginalized(true);
            }
            ptw.v_xyz = v;
            optimizer.addVertex(v);
        }
        // 准备相机参数
        auto *camera = new g2o::CameraParameters((fx + fy) / 2, {cx, cy}, 0);
        camera->setId(0);
        optimizer.addParameter(camera);

        // 添加边
        int edge_id = 0;
        // 历史帧
        for (const auto &frame:history_frames) {
            for (const auto &fp: frame->frame_points) {
                if (fp->map_point->look_cnt < 2) continue;
                auto *edge = new g2o::EdgeProjectXYZ2UV();
                edge->setId(edge_id++);
                edge->setVertex(0, fp->map_point->v_xyz);
                edge->setVertex(1, frame->v_map);
                edge->setMeasurement({fp->pt.x, fp->pt.y});
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setParameterId(0, 0);
                edge->setRobustKernel(new g2o::RobustKernelHuber());
                optimizer.addEdge(edge);
            }
        }
        // 当前帧
        std::vector<g2o::EdgeProjectXYZ2UV *> edges;
        std::vector<bool> is_outlier;
        for (const auto &fp: current_frame->frame_points) {
            if (fp->map_point->look_cnt < 2) continue;
            auto *edge = new g2o::EdgeProjectXYZ2UV();
            edge->setId(edge_id++);
            edge->setVertex(0, fp->map_point->v_xyz);
            edge->setVertex(1, current_frame->v_map);
            edge->setMeasurement({fp->pt.x, fp->pt.y});
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setParameterId(0, 0);
            edge->setRobustKernel(new g2o::RobustKernelHuber());
            optimizer.addEdge(edge);
            edges.emplace_back(edge);
            is_outlier.emplace_back(false);
        }

        // 开始优化
        const double chi2_th = 5.991;
        for (int iteration = 0; iteration < 4; ++iteration) {
            current_frame->v_map->setEstimate({current_frame->Tcw.rotationMatrix(), current_frame->Tcw.translation()});
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            // 得出姿态矩阵
            Eigen::Isometry3d Tcw = current_frame->v_map->estimate();
            current_frame->Tcw = {Tcw.rotation(), Tcw.translation()};

            // count the outliers
            for (size_t i = 0; i < edges.size(); ++i) {
                auto e = edges[i];
                if (is_outlier[i]) {
                    e->computeError();
                }
                if (e->chi2() > chi2_th) {
                    is_outlier[i] = true;
                    e->setLevel(1);
                } else {
                    is_outlier[i] = false;
                    e->setLevel(0);
                }

                if (iteration == 2) {
                    e->setRobustKernel(nullptr);
                }
            }
        }

        // 从frame points中剔除outliers
        std::vector<ObjManager<FramePoint>::Ptr> new_frame_points;
        for (int i = 0, outlier_id = 0; i < current_frame->frame_points.size(); i++) {
            if (current_frame->frame_points[i]->map_point->look_cnt < 2 || !is_outlier[outlier_id++]) {
                new_frame_points.emplace_back(current_frame->frame_points[i]);
            }
        }
        current_frame->frame_points = std::move(new_frame_points);

        // 更新地图点坐标
        for (const auto &[id, p_ptw]: ObjManager<MapPoint>::set()) {
            auto &ptw = *p_ptw.lock();
            if (2 <= ptw.look_cnt && ptw.look_cnt < point_world_look_cnt) {
                ptw.pt = ptw.v_xyz->estimate();
            }
        }
    }

    std::list<ObjManager<Frame>::Ptr> history_frames;
    ObjManager<Frame>::Ptr current_frame;

    // parameters
    size_t max_history_frame_size{};
    size_t max_frame_point_size{};
    size_t point_world_look_cnt{};
    cv::Mat K, C;
    double cx, cy, fx, fy;
};

int main() {
    cv::VideoCapture cam("/home/xinyang/Videos/part-monoslam.mp4");
//    cv::VideoCapture cam(0);
//    MindVision cam("../config/MV-SUA133GC.Config");
//    assert(cam.init());

    Slam slam("../config/param.yml");

    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    vis.setViewerPose(cam_pose);
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget("World", world_coor);
    vis.showWidget("Camera", camera_coor);

    int k = 0;
    while (k != 'q') {
        cv::Mat src, im2show;
        ObjManager<Frame>::Ptr frame;
        cam.read(src);
        cv::resize(src, src, {640, 384});
        im2show = src.clone();
        {
            boost::timer::auto_cpu_timer timer("total: %ws\n");
            frame = slam(src);
            std::cout << "R:\n" << frame->Tcw.rotationMatrix() << std::endl;
            std::cout << "t:\n" << frame->Tcw.translation() << std::endl;
        }

        for (const auto &fp: frame->frame_points) {
            cv::Scalar color;
            if (fp->map_point->look_cnt >= 2) {
                color = {0, 255, 0};
            } else {
                color = {0, 0, 255};
            }
            cv::circle(im2show, fp->pt, 3, color, 2);
        }

        Sophus::SE3d Tcw = frame->Tcw.inverse();
        cv::Affine3d M(
                cv::Affine3d::Mat3(
                        Tcw.rotationMatrix()(0, 0), Tcw.rotationMatrix()(0, 1), Tcw.rotationMatrix()(0, 2),
                        Tcw.rotationMatrix()(1, 0), Tcw.rotationMatrix()(1, 1), Tcw.rotationMatrix()(1, 2),
                        Tcw.rotationMatrix()(2, 0), Tcw.rotationMatrix()(2, 1), Tcw.rotationMatrix()(2, 2)
                ),
                cv::Affine3d::Vec3(
                        Tcw.translation()(0, 0), Tcw.translation()(1, 0), Tcw.translation()(2, 0)
                )
        );

        cv::imshow("keypoint", im2show);
        vis.setWidgetPose("Camera", M);
        vis.spinOnce(0, false);
        k = cv::waitKey(1);
    }

    return 0;
}
