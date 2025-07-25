#include <cv_bridge/cv_bridge.h>
#include <hy_common/logger/logger.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32MultiArray.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>

#include "hy_common/config.h"
#include "hy_common/image/tag_detector.h"
#include "hy_manipulation_controllers/core/arm_controller.h"
using namespace std;
using namespace cv;
using namespace hy_common;
//线程防止堵塞
template <typename T>
class ThreadSafeQueue {
 public:
  void push(const T& val) {
    std::lock_guard<std::mutex> lock(m_);
    q_.push(val);
    cv_.notify_one();
  }
  bool pop(T& val) {
    std::unique_lock<std::mutex> lock(m_);
    if (q_.empty()) return false;
    val = std::move(q_.front());
    q_.pop();
    return true;
  }
  void wait_and_pop(T& val) {
    std::unique_lock<std::mutex> lock(m_);
    cv_.wait(lock, [this] { return !q_.empty(); });
    val = std::move(q_.front());
    q_.pop();
  }

 private:
  std::queue<T> q_;
  std::mutex m_;
  std::condition_variable cv_;
};
//读取内参
bool loadCameraParams(const std::string& yaml_path, cv::Mat& cameraMatrix,
                      cv::Mat& distCoeffs) {
  cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    LOG_INFO("Cannot open camera yaml file: {}", yaml_path);
    return false;
  }
  cv::FileNode n = fs["camera_matrix"];
  std::vector<double> camera_data;
  n["data"] >> camera_data;
  cameraMatrix = cv::Mat(3, 3, CV_64F, camera_data.data()).clone();

  fs["distortion_coefficients"]["data"] >> camera_data;
  distCoeffs = cv::Mat(1, 5, CV_64F, camera_data.data()).clone();

  fs.release();
  return true;
}

struct TagDetectTask {
  cv::Mat undistorted_frame;
  std_msgs::Header header;
};
//识别结果
struct TagDetectResult {
  TagDetectTask task;
  std::vector<TagInfo> tags;
};
// tag检测线程
void tag_detect_thread(ThreadSafeQueue<TagDetectTask>& in_q,
                       ThreadSafeQueue<TagDetectResult>& out_q,
                       std::atomic<bool>& running) {
  TagDetector tag_detector;
  while (running) {
    TagDetectTask task;
    in_q.wait_and_pop(task);
    if (!running) break;
    TagDetectResult result;
    result.task = task;
    cv::Mat grey_img;
    cv::cvtColor(task.undistorted_frame, grey_img, COLOR_BGR2GRAY);
    tag_detector.DoDetection2d(grey_img, result.tags);
    out_q.push(result);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_tag_detector_node");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  std::string source_dir = std::string(DK_SOURCE_DIR);
  std::string cam_yaml_path =
      source_dir + "/params/config/camera_intrinsic.yaml";

  // 加载相机内参
  cv::Mat cameraMatrix, distCoeffs;
  if (!loadCameraParams(cam_yaml_path, cameraMatrix, distCoeffs)) {
    return -1;
  }
  LOG_INFO("Loaded camera intrinsics from: {}", cam_yaml_path);

  image_transport::Publisher image_pub = it.advertise("tag_detection/image", 1);
  ros::Publisher error_pub =
      nh.advertise<std_msgs::Int32MultiArray>("tag_detection/error_xy", 10);

  const float tag_size = 0.05;  // 5cm
  float half_size = tag_size / 2.0f;
  std::vector<cv::Point3f> obj_pts{{-half_size, -half_size, 0},
                                   {half_size, -half_size, 0},
                                   {half_size, half_size, 0},
                                   {-half_size, half_size, 0}};

  // 打开摄像头
  VideoCapture cap(0);
  if (!cap.isOpened()) {
    ROS_ERROR("Can not open the camera!");
    return -1;
  }
  cap.set(CAP_PROP_FRAME_WIDTH, 640);
  cap.set(CAP_PROP_FRAME_HEIGHT, 480);
  //实时检测
  ThreadSafeQueue<TagDetectTask> in_q;
  ThreadSafeQueue<TagDetectResult> out_q;
  std::atomic<bool> running(true);
  std::thread worker(tag_detect_thread, std::ref(in_q), std::ref(out_q),
                     std::ref(running));

  ros::Rate loop_rate(200);
  while (ros::ok()) {
    cv::Mat frame, undistorted_frame;
    cap >> frame;
    if (frame.empty()) {
      LOG_WARN("Don not get the pic!");
      continue;
    }
    // 去畸变
    cv::undistort(frame, undistorted_frame, cameraMatrix, distCoeffs);
    resize(undistorted_frame, undistorted_frame, Size(640, 480));

    TagDetectTask task;
    task.undistorted_frame = undistorted_frame.clone();
    task.header.stamp = ros::Time::now();
    in_q.push(task);
    //检测结果
    TagDetectResult result;
    if (out_q.pop(result)) {
      Mat& display_img = result.task.undistorted_frame;
      auto& tags = result.tags;
      int cx = display_img.cols / 2;
      int cy = display_img.rows / 2;
      circle(display_img, Point(cx, cy), 5, Scalar(0, 0, 255), -1);

      int dx = 0, dy = 0;
      int fontface = FONT_HERSHEY_SIMPLEX;
      double fontscale = 1.0;

      for (size_t i = 0; i < tags.size(); ++i) {
        // 边框
        line(display_img, tags[i].tag_image_points_[0],
             tags[i].tag_image_points_[1], Scalar(0, 255, 0), 2);  // green
        line(display_img, tags[i].tag_image_points_[1],
             tags[i].tag_image_points_[2], Scalar(0, 0, 255), 2);  // red
        line(display_img, tags[i].tag_image_points_[2],
             tags[i].tag_image_points_[3], Scalar(255, 0, 0), 2);  // blue
        line(display_img, tags[i].tag_image_points_[3],
             tags[i].tag_image_points_[0], Scalar(255, 0, 0), 2);  // blue

        // Tag中心
        int tx = int(tags[i].tag_image_center_.x);
        int ty = int(tags[i].tag_image_center_.y);
        circle(display_img, Point(tx, ty), 5, Scalar(0, 255, 255), -1);

        // 显示Tag id
        stringstream id_ss;
        id_ss << tags[i].id_;
        string id_text = id_ss.str();
        int baseline = 0;
        Size textsize = getTextSize(id_text, fontface, fontscale, 1, &baseline);
        putText(display_img, id_text,
                Point(tx - textsize.width / 2, ty + textsize.height / 2),
                fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

        // PnP三维位姿

        std::vector<cv::Point2f> img_pts;
        for (int j = 0; j < 4; ++j)
          img_pts.push_back(tags[i].tag_image_points_[j]);
        cv::Mat rvec, tvec;
        bool ok =
            solvePnP(obj_pts, img_pts, cameraMatrix, distCoeffs, rvec, tvec);
        if (ok) {
          cv::Mat R_cv;
          cv::Rodrigues(rvec, R_cv);
          Eigen::Matrix3f R_eigen;
          Eigen::Vector3f t_eigen;
          for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c)
              R_eigen(r, c) = static_cast<float>(R_cv.at<double>(r, c));
            t_eigen(r) = static_cast<float>(tvec.at<double>(r, 0));
          }
          Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
          T.block<3, 3>(0, 0) = R_eigen;
          T.block<3, 1>(0, 3) = t_eigen;

          hy_common::geometry::Transform3D end_pose(T);

          // 提取xyz rpy
          float x, y, z, roll, pitch, yaw;
          end_pose.GetXYZ(x, y, z);
          end_pose.GetRPY(roll, pitch, yaw);

          // 打印到图片，三行
          int fontface = FONT_HERSHEY_SIMPLEX;
          double fontscale = 1.0;
          int thickness = 2;
          cv::Scalar color = cv::Scalar(0, 255, 255);  // red

          char line1[128], line2[128], line3[128];
          snprintf(line1, sizeof(line1), "x=%.3f, y=%.3f, z=%.3f", x, y, z);
          snprintf(line2, sizeof(line2), "r=%.3f, p=%.3f, y=%.3f", roll, pitch,
                   yaw);
          snprintf(line3, sizeof(line3), "dx=%d, dy=%d", tx - cx, ty - cy);

          int base_x = 10, base_y = 40, line_gap = 40;
          putText(display_img, line1, Point(base_x, base_y), fontface,
                  fontscale, color, thickness);
          putText(display_img, line2, Point(base_x, base_y + line_gap),
                  fontface, fontscale, color, thickness);
          putText(display_img, line3, Point(base_x, base_y + 2 * line_gap),
                  fontface, fontscale, color, thickness);
        }

        // // 偏差
        // if (i == 0) {
        //   dx = tx - cx;
        //   dy = ty - cy;
        //   stringstream err_ss;
        //   err_ss << "error_x:" << dx << ", error_y:" << dy;
        //   putText(display_img, err_ss.str(), Point(10, 60), fontface, 1.0,
        //           Scalar(0, 255, 255), 2);
        // }
      }

      sensor_msgs::ImagePtr img_msg =
          cv_bridge::CvImage(result.task.header, "bgr8", display_img)
              .toImageMsg();
      image_pub.publish(img_msg);

      std_msgs::Int32MultiArray error_msg;
      error_msg.data.clear();
      error_msg.data.push_back(dx);
      error_msg.data.push_back(dy);
      error_pub.publish(error_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  running = false;
  in_q.push(TagDetectTask());
  worker.join();
  cap.release();
  return 0;
}
