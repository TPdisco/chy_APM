#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <memory>

class StereoSplitNode : public rclcpp::Node
{
public:
    StereoSplitNode() : Node("stereo_split_node")
    {
        // 声明参数（默认路径可修改）
        this->declare_parameter<std::string>("left_calibration_file",
            "/home/disco/study/chy/src/chy_camera/biaoding/left.yaml");
        this->declare_parameter<std::string>("right_calibration_file",
            "/home/disco/study/chy/src/chy_camera/biaoding/right.yaml");

        std::string left_calib_file = this->get_parameter("left_calibration_file").as_string();
        std::string right_calib_file = this->get_parameter("right_calibration_file").as_string();

        // QoS 配置（与 Python 一致）
        rclcpp::QoS camera_qos(30);
        camera_qos.reliable();
        camera_qos.durability_volatile();

        // 订阅原始拼接图像
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/stereo/image_raw", camera_qos,
            std::bind(&StereoSplitNode::image_callback, this, std::placeholders::_1));

        // 发布左右图像及对应的 CameraInfo
        left_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_raw", camera_qos);
        right_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_raw", camera_qos);
        left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/left/camera_info", camera_qos);
        right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/right/camera_info", camera_qos);

        // 加载标定文件
        left_camera_info_ = load_camera_info(left_calib_file);
        right_camera_info_ = load_camera_info(right_calib_file);

        // 检查加载是否成功（通过 width 是否 >0 判断）
        if (left_camera_info_.width == 0 || right_camera_info_.width == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera calibration files. Shutting down.");
            rclcpp::shutdown();
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        // 验证输入图像宽度是否匹配标定（2倍左目宽度）
        if (msg->width != left_camera_info_.width * 2)
        {
            RCLCPP_WARN(this->get_logger(),
                "Input image width (%d) is not twice the calibration width (%d)!",
                msg->width, left_camera_info_.width);
            return;
        }

        cv::Mat cv_image;

        // 解码图像（支持 rgb8/bgr8 直接转换，否则尝试 MJPEG 解码）
        if (msg->encoding == "rgb8" || msg->encoding == "bgr8")
        {
            cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        else
        {
            std::vector<uint8_t> data(msg->data.begin(), msg->data.end());
            cv_image = cv::imdecode(data, cv::IMREAD_COLOR);
            if (cv_image.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "MJPEG decoding failed");
                return;
            }
        }

        int height = cv_image.rows;
        int width = cv_image.cols;
        int half_width = width / 2;

        // 分割左右图像
        cv::Mat left_img = cv_image(cv::Rect(0, 0, half_width, height));
        cv::Mat right_img = cv_image(cv::Rect(half_width, 0, half_width, height));

        // 创建新的 header，使用当前 ROS 时间
        std_msgs::msg::Header new_header;
        new_header.stamp = rclcpp::Clock().now();   // 获取当前 ROS 时间
        new_header.frame_id = "camera_link";        // 根据需要设置

        // 创建图像消息，使用新的 header
        auto left_msg = cv_bridge::CvImage(new_header, "bgr8", left_img).toImageMsg();
        auto right_msg = cv_bridge::CvImage(new_header, "bgr8", right_img).toImageMsg();
        // // 创建图像消息，复用原始时间戳
        // auto left_msg = cv_bridge::CvImage(msg->header, "bgr8", left_img).toImageMsg();
        // auto right_msg = cv_bridge::CvImage(msg->header, "bgr8", right_img).toImageMsg();

        // 设置 frame_id（可选，与 Python 一致）
        left_msg->header.frame_id = "camera_link";
        right_msg->header.frame_id = "camera_link";

        // 复制 CameraInfo 并设置时间戳和 frame_id
        auto left_info = std::make_shared<sensor_msgs::msg::CameraInfo>(left_camera_info_);
        auto right_info = std::make_shared<sensor_msgs::msg::CameraInfo>(right_camera_info_);
        // left_info->header = msg->header;
        // right_info->header = msg->header;
        left_info->header = new_header;
        right_info->header = new_header;
        left_info->header.frame_id = "camera_link";
        right_info->header.frame_id = "camera_link";

        // 原子性发布（先 CameraInfo，后图像）
        left_info_pub_->publish(*left_info);
        right_info_pub_->publish(*right_info);
        left_pub_->publish(*left_msg);
        right_pub_->publish(*right_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published stereo data at timestamp %d.%d",
                     msg->header.stamp.sec, msg->header.stamp.nanosec);
    }

    sensor_msgs::msg::CameraInfo load_camera_info(const std::string &file_path)
    {
        sensor_msgs::msg::CameraInfo camera_info;
        try
        {
            YAML::Node config = YAML::LoadFile(file_path);

            camera_info.width = config["image_width"].as<int>();
            camera_info.height = config["image_height"].as<int>();
            camera_info.distortion_model = config["distortion_model"].as<std::string>();

            // 相机矩阵 K (3x3)
            auto k_data = config["camera_matrix"]["data"].as<std::vector<double>>();
            std::copy(k_data.begin(), k_data.end(), camera_info.k.begin());

            // 畸变系数 D (变长)
            camera_info.d = config["distortion_coefficients"]["data"].as<std::vector<double>>();

            // 校正矩阵 R (3x3)
            auto r_data = config["rectification_matrix"]["data"].as<std::vector<double>>();
            std::copy(r_data.begin(), r_data.end(), camera_info.r.begin());

            // 投影矩阵 P (3x4)
            auto p_data = config["projection_matrix"]["data"].as<std::vector<double>>();
            std::copy(p_data.begin(), p_data.end(), camera_info.p.begin());

            // 输出基线信息（与 Python 一致）
            double baseline = -camera_info.p[3] / camera_info.p[0];  // P[0,3] = -f*Tx
            RCLCPP_INFO(this->get_logger(), "Camera baseline: %.3f m", baseline);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera calibration file %s: %s",
                         file_path.c_str(), e.what());
            // 返回默认构造的 CameraInfo（width 为 0，用于上层检查）
        }
        return camera_info;
    }

    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

    sensor_msgs::msg::CameraInfo left_camera_info_;
    sensor_msgs::msg::CameraInfo right_camera_info_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoSplitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
