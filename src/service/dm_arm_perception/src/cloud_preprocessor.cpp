#include <future>
#include <atomic>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include "dm_arm_msgs/CommandOctomap.h" // IWYU pragma: keep

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

class CloudProprocessor {
public:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;

    CloudProprocessor()
        : nh_(), pnh_("~"),
        color_sub_(nh_, get_param<std::string>("topics/color_image", "/dm_arm/camera/orbbec/color/image_raw"), 1),
        depth_sub_(nh_, get_param<std::string>("topics/depth_image", "/dm_arm/camera/orbbec/depth_registered/image_raw"), 1),
        depth_info_sub_(nh_, get_param<std::string>("topics/depth_info", "/dm_arm/camera/orbbec/depth_registered/camera_info"), 1),
        tf_listener_(tf_buffer_) {

        target_frame_ = get_param<std::string>("target_frame", "base_link");
        pixel_stride_ = get_param<int>("pixel_stride", 4);
        frame_skip_ = get_param<int>("frame_skip", 1);

        min_depth_ = get_param<double>("min_depth", 0.15);
        max_depth_ = get_param<double>("max_depth", 1.50);

        min_x_ = get_param<double>("min_x", 0.0);
        max_x_ = get_param<double>("max_x", 0.8);
        min_y_ = get_param<double>("min_y", -0.5);
        max_y_ = get_param<double>("max_y", 0.5);
        min_z_ = get_param<double>("min_z", 0.0);
        max_z_ = get_param<double>("max_z", 1.2);

        voxel_leaf_ = get_param<double>("voxel_leaf", 0.015);
        sor_mean_k_ = get_param<int>("sor_mean_k", 20);
        sor_stddev_ = get_param<double>("sor_stddev", 1.0);

        publish_raw_ = get_param<bool>("publish_raw", true);
        publish_base_ = get_param<bool>("publish_base", true);
        publish_filtered_ = get_param<bool>("publish_filtered", true);

        raw_pub_ = nh_.advertise <sensor_msgs::PointCloud2>("/dm_arm/perception/cloud/raw", 1);
        base_pub_ = nh_.advertise <sensor_msgs::PointCloud2>("/dm_arm/perception/cloud/base", 1);
        filtered_pub_ = nh_.advertise <sensor_msgs::PointCloud2>("/dm_arm/perception/cloud/filtered", 1);

        octomap_srv_ = nh_.advertiseService("/dm_arm/perception/set_octomap_enabled", &CloudProprocessor::on_request, this);
        clear_octomap_client_ = nh_.serviceClient<std_srvs::Empty>("/clear_octomap");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), color_sub_, depth_sub_, depth_info_sub_);
        sync_->registerCallback(boost::bind(&CloudProprocessor::callback, this, _1, _2, _3));

        ROS_INFO("CloudPreprocessor 已启动，订阅彩色图像: %s, 深度图像: %s, 相机信息: %s", color_sub_.getTopic().c_str(), depth_sub_.getTopic().c_str(), depth_info_sub_.getTopic().c_str());
    }

private:
    template<typename T>
    T get_param(const std::string& name, const T& default_value) {
        T value;
        pnh_.param(name, value, default_value);

        return value;
    }

    void callback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& depth_info_msg) {
        if(frame_skip_ > 1 && (frame_count_++ % frame_skip_ != 0)) return;

        cv_bridge::CvImageConstPtr color_cv_ptr;
        cv_bridge::CvImageConstPtr depth_cv_ptr;
        try {
            color_cv_ptr = cv_bridge::toCvShare(color_msg, "bgr8");
            depth_cv_ptr = cv_bridge::toCvShare(depth_msg);
        }
        catch(const cv_bridge::Exception& e) {
            ROS_WARN("cv_bridge 异常: %s", e.what());
            return;
        }

        if(depth_cv_ptr->image.type() != CV_32FC1) {
            ROS_WARN("不支持的深度图像类型（仅支持 CV_32FC1）: %d", depth_cv_ptr->image.type());
            return;
        }

        const auto& color = color_cv_ptr->image;
        const auto& depth = depth_cv_ptr->image;

        if(color.rows != depth.rows || color.cols != depth.cols) {
            ROS_WARN("彩色图像和深度图像尺寸不匹配: color(%d x %d), depth(%d x %d)", color.cols, color.rows, depth.cols, depth.rows);
            return;
        }

        const double fx = depth_info_msg->K[0];
        const double fy = depth_info_msg->K[4];
        const double cx = depth_info_msg->K[2];
        const double cy = depth_info_msg->K[5];

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        raw_cloud_ptr->header.frame_id = depth_msg->header.frame_id;
        raw_cloud_ptr->is_dense = false;

        for(int v = 0; v < depth.rows; v += pixel_stride_) {
            for(int u = 0; u < depth.cols; u += pixel_stride_) {
                const float z = depth.at<float>(v, u);
                if(!std::isfinite(z) || z < min_depth_ || z > max_depth_) continue;

                const auto& bgr = color.at<cv::Vec3b>(v, u);

                pcl::PointXYZRGB point;
                point.x = static_cast<float>((u - cx) * z / fx);
                point.y = static_cast<float>((v - cy) * z / fy);
                point.z = z;
                point.b = bgr[0];
                point.g = bgr[1];
                point.r = bgr[2];
                raw_cloud_ptr->points.push_back(point);
            }
        }

        raw_cloud_ptr->width = static_cast<uint32_t>(raw_cloud_ptr->points.size());
        raw_cloud_ptr->height = 1;

        sensor_msgs::PointCloud2 raw_msg;
        pcl::toROSMsg(*raw_cloud_ptr, raw_msg);
        raw_msg.header = depth_msg->header;

        if(publish_raw_) raw_pub_.publish(raw_msg);

        sensor_msgs::PointCloud2 base_msg = raw_msg;
        if(!target_frame_.empty() && raw_msg.header.frame_id != target_frame_) {
            const ros::Time query_stamp = raw_msg.header.stamp;
            geometry_msgs::TransformStamped tf;
            bool tf_ok = false;

            try {
                tf = tf_buffer_.lookupTransform(target_frame_, raw_msg.header.frame_id, query_stamp, ros::Duration(0.10));
                tf_ok = true;
            }
            catch(const tf2::ExtrapolationException& e) {
                ROS_WARN("TF extrapolation at image stamp %.6f, fallback to latest transform: %s", query_stamp.toSec(), e.what());

                try {
                    tf = tf_buffer_.lookupTransform(target_frame_, raw_msg.header.frame_id, ros::Time(0), ros::Duration(0.10));
                    tf_ok = true;
                }
                catch(const tf2::TransformException& e2) {
                    ROS_WARN("TF fallback failed: %s", e2.what());
                    return;
                }
            }
            catch(const tf2::TransformException& e) {
                ROS_WARN("TF transform failed: %s", e.what());
                return;
            }

            if(tf_ok) {
                tf2::doTransform(raw_msg, base_msg, tf);
                base_msg.header.frame_id = target_frame_;

                if(!tf.header.stamp.isZero()) base_msg.header.stamp = tf.header.stamp;
                else base_msg.header.stamp = query_stamp;
            }
        }

        if(publish_base_) base_pub_.publish(base_msg);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(base_msg, *base_cloud_ptr);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_x_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_y_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_z_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(base_cloud_ptr);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(min_x_, max_x_);
        pass.filter(*filtered_x_ptr);

        pass.setInputCloud(filtered_x_ptr);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(min_y_, max_y_);
        pass.filter(*filtered_y_ptr);

        pass.setInputCloud(filtered_y_ptr);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_z_, max_z_);
        pass.filter(*filtered_z_ptr);

        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        voxel.setInputCloud(filtered_z_ptr);
        voxel.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
        voxel.filter(*voxel_cloud_ptr);

        if(static_cast<int>(voxel_cloud_ptr->points.size()) > sor_mean_k_) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
            sor.setInputCloud(voxel_cloud_ptr);
            sor.setMeanK(sor_mean_k_);
            sor.setStddevMulThresh(sor_stddev_);
            sor.filter(*final_cloud_ptr);
        }
        else final_cloud_ptr = voxel_cloud_ptr;

        final_cloud_ptr->header.frame_id = target_frame_.empty() ? base_msg.header.frame_id : target_frame_;
        final_cloud_ptr->width = static_cast<uint32_t>(final_cloud_ptr->points.size());
        final_cloud_ptr->height = 1;
        final_cloud_ptr->is_dense = false;

        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*final_cloud_ptr, filtered_msg);
        filtered_msg.header.stamp = base_msg.header.stamp;
        filtered_msg.header.frame_id = final_cloud_ptr->header.frame_id;

        if(publish_filtered_ && octomap_enabled_) filtered_pub_.publish(filtered_msg);
    }

    bool on_request(dm_arm_msgs::CommandOctomapRequest& req, dm_arm_msgs::CommandOctomapResponse& res) {
        octomap_enabled_ = req.enabled;

        if(req.clear_octomap) {
            if(octomap_clearing_.exchange(true)) {
                ROS_WARN("正在清除 Octomap，请稍后再试");
                res.success = false;
                res.message = "Clear Octomap in progress, please try again later";
                return true;
            }

            clear_octomap_future_ = std::async(std::launch::async, [this]() {
                std_srvs::Empty srv;
                if(!clear_octomap_client_.waitForExistence(ros::Duration(0.5))) {
                    ROS_WARN("等待 clear_octomap 服务超时，无法清除 Octomap");
                    octomap_clearing_.store(false);
                    return;
                }
                if(!clear_octomap_client_.call(srv)) {
                    ROS_WARN("调用 clear_octomap 服务失败，无法清除 Octomap");
                    octomap_clearing_.store(false);
                    return;
                }
                ROS_INFO("Octomap 已清除");
                octomap_clearing_.store(false);
                });

            res.message = "Clearing Octomap...(asynchronous)";
        }

        res.success = true;
        res.message += octomap_enabled_ ? "; Octomap enabled" : "; Octomap disabled";
        ROS_INFO("Octomap 已%s", octomap_enabled_ ? "启用" : "禁用");
        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    message_filters::Subscriber<sensor_msgs::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    ros::Publisher raw_pub_;
    ros::Publisher base_pub_;
    ros::Publisher filtered_pub_;

    ros::ServiceServer octomap_srv_;
    ros::ServiceClient clear_octomap_client_;
    std::future<void> clear_octomap_future_;
    std::atomic<bool> octomap_clearing_{ false };

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string target_frame_;
    int pixel_stride_{ 4 };
    int frame_skip_{ 1 };
    int frame_count_{ 0 };

    double min_depth_{ 0.15 };
    double max_depth_{ 1.50 };

    double min_x_{ 0.0 };
    double max_x_{ 0.8 };
    double min_y_{ -0.5 };
    double max_y_{ 0.5 };
    double min_z_{ 0.0 };
    double max_z_{ 1.2 };

    double voxel_leaf_{ 0.015 };
    int sor_mean_k_{ 20 };
    double sor_stddev_{ 1.0 };

    bool publish_raw_{ true };
    bool publish_base_{ true };
    bool publish_filtered_{ true };
    bool octomap_enabled_{ true };
};

// ! ========================= 私 有 函 数 实 现 ========================= ! //

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_preprocessor");
    CloudProprocessor processor;
    ros::spin();

    return 0;
}
