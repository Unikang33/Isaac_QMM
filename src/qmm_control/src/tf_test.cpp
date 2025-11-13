#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <optional>
#include <cmath>
#include <iomanip>

class TFSubscriber : public rclcpp::Node
{
public:
    TFSubscriber() : Node("tf_subscriber_node") {
        tf_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf",
            10,
            std::bind(&TFSubscriber::tf_callback, this, std::placeholders::_1));        
    }
    
    std::optional<Eigen::Matrix4d> get_world_to_sgr532_link1() const {
        return world_to_sgr532_link1_;
    }
    
    std::optional<Eigen::Matrix4d> get_sgr532_link1_to_link2() const {
        return sgr532_link1_to_link2_;
    }
    
    std::optional<Eigen::Matrix4d> get_sgr532_link2_to_link3() const {
        return sgr532_link2_to_link3_;
    }
    
    std::optional<Eigen::Matrix4d> get_sgr532_link3_to_link4() const {
        return sgr532_link3_to_link4_;
    }
    
    std::optional<Eigen::Matrix4d> get_sgr532_link4_to_link5() const {
        return sgr532_link4_to_link5_;
    }
    
    std::optional<Eigen::Matrix4d> get_sgr532_link5_to_link6() const {
        return sgr532_link5_to_link6_;
    }
    
    std::optional<Eigen::Matrix4d> get_sgr532_base_link_to_base() const {
        return sgr532_base_link_to_base_;
    }
    
    std::optional<Eigen::Matrix4d> get_sgr532_link1_to_base_link() const {
        return sgr532_link1_to_base_link_;
    }

private:
    Eigen::Matrix4d transform_to_matrix(const geometry_msgs::msg::TransformStamped& transform) {
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        
        matrix(0, 3) = transform.transform.translation.x;
        matrix(1, 3) = transform.transform.translation.y;
        matrix(2, 3) = transform.transform.translation.z;
        
        Eigen::Quaterniond q(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z
        );
        
        Eigen::Matrix3d rotation = q.toRotationMatrix();
        matrix.block<3, 3>(0, 0) = rotation;
        
        return matrix;
    }
    
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto& transform : msg->transforms)
        {
            std::string parent_frame = transform.header.frame_id;
            std::string child_frame = transform.child_frame_id;
            
            if (parent_frame == "world" && child_frame == "sgr532_link1") {
                world_to_sgr532_link1_ = transform_to_matrix(transform);
            }
            else if (parent_frame == "sgr532_link1" && child_frame == "sgr532_link2") {
                sgr532_link1_to_link2_ = transform_to_matrix(transform);
            }
            else if (parent_frame == "sgr532_link1" && child_frame == "sgr532_base_link") {
                sgr532_link1_to_base_link_ = transform_to_matrix(transform);
            }
            else if (parent_frame == "sgr532_link2" && child_frame == "sgr532_link3") {
                sgr532_link2_to_link3_ = transform_to_matrix(transform);
            }
            else if (parent_frame == "sgr532_link3" && child_frame == "sgr532_link4") {
                sgr532_link3_to_link4_ = transform_to_matrix(transform);
            }
            else if (parent_frame == "sgr532_link4" && child_frame == "sgr532_link5") {
                sgr532_link4_to_link5_ = transform_to_matrix(transform);
            }
            else if (parent_frame == "sgr532_link5" && child_frame == "sgr532_link6") {
                sgr532_link5_to_link6_ = transform_to_matrix(transform);
            }
            else if (parent_frame == "sgr532_base_link" && child_frame == "base") {
                sgr532_base_link_to_base_ = transform_to_matrix(transform);
            }
        }
    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
    
    std::optional<Eigen::Matrix4d> world_to_sgr532_link1_;
    std::optional<Eigen::Matrix4d> sgr532_link1_to_link2_;
    std::optional<Eigen::Matrix4d> sgr532_link1_to_base_link_;
    std::optional<Eigen::Matrix4d> sgr532_link2_to_link3_;
    std::optional<Eigen::Matrix4d> sgr532_link3_to_link4_;
    std::optional<Eigen::Matrix4d> sgr532_link4_to_link5_;
    std::optional<Eigen::Matrix4d> sgr532_link5_to_link6_;
    std::optional<Eigen::Matrix4d> sgr532_base_link_to_base_;
};

// 회전 행렬에서 Roll, Pitch, Yaw 추출 (ZYX Euler angles)
Eigen::Vector3d GetRollPitchYaw(const Eigen::Matrix4d& pose) {
    Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
    
    double roll, pitch, yaw;
    
    // pitch 계산 (Y축 회전)
    double sin_pitch = -R(0, 2);
    
    // Gimbal lock 체크
    if (std::abs(sin_pitch) >= 1.0) {
        // Gimbal lock 발생 (pitch = ±90°)
        pitch = std::copysign(M_PI / 2.0, sin_pitch);
        roll = 0.0;
        yaw = std::atan2(-R(1, 0), R(1, 1));
    } else {
        pitch = std::asin(sin_pitch);
        roll = std::atan2(R(1, 2), R(2, 2));
        yaw = std::atan2(R(0, 1), R(0, 0));
    }
    
    return Eigen::Vector3d(roll, pitch, yaw);
}

// Radian을 Degree로 변환
Eigen::Vector3d RadiansToDegrees(const Eigen::Vector3d& radians) {
    return radians * 180.0 / M_PI;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TFSubscriber>();

    rclcpp::Rate rate(10);
    while(rclcpp::ok()) {
        rclcpp::spin_some(node);

        auto tf_world_to_link1 = node->get_world_to_sgr532_link1();
        auto tf_link1_to_link2 = node->get_sgr532_link1_to_link2();
        auto tf_link1_to_base_link = node->get_sgr532_link1_to_base_link();
        auto tf_link2_to_link3 = node->get_sgr532_link2_to_link3();
        auto tf_link3_to_link4 = node->get_sgr532_link3_to_link4();
        auto tf_link4_to_link5 = node->get_sgr532_link4_to_link5();
        auto tf_link5_to_link6 = node->get_sgr532_link5_to_link6();
        auto tf_base_link_to_base = node->get_sgr532_base_link_to_base();

        if (tf_world_to_link1.has_value() && 
            tf_link1_to_link2.has_value() && 
            tf_link1_to_base_link.has_value() &&
            tf_link2_to_link3.has_value() &&
            tf_link3_to_link4.has_value() &&
            tf_link4_to_link5.has_value() &&
            tf_link5_to_link6.has_value() &&
            tf_base_link_to_base.has_value())
        {
            // World to sgr532_link6 변환 계산
            Eigen::Matrix4d tf_world_to_link6 = tf_world_to_link1.value() * 
                                                 tf_link1_to_link2.value() * 
                                                 tf_link2_to_link3.value() *
                                                 tf_link3_to_link4.value() *
                                                 tf_link4_to_link5.value() *
                                                 tf_link5_to_link6.value();
            
            // World to GO1 base 변환 계산
            Eigen::Matrix4d tf_world_to_go1_base = tf_world_to_link1.value() * 
                                                     tf_link1_to_base_link.value() * 
                                                     tf_base_link_to_base.value();
            
            // World to sgr532_base_link (매니퓰레이터 베이스) 변환 계산
            Eigen::Matrix4d tf_world_to_manip_base = tf_world_to_link1.value() * 
                                                       tf_link1_to_base_link.value();
            
            // sgr532_base_link to sgr532_link6 변환 계산 (매니퓰레이터 베이스 기준)
            Eigen::Matrix4d tf_manip_base_to_link6 = tf_world_to_manip_base.inverse() * tf_world_to_link6;
            
            // World to sgr532_link6 출력
            std::cout << "\n========== World to sgr532_link6 Transform ==========" << std::endl;
            
            // Position 출력
            std::cout << "Position (x, y, z): " 
                      << std::fixed << std::setprecision(6)
                      << tf_world_to_link6(0, 3) << ", "
                      << tf_world_to_link6(1, 3) << ", "
                      << tf_world_to_link6(2, 3) << std::endl;
            
            // Roll, Pitch, Yaw 계산 및 출력
            Eigen::Vector3d rpy_rad = GetRollPitchYaw(tf_world_to_link6);
            Eigen::Vector3d rpy_deg = RadiansToDegrees(rpy_rad);
            
            std::cout << "Orientation (Roll, Pitch, Yaw):" << std::endl;
            std::cout << "  Radians: " << rpy_rad[0] << ", " << rpy_rad[1] << ", " << rpy_rad[2] << std::endl;
            std::cout << "  Degrees: " << rpy_deg[0] << "°, " << rpy_deg[1] << "°, " << rpy_deg[2] << "°" << std::endl;
            
            std::cout << "\nTransformation Matrix:" << std::endl;
            std::cout << tf_world_to_link6 << std::endl;
            std::cout << "=====================================================\n" << std::endl;
            
            // sgr532_base_link (매니퓰레이터 베이스) to sgr532_link6 출력
            std::cout << "\n========== sgr532_base_link to sgr532_link6 Transform ==========" << std::endl;
            
            // Position 출력
            std::cout << "Position (x, y, z): " 
                      << std::fixed << std::setprecision(6)
                      << tf_manip_base_to_link6(0, 3) << ", "
                      << tf_manip_base_to_link6(1, 3) << ", "
                      << tf_manip_base_to_link6(2, 3) << std::endl;
            
            // Roll, Pitch, Yaw 계산 및 출력
            Eigen::Vector3d manip_rpy_rad = GetRollPitchYaw(tf_manip_base_to_link6);
            Eigen::Vector3d manip_rpy_deg = RadiansToDegrees(manip_rpy_rad);
            
            std::cout << "Orientation (Roll, Pitch, Yaw):" << std::endl;
            std::cout << "  Radians: " << manip_rpy_rad[0] << ", " << manip_rpy_rad[1] << ", " << manip_rpy_rad[2] << std::endl;
            std::cout << "  Degrees: " << manip_rpy_deg[0] << "°, " << manip_rpy_deg[1] << "°, " << manip_rpy_deg[2] << "°" << std::endl;
            
            std::cout << "\nTransformation Matrix:" << std::endl;
            std::cout << tf_manip_base_to_link6 << std::endl;

            std::cout << "World to GO1 base Transform:" << std::endl;
            std::cout << tf_world_to_go1_base << std::endl;

            std::cout << "World to sgr532_base_link Transform:" << std::endl;
            std::cout << tf_world_to_manip_base << std::endl;

            std::cout << "================================================================\n" << std::endl;
        }
        
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}

