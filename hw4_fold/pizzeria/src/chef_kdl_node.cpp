#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>
#include <string>
#include <vector>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

// --- NUOVO: Header per il messaggio Bool ---
#include "std_msgs/msg/bool.hpp" 

#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

#include "pizzeria/kdl_robot.h"
#include "pizzeria/kdl_control.h"
#include "pizzeria/kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

enum RobotState {
    WAIT_FOR_TAG, 
    APPROACH,     
    DESCEND,      
    GRASP,        
    LIFT,
    RETURN,           
    LOWER_TO_WAITER,  
    RESET_TO_HOME,    
    FINISH        
};

class ChefKDLNode : public rclcpp::Node
{
    public:
        ChefKDLNode()
        : Node("chef_kdl_node"), 
        node_handle_(std::shared_ptr<ChefKDLNode>(this))
        {
            // ... (Codice esistente invariato) ...
            declare_parameter("cmd_interface", "position");
            get_parameter("cmd_interface", cmd_interface_);
            traj_type_ = "linear"; 
            
            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false; 
            robot_state_ = WAIT_FOR_TAG; 
            pizza_attached_ = false; 

            std::string robot_desc_string;
            declare_parameter("robot_description", "");
            get_parameter("robot_description", robot_desc_string);
            
            if(robot_desc_string.empty()) {
                 RCLCPP_ERROR(get_logger(), "Parametro robot_description vuoto!");
                 return;
            }

            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(robot_desc_string, robot_tree)){
                RCLCPP_ERROR(get_logger(), "Failed to construct kdl tree");
                return;
            }
            
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;
            q_max.data <<  2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96;
            robot_->setJntLimits(q_min,q_max);            
            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj);
            desired_commands_.resize(nj);

            // --- Subscribers ---
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&ChefKDLNode::joint_state_subscriber, this, std::placeholders::_1));

            arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10, std::bind(&ChefKDLNode::aruco_callback, this, std::placeholders::_1));

            cmdPublisher_ = this->create_publisher<FloatArray>("/chef_position_controller/commands", 10);

            // --- NUOVO: Publisher per dare il via al Waiter ---
            start_delivery_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/start_delivery", 10);

            set_entity_pose_client_ = this->create_client<ros_gz_interfaces::srv::SetEntityPose>(
                "/world/pizzeria_world/set_pose"
            );

            RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
            while(!joint_state_available_){
                rclcpp::spin_some(node_handle_);
            }

            // Update robot
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            // Setup Target Iniziali
            init_cart_pose_ = robot_->getEEFrame();
            target_pos_hover_ = Eigen::Vector3d(0.2, 0.5, 1.25);
            target_pos_grasp_ = Eigen::Vector3d(0.2, 0.5, 1.05); 
            target_rotation_ = KDL::Rotation::RotY(M_PI); 
            target_pos_lower_ = Eigen::Vector3d(0.0, 0.0, 0.0);

            traj_duration_ = 1.0; 
            double acc_duration = 0.3;
            Eigen::Vector3d start_pos_vec(init_cart_pose_.p.data);
            planner_ = KDLPlanner(traj_duration_, acc_duration, start_pos_vec, target_pos_hover_);

            timer_ = this->create_wall_timer(std::chrono::milliseconds(20), 
                                             std::bind(&ChefKDLNode::control_loop, this));
            
            RCLCPP_INFO(this->get_logger(), "Node Ready. SERVICE TELEPORT MODE.");
        }

    private:

        void teleport_pizza() {
            // (Codice esistente invariato)
            if (!set_entity_pose_client_->service_is_ready()) return; 
            KDL::Frame current_ee_pose = robot_->getEEFrame();
            auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
            request->entity.name = "my_pizza"; 
            request->pose.position.x = current_ee_pose.p.x();
            request->pose.position.y = current_ee_pose.p.y();
            request->pose.position.z = current_ee_pose.p.z() - 0.10;
            double qx, qy, qz, qw;
            current_ee_pose.M.GetQuaternion(qx, qy, qz, qw);
            request->pose.orientation.x = qx;
            request->pose.orientation.y = qy;
            request->pose.orientation.z = qz;
            request->pose.orientation.w = qw;
            auto result_future = set_entity_pose_client_->async_send_request(request);
        }

        void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            // (Codice esistente invariato)
            if (robot_state_ == WAIT_FOR_TAG) {
                RCLCPP_INFO(get_logger(), "!!! ARUCO DETECTED !!!");
                robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
                init_cart_pose_ = robot_->getEEFrame();
                waiter_pos_ = Eigen::Vector3d(init_cart_pose_.p.data); 
                Eigen::Vector3d start_pos_vec(init_cart_pose_.p.data);
                planner_ = KDLPlanner(traj_duration_, 0.5, start_pos_vec, target_pos_hover_);
                robot_state_ = APPROACH;
                t_ = 0; 
                (void)msg; 
            }
        }

        void control_loop(){
            
            if (robot_state_ == WAIT_FOR_TAG) return;

            // (Codice esistente per calcolo cinematica e stati...)
            double dt = 0.02; 
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            KDL::Frame current_pose = robot_->getEEFrame();
            Eigen::Vector3d current_pos_vec(current_pose.p.data);

            if (pizza_attached_) teleport_pizza();

            double error_norm = 100.0; 
            
            if (robot_state_ == APPROACH) error_norm = (current_pos_vec - target_pos_hover_).norm();
            else if (robot_state_ == DESCEND) error_norm = (current_pos_vec - target_pos_grasp_).norm();
            else if (robot_state_ == LIFT) error_norm = (current_pos_vec - target_pos_hover_).norm();
            else if (robot_state_ == RETURN) error_norm = (current_pos_vec - waiter_pos_).norm();
            else if (robot_state_ == LOWER_TO_WAITER) error_norm = (current_pos_vec - target_pos_lower_).norm();
            else if (robot_state_ == RESET_TO_HOME) error_norm = (current_pos_vec - waiter_pos_).norm();

            if (t_ <= traj_duration_) t_ += dt;

            // --- MACCHINA A STATI ---
            if (t_ >= traj_duration_) {
                
                // ... (Altri stati invariati: APPROACH, DESCEND, GRASP, LIFT, RETURN, LOWER_TO_WAITER) ...
                if (robot_state_ == APPROACH) {
                    if (error_norm < 0.05) { 
                        RCLCPP_INFO(get_logger(), "Hover Reached. Descending...");
                        robot_state_ = DESCEND; t_ = 0; traj_duration_ = 1.5; 
                        planner_ = KDLPlanner(traj_duration_, 0.5, target_pos_hover_, target_pos_grasp_);
                        init_cart_pose_.M = target_rotation_; 
                    }
                } 
                else if (robot_state_ == DESCEND) {
                      if (error_norm < 0.02) { 
                        RCLCPP_INFO(get_logger(), "Grasp Reached.");
                        pizza_attached_ = true; robot_state_ = GRASP; t_ = 0; traj_duration_ = 2.0; 
                        planner_ = KDLPlanner(traj_duration_, 0.5, target_pos_grasp_, target_pos_grasp_);
                        init_cart_pose_.M = target_rotation_;
                      }
                }
                else if (robot_state_ == GRASP) {
                    RCLCPP_INFO(get_logger(), "Lifting...");
                    robot_state_ = LIFT; t_ = 0; traj_duration_ = 3.0; 
                    planner_ = KDLPlanner(traj_duration_, 0.5, target_pos_grasp_, target_pos_hover_);
                    init_cart_pose_.M = target_rotation_;
                }
                else if (robot_state_ == LIFT) {
                    if (error_norm < 0.05) {
                         RCLCPP_INFO(get_logger(), "Moving to Waiter...");
                         robot_state_ = RETURN; t_ = 0; traj_duration_ = 3.0; 
                         planner_ = KDLPlanner(traj_duration_, 0.5, target_pos_hover_, waiter_pos_);
                         init_cart_pose_.M = target_rotation_; 
                    }
                }
                else if (robot_state_ == RETURN) {
                    if (error_norm < 0.05) {
                         RCLCPP_INFO(get_logger(), "Lowering...");
                         robot_state_ = LOWER_TO_WAITER; t_ = 0; traj_duration_ = 3.0;
                         target_pos_lower_ = waiter_pos_; target_pos_lower_.z() -= 0.20; 
                         planner_ = KDLPlanner(traj_duration_, 0.5, waiter_pos_, target_pos_lower_);
                         init_cart_pose_.M = target_rotation_;
                    }
                }
                else if (robot_state_ == LOWER_TO_WAITER) {
                    if (error_norm < 0.02) {
                         RCLCPP_INFO(get_logger(), "Pizza Placed. Resetting Home...");
                         pizza_attached_ = false; robot_state_ = RESET_TO_HOME; t_ = 0; traj_duration_ = 3.0;
                         planner_ = KDLPlanner(traj_duration_, 0.5, target_pos_lower_, waiter_pos_);
                         init_cart_pose_.M = target_rotation_; 
                    }
                }
                // --- QUI LA MODIFICA PRINCIPALE ---
                else if (robot_state_ == RESET_TO_HOME) {
                    if (error_norm < 0.02) {
                         RCLCPP_INFO(get_logger(), "Back at Start Position.");
                         
                         // --- NUOVO: Invio segnale al Waiter ---
                         std_msgs::msg::Bool msg;
                         msg.data = true;
                         start_delivery_publisher_->publish(msg);
                         RCLCPP_INFO(get_logger(), ">> SIGNAL SENT: Waiter can GO! <<");
                         // --------------------------------------

                         robot_state_ = FINISH;
                    }
                }
            }

            // ... (Cinematica invariata) ...
             if (robot_state_ != FINISH){
                trajectory_point p = planner_.linear_traj_trapezoidal(t_);
                KDL::Rotation R_start = init_cart_pose_.M;
                KDL::Rotation R_end = target_rotation_;
                double s = (traj_duration_ > 0) ? (t_ / traj_duration_) : 1.0;
                if (s > 1.0) s = 1.0;
                KDL::Rotation R_diff = R_start.Inverse() * R_end; 
                KDL::Vector rot_axis;
                double rot_angle = R_diff.GetRotAngle(rot_axis); 
                double scaled_angle = rot_angle * s;
                KDL::Rotation current_des_rot = R_start * KDL::Rotation::Rot(rot_axis, scaled_angle);
                KDL::Frame desFrame;
                desFrame.M = current_des_rot;
                desFrame.p = toKDL(p.pos);
                joint_positions_cmd_ = joint_positions_; 
                robot_->getInverseKinematics(desFrame, joint_positions_cmd_); 
                for (unsigned int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            } 
            else {
                iteration_++;
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
            joint_state_available_ = true;
            if (sensor_msg.position.size() >= 7) {
                for (unsigned int i = 0; i < 7; i++){
                    joint_positions_.data[i] = sensor_msg.position[i];
                    joint_velocities_.data[i] = sensor_msg.velocity[i];
                }
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_; 
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        
        // --- NUOVO: Dichiarazione Publisher ---
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_delivery_publisher_;

        rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_entity_pose_client_;

        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::Node::SharedPtr node_handle_;
        // ... (resto delle variabili invariato)
        std::vector<double> desired_commands_;
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
        bool joint_state_available_;
        bool pizza_attached_; 
        double t_;
        double traj_duration_;
        int iteration_;
        RobotState robot_state_;
        std::string cmd_interface_;
        std::string traj_type_;
        KDL::Frame init_cart_pose_;
        Eigen::Vector3d target_pos_hover_;
        Eigen::Vector3d target_pos_grasp_;
        Eigen::Vector3d waiter_pos_; 
        Eigen::Vector3d target_pos_lower_;
        KDL::Rotation target_rotation_;
};

int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChefKDLNode>());
    rclcpp::shutdown();
    return 1;
}