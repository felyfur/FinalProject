#include <chrono>
#include <memory>
#include <thread>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "ros_gz_interfaces/srv/delete_entity.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class PizzaDeliveryNode : public rclcpp::Node
{
public:
    PizzaDeliveryNode() : Node("pizza_delivery_node")
    {
        // 1. Publisher per Initial Pose
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // 2. Subscriber per segnale Chef
        chef_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start_delivery", 10,
            std::bind(&PizzaDeliveryNode::chefCallback, this, std::placeholders::_1));

        // 3. Action Client per Navigazione
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // 4. Service Client per cancellare la pizza
        delete_entity_client_ = this->create_client<ros_gz_interfaces::srv::DeleteEntity>(
            "/delete_entity");

        // Avviamo il thread logico separato per non bloccare i callback ROS
        delivery_thread_ = std::thread(&PizzaDeliveryNode::executionLoop, this);
    }

    ~PizzaDeliveryNode()
    {
        if (delivery_thread_.joinable()) {
            delivery_thread_.join();
        }
    }

private:
    bool delivery_requested_ = false;
    bool initial_pose_set_ = false;

    // --- ROS HANDLES ---
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr chef_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr delete_entity_client_;
    std::thread delivery_thread_;

    // --- CALLBACKS ---
    void chefCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Signal received from Chef! Starting delivery...");
            delivery_requested_ = true;
        }
    }

    // --- HELPERS ---
    void setInitialPose()
    {
        // Aspettiamo un attimo che i subscriber si connettano
        std::this_thread::sleep_for(2s);
        
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.orientation.w = 1.0;
        // Covariance dummy per amcl (anche se qui usiamo fake localization/sim)
        msg.pose.covariance[0] = 0.25;
        msg.pose.covariance[7] = 0.25;
        msg.pose.covariance[35] = 0.0685;

        initial_pose_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Initial Pose Published.");
        initial_pose_set_ = true;
    }

    bool moveToPose(double x, double y)
    {
        if (!nav_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Navigating to: x=%f, y=%f", x, y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        // Sincronizzazione tramite Future (blocca solo questo thread, non il nodo)
        auto goal_handle_future = nav_client_->async_send_goal(goal_msg, send_goal_options);
        
        if (goal_handle_future.wait_for(10s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Send goal failed");
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }

        auto result_future = nav_client_->async_get_result(goal_handle);
        
        // Attendiamo il completamento della navigazione
        // (Polling semplice per controllare se dobbiamo uscire)
        while (result_future.wait_for(1s) != std::future_status::ready) {
            if (!rclcpp::ok()) return false;
        }

        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Navigation failed or canceled");
            return false;
        }
    }

    void deletePizzaEntity()
    {
        if (!delete_entity_client_->wait_for_service(2s)) {
            RCLCPP_WARN(this->get_logger(), "Delete Entity Service not available");
            return;
        }
        
        auto request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
        request->entity.name = "my_pizza";
        request->entity.type = 2; // MODEL

        auto result_future = delete_entity_client_->async_send_request(request);
        
        // Attendiamo il risultato
        if (result_future.wait_for(3s) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Pizza deleted (or request sent).");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to call delete service.");
        }
    }

    // --- MAIN LOGIC LOOP ---
    void executionLoop()
    {
        // Setup iniziale
        setInitialPose();
        // Flag per sapere se abbiamo già stampato il messaggio di attesa
        bool waiting_msg_printed = false;

        while (rclcpp::ok())
        {
            // 1. Wait for Chef Signal
            if (!delivery_requested_) {
                if (!waiting_msg_printed) {
                    RCLCPP_INFO(this->get_logger(), "Waiting for the Chef message...");
                    waiting_msg_printed = true; // Segna come stampato
                }
                std::this_thread::sleep_for(100ms);
                continue;
            }
            // Resettiamo il flag di stampa per la PROSSIMA volta che aspetteremo
            waiting_msg_printed = false;

            // Reset flag
            delivery_requested_ = false;

            // 2. Go to Table
            if (moveToPose(7.0, 0.3)) {
                RCLCPP_INFO(this->get_logger(), "Arrived at table. Unloading...");
                std::this_thread::sleep_for(2s);
                deletePizzaEntity();
            }

            // 3. Return to Base
            RCLCPP_INFO(this->get_logger(), "Returning to Chef...");
            moveToPose(0.18, -0.15);
            
            RCLCPP_INFO(this->get_logger(), "Back at base. Ready for next order.");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PizzaDeliveryNode>();
    rclcpp::spin(node); // Il main thread gestisce i callback
    rclcpp::shutdown();
    return 0;
}