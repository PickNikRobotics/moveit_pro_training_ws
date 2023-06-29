#pragma once

#include <behaviortree_cpp/action_node.h>

#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <fp/all.hpp>
#include <rclcpp/client.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <builtin_interfaces/msg/duration.hpp>

using GetIK = moveit_msgs::srv::GetPositionIK;

namespace ik_batch
{
    constexpr auto kPortIDPoseArray = "pose_array";
    constexpr auto kPortIDResultsPoseArray = "pose_array_results";
    constexpr auto kServiceName = "/compute_ik";

class IkBatch : public moveit_studio::behaviors::AsyncBehaviorBase
{
public:

    class ClientInterfaceBase
    {
    public:
        ~ClientInterfaceBase() = default;

        virtual void initialize(const std::string& service_name,
                                std::chrono::duration<double> wait_for_server_timeout,
                                std::chrono::duration<double> response_timeout) = 0;

        virtual bool waitForServiceServer() const = 0;

        virtual fp::Result<typename GetIK::Response> syncSendRequest(const typename GetIK::Request& request) = 0;

        virtual void cancelRequest() = 0;
    };

    class RclcppClientInterface : public ClientInterfaceBase
    {
    public:

        RclcppClientInterface(const std::shared_ptr<rclcpp::Node> node);


        void initialize(const std::string& service_name, std::chrono::duration<double> wait_for_server_timeout,
                        std::chrono::duration<double> result_timeout) override;
        
        bool waitForServiceServer() const override;


        fp::Result<typename GetIK::Response> syncSendRequest(const typename GetIK::Request& request) override;
        
        void cancelRequest() override;

    private:
        std::shared_ptr<rclcpp::Node> node_;

        std::shared_ptr<rclcpp::CallbackGroup> reentrant_callback_group_;

        std::shared_ptr<rclcpp::Client<GetIK>> client_;

        std::chrono::duration<double> wait_for_server_timeout_;

        std::chrono::duration<double> result_timeout_;

        std::atomic_bool request_canceled_;
    };


    IkBatch(const std::string& name, const BT::NodeConfiguration& config,
                              const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

    IkBatch(const std::string& name, const BT::NodeConfiguration& config,
                              const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources,
                              std::unique_ptr<ClientInterfaceBase> client_interface);

    virtual ~IkBatch() = default;

    static BT::PortsList providedPorts();


protected:

    fp::Result<std::string> getServiceName(){
        return kServiceName;
    }

    fp::Result<std::chrono::duration<double>> getResponseTimeout()
    {
        return std::chrono::duration<double>{ -1.0 };
    };

    fp::Result<typename GetIK::Request> createRequest(const geometry_msgs::msg::Pose& p, const std::string& frame_id){
        GetIK::Request r;
        r.ik_request.group_name = "manipulator";
        r.ik_request.avoid_collisions = true;
        r.ik_request.pose_stamped.header.frame_id = frame_id;
        r.ik_request.pose_stamped.pose = p;
        builtin_interfaces::msg::Duration d;
        d.sec = 1.0;
        r.ik_request.timeout = d;
        r.ik_request.robot_state.joint_state.name = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"};
        r.ik_request.robot_state.joint_state.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        return r;
    }

    fp::Result<bool> processResponse(const typename GetIK::Response& response)
    {

//        setOutput(kPortIDResultsPoseArray, detection.pose);
        return response.error_code.val == response.error_code.SUCCESS;
    };

private:
    fp::Result<bool> doWork() override;
    fp::Result<void> doHalt() override;
    std::unique_ptr<ClientInterfaceBase> client_interface_;

    std::shared_future<fp::Result<bool>>& getFuture() override
    {
        return response_future_;
    }

    std::shared_future<fp::Result<bool>> response_future_;

    geometry_msgs::msg::PoseArray pose_array_;
    geometry_msgs::msg::PoseArray pose_array_result_;





};
}  // namespace ik_batch
