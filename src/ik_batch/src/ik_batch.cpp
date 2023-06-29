#include <ik_batch/ik_batch.hpp>


namespace
{
    constexpr std::chrono::seconds kTimeoutWaitForServiceServer{ 3 };
constexpr std::chrono::milliseconds kResponseBusyWaitPeriod{ 100 };

inline bool clientMustBeRecreated(const std::shared_ptr<rclcpp::Client<GetIK>>& client,
                                  const std::string& new_service_name)
{
    if (client == nullptr)
    {
        return true;
    }

    if (client->get_service_name() != new_service_name)
    {
        return true;
    }

    return false;
}
}  // namespace

namespace ik_batch
{




    IkBatch::RclcppClientInterface::RclcppClientInterface(
            const std::shared_ptr<rclcpp::Node> node)
            : node_{ node }
            , reentrant_callback_group_{ node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant) }
            , request_canceled_{ false }
    {
    }

    void IkBatch::RclcppClientInterface::initialize(
            const std::string& service_name, std::chrono::duration<double> wait_for_server_timeout,
            std::chrono::duration<double> result_timeout)
    {
        if (clientMustBeRecreated(client_, service_name))
        {
            client_ = node_->create_client<GetIK>(service_name, rmw_qos_profile_services_default, reentrant_callback_group_);
        }

        wait_for_server_timeout_ = wait_for_server_timeout;
        result_timeout_ = result_timeout;
    }

    bool IkBatch::RclcppClientInterface::waitForServiceServer() const
    {
        return client_->wait_for_service(wait_for_server_timeout_);
    }

    fp::Result<typename GetIK::Response>
    IkBatch::RclcppClientInterface::syncSendRequest(const typename GetIK::Request& request)
    {
        auto response_future = client_->async_send_request(std::make_shared<typename GetIK::Request>(request));

        // Race between the following:
        // 1) the response future completes
        // 2a) (if result_timeout < 0.0)  forever
        // 2b) (if result_timeout >= 0.0) result_timeout expires
        const auto start_time = std::chrono::steady_clock::now();
        while (true)
        {
            // Done: the response has been returned
            if (response_future.wait_for(kResponseBusyWaitPeriod) == std::future_status::ready)
            {
                return *response_future.get();
            }

            // Done: the caller has requested to cancel
            if (request_canceled_.exchange(false))
            {
                return tl::make_unexpected(fp::Timeout("Request canceled before service response."));
            }

            // Done: a timeout was specified, and was reached
            const auto waiting_duration = std::chrono::steady_clock::now() - start_time;
            if (result_timeout_.count() >= 0.0 && waiting_duration >= result_timeout_)
            {
                return tl::make_unexpected(fp::Timeout("Timed out waiting for service response."));
            }
        }
    }

    void IkBatch::RclcppClientInterface::cancelRequest()
    {
        request_canceled_ = true;
    }



    IkBatch::IkBatch(const std::string& name,
                               const BT::NodeConfiguration& config,
                               const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
            : IkBatch(name, config, shared_resources,
                                                  std::make_unique<RclcppClientInterface>(shared_resources->node))
    {
    }

    IkBatch::IkBatch(const std::string& name,
                     const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources,
                     std::unique_ptr<ClientInterfaceBase> client_interface)
            : AsyncBehaviorBase(name, config, shared_resources), client_interface_(std::move(client_interface))
    {
    }


BT::PortsList IkBatch::providedPorts()
{
  return BT::PortsList({
//          BT::InputPort<geometry_msgs::msg::PoseArray>(kPortIDPoseArray),
//          BT::OutputPort<geometry_msgs::msg::PoseArray>(kPortIDResultsPoseArray),

  });

}

    fp::Result<bool> IkBatch::doWork() {
        const auto response_timeout = getResponseTimeout();
        if (!response_timeout.has_value()) {
            return tl::make_unexpected(
                    fp::Internal(
                            "Failed to get value for response timeout duration: " + response_timeout.error().what));
        }

        const auto service_name = getServiceName();
        if (service_name.has_value()) {
            client_interface_->initialize(service_name.value(), kTimeoutWaitForServiceServer, response_timeout.value());
        } else {
            return tl::make_unexpected(fp::Internal("Failed to get service name: " + service_name.error().what));
        }

        if (!client_interface_->waitForServiceServer()) {
            return tl::make_unexpected(fp::Internal("Server not available for service " + service_name.value()));
        }

        // Halting is allowed once the service connection has been established.
        notifyCanHalt();


//        const auto pose_array = getInput<geometry_msgs::msg::PoseArray>(kPortIDPoseArray);
        pose_array_.poses.clear();

        pose_array_.header.frame_id = "base_link";
          geometry_msgs::msg::Pose pose;
          pose.position.x = 0.5;
          pose.position.y = 0.5;
          pose.position.z = 0.5;
          pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;
          pose.orientation.w = 1.0;
        pose_array_.poses.push_back(pose);

        for (const auto &p: pose_array_.poses){


            const auto request = createRequest(p, pose_array_.header.frame_id);

            if (!request.has_value()) {
                return tl::make_unexpected(fp::Internal("Failed to create service request: " + request.error().what));
            }

            // TODO(livnaov93): add for loop here to iterate through poses
            if (const auto response = client_interface_->syncSendRequest(request.value()); response.has_value()) {
                std::cerr<<"ik call!"<<std::endl;

                if (processResponse(response.value())){
                    pose_array_.poses.push_back(p);
                }
            } else {
                return tl::make_unexpected(
                        fp::Internal("Failed to send request to service server: " + response.error().what));
            }
     }

        setOutput(kPortIDResultsPoseArray, detection.pose);

        return true;
}

fp::Result<void> IkBatch::doHalt()
{

    client_interface_->cancelRequest();

    return {};
}


}  // namespace ik_batch
