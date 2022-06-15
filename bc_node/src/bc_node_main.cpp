#include "bc_node/bc_node.hpp"

int main(int argc, char* argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    /// @brief Create and initialize node
    const auto node_options{rclcpp::NodeOptions().use_intra_process_comms(false)};
    auto node{std::make_shared<bc_node::BCNode>("broadcaster", node_options)};

    if (!node->init())
    {
        rclcpp::shutdown();
        return 0;
    }

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}