#include "brain_log.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"

BrainLog::BrainLog(Brain *argBrain) : brain(argBrain)
{

}

void BrainLog::debug(string_view entity_path, const string& message) const
{
    std::ostringstream oss;
    oss << "[" << entity_path << "]";
    if (!message.empty()) {
        oss << " " << message;
    }
    RCLCPP_DEBUG(brain->get_logger(), "%s", oss.str().c_str());
}

void BrainLog::log(string_view entity_path, const string& message) const
{
    std::ostringstream oss;
    oss << "[" << entity_path << "]";
    if (!message.empty()) {
        oss << " " << message;
    }
    RCLCPP_INFO(brain->get_logger(), "%s", oss.str().c_str());
}

void BrainLog::warn(string_view entity_path, const string& message) const
{
    std::ostringstream oss;
    oss << "[" << entity_path << "]";
    if (!message.empty()) {
        oss << " " << message;
    }
    RCLCPP_WARN(brain->get_logger(), "%s", oss.str().c_str());
}

void BrainLog::error(string_view entity_path, const string& message) const
{
    std::ostringstream oss;
    oss << "[" << entity_path << "]";
    if (!message.empty()) {
        oss << " " << message;
    }
    RCLCPP_ERROR(brain->get_logger(), "%s", oss.str().c_str());
}

rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr 
BrainLog::get_or_create_publisher(const string& entity_path)
{
    auto it = scalar_publishers_.find(entity_path);
    if (it == scalar_publishers_.end()) {
        // Create a new publisher with topic name /booster_soccer/log/scalars/{entity_path}
        string topic_name = "/booster_soccer/log/scalars/" + entity_path;
        auto publisher = brain->create_publisher<diagnostic_msgs::msg::KeyValue>(topic_name, 10);
        scalar_publishers_[entity_path] = publisher;
        return publisher;
    }
    return it->second;
}

void BrainLog::log_scalar(const string& entity_path, const string& label, double value)
{
    auto publisher = get_or_create_publisher(entity_path);
    
    diagnostic_msgs::msg::KeyValue msg;
    msg.key = label;
    msg.value = std::to_string(value);
    
    publisher->publish(msg);
}

void BrainLog::log_scalar(const string& entity_path, const string& label, int value)
{
    auto publisher = get_or_create_publisher(entity_path);
    
    diagnostic_msgs::msg::KeyValue msg;
    msg.key = label;
    msg.value = std::to_string(value);
    
    publisher->publish(msg);
}
