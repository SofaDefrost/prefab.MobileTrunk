#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <filesystem>

using std::placeholders::_1;

class SimpleBagRecorder : public rclcpp::Node
{
public:
    SimpleBagRecorder(double vitesse_angulaire=-10., double vitesse_lineaire=10.) : Node("simple_bag_recorder"),
                      m_vitesse_angulaire(vitesse_angulaire), m_vitesse_lineaire(vitesse_lineaire)
        {
          // Get parameters from launch file
          vitesse_angulaire = this->declare_parameter("vitesse_angulaire", 0.0);
          vitesse_lineaire = this->declare_parameter("vitesse_lineaire", 0.0);

          m_vitesse_angulaire = this->get_parameter("vitesse_angulaire").get_value<double>();
          m_vitesse_lineaire = this->get_parameter("vitesse_lineaire").get_value<double>();

          // Create base directory if it doesn't exist
          if(!std::filesystem::exists("record_folder"))
          {
            std::filesystem::create_directory("record_folder");
          }

          // If the folder already exist move to the base directory
          std::filesystem::current_path("record_folder");

          //Check if the velocities are different from zero bzfor creating the folder
          if(m_vitesse_angulaire !=-10. && m_vitesse_lineaire !=-10.)
          {
            std::string base_folder_name = "w_" + std::to_string(m_vitesse_angulaire) + "_v_"+
                                            std::to_string(m_vitesse_lineaire);

            //create the new folder withthe velocities as forder name
            if(!std::filesystem::exists(base_folder_name))
            {
              std::filesystem::create_directory(base_folder_name);

            }
            else
            {
              std::filesystem::remove_all(base_folder_name);
              std::filesystem::create_directory(base_folder_name);

            }

          const rosbag2_cpp::StorageOptions storage_options({base_folder_name, "sqlite3"});

          const rosbag2_cpp::ConverterOptions converter_options(
                 {rmw_get_serialization_format(),
                 rmw_get_serialization_format()});   

          writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

          writer_->open(storage_options, converter_options);

          writer_->create_topic(
              {"/summit_xl/robotnik_base_control/odom",
                   "nav_msgs/msg/Odometry",
                  rmw_get_serialization_format(),
                    ""}); 
          
          writer_->create_topic(
                  {"/summit_xl/joint_states",
                                "sensor_msgs/msg/JointState",
                                rmw_get_serialization_format(),
                        ""});

          subscription_odom = create_subscription<nav_msgs::msg::Odometry>(
                "/summit_xl/robotnik_base_control/odom", 10, std::bind(&SimpleBagRecorder::odom_topic_callback, this, _1));

          subscription_join_state = create_subscription<sensor_msgs::msg::JointState>(
                "/summit_xl/joint_states", 10, std::bind(&SimpleBagRecorder::join_state_topic_callback, this, _1));           
        } 
        else{
                RCLCPP_ERROR(get_logger(), "Invalid velocities");
        }

    }
private:
    void odom_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

      bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
        new rcutils_uint8_array_t,
        [this](rcutils_uint8_array_t *msg) {
          auto fini_return = rcutils_uint8_array_fini(msg);
          delete msg;
          if (fini_return != RCUTILS_RET_OK) {
            RCLCPP_ERROR(get_logger(),
              "Failed to destroy serialized message %s", rcutils_get_error_string().str);
          }
        });

      *bag_message->serialized_data = msg->release_rcl_serialized_message();
       //Convert the received message to a serialized message and store it in the bag message
      //rosbag2_cpp::typesupport_helpers::to_storage_format(
      //*msg, rmw_get_serialization_format(), *bag_message->serialized_data);

      bag_message->topic_name = "/summit_xl/robotnik_base_control/odom";
      if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
          rcutils_get_error_string().str);
      }

      writer_->write(bag_message);
    }

    void join_state_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

      bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
        new rcutils_uint8_array_t,
        [this](rcutils_uint8_array_t *msg) {
          auto fini_return = rcutils_uint8_array_fini(msg);
          delete msg;
          if (fini_return != RCUTILS_RET_OK) {
            RCLCPP_ERROR(get_logger(),
              "Failed to destroy serialized message %s", rcutils_get_error_string().str);
          }
        });

      *bag_message->serialized_data = msg->release_rcl_serialized_message();
       //Convert the received message to a serialized message and store it in the bag message
      //rosbag2_cpp::typesupport_helpers::to_storage_format(
      //*msg, rmw_get_serialization_format(), *bag_message->serialized_data);

      bag_message->topic_name = "/summit_xl/joint_states";
      if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
          rcutils_get_error_string().str);
      }

      writer_->write(bag_message);
    }

    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_odom;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_join_state;
    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
    double  m_vitesse_angulaire, m_vitesse_lineaire;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
  }
