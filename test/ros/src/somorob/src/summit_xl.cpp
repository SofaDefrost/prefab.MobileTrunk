#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class TeleopTwistTest : public rclcpp::Node
{
public:
  TeleopTwistTest(double vitesse_angulaire = 0, double vitesse_lineaire = 0)
    : Node("teleop_twist_test"), m_vitesse_angulaire(vitesse_angulaire), m_vitesse_lineaire(vitesse_lineaire)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/summit_xl/robotnik_base_control/cmd_vel", 100);
    timer_ = this->create_wall_timer(10ms, std::bind(&TeleopTwistTest::timer_callback, this));
    vitesse_angulaire = this->declare_parameter("vitesse_angulaire", 0.0);
    vitesse_lineaire = this->declare_parameter("vitesse_lineaire", 0.0);

    m_vitesse_lineaire = this->get_parameter("vitesse_lineaire").get_value<double>();
    m_vitesse_angulaire = this->get_parameter("vitesse_angulaire").get_value<double>();
  }

private:
  void timer_callback()
  {
    time_now_ = this->get_clock()->now();

    auto dt_duration = time_now_ - time_s_;
    double dt = dt_duration.seconds();

    if (dt > 0.0 && dt < 10.0)
    {
      // RCLCPP_INFO(this->get_logger(), "etape 1 = %f", dt);
      geometry_msgs::msg::Twist twist;
      twist.linear.x = m_vitesse_lineaire; twist.linear.y = 0.0; twist.linear.z = 0.0; 
      twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.;
      publisher_->publish(twist);
    }

    if (dt > 10.0 && dt < 20.0)
    {
      // RCLCPP_INFO(this->get_logger(), "etape 2 = %f", dt);
      geometry_msgs::msg::Twist twist;
      twist.linear.x = 0.; twist.linear.y = 0.0; twist.linear.z = 0.0;
      twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = m_vitesse_angulaire;
      publisher_->publish(twist);
    }

    if (dt > 20.0 && dt < 30.0)
    {
      // RCLCPP_INFO(this->get_logger(), "etape 3 = %f", dt);
      geometry_msgs::msg::Twist twist;
      twist.linear.x = m_vitesse_lineaire; twist.linear.y = 0.0; twist.linear.z = 0.0;
      twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.;
      publisher_->publish(twist);
    }

    if (dt > 30.0 && dt < 40.0)
    {
      // RCLCPP_INFO(this->get_logger(), "etape 4 = %f", dt);
      geometry_msgs::msg::Twist twist;
      twist.linear.x = 0.; twist.linear.y = 0.0; twist.linear.z = 0.0;
      twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = m_vitesse_angulaire;
      publisher_->publish(twist);
    }

    if (dt > 40.0 && dt < 50.0)
    {
      // RCLCPP_INFO(this->get_logger(), "etape 5 = %f", dt);
      geometry_msgs::msg::Twist twist;
      twist.linear.x = m_vitesse_lineaire; twist.linear.y = 0.0; twist.linear.z = 0.0;
      twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.;
      publisher_->publish(twist);
    }

    if (dt > 50.0 && dt < 60.0)
    {
      // RCLCPP_INFO(this->get_logger(), "etape 6 = %f", dt);
      geometry_msgs::msg::Twist twist;
      twist.linear.x = 0.; twist.linear.y = 0.0; twist.linear.z = 0.0;
      twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = m_vitesse_angulaire;
      publisher_->publish(twist);
    }

    if (dt > 60.0 && dt < 70.0)
    {
      // RCLCPP_INFO(this->get_logger(), "etape 7 = %f", dt);
      geometry_msgs::msg::Twist twist;
      twist.linear.x = m_vitesse_lineaire; twist.linear.y = 0.0; twist.linear.z = 0.0;
      twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.;
      publisher_->publish(twist);
    }
    if (dt > 70.0)
    {
      RCLCPP_INFO(this->get_logger(), "Job done = %f", dt);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time time_now_;
  rclcpp::Time time_s_ = this->get_clock()->now();
  double m_vitesse_angulaire, m_vitesse_lineaire;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopTwistTest>());
  rclcpp::shutdown();

  return 0;
}