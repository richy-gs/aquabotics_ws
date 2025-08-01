#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

using std::placeholders::_1;
  
class JoyController : public rclcpp::Node
{
public:
  JoyController()
  : Node("joy_controller")
  {
    // Suscríbete a /joy con QoS 10
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&JoyController::joy_callback, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "JoyController iniciado, esperando /joy …");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Asegurarnos de que tenemos al menos 6 ejes y 1 botón
    if (msg->axes.size() < 6 || msg->buttons.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Joy message sin suficientes ejes (%zu) o botones (%zu)",
                  msg->axes.size(), msg->buttons.size());
      return;
    }

    // Mapea índices de axes[0..5]
    double lx = msg->axes[0];  // left stick horizontal
    double ly = msg->axes[1];  // left stick vertical
    double lt = msg->axes[2];  // left trigger
    double rx = msg->axes[3];  // right stick horizontal
    double ry = msg->axes[4];  // right stick vertical
    double rt = msg->axes[5];  // right trigger

    // Botón “Cross” (A) es buttons[0]
    int cross_button = msg->buttons[0];

    // Aquí puedes implementar tu lógica de control:
    // Por ejemplo, imprimir valores:
    RCLCPP_INFO(this->get_logger(),
      "LX: %.2f  LY: %.2f  RX: %.2f  RY: %.2f  LT: %.2f  RT: %.2f  A: %d",
      lx, ly, rx, ry, lt, rt, cross_button);

    // O: convertir estos valores a comandos para tu submarino…
    // control_submarine(lx, ly, rx, ry, lt, rt, cross_button);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyController>());
  rclcpp::shutdown();
  return 0;
}
