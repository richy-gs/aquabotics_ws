#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

using std::placeholders::_1;
using std::clamp;

class JoyController : public rclcpp::Node
{
public:
  JoyController()
  : Node("joy_controller"),
    range_freq_(5.0),
    latest_axes_(6, 0.0),
    last_axes_(6, 0.0)
  {
    // Publisher a /xbox/motor_command
    motor_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
      "xbox/motor_command", rclcpp::QoS(10));

    // Subscriber a /joy
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&JoyController::joy_callback, this, _1)
    );

    // Timer a 50 Hz (20 ms)
    auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / range_freq_));
    timer_ = this->create_wall_timer(
      period,
      std::bind(&JoyController::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(),
                "JoyController iniciado:  subscríbete a /joy, "
                "publica en /xbox/motor_command a %.1f Hz",
                range_freq_);
  }

private:
  // Mapea [0..1] a PWM [0..150]
  int mapSpeed(double v) {
    return static_cast<int>(std::round(clamp(v, 0.0, 1.0) * 150.0));
  }

  // Mapea trigger [-1..1] a [0..150]
  int mapSpeed2(double v) {
    double vv = clamp(v, -1.0, 1.0);
    double norm = (1.0 - vv) / 2.0;     // 1=>0, -1=>1
    return static_cast<int>(std::round(norm * 150.0));
  }

  // Encola y publica un comando para un motor
  void publishCommand(int index, int direction, int speed) {
    std_msgs::msg::Int32MultiArray cmd;
    cmd.data = { index, direction, speed };
    motor_pub_->publish(cmd);
  }

  // Actualiza el buffer de ejes al llegar un joy_msg
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->axes.size() < 6) {
      RCLCPP_WARN(this->get_logger(),
        "Joy solo tiene %zu ejes (se requieren 6).",
        msg->axes.size());
      return;
    }
    for (size_t i = 0; i < 6; ++i) {
      latest_axes_[i] = msg->axes[i];
    }
  }

  // Se llama cada 20 ms; publica si hubo cambio >0.01 en algún eje
  void timer_callback()
  {
    const double threshold = 0.01;
    bool changed = false;
    for (size_t i = 0; i < 6; ++i) {
      if (std::fabs(latest_axes_[i] - last_axes_[i]) > threshold) {
        changed = true;
        break;
      }
    }
    if (!changed) {
      return;  // nada que publicar
    }

    // Guardamos para la próxima comparación
    last_axes_ = latest_axes_;

    // Extraemos valores
    double lx = latest_axes_[0];
    double ly = latest_axes_[1];
    double lt = latest_axes_[2];
    double rx = latest_axes_[3];
    double ry = latest_axes_[4];
    double rt = latest_axes_[5];

    // ----- 1) Horizontal: mezcla ly + lx -----
    double left_val  = clamp(ly + lx, -1.0, 1.0);
    double right_val = clamp(ly - lx, -1.0, 1.0);
    int dir_l   = (left_val  >= 0) ? 1 : -1;
    int speed_l = mapSpeed(std::fabs(left_val));
    int dir_r   = (right_val >= 0) ? 1 : -1;
    int speed_r = mapSpeed(std::fabs(right_val));
    publishCommand(0, dir_l, speed_l);
    publishCommand(1, dir_r, speed_r);

    // ----- 2) Vertical: triggers suben/bajan -----
    const double dead_axe_z = 1.0 - 0.1;  // disparador a >90% recorrido
    if (rt < dead_axe_z || lt < dead_axe_z) {
      // Subir con RT
      if (rt < dead_axe_z) {
        int sp = mapSpeed2(rt);
        for (int idx = 2; idx <= 5; ++idx) {
          publishCommand(idx, 1, sp);
        }
      }
      // Bajar con LT
      else if (lt < dead_axe_z) {
        int sp = mapSpeed2(lt);
        for (int idx = 2; idx <= 5; ++idx) {
          publishCommand(idx, -1, sp);
        }
      }
    }
    // ----- 3) Sin triggers verticales: detenemos esos motores -----
    else {
      for (int idx = 2; idx <= 5; ++idx) {
        publishCommand(idx, 0, 0);
      }
    }
  }

  double range_freq_;
  std::vector<double> latest_axes_, last_axes_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr    joy_sub_;
  rclcpp::TimerBase::SharedPtr                              timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyController>());
  rclcpp::shutdown();
  return 0;
}

