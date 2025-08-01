
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <algorithm>
#include <cmath>

using std::placeholders::_1;
using std::clamp;

class JoyController : public rclcpp::Node
{
public:
  JoyController()
  : Node("joy_controller")
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

    // Timer
    

    RCLCPP_INFO(this->get_logger(), "JoyController iniciado, esperando /joy …");
  }

private:
  // Mapea valor [0..1] a PWM [0..200]
  int mapSpeed(double v) {
    return static_cast<int>(round(clamp(v, 0.0, 1.0) * 200.0));
  }

  int mapSpeed2(double v) {
  // 1) Aseguramos que v no salga de [-1,1]
  double vv = std::clamp(v, -1.0, 1.0);
  // 2) Normalizamos a [0,1]
  double norm = (1.0 - vv) / 2.0;
  // 3) Escalamos a [0,200] y redondeamos
  return static_cast<int>(std::round(norm * 200.0));
}


  void publishCommand(int index, int direction, int speed) {
    std_msgs::msg::Int32MultiArray cmd;
    cmd.data = { index, direction, speed };
    motor_pub_->publish(cmd);
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Validar ejes y botones
    if (msg->axes.size() < 6) {
      RCLCPP_WARN(this->get_logger(),
        "Joy solo tiene %zu ejes (se requieren 6).",
        msg->axes.size());
      return;
    }

    // Leer controles
    double lx = msg->axes[0];
    double ly = msg->axes[1];
    double lt = msg->axes[2];
    double rx = msg->axes[3];
    double ry = msg->axes[4];
    double rt = msg->axes[5];

    // --- 1) Horizontal: mezcla ly (avance) + lx (giro) ---
    double left_val  = clamp(ly + lx, -1.0, 1.0);
    double right_val = clamp(ly - lx, -1.0, 1.0);

    int dir_l   = (left_val  >= 0) ? 1 : -1;
    int speed_l = mapSpeed(std::fabs(left_val));
    int dir_r   = (right_val >= 0) ? 1 : -1;
    int speed_r = mapSpeed(std::fabs(right_val));

    // Motor 0 = izquierda, Motor 1 = derecha
    publishCommand(0, dir_l, speed_l);
    publishCommand(1, dir_r, speed_r);

    // --- 2) Vertical: triggers suben/bajan ---
    const double dead_axe_z = 1 - 0.1;
    if (rt < dead_axe_z || lt < dead_axe_z) {
      // Subir
      if (rt < dead_axe_z) {
        int sp = mapSpeed2(rt);
        for (int idx = 2; idx <= 5; ++idx) {
          publishCommand(idx, 1, sp);
        }
      }
      // Bajar
      else if (lt < dead_axe_z) {
        int sp = mapSpeed2(lt);
        for (int idx = 2; idx <= 5; ++idx) {
          publishCommand(idx, -1, sp);
        }
      }
    }
    // --- 3) Roll: diferencial vertical izquierda vs derecha ---
    // const double dead_roll = 0.0;
    // else if (std::fabs(rx) > d`ead_roll && std::fabs(rx) > std::fabs(ry)) {
    //   int sp = mapSpeed(std::fabs(rx));
    //   int dir_lr = (rx >= 0) ? 1 : -1;   // izquierda hacia arriba si rx+
    //   int dir_rr = -dir_lr;             // derecha al revés
    //   // Índices 2 y 3 = vertical izquierda; 4 y 5 = vertical derecha
    //   publishCommand(2, dir_lr, sp);
    //   publishCommand(3, dir_lr, sp);
    //   publishCommand(4, dir_rr, sp);
    //   publishCommand(5, dir_rr, sp);
    // }
    // // --- 4) Pitch: diferencial front/back ---
    // else if (std::fabs(ry) > dead) {
    //   int sp = mapSpeed(std::fabs(ry));
    //   int dir_front = (ry >= 0) ? -1 : 1; // nariz arriba => front down
    //   int dir_back  = -dir_front;         // back al revés
    //   // Front: índices 2 y 4; Back: 3 y 5
    //   publishCommand(2,   dir_front, sp);
    //   publishCommand(4, dir_front, sp);
    //   publishCommand(3, dir_back,  sp);
    //   publishCommand(5, dir_back,  sp);
    // }
    // // --- 5) Ningún vertical activo: detenerlos ---
    else {
      for (int idx = 2; idx <= 5; ++idx) {
        publishCommand(idx, 0, 0);
      }
    }
  }

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr    joy_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyController>());
  rclcpp::shutdown();
  return 0;
}
