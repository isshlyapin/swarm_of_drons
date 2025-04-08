#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class SimClockPublisher : public rclcpp::Node
{
public:
  SimClockPublisher() : Node("sim_clock"), sim_time_(0, 0)
  {
    // Объявляем параметры
    this->declare_parameter("time_scale", 1.0);   // Масштаб времени (1.0 = реальное время)
    this->declare_parameter("publish_rate", 100); // Частота публикации в Гц
    this->declare_parameter("start_paused", false); // Начать на паузе
    
    paused_          = this->get_parameter("start_paused").as_bool();
    time_scale_      = this->get_parameter("time_scale").as_double();
    int publish_rate = this->get_parameter("publish_rate").as_int();
    
    // Создаем издателя для топика /clock
    clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    
    // Создаем сервисы для управления симуляцией
    pause_service_ = this->create_service<std_srvs::srv::SetBool>(
      "~/pause", 
      std::bind(&SimClockPublisher::handle_pause, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/reset",
      std::bind(&SimClockPublisher::handle_reset, this, 
                std::placeholders::_1, std::placeholders::_2));
                
    set_time_scale_service_ = this->create_service<std_srvs::srv::SetBool>(
      "~/set_time_scale",
      std::bind(&SimClockPublisher::handle_set_time_scale, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    // Сохраняем время начала для расчета прошедшего времени
    real_time_start_ = this->get_clock()->now();
    
    // Таймер для публикации времени
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / publish_rate),
      std::bind(&SimClockPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Simulated time started.");
    RCLCPP_INFO(this->get_logger(), "Time scale: %.2f", time_scale_);
    RCLCPP_INFO(this->get_logger(), "Status: %s", paused_ ? "paused" : "running");
  }

private:
  void timer_callback()
  {
    if (!paused_) {
      // Получаем реальное прошедшее время с момента начала
      auto current_real_time = this->get_clock()->now();
      auto elapsed_real = current_real_time - real_time_start_;
      
      // Рассчитываем симулированное время с учетом масштаба
      sim_time_ = rclcpp::Time(0, 0) + 
                 rclcpp::Duration(elapsed_real.seconds() * time_scale_,
                                 elapsed_real.nanoseconds() * time_scale_);
    }
    
    // Публикуем текущее симулированное время в /clock
    auto msg = rosgraph_msgs::msg::Clock();
    msg.clock = sim_time_;
    clock_publisher_->publish(msg);
  }
  
  void handle_pause(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    paused_ = request->data;
    
    if (!paused_) {
      // При снятии с паузы, обновляем начальное время
      real_time_start_ = this->get_clock()->now() - 
                        rclcpp::Duration(sim_time_.seconds() / time_scale_,
                                        sim_time_.nanoseconds() / time_scale_);
    }
    
    response->success = true;
    response->message = paused_ ? "Simulation paused" : "Simulation resumed";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }
  
  void handle_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    sim_time_ = rclcpp::Time(0, 0);
    real_time_start_ = this->get_clock()->now();
    
    response->success = true;
    response->message = "Simulation time reset";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }
  
  void handle_set_time_scale(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    double new_scale = request->data;
    
    if (new_scale <= 0.0) {
      response->success = false;
      response->message = "Time scale must be a positive number";
    } else {
      // Сохраняем текущее симулированное время
      auto current_sim_time = sim_time_;
      
      // Обновляем масштаб времени
      time_scale_ = new_scale;
      
      // Обновляем начальное время с учетом нового масштаба
      real_time_start_ = this->get_clock()->now() - 
                        rclcpp::Duration(current_sim_time.seconds() / time_scale_,
                                        current_sim_time.nanoseconds() / time_scale_);
      
      response->success = true;
      response->message = "Time scale set to " + std::to_string(time_scale_);
    }
    
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }
  
  // Время симуляции
  rclcpp::Time sim_time_;
  
  // Реальное время начала симуляции
  rclcpp::Time real_time_start_;
  
  // Таймер для публикации
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Издатель времени
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  
  // Сервисы для управления симуляцией
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_time_scale_service_;
  
  // Параметры симуляции
  double time_scale_;
  bool paused_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimClockPublisher>());
  rclcpp::shutdown();
  return 0;
}
