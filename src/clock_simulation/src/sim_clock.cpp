#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "clock_simulation_interfaces/srv/time_scale.hpp"
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

class SimClockPublisher : public rclcpp::Node {
public:
    SimClockPublisher() : Node("sim_clock"), sim_time_(0, 0) {
        // Объявляем параметры
        this->declare_parameter("time_scale", 1);       // Масштаб времени (1.0 = реальное время)
        this->declare_parameter("publish_rate", 100);   // Частота публикации в Гц
        this->declare_parameter("start_paused", false); // Начать на паузе

        paused_          = this->get_parameter("start_paused").as_bool();
        time_scale_      = this->get_parameter("time_scale").as_double();
        int publish_rate = this->get_parameter("publish_rate").as_int();

        // Создаем издателя для топика /clock
        clock_publisher_ =
            this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Создаем сервисы для управления симуляцией
        pause_service_ = this->create_service<std_srvs::srv::SetBool>(
            "~/pause", std::bind(&SimClockPublisher::handle_pause, this,
                                 std::placeholders::_1, std::placeholders::_2));

        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/reset", std::bind(&SimClockPublisher::handle_reset, this,
                                 std::placeholders::_1, std::placeholders::_2));

        set_time_scale_service_ =
            this->create_service<clock_simulation_interfaces::srv::TimeScale>(
                "~/time_scale", std::bind(&SimClockPublisher::handle_set_time_scale, this,
                                          std::placeholders::_1, std::placeholders::_2));

        // Сохраняем время начала для расчета прошедшего времени
        last_real_time_ = this->now();

        // Таймер для публикации времени
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / publish_rate),
            std::bind(&SimClockPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Simulated time started.");
        RCLCPP_INFO(this->get_logger(), "Time scale: %.2f", time_scale_);
        RCLCPP_INFO(this->get_logger(), "Status: %s", paused_ ? "paused" : "running");
    }

private:
    void timer_callback() {
        if (!paused_) {
            // Получаем реальное прошедшее время с момента начала
            auto current_real_time = this->now();
            auto elapsed_real_time = current_real_time - last_real_time_;

            // Обновляем последнее реальное время
            last_real_time_ = current_real_time;

            // Рассчитываем симулированное время с учетом масштаба
            double dtime = elapsed_real_time.seconds() +
                           elapsed_real_time.nanoseconds() * 1e-9;

            double sim_dtime = dtime * time_scale_;

            sim_time_ += rclcpp::Duration(static_cast<int>(sim_dtime),
                                          static_cast<int>((sim_dtime - static_cast<int>(sim_dtime)) * 1e9));
        }

        // Публикуем текущее симулированное время в /clock
        auto msg = rosgraph_msgs::msg::Clock();
        msg.clock = sim_time_;
        clock_publisher_->publish(msg);
    }

    void handle_pause(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        paused_ = request->data;

        if (!paused_) {
            // При снятии с паузы, обновляем последнее реальное время
            last_real_time_ = this->now();
        }

        response->success = true;
        response->message = paused_ ? "Simulation paused" : "Simulation resumed";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void handle_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        sim_time_ = rclcpp::Time(0, 0);
        last_real_time_ = this->now();

        response->success = true;
        response->message = "Simulation time reset";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void handle_set_time_scale(const std::shared_ptr<clock_simulation_interfaces::srv::TimeScale::Request> request,
                               std::shared_ptr<clock_simulation_interfaces::srv::TimeScale::Response> response) {
        double new_scale = request->scale;

        if (new_scale <= 0.0) {
            response->success = false;
            response->message = "Time scale must be a positive number";
        } else {
            // Обновляем масштаб времени
            time_scale_ = new_scale;

            response->success = true;
            response->message =
                "Time scale set to " + std::to_string(time_scale_);
        }

        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

  private:
    // Время симуляции
    rclcpp::Time sim_time_;

    // Последнее зафиксированное реальное время
    rclcpp::Time last_real_time_;

    // Таймер для публикации
    rclcpp::TimerBase::SharedPtr timer_;

    // Издатель времени
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

    // Сервисы для управления симуляцией
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Service<clock_simulation_interfaces::srv::TimeScale>::SharedPtr set_time_scale_service_;

    // Параметры симуляции
    bool paused_;
    double time_scale_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimClockPublisher>());
    rclcpp::shutdown();
    return 0;
}
