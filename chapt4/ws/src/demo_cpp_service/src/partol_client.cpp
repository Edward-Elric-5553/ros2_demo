#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/partol.hpp"
#include <chrono>
#include <ctime>

using namespace std::chrono_literals;
using Partol = chapt4_interfaces::srv::Partol;

class PartolClient: public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Partol>::SharedPtr partol_client_;

public: 
    PartolClient():Node("partol_client")
    {
        srand(time(NULL));
        partol_client_ = this->create_client<Partol>("partol");
        timer_ = this->create_wall_timer(10s, [&]()->void{
            while (!this->partol_client_->wait_for_service(1s)){
                if (!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "rclcpp service stop ....");
                    return ;
                }
                RCLCPP_INFO(this->get_logger(), "wait for serveice ready ...");
            }

            auto request = std::make_shared<Partol::Request>();
            request->target_x = rand() % 15;
            request->target_y = rand() % 15;
            RCLCPP_INFO(this->get_logger(), "target x : %f , target y : %f", request->target_x, request->target_y);
            this->partol_client_->async_send_request(request, [&](
                rclcpp::Client<Partol>::SharedFuture result_future
            )->void{
                auto response = result_future.get();
                if (response->result == Partol::Response::SUCCESS){
                    RCLCPP_INFO(this->get_logger(), "Request SUCCESS");
                }
                if (response->result == Partol::Response::FAIL){
                    RCLCPP_INFO(this->get_logger(), "Request FAIL");
                }

            });
        });

    }
};

int main(int argc, char* argv[]){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PartolClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}