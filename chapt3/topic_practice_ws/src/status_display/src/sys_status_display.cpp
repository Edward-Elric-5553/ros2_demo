#include <QApplication>
#include <QLabel>
#include <QString>
#include "rclcpp/rclcpp.hpp"
#include "status_interfaces/msg/system_status.hpp"

using SysStatus = status_interfaces::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node{

    private:
        rclcpp::Subscription<SysStatus>::SharedPtr subscriber_;
        QLabel *label_;
    public:
        SysStatusDisplay():Node("sys_status_display"){
            label_ = new QLabel();
            subscriber_ = this->create_subscription<SysStatus>("sys_status", 10, [&](
                const SysStatus::SharedPtr msg
            )->void{
                label_->setText(get_qstr_from_msg(msg));
            });

            label_->setText(get_qstr_from_msg(std::make_shared<SysStatus>()));
            label_->show();
        }

        QString get_qstr_from_msg(const SysStatus::SharedPtr msg){

            std::stringstream show_str;
            show_str << "==============System Status==============\n" 
            << "System Time: \t" << msg->stamp.sec << "\ts\n" 
            << "System Name: \t" << msg->host_name << "\ts\n" 
            << "CPU Percent: \t" << msg->cpu_percent << "\ts\n" 
            << "MEM Percent: \t" << msg->memory_percent << "\ts\n" 
            << "MEM Total: \t" << msg->memory_total << "\ts\n"
            << "MEM AVAIL: \t" << msg->memory_available << "\ts\n" 
            << "NET SENT: \t" << msg->net_sent << "\ts\n" 
            << "NET RECV: \t" << msg->net_recv << "\ts\n"
            << "==========================================\n" ;

            return QString::fromStdString(show_str.str());
        }
};




int main(int argc, char* argv[]){

    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<SysStatusDisplay>();
    std::thread spin_thread([&]()->void{
        rclcpp::spin(node);
    });
    spin_thread.detach();

    app.exec();

    return 0;
}