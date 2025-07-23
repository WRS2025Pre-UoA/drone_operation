#ifndef DRONE_GUI_COMPONENT_HPP
#define DRONE_GUI_COMPONENT_HPP

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <algorithm>


#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/type_adapter.hpp>

#include "drone_operation/gui_tool.hpp"
#include "drone_operation/cv_mat_type_adapter.hpp"

using namespace std::chrono_literals;

namespace component_operator_gui
{
class DroneGUI : public rclcpp::Node
{
public:
    using MyAdaptedType = rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>;

    // ボタンの設定-------------------------------------------------
    cv::Mat mat;// ボタンの画像
    // 描画設定
    int width = 300, height = 350;//画面サイズ
    int btn_width = 140, btn_height = 50;//ボタンのサイズ
    int x_offset = 5,y_offset = 25;//ボタンの最初の位置
    int btn_per_row = 2;//一行に何個ボタン設置するか
    int btn_space_row = 5, btn_space_col = 30;//ボタン間のスペース

    // 中身
    std::vector<std::string> buttons_name_;//表示しているボタンのリスト
    std::vector<Button> buttons_; // ボタン位置、サイズのリスト
    cv::Size btn_size = cv::Size(btn_width,btn_height);

    std::vector<Button> another_box_;

    // 最新の値を格納確認するリスト----------------------------------
    // std::vector<std::map<std::string, bool>> receive_list;
    std::string latest_topic = "None";
    bool latest_qr = false;

    // バルブの開閉状況カウント変数 open / close---------------------
    int bulb_state_count = 0;

    // 画像分配用---------------------------------------------------
    std::map<std::string, bool> bool_flags_ = {
        {"pressure",false},
        {"qr",false},
        {"cracks",false}
    };// 連億処理信号とそれに対応する送り先のmap
    std::map<std::string, rclcpp::Time> start_times_ = {
        {"pressure",rclcpp::Time(0)},
        {"qr",rclcpp::Time(0)},
        {"cracks",rclcpp::Time(0)}
    };// 信号を受け取ってから1secとカウントするため
    double check_duration_sec;

    // ドローンからの画像をcv::Matで一時保存 sensor_msgs -> cv::Mat
    cv::Mat temporary_image;
    cv::Mat temp_image_with_rect; // 切り抜き枠(長方形)を設けた画像　rqtで表示する
    // 切り抜き領域座標
    // cv::Point top_left,buttom_right;// 左上(x,y) 右下(x,y)
    // int rect_width, rect_height;
    // 切り抜かれた画像
    cv::Mat cropped_temp_image;

    // 受け取ったメッセージを格納-------------------------------------
    std_msgs::msg::String qr_id, result_data;// 確認ノードへ報告を行う時に必要なid 結果を格納
    cv::Mat receive_image, receive_qr_image;// sensor_msgsで送られてくるので一時的にcv::Matへ

     // 画面が起動しているかflag
    bool send_confirm_flag = false;
    std::vector<std::string> trigger_list = {"pressure", "qr", "cracks"};
    std::vector<std::string> confirm_list = {"pressure", "cracks"};

    explicit DroneGUI(const rclcpp::NodeOptions &options);
    DroneGUI() : DroneGUI(rclcpp::NodeOptions{}) {}

private:
    cv ::Mat setup();// ボタン画面生成生成
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);// 検出結果をうけとった時に行う処理関数
    void timer_callback();// 定期的にボタン画像を流す
    void mouse_click_callback(const geometry_msgs::msg::Point::SharedPtr msg);// ボタン画面にクリックした時の座標をもとに行う処理関数
    void rewriteButton(Button btn, std::string text, cv::Scalar color) const;// 指定したボタンの色、表示内容を変更
    void rewriteMessage();
    void process(std::string topic_name);// クリックしたボタンに対応した処理を行う関数
    void publish_images();// 連続処理ノードへ画像を流す関数

    rclcpp::Publisher<MyAdaptedType>::SharedPtr publish_gui_;// ボタン画面を流すpublisher
    rclcpp::Publisher<MyAdaptedType>::SharedPtr publish_operator_image_;// 受け取ったドローンの画像に切り抜き枠を設けた画像を流すpublisher
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr click_;// ボタンクリック座標を受け取るsubscriber
    rclcpp::TimerBase::SharedPtr view_;// ボタン画面を定期的に流すタイマー
    rclcpp::TimerBase::SharedPtr timer_;// 一定間隔で画像を流す
    rclcpp::TimerBase::SharedPtr color_reset_timer_;// 一定時間待って、ボタンの色を白に戻す
     rclcpp::TimerBase::SharedPtr message_reset_timer_;// 
    rclcpp::TimerBase::SharedPtr reopen_window_;// 確認画面が表示されてるときにsendボタンが押されたとき

    std::map<std::string, std::shared_ptr<rclcpp::Publisher<MyAdaptedType>>> image_publishers_;// 連続処理ノードに画像を流す

    std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> receive_data_;// 検出結果をうけとる　
    std::map<std::string, rclcpp::Subscription<MyAdaptedType>::SharedPtr> receive_image_;// 検出画像をうけとる

    rclcpp::Subscription<MyAdaptedType>::SharedPtr receive_raw_image_;// DRONEから来る生画像

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dt_qr_id_publisher_;// 確認ノードへidを送る
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dt_data_publisher_;// 確認ノードへ検出結果を送る
    rclcpp::Publisher<MyAdaptedType>::SharedPtr dt_image_publisher_;// 確認ノードへ検出画像を送る
    rclcpp::Publisher<MyAdaptedType>::SharedPtr dt_qr_image_publisher_;// 確認ノードへ検出画像を送る
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dt_flag_subscriber_;// 送信をしたか否か send or back
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dt_flag_;// 画面起動の信号
};

} // namespace component_operator_gui

#endif // DRONE_GUI_COMPONENT_HPP