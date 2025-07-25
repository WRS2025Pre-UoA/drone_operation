#include "drone_operation/operator_gui_component.hpp"

namespace component_operator_gui
{
DroneGUI::DroneGUI(const rclcpp::NodeOptions &options) 
    : Node("drone_gui", options)
{
    // ミッションごとにモードを変更------------------------------------------------------------------------------------------
    this->declare_parameter("mode", "P6");
    std::string param = this->get_parameter("mode").as_string();
    RCLCPP_INFO(this->get_logger(), "Received mode: %s", param.c_str());

    // ドローンから来る画像に長方形の枠を設ける-------------------------------------------------------------------------------
    this->declare_parameter("top_left_x", 100);
    this->declare_parameter("top_left_y", 100);
    this->declare_parameter("rect_width", 300);
    this->declare_parameter("rect_height", 300);

    int top_left_x,top_left_y,rect_width,rect_height;
    this->get_parameter("top_left_x", top_left_x);
    this->get_parameter("top_left_y", top_left_y);
    this->get_parameter("rect_width", rect_width);
    this->get_parameter("rect_height", rect_height);
    RCLCPP_INFO_STREAM(this->get_logger(), "Received top_left_x: "<< top_left_x);
    RCLCPP_INFO_STREAM(this->get_logger(), "Received top_left_y: "<< top_left_y);
    RCLCPP_INFO_STREAM(this->get_logger(), "Received rect_width: "<< rect_width);
    RCLCPP_INFO_STREAM(this->get_logger(), "Received rect_height: "<< rect_height);
    
    cv::Point top_left = cv::Point(top_left_x, top_left_y);
    cv::Point buttom_right = cv::Point(top_left_x+rect_width, top_left_y+rect_height);
    // 画像送信時間、間隔をパラメータにする------------------------------------------------------------------------------------
    this->declare_parameter<double>("check_duration_sec",1.0);
    this->declare_parameter<int>("timer_interval_ms",100);

    int timer_interval_ms;
    this->get_parameter("check_duration_sec", check_duration_sec);
    this->get_parameter("timer_interval_ms", timer_interval_ms);
    RCLCPP_INFO_STREAM(this->get_logger(),"Interval: " << timer_interval_ms << " ms, duration_time: " << check_duration_sec);

    // ミッションごとのボタン表示名------------------------------------------------------------------------------------------
    std::map<std::string, std::vector<std::string>> pub_sub_topics = {
        {"P1", {"pressure", "qr", "send"}},
        {"P2", {"pressure", "qr", "V_state", "send"}},
        {"P3", {"cracks", "qr", "send"}},
        {"P4", {"qr", "disaster", "send"}},
        {"P6", {"pressure", "qr", "disaster", "missing", "send"}}
    };

    // 連続処理用の画像を飛ばす image_publisher初期化-----------------------------------------------------------------------------
    if (pub_sub_topics.find(param) != pub_sub_topics.end()) {
        for (const auto &topic : pub_sub_topics[param]) {
            if(std::find(trigger_list.begin(), trigger_list.end(), topic) != trigger_list.end()){
                image_publishers_[topic] = this->create_publisher<MyAdaptedType>(topic+"_image", 10);
                RCLCPP_INFO(this->get_logger(), "Created publisher for topic: %s", topic.c_str());
            }
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid profile: %s", param.c_str());
    } 

    // 連続処理ノードから送られてくる結果、画像を受け取るsubscriber初期化----------------------------------------------------------
    if (pub_sub_topics.find(param) != pub_sub_topics.end()) {
        for (const auto &topic : pub_sub_topics[param]) {
            if(std::find(trigger_list.begin(), trigger_list.end(), topic) != trigger_list.end()){
                receive_data_[topic] = this->create_subscription<std_msgs::msg::String>(topic+"_result_data", 10,
                    [this, topic](const std_msgs::msg::String::SharedPtr msg){
                    if(topic == "qr") {
                        latest_qr = true;
                        qr_id = *msg;
                        dt_qr_id_publisher_->publish(*msg);
                    }
                    else {
                        latest_topic = topic;
                        dt_data_publisher_->publish(*msg);
                    }
                    
                });// 受け取り時の処理
                receive_image_[topic] = this->create_subscription<MyAdaptedType>(topic+"_result_image",10,
                    [this,topic](const cv::Mat msg){
                        
                    if(not(topic == "qr")) {
                        receive_image = msg;
                        cv::Mat result_image = msg;
                        dt_image_publisher_->publish(result_image);
                    }
                    else if(topic == "qr"){
                        receive_qr_image = msg;
                        cv::Mat qr_image = msg;
                        dt_qr_image_publisher_->publish(qr_image);
                    }
                });// 受け取り時の処理
                RCLCPP_INFO(this->get_logger(), "Created subscriber for topic: %s", topic.c_str());
            }
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid profile: %s", param.c_str());
    } 

    // Droneから未加工な画像(pressureなどが処理に掛ける画像)　disaster_reportやdebris_removalのため
    receive_raw_image_ = this->create_subscription<MyAdaptedType>("raw_image",10,
        [this, rect_width, rect_height, top_left, buttom_right](const cv::Mat msg){
            temporary_image = msg;
            // RCLCPP_INFO_STREAM(this->get_logger(), "receive image from drone");
            temp_image_with_rect = temporary_image.clone();
            bool is_rect_inside_image =
                top_left.x >= 0 && top_left.y >= 0 &&
                (top_left.x + rect_width) <= temp_image_with_rect.cols &&
                (top_left.y + rect_height) <= temp_image_with_rect.rows;
            if(!is_rect_inside_image){
                RCLCPP_WARN_STREAM(this->get_logger(), "Requested crop area is outside image bounds. Skipping crop.");
                cropped_temp_image = temp_image_with_rect.clone();  // fallback  
            }else{
                cv::rectangle(temp_image_with_rect, top_left, buttom_right, cv::Scalar(255, 0, 0), 5);
                cropped_temp_image = cv::Mat(temporary_image, cv::Rect(top_left.x, top_left.y, rect_width, rect_height));
                // RCLCPP_INFO_STREAM(this->get_logger(), "remake img w: " << temp_image_with_rect.cols << " h: " << temp_image_with_rect.rows);
                // RCLCPP_INFO_STREAM(this->get_logger(), "cropped img w: " << cropped_temp_image.cols << " h: " << cropped_temp_image.rows);
            }
        });

    // デジタルツインへ上げるpublisher初期化-----------------------------------------------------------------------------------
    dt_qr_id_publisher_ = (this->create_publisher<std_msgs::msg::String>("qr_id", 1));// 送信上限1でもいい気がする
    dt_data_publisher_ = (this->create_publisher<std_msgs::msg::String>("result_data", 1));
    dt_image_publisher_ = (this->create_publisher<MyAdaptedType>("result_image", 10));// Myadaptation
    dt_qr_image_publisher_ = (this->create_publisher<MyAdaptedType>("result_qr_image", 10));// Myadaptation

    // デジタルツインからの信号----------------------------------------------------------------------------------------------
    dt_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("send_flag",1,
        [this](const std_msgs::msg::Bool::SharedPtr msg){
            if(msg->data){
                latest_topic = "None";
                latest_qr = false;
                receive_qr_image.release();// 勝手に更新されるのかな
                receive_image.release();// 勝手に更新されるのかな
            }
            send_confirm_flag = false;// backかsendを押して画面を閉じたとする
            // RCLCPP_INFO_STREAM(this->get_logger(),"CLOSE");
        });

    // 確認画面起動---------------------------------------------------------------------------------------------------------
    dt_flag_ = this->create_publisher<std_msgs::msg::Bool>("startUp",1);

    // ボタン表示設定-------------------------------------------------------------------------------------------------------
    // ミッションごとに表示するボタンのリストを作成
    buttons_name_ = pub_sub_topics[param];

    // ボタンの画像を送信するpublisher初期化
    publish_gui_ = this->create_publisher<MyAdaptedType>("drone_gui_with_buttons",1);
    // ドローンからの画像(切り抜き領域込み)を送信するPublisher初期化
    publish_operator_image_ = this->create_publisher<MyAdaptedType>("drone_image_with_rect",1);

    // ボタンのクリック判定subscriberを作成
    click_ = this->create_subscription<geometry_msgs::msg::Point>(
        "drone_gui_with_buttons_mouse_left", 10, std::bind(&DroneGUI::mouse_click_callback, this, std::placeholders::_1));

    // ボタンの画面生成
    mat = setup();

    // 定期的にボタン画面をpublishするtimer関数
    view_ = this->create_wall_timer(100ms, std::bind(&DroneGUI::timer_callback, this));

    // 連続処理へ
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_interval_ms), std::bind(&DroneGUI::publish_images, this));
}

// 画面初期化
cv::Mat DroneGUI::setup(){
    DrawTool canvas(width,height,0);//画面描画

    for(size_t i = 0; i < buttons_name_.size(); i++){
        int row = i / btn_per_row;  // 行数
        int col = i % btn_per_row;  // 列数

        // ボタン位置を更新
        Button btn(cv::Point(x_offset + col * (btn_width + btn_space_row), y_offset + row * (btn_height + btn_space_col)),cv::Size(btn_width,btn_height));
        buttons_.push_back(btn); // ボタンをリストに追加
        canvas.drawButton_new(btn, buttons_name_[i], cv::Scalar(255, 255, 255), -1, cv::LINE_8, 0.78, cv::Scalar(0,0,0), 2);
    }

    Button receive_qr_box_(cv::Point(x_offset,buttons_.back().pos.y+btn_height+10),cv::Size(btn_width*2+btn_space_row,btn_height));// 今 qrを受け取っているかを表示
    another_box_.push_back(receive_qr_box_);
    canvas.drawButton_new(another_box_[0], "qr: None", cv::Scalar(0, 0, 0), -1, cv::LINE_AA, 1, cv::Scalar(255,255,255), 2);

    Button receive_box_(cv::Point(x_offset,another_box_[0].pos.y+btn_height-5),cv::Size(btn_width*2+btn_space_row,btn_height));// 今 何を受け取っているかを表示
    another_box_.push_back(receive_box_);
    canvas.drawButton_new(another_box_[1], "Other: None", cv::Scalar(0, 0, 0), -1, cv::LINE_AA, 1, cv::Scalar(255,255,255), 2);

    return canvas.getImage();
}

// 定期的にpublish
void DroneGUI::timer_callback() {
    rewriteMessage();
    if(not(mat.empty())){
        publish_gui_->publish(mat);
        // RCLCPP_INFO_STREAM(this->get_logger(), "publish button gui");
    }
    if(not(temp_image_with_rect.empty())){
        publish_operator_image_->publish(temp_image_with_rect);
        // RCLCPP_INFO_STREAM(this->get_logger(), "publish remake image");
    }
}

// ボタンごとの信号処理--------------------------------------------------------------------------------------------------------------------------------------------------------
void DroneGUI::process(std::string topic_name) {
    if(std::find(trigger_list.begin(), trigger_list.end(), topic_name) != trigger_list.end()){ //被災者の顔写真を送るのか、QRのデコードならいらないかも
        bool_flags_[topic_name] = true;
        start_times_[topic_name] = this->now();
        RCLCPP_INFO_STREAM(this->get_logger(),"Push button: " << topic_name);
    }
    else if(not(topic_name == "send")){ //Drone PCから送られてきた画像をそのまま流す

        result_data.data = "OK";// disaster, debrisは状況によって変更

        if(topic_name == "V_state" && bulb_state_count == 1){
            result_data.data = "OPEN";
        }
        else if(topic_name == "V_state" && bulb_state_count == 0){
            result_data.data = "CLOSE";
        }
        
        cv::Mat result_image = cropped_temp_image;
        RCLCPP_INFO_STREAM(this->get_logger(), "Send image size: " << result_image.size());
        dt_data_publisher_->publish(result_data);
        dt_image_publisher_->publish(result_image);
        latest_topic = topic_name;
    }

    if(topic_name == "send"){ 
        std_msgs::msg::Bool msg;
        if(!send_confirm_flag){
            msg.data = !send_confirm_flag;
            dt_flag_->publish(msg);// 表示の信号
            send_confirm_flag = true;// 表示
            // RCLCPP_INFO_STREAM(this->get_logger(),"OPEN Window");
        }else if(send_confirm_flag){
            msg.data = !send_confirm_flag;
            dt_flag_->publish(msg);// 閉じる信号
            send_confirm_flag = false;// 閉じた
            // RCLCPP_INFO_STREAM(this->get_logger(),"CLOSE Window");
            // 50ms後にもう一度起動の信号を送信
            reopen_window_ = this->create_wall_timer(
                50ms,  // 1秒後
                [this]() {
                    std_msgs::msg::Bool re_msg;
                    re_msg.data = !this->send_confirm_flag;
                    // RCLCPP_INFO_STREAM(this->get_logger(),"Message: " << !this->send_confirm_flag);
                    dt_flag_->publish(re_msg);// 再度表示の信号
                    this->send_confirm_flag = true;
                    reopen_window_->cancel(); // 処理を一回で止める
                    reopen_window_.reset();
                }
            );           
        }  
    }

}

// クリック判定--------------------------------------------------------------------------------------------------------------------------------------------------------
void DroneGUI::mouse_click_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    cv::Point point(msg->x, msg->y);
    for(size_t i = 0; i < buttons_.size(); i++){
        
        // ボタンの矩形領域を定義
        cv::Rect button_rect(buttons_[i].pos, buttons_[i].size);
        // クリック位置がボタンの範囲内にあるかチェック
        if (button_rect.contains(point)) {
            std::string button_name = buttons_name_[i];

            if(button_name == "V_state" && bulb_state_count == 0){
                bulb_state_count = 1;
                rewriteButton(buttons_[i], button_name, cv::Scalar(0,0,255));
            }
            else if(button_name == "V_state" && bulb_state_count == 1){
                bulb_state_count = 0;
                rewriteButton(buttons_[i], button_name, cv::Scalar(255,0,0));
            }
            else rewriteButton(buttons_[i], button_name, cv::Scalar(0,0,255));
            // クリックされたボタンを赤色にした状態でGUIを再描画
            publish_gui_->publish(mat);
            
            process(button_name);

            // 1秒後に色を戻すタイマーを作成
            color_reset_timer_ = this->create_wall_timer(
                100ms,  // 1秒後
                [this, i, button_name]() {
                    rewriteButton(buttons_[i], button_name, cv::Scalar(255, 255, 255));
                    publish_gui_->publish(mat);
                    color_reset_timer_->cancel(); // 一回で止める
                }
            );
            publish_gui_->publish(mat);

            // RCLCPP_INFO(this->get_logger(), "Button '%s' clicked",button_name_.c_str());
        }
    }
}

// ボタンの色変更--------------------------------------------------------------------------------------------------------------------------------------------------------
void DroneGUI::rewriteButton(Button btn, std::string text, cv::Scalar color) const {
    cv::Point sp = cv::Point(btn.pos.x, btn.pos.y);
    cv::Point ep = cv::Point(btn.pos.x + btn.size.width, btn.pos.y + btn.size.height);
    cv::rectangle(mat, sp, ep, color, cv::FILLED);
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.78, 2, &baseline);
    cv::Point tp(sp.x + (btn.size.width - text_size.width) / 2, sp.y + (btn.size.height + text_size.height) / 2);
    cv::putText(mat, text, tp, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.78, cv::Scalar(0, 0, 0), 2);
}

// 画面表示文字の変更--------------------------------------------------------------------------------------------------------------------------------------------------------
void DroneGUI::rewriteMessage(){
// 値が更新されたらその都度-------------------------------------------------------------------------------
    std::string qr_text,receive_text;
    std::vector<std::string> text_list;

    if(latest_qr) qr_text ="qr: True";
    else qr_text = "qr: None";
    text_list.push_back(qr_text);

    if(latest_topic != "None"){
        receive_text = "Other: " + latest_topic;
        if(latest_topic == "V_state" and bulb_state_count == 1) receive_text += "OP";
        else if(latest_topic == "V_state" and bulb_state_count==0) receive_text += "CL";
    }
    else receive_text = "Other: None";
    text_list.push_back(receive_text);

    // qr-----------------------------------------
    for(int n = 0; n < 2; n++){
        // まず黒いボックスで上書き
        cv::Point receive_btn_sp = cv::Point(another_box_[n].pos.x, another_box_[n].pos.y);
        cv::Point receive_btn_ep = cv::Point(receive_btn_sp.x+another_box_[n].size.width,receive_btn_sp.y+another_box_[n].size.height);
        cv::rectangle(mat, receive_btn_sp, receive_btn_ep, 0, cv::FILLED);

        // その後文字を入力
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(text_list[n], cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 1, 2, &baseline);
        cv::Point tp = cv::Point(another_box_[n].pos.x + (another_box_[n].size.width - text_size.width) / 2, another_box_[n].pos.y+ (another_box_[n].size.height + text_size.height) / 2);
        cv::Scalar color = cv::Scalar(255, 255, 255);
        cv::putText(mat, text_list[n], tp, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 1, color, 2);
    }
}

void DroneGUI::publish_images()
{
    for (auto &[key, flag] : bool_flags_)
    {
        if (flag)
        {
            if((this->now() - start_times_[key]).seconds() < check_duration_sec)
            {
                std::unique_ptr<cv::Mat> msg_image = std::make_unique<cv::Mat>(cropped_temp_image);
                RCLCPP_INFO_STREAM(this->get_logger(),"Publish image address: "<< &(msg_image->data));
                image_publishers_[key]->publish(std::move(msg_image));
            } else {
                flag = false;
                start_times_[key] = rclcpp::Time(0); // 任意
                cv::Mat black_image = cv::Mat::zeros(480,640,CV_8UC1);
                std::unique_ptr<cv::Mat> msg_image = std::make_unique<cv::Mat>(black_image);
                image_publishers_[key]->publish(std::move(msg_image));
            }
        }
    }
}

} // namespace component_operator_gui

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_operator_gui::DroneGUI)