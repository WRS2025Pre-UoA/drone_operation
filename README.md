# drone_operation
## 内容
 - ドローン用PC上でボタン付きのGUI画面を表示するノード。
## 挙動の仕様
### ノード起動時、以下のパラメータを取得する
 - mode：実行するミッション番号 P1~P4,P6
 - check_duration_sec：画像を何秒間送信を行うか (秒)
 - timer_interval_ms：画像分配関数の実行間隔 (ミリ秒)
### GUI画面の使用方法(rqt)
 - rqtでplugins -> visualization -> Image Viewを選択
 - 以下のトピックを選択して表示
    - /drone_gui_with_buttons：ボタン付きGUI画面
    - /drone_gui_with_buttons_mouse_left：マウスクリックの検知

### pressure,qr,crakcsのボタンを押した場合
 - check_duration_sec / timer_interval_ms枚の画像を送信後、送信終了の合図として黒画像を流す
### 画面下部のデータ表示
 - 現在保持している報告データの状態が簡易的に表示される
    - qr : True / False  qrデータを保持している(T)、していない(F)
    - other : ~~ qrデータ以外の報告結果が何かを表示
        - 表示内容
        - | 表示名 | 内容 |
            | :----: | :----: |
            | pressure | メータの検出結果 |
            | cracks | テストピース(クラック)の検出結果 |
            | V_stateOP | バルブ状況報告 OPEN |
            | V_stateCL | バルブ状況報告 CLOSE |
            | disaster | 被災状況報告 |
            | missing | 行方不明者報告 |

- sendボタンを押した場合、確認画面ノード(misora2_dt_client)を表示
- 送信が正常に完了した場合 -> 保持しているデータを初期化する
- 送信が未完了の場合 -> そのまま保持
### ドローンからの生画像をクロップ
- ドローンはオペレータ画面を画像として流すので、四隅にある機体情報などがその後の処理に悪影響を及ぼす
- 実行時にパラメータでクロップ領域(左頂点 p1(x,y),サイズ S(w,h))を宣言し、流れてくる画像は宣言したクロップ領域に基づいて切り抜きその後の処理に移る
## 実行コード
~~~bash!
git clone git@github.com:WRS2025Pre-UoA/drone_operation.git
cd [ワーkスペース]
colcon build
source install/setup.bash
ros2 run drone_operation operator_gui_node --ros-args -p mode:=P<1~4,6> -p check_duration_sec:=1.0 -p timer_interval_ms:=500 -p top_left_x:=100 -p top_left_y:=100 -p rect_width:=300 -p rect_height:=300
~~~
