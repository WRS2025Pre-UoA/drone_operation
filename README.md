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