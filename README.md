三角波比較法によるVVVFインバータプログラムです。
ハイサイド側の信号しか生成していませんが、一応ローサイドのできると思います。（使用ピンはちゃんと見ないと起動時にリセットしまくる。）
その際にデッドタイムの生成はないので注意してください。
タイマー割り込み周波数は20kHz、ディスプレイ表示などは未実装です。

3+6nパルスの生成は大丈夫ですが、5パルスなどの生成がうまくできてません。（左右非対称）
電圧のV/f一定制御もできていません。（割と簡単にできると思う）

回生ブレーキの実装を目指してこれから変更していく。
