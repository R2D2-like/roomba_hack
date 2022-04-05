---
title: ロボットシステムにおけるセンシング・アクチュエーション・通信①
date: '2022-04-05'
type: book
weight: 21
---

センサの値を読み取りロボットを動かしてみよう
<!--more-->

## Learn

### ロボットセンサの基礎知識

ロボットが動作するために必要なセンサは大きく2種類に分けられる。

1つ目が外界センサで、これはロボットが行動する環境の情報を取得するためのセンサーである。
具体的なセンサとして、
- LiDAR
- デプスカメラ
- ホイールエンコーダ 
- IMU

などがあげられる。

センサのノイズの影響を軽減するため、複数のセンサを組み合わせて利用されることもある。

2つ目は内界センサで、これは(ロボットアームのような変形可能な)ロボットが自身の内部状態を把握し、位置や姿勢を制御するために使われるセンサーである。
- 関節位置・角度センサ
- 関節姿勢センサ

などが内界センサである。

参考: https://www.jsme.or.jp/jsme-medwiki/14:1013897#:~:text=robot%20sensor

## 演習

{{< spoiler text="【jetson】ブランチ切り替え" >}}
```shell
cd roomba_hack
git fetch
git checkout lec_0405 
```
{{< /spoiler >}}

{{< spoiler text="【開発マシン】ブランチ切り替え" >}}
```shell
cd roomba_hack
git fetch
マージ
```
{{< /spoiler >}}

{{< spoiler text="【jetson・開発マシン】それぞれdockerコンテナを起動" >}}
```shell
cd roomba_hack
./RUN-DOCKER-CONTAINER.sh
(docker) roomba_mode
```
{{< /spoiler >}}

{{< spoiler text="【jetson】ルンバを起動" >}}
```shell
(docker) roslaunch roomba_bringup bringup.launch
```
{{< /spoiler >}}

{{< spoiler text="【jetson】RealSenseを起動" >}}
```shell
 cd realsense_docker
 ./launch_realsense.sh
```
{{< /spoiler >}}

### ROSメッセージの可視化


