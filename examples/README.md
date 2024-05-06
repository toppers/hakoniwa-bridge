# これは何？

箱庭ブリッジの簡単なサンプルコードです。

このサンプルコードを利用すると以下を試すことができます。

* バーチャル側の箱庭アセットからPDUデータを書き込みすると、エッジ側でそのデータを参照できる
* エッジ側でPDUデータを配信すると、バーチャル側の箱庭アセットでそのデータを参照できる

# 構成

サンプルプログラムの構成は下図のとおりです。

![example](../images/example.png)

バーチャル側の箱庭アセット HakoAssetSample は、箱庭PDUデータの読み書きを行います。

HakoAssetSample が読み込みするデータは、`RobotAvator_cmd_pos` です。

HakoAssetSample が書き込みするデータは、`RobotAvator_baggage_sensor` と `RobotAvator_bumper_sensor` です。

これらのデータは、ShmProxyを通して、エッジ側（Real）と共有されます。

リアル側では、RosProxyを通して、ROSトピックとして配信/購読できます。

HakoAssetSample が書き込みしたデータ`RobotAvator_baggage_sensor` と `RobotAvator_bumper_sensor` は、ros2 topic echo で参照できます。


HakoAssetSample が読み込みするデータである `RobotAvator_cmd_pos` は、ros2 topic echo コマンドで送信できます。


# 準備

事前に、バーチャル側とエッジ側のインストールを実施してください。

# 実行方法


## エッジ側

RosProxy を起動します。

```
ros2 run hako_ros_proxy hako_ros_proxy_node 
```

HakoAssetSample が書き込みしたデータ`RobotAvator_baggage_sensor` と `RobotAvator_bumper_sensor` を、ros2 topic echo で参照します。

RobotAvator_baggage_sensor:
```
ros2 topic echo /RobotAvator_baggage_sensor
```

RobotAvator_bumper_sensor:
```
ros2 topic echo /RobotAvator_bumper_sensor:
```

## バーチャル側

ShmProxy を起動します。

```
 ./cmake-build/shm-proxy/shm-proxy ShmProxy ../third-party/hakoniwa-ros2pdu/config/custom.json 20 master
```

サンプルプログラムを起動します。

```
TODO
```
