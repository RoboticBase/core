# RoboticBase-core

This repository is a core components of "RoboticBase".

## Description
"RoboticBase" is a robot management platform based on [FIWARE](http://www.fiware.org/) which enables you to manage and operate many kinds of robots and IoT devices as interactions of contexts.

"RoboticBase" allows robots to collaborate with IoT devices, Open Data, human beings and so on. You can connect a robot to "RoboticBase" using the open APIs of the robot, and operate the robot through those APIs. In turn, "RoboticBase" has an ability to manage ROS. If you connect a ROS robot to "RoboticBase", you can operate the robot directly without restrictions.  
For example, you can deploy a ROS program to the robot and access the raw data of the robot through "RoboticBase".

![roboticbase-core-architecture.png](/docs/images/roboticbase-core-architecture.png)

|component|summary|
|:--|:--|
|[kubernetes](https://kubernetes.io/)|Container Orchestration Platform|
|[ambassador](https://www.getambassador.io/)|API Gateway|
|fiware-ambssador-auth|Authorization and Authentication component working with ambassador|
|fiware-cmd-proxy|Business Logic component working with FIWARE orion|
|[FIWARE orion](https://catalogue-server.fiware.org/enablers/publishsubscribe-context-broker-orion-context-broker)|Publish/Subscribe Context Broker|
|[FIWARE cygnus](https://catalogue-server.fiware.org/enablers/cygnus)|Data collection and Persistence Agent|
|[FIWARE iotagent-ul](https://catalogue-server.fiware.org/enablers/backend-device-management-idas)|Backend Device Management Agent|
|[RabbitMQ](https://www.rabbitmq.com/)|Distributed Message Queue|
|[MongoDB](https://www.mongodb.com/)|Document-oriented NoSQL Database|
|[Prometheus](https://prometheus.io/)|Monitoring and Alerting toolkit|
|[Grafana](https://grafana.com/)|Analytics and Alerting platform for time series metrics|
|[Elasticsearch](https://www.elastic.co/products/elasticsearch)|Distributed search and analytics engine|
|[fluentd](https://www.fluentd.org/)|Data collector for unified logging layer|
|[Kibana](https://www.elastic.co/products/kibana)|Visualize the Elasticsearch data|

|gamepad|summary|
|:--|:--|
|[gamepad](https://github.com/tech-sketch/fiware-gamepad-controller)|Gamepad Controller connecting FIWARE|

|robot(Android)|summary|
|:--|:--|
|[robot(Android)](https://github.com/tech-sketch/fiware_xperiahello)|Android Application for [Xperia Hello!](https://www.sonymobile.co.jp/product/smartproducts/g1209/)|

|turtlebot3|summary|
|:--|:--|
|[deployer](https://github.com/tech-sketch/mqtt-kube-operator)|MQTT client to deploy (or delete) a resource to its own Kubernetes|
|[bridge](https://github.com/tech-sketch/fiware_ros_turtlebot3_bridge)|ROS package to act as a bridge FIWARE orion and ROS|
|[operator](https://github.com/tech-sketch/fiware_ros_turtlebot3_operator)|ROS package to control turtlebot3 (simulator and physical robot)|

## An experiment to prove our concept
We and University of Aizu have been performed an experiment to guide a visitor by collaborating with heterogeneous robots, IoT devices and people through this Robot Platform on Nov. 6th - 8th , 2018.

Please see this repository [ogcaizu/ogc-poc1](https://github.com/ogcaizu/ogc-poc1).

[![video](http://img.youtube.com/vi/D9NPxxYgPa0/0.jpg)](https://youtu.be/D9NPxxYgPa0)

## Requirements

* kubernetes client PC

||version|
|:--|:--|
|OS|macOS Sierra 10.12.6|
|azure cli|2.0.45|
|kubectl|1.11.2|
|helm|2.10.0|
|envsubst|0.19.8.1|

* minikube
    * when you use monitoring & logging, you have to give **4 cpu & 8192 MB memories** to minikube.

||version|
|:--|:--|
|OS|macOS Sierra 10.12.6|
|VirtualBox|5.2.12 r122591|
|minikube|0.28.2|
|kubernetes|1.10.0|

* Azure AKS
    * when you use monitoring & logging, you have to use the vm series which supports `Premium Storage` such as `Dsv3-series`.

||version|
|:--|:--|
|region|japaneast|
|kubernetes|1.11.2|

## getting started

1. install jupyter notebook

    ```bash
    $ cd docs
    $ ./setup_jupyter_notebook.sh
    ```
1. start jupyter notebook

    ```bash
    $ ./start_jupyter_notebook.sh
    ```

### Microsoft Azure AKS

1. setup environment variables

    ```bash
    $ cp azure_aks/env.template azure_aks/env
    $ vi env
    ```
1. prepare Microsoft Azure AKS -- [/docs/azure_aks/01_prepare_aks.ipynb](/docs/azure_aks/01_prepare_aks.ipynb).
1. start pods on Azure AKS -- [/docs/azure_aks/02_start_pods.ipynb](/docs/azure_aks/02_start_pods.ipynb).
1. register iot device & robot to FIWARE  -- [/docs/azure_aks/03_register_device.ipynb](/docs/azure_aks/03_register_device.ipynb).
1. register business logic to FIWARE -- [/docs/azure_aks/04_register_business_logic.ipynb](/docs/azure_aks/04_register_business_logic.ipynb).
1. start monitoring and logging on Azure AKS -- [/doss/azure_aks/05_start_monitoring_and_logging.ipynb](/docs/azure_aks/05_start_monitoring_and_logging.ipynb).
1. prepare minikube in turtlebot3, and start `mqtt-kube-operator` in order to enable remote deployment -- [/docs/azure_aks/06_prepare_remote_deploy.ipynb](/docs/azure_aks/06_prepare_remote_deploy.ipynb).
1. deploy programs to turtlebot3 through FIWARE -- [/docs/azure_aks/07_deploy_containers_to_turtlebot3.ipynb](/docs/azure_aks/07_deploy_containers_to_turtlebot3.ipynb).
1. operate turtlebot3 step by step using [/docs/azure_aks/08_operate_turtlebot3.ipynb](/docs/azure_aks/08_operate_turtlebot3.ipynb).
1. visualize the data of turtlebot3 step by step using [/docs/azure_aks/09_visualize_data.ipynb](/docs/azure_aks/09_visualize_data.ipynb).
1. delete programs from turtlebot3 through FIWARE -- [/docs/azure_aks/10_delete_containers_from_turtlebot3.ipynb](/docs/azure_aks/10_delete_containers_from_turtlebot3.ipynb).

### minikube

1. setup environment variables

    ```bash
    $ cp minikube/env.template minikube/env
    $ vi env
    ```
1. prepare minikube -- [/docs/minikube/01_prepare_minikube.ipynb](/docs/minikube/01_prepare_minikube.ipynb).
1. start pods on minikube -- [/docs/minikube/02_start_pods.ipynb](/docs/minikube/02_start_pods.ipynb).
1. register iot device & robot to fiware -- [/docs/minikube/03_register_device.ipynb](/docs/minikube/03_register_device.ipynb).
1. register business logic to FIWARE -- [/docs/minikube/04_register_business_logic.ipynb](/docs/minikube/04_register_business_logic.ipynb).
1. start monitoring and logging on minikube -- [/doss/minikube/05_start_monitoring_and_logging.ipynb](/docs/minikube/05_start_monitoring_and_logging.ipynb).
1. prepare minikube in turtlebot3, and start `mqtt-kube-operator` in order to enable remote deployment -- [/docs/minikube/06_prepare_remote_deploy.ipynb](/docs/minikube/06_prepare_remote_deploy.ipynb).
1. deploy programs to turtlebot3 through FIWARE -- [/docs/minikube/07_deploy_containers_to_turtlebot3.ipynb](/docs/minikube/07_deploy_containers_to_turtlebot3.ipynb).
1. operate turtlebot3 step by step using [/docs/minikube/08_operate_turtlebot3.ipynb](/docs/minikube/08_operate_turtlebot3.ipynb).
1. visualize the data of turtlebot3 step by step using [/docs/minikube/09_visualize_data.ipynb](/docs/minikube/09_visualize_data.ipynb).
1. delete programs from turtlebot3 through FIWARE -- [/docs/minikube/10_delete_containers_from_turtlebot3.ipynb](/docs/minikube/10_delete_containers_from_turtlebot3.ipynb).


## Related Repositories (Cloud)
### FIWARE components
* [telefonicaid/fiware-orion](https://github.com/telefonicaid/fiware-orion)
    * Orion is a FIWARE's reference implementation of the Publish/Subscribe Context Broker.
* [telefonicaid/iotagent-ul](https://github.com/telefonicaid/iotagent-ul)
    * IotAgent-UL is a bridge that can be used to communicate devices using the Ultralight 2.0 protocol and Orion.
        * Ultralight 2.0 is a lightweight text based protocol aimed to constrained devices and communications where the bandwidth and device memory may be limited resources.
* [telefonicaid/fiware-cygnus](https://github.com/telefonicaid/fiware-cygnus)
    * Cygnus is a connector in charge of persisting certain sources of data in certain configured third-party storages, creating a historical view of such data.
        * In this demonstration, historical data are stored to mongodb.

### Business Logic components
* [tech-sketch/fiware-cmd-proxy](https://github.com/tech-sketch/fiware-cmd-proxy)
    * A web application working with [FIWARE orion context broker](https://github.com/telefonicaid/fiware-orion) in order to receive a command from gamepad or web controler and to send a command to ROS robot.
* [tech-ksetch/fiware-robot-visualization](https://github.com/tech-sketch/fiware-robot-visualization)
    * A web application working with [FIWARE cygnus](https://github.com/telefonicaid/fiware-cygnus) in order to visualize the locus of ROS robot.

### Support components
* [tech-sketch/fiware-ambassador-auth](https://github.com/tech-sketch/fiware-ambassador-auth)
    * A REST API component working with [Ambassador](https://www.getambassador.io/) on Kubernetes in order to authorize and authanticate the client.
    * Bearar Authenticaton and Basic Authentication are supported.

## Related Repositories (Device & Robot)
### gamepad controller
* [tech-sketch/fiware-gamepad-controller](https://github.com/tech-sketch/fiware-gamepad-controller)
    * A python3.6 application in order to receive gamepad events and to send a command corresponding the event to FIWARE.

### android application for Xperia Hello!
* [tech-sketch/fiware_xperiahello](https://github.com/tech-sketch/fiware_xperiahello)
    * An android application for Xperia Hello! It connect to FIWARE using MQTT(S).

### ROS package
* [tech-sketch/fiware_ros_turtlebot3_bridge](https://github.com/tech-sketch/fiware_ros_turtlebot3_bridge)
    * A [ROS](http://wiki.ros.org/) pakage witten by python2 in order to act as a bridge between FIWARE and ROS nodes.
    * When a MQTT message is received from a MQTT topic, this package create ROS message and publish a ROS message to a ROS topic.
    * At the opposite, when a ROS message is received from a ROS topic, this package publish a MQTT message to a MQTT topic.
* [tech-sketch/fiware_ros_turtlebot3_operator](https://github.com/tech-sketch/fiware_ros_turtlebot3_operator)
    * A [ROS](http://wiki.ros.org/) pakage witten by python2 in order to control "turtlebot3" and receive its odometries.
    * You can use this package with either actual robot or simulator.

### Support components
* [tech-sketch/mqtt-kube-operator](https://github.com/tech-sketch/mqtt-kube-operator)
    * A MQTT client to deploy (or delete) a resource to its own Kubernetes.

## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2018 [TIS Inc.](https://www.tis.co.jp/)
