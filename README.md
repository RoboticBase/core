# container-centric fiware demonstaration

This repository construct a container-centric [FIWARE](http://www.fiware.org/) demonstration on [Kubernetes](https://kubernetes.io/).

![conrainer-centric-fiware-demonstration.png](/docs/images/container-centric-fiware-demonstration.png)

In the current version, Microsoft Azure AKS (preview) and minikube is tested.

## Requirements

* kubernetes client PC

||version|
|:--|:--|
|OS|macOS Sierra 10.12.6|
|azure cli|2.0.31|
|kubectl|1.10.2|
|helm|2.8.2|
|envsubst|0.19.8.1|

* minikube host PC

||version|
|:--|:--|
|OS|macOS Sierra 10.12.6|
|VirtualBox|5.2.12 r122591|
|minikube|0.27.0|

* Kubernetes

||version|
|:--|:--|
|Azure AKS|1.9.6|
|minikube|1.9.4|

## getting started
### Microsoft Azure AKS

1. [prepare Microsoft Azure AKS](/docs/azure_aks/1_prepare_aks.md)
1. [start containers on Kubernetes](/docs/azure_aks/2_start_containers.md)
1. [configure fiware](/docs/azure_aks/3_configure_fiware.md)
1. [operate 'turtlesim'](/docs/azure_aks/4_operate_turtlesim.md)
1. [operate 'gopigo'](/docs/azure_aks/5_operate_gopigo.md) (if gopigo is available)

### minikube

1. [prepare minikube](/docs/minikube/1_prepare_minikube.md)
1. [start containers on Kubernetes](/docs/minikube/2_start_containers.md)
1. [configure fiware](/docs/minikube/3_configure_fiware.md)
1. [operate 'turtlesim'](/docs/minikube/4_operate_turtlesim.md)
1. [operate 'gopigo'](/docs/minikube/5_operate_gopigo.md) (if gopigo is available)

## Related Repositories
### customized FIWARE components
* [tech-sketch/iotagent-ul](https://github.com/tech-sketch/iotagent-ul)
    * original: [telefonicaid/iotagent-ul](https://github.com/telefonicaid/iotagent-ul)
    * What's the problem?
        * Let's say that you want a iotagent-ul SERVICE which has multiple iotagent-ul PODs on your Kubernetes.
        * When you put a message to iotagent-ul by using HTTP, there is no problem because iotagent-ul SERVICE routes a HTTP message to only one POD.
        * But when you put a message to iotagent-ul by using MQTT, unfortunatly a MQTT message is processed as many times as the number of iotagent-ul PODs. Because the each iotagent-ul PODs subscribes for the same topic of MQTT Broker, so a MQTT message published that topic is proccessed by each PODs individually.
    * How to treat this
        * When a MQTT message is received, the customized iotagent-ul calls a REST API endpoint before processing the MQTT message.
        * If the REST API returns `200 OK`, the customized iotagent-ul continues processing the MQTT message as ordinally.
        * But if the REST API returns `409 Conflict`, the customized iotagent-ul stops processing.
        * To do so, the cluster of iotagent-ul PODs processes only once for a MQTT message.

### FIWARE support components
* [tech-sketch/fiware-ambassador-auth](https://github.com/tech-sketch/fiware-ambassador-auth)
    * A REST API component working with [Ambassador](https://www.getambassador.io/) on Kubernetes in order to authorize and authanticate the client.
    * Bearar Authenticaton and Basic Authentication are supported.
* [tech-sketch/fiware-mqtt-msgfilter](https://github.com/tech-sketch/fiware-mqtt-msgfilter)
    * A REST API component working with [tech-sketch/iotagent-ul](https://github.com/tech-sketch/iotagent-ul) and [etcd](https://coreos.com/etcd/docs/latest/) in order to check the message duplication.

### Business Logic components
* [tech-sketch/fiware-cmd-proxy](https://github.com/tech-sketch/fiware-cmd-proxy)
    * A REST API component working with [FIWARE orion context broker](https://github.com/telefonicaid/fiware-orion) in order to receive a command from gamepad or web controler and to send a command to ROS robot.

### gamepad controller
* [tech-sketch/fiware-gamepad-controller](https://github.com/tech-sketch/fiware-gamepad-controller)
    * A python3.6 application in order to receive gamepad events and to send a command corresponding the event to MQTT broker.

### ROS package
* [tech-sketch/fiware-ros-turtlesim](https://github.com/tech-sketch/fiware-ros-turtlesim)
    * A [ROS](http://wiki.ros.org/) pakage witten by python2 in order to act as a bridge between MQTT broker and ROS nodes.
    * When a MQTT message is received from a MQTT topic, this package publish a seriese of ROS message to a ROS topic for turtlesim.
    * At the opposite, when a ROS message is received from a ROS topic, this package publish a MQTT message to a MQTT topic.
* [tech-sketch/fiware-ros-gopigo](https://github.com/tech-sketch/fiware-ros-gopigo)
    * A [ROS](http://wiki.ros.org/) pakage witten by python2 in order to act as a bridge between MQTT broker and ROS nodes.
    * When a MQTT message is received from a MQTT topic, this package publish a seriese of ROS message to a ROS topic for gopigo.

## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2018 [TIS Inc.](https://www.tis.co.jp/)
