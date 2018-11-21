# Change Log

## [Unreleased]

## [0.3.0]
### Added
* employed "Cygnus ElasticsearchSink"
* employed "ROS bridge and ROS operator"

## [0.2.0]
### Changed
* testd on Azure AKS(1.11.2) and minikube(1.10.0)
* employed "RabbitMQ" as MQTT Broker and Message Queue instead of "VerneMQ".
    * "etcd" and "fiware-mqtt-msgfilter" were no longer needed.

### Added
* employed "Prometheus & Grafana" as monitoring of Kubernetes.
* employed "Elasticsearch & fluentd & Kibana" as logging of Kubernetes.
* added "[mqtt-kube-operator](https://github.com/tech-sketch/mqtt-kube-operator)" in order to enable remote deployment of ROS programs.

### Removed
* discontinued to use "VerneMQ"
* discontinued to use "etcd"
* discontinued to use "fiware-mqtt-msgfilter"

## [0.1.1]
### Changed
* testd on Azure AKS(1.10.3) and minikube(1.10.0)

### Added
* employed "jupyter notebook" as operable documents.
* employed "turtlebot3 (simulator and real robot)" as ROS robot.

### Removed
* discontinued to use "turtlesim" and "gopigo".

## [0.1.0]
### Added
* deployed FIWARE (orion, iotagent-ul, cygnus) on Kubernetes.
    * tested on Azure AKS(1.9.6) and minikube(1.9.4).
* employed "Ambassador" as API Gateway.
* employed "VerneMQ" as MQTT Broker.
    * in order to ignore duplicated messages, "etcd" and "[fiware-mqtt-msgfilter](https://github.com/tech-sketch/fiware-mqtt-msgfilter)" were used.
* employed "turtlesim" and "gopigo" as ROS robot.
