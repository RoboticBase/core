# Change Log

## [Unreleased]
* We will employ "wirecloud" as a core component of RoboticBase-core
* We will employ`"sth-comet" as a core component of RoboticBase-core
* We will employ`"kurento" as a core component of RoboticBase-core

## [0.4.0]
### Changed
* split repository ([core](https://github.com/RoboticBase/core) and [example-turtlebot3](https://github.com/RoboticBase/example-turtlebot3))
* update components to adjust [FIWARE Release 7.5.1](https://github.com/Fiware/catalogue/releases)

* rabbitmq
    * no change
* mongodb
    * 3.6 -> 4.0.6
    * use healm (stable/mongodb-replicaset)
* ambassador
    * 0.39.0 -> 0.50.0
    * change the "apiVersion" of deployment
* fiware-ambassador-auth -> auth
    * 0.1.1 -> 0.2.0
    * change the "apiVersion" of deployment
    * change docker repository (techsketch/fiware-ambassador-auth -> roboticbase/fiware-ambassador-auth)
    * change the format of `auth-token.json`
    * move to `auth` directory
* orion
    * 1.15.1 -> 2.1.0
    * split the single yaml to service yaml and deployment yaml
    * change the "apiVersion" of deployment
    * add a routing rule which uses a regex of host name
* idas
    * private image(tech-sketch/iotagent-ul:290a1fa) -> fiware/iotagent-ul:1.8.0
    * split the single yaml to service yaml and deployment yaml
    * rename configuration file (`config.js` -> `rb-config.js`)
    * remove `iotagent-ul/Dockerfile`
    * change the "apiVersion" of deployment
    * add a routing rule which uses a regex of host name
* cygnus
    * techsketch/fiware-cygnus:1.9.0.elasticsearch -> fiware/cygnus-ngsi:1.10.0
    * split the single yaml to service yaml and deployment yaml
    * change the "apiVersion" of deployment
    * change CYGNUS_MONGO_ATTR_PERSISTENCE (column -> row)
        * because `sth-comet` can't recognize "column" style record
* cygnus-elasticsearch
    * techsketch/fiware-cygnus:1.9.0.elasticsearch -> roboticbase/fiware-cygnus:1.9.0.elasticsearch
* prometheus
    * v2.3.2 -> v2.7.1
    * deployCoreDNS false -> true
    * deployKubeDNS true -> false
* alertmanager
    * v0.15.1 -> v0.16.1
* grafana
    * 5.2.2 -> 5.4.3
* elasticsearch
    * v6.2.5 -> v6.3.0
* fluentd
    * v2.2.0 -> v2.4.0
* kibana
    * 6.2.4 -> 6.3.2

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
