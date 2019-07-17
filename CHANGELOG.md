# Change Log

## [Unreleased]
* We will employ "wirecloud" as a core component of RoboticBase-core
* We will employ "kurento" as a core component of RoboticBase-core

## [0.4.4]
### Changed
* updated components and documents to adjust [FIWARE Release 7.7.1](https://github.com/FIWARE/catalogue/releases/tag/FIWARE_7.7.1)
    * Kubernetes

        |component|version|(previous version)|
        |:--|:--|:--|
        |Azure AKS|1.13.7|(1.13.5)|
        |minikube|1.14.4|(1.14.1)|
    * Support components

        |component|version|(previous version)|
        |:--|:--|:--|
        |mongodb|4.1.13|(4.1.10)|
        |rabbitmq|3.7.16|(3.7.14)|
        |ambassador|0.73.0|(0.53.1)|
        |auth|0.3.0|(0.2.0)|
    * FIWARE components

        |component|version|(previous version)|
        |:--|:--|:--|
        |orion|2.2.0|-|
        |iotagent-ul|1.9.0|(1.8.0)|
        |iotagent-json|1.10.0|(1.9.0)|
        |cygnus|1.15.0|(1.10.0)|
        |sth-comet|2.5.0|-|
    * Monitoring components

        |component|version|(previous version)|
        |:--|:--|:--|
        |prometheus|2.10.0|-|
        |alertmanager|0.17.0|-|
        |grafana|6.2.4|-|
    * Logging components

        |component|version|(previous version)|
        |:--|:--|:--|
        |elasticsearch|6.3.0|-|
        |fluentd|2.4.0|-|
        |kibana|6.3.2|-|
        |curator|5.6.0|-|
* [fiware/cygnus-ngsi:1.15.0](https://hub.docker.com/r/fiware/cygnus-ngsi/tags) was used as the container of cygnus-elasticsearch instead of [roboticbase/fiware-cygnus:1.9.0.elasticsearch](https://hub.docker.com/r/roboticbase/fiware-cygnus/tags) because `NGSIelasticsearchSink` had been merged to [fiware/cygnus-ngsi](https://hub.docker.com/r/fiware/cygnus-ngsi) at `1.15.0`.

## [0.4.3]
### Added
* added the function to back up mongodb data to Azure Blob Storage periodically (in the case of AKS).
* employed the stable HELM [stable/prometheus-operator](https://github.com/helm/charts/tree/master/stable/prometheus-operator) for deploying Prometheus and Grafana instead of the [deprecated coreos's HELM](https://github.com/coreos/prometheus-operator/tree/master/helm).
* employed below FIWARE components:

    |component|version|
    |:--|:--|
    |sth-comet|2.5.0|
    |iotagent-json|1.9.0|

## [0.4.2]
### Changed
* created the subdomains and routing rules of "kibana" and "grafana", and expose them to Internet (in the case of AKS).
* updated the `auth` component to be able to change the auth tokens dynamically.

## [0.4.1]
### Changed
* updated components and documents to adjust [FIWARE Release 7.6](https://github.com/FIWARE/catalogue/releases/tag/FIWARE_7.6)
    * FIWARE components

        |component|version|(previous version)|
        |:--|:--|:--|
        |orion|2.2.0|(2.1.0)|
        |iotagent-ul|1.8.0|-|
        |cygnus|1.10.0|-|
    * Support components

        |component|version|(previous version)|
        |:--|:--|:--|
        |mongodb|4.1.10|(4.0.6)|
        |rabbitmq|3.7.14|-|
        |ambassador|0.53.1|(0.50.2)|
        |auth|0.2.0|-|
    * Monitoring components

        |component|version|(previous version)|
        |:--|:--|:--|
        |prometheus|2.9.1|2.7.1|
        |alertmanager|0.16.2|0.16.1|
        |grafana|6.1.4|5.4.3|
    * Logging components

        |component|version|(previous version)|
        |:--|:--|:--|
        |elasticsearch|6.3.0|-|
        |fluentd|2.4.0|-|
        |kibana|6.3.2|-|
        |curator|5.6.0|5.5.4|

## [0.4.0]
### Changed
* splitted repository ([core](https://github.com/RoboticBase/core) and [example-turtlebot3](https://github.com/RoboticBase/example-turtlebot3))
* updated components and documents to adjust [FIWARE Release 7.5.1](https://github.com/Fiware/catalogue/releases)

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
    * change `CYGNUS_MONGO_ATTR_PERSISTENCE` (column -> row)
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
* Kubernetes
    * 1.10.0 -> 1.12.5
* minikube
    * 0.28.2 -> 0.34.1

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
