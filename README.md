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
|[fiware-ambassador-auth](https://github.com/RoboticBase/fiware-ambassador-auth)|Authorization and Authentication component working with ambassador|
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

## An experiment to prove our concept
We and University of Aizu have been performed an experiment to guide a visitor by collaborating with heterogeneous robots, IoT devices and people through this Robot Platform on Nov. 6th - 8th , 2018.

Please see this repository [ogcaizu/ogc-poc1](https://github.com/ogcaizu/ogc-poc1).

[![video](http://img.youtube.com/vi/D9NPxxYgPa0/0.jpg)](https://youtu.be/D9NPxxYgPa0)

## Requirements

### When you use macOS,

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
|minikube|0.34.1|
|kubernetes|1.12.5|

### When you use Ubuntu,
* kubernetes client PC

||version|
|:--|:--|
|OS|Ubuntu 16.04|
|kubectl|1.12.2|
|helm|2.11.0|
|envsubst|0.19.7|

* minikube
    * when you use monitoring & logging, you have to give **4 cpu & 8192 MB memories** to minikube.

||version|
|:--|:--|
|OS|Ubuntu 16.04|
|VirtualBox|5.2.14 r123301|
|minikube|0.34.1|
|kubernetes|1.12.5|


* Azure AKS
    * when you use monitoring & logging, you have to use the vm series which supports `Premium Storage` such as `Dsv3-series`.

||version|
|:--|:--|
|region|japaneast|
|kubernetes|1.12.5|

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

### Microsoft Azure AKS using macOS

1. setup environment variables

    ```bash
    $ cp azure_aks/env.template azure_aks/env
    $ vi env
    ```
1. prepare Microsoft Azure AKS -- [/docs/azure_aks/macOS/01_prepare_aks.ipynb](/docs/azure_aks/macOS/01_prepare_aks.ipynb).
1. start pods on Azure AKS -- [/docs/azure_aks/macOS/02_start_pods.ipynb](/docs/azure_aks/macOS/02_start_pods.ipynb).
1. start monitoring and logging on Azure AKS -- [/doss/azure_aks/macOS/03_start_monitoring_and_logging.ipynb](/docs/azure_aks/macOS/03_start_monitoring_and_logging.ipynb).

### Microsoft Azure AKS using Ubuntu

1. setup environment variables

    ```bash
    $ cp azure_aks/env.template azure_aks/env
    $ vi env
    ```
    1. prepare Microsoft Azure AKS -- [/docs/azure_aks/macOS/01_prepare_aks.ipynb](/docs/azure_aks/macOS/01_prepare_aks.ipynb).
    1. start pods on Azure AKS -- [/docs/azure_aks/macOS/02_start_pods.ipynb](/docs/azure_aks/macOS/02_start_pods.ipynb).
    1. start monitoring and logging on Azure AKS -- [/doss/azure_aks/macOS/03_start_monitoring_and_logging.ipynb](/docs/azure_aks/macOS/03_start_monitoring_and_logging.ipynb).

### minikube on macOS

1. setup environment variables

    ```bash
    $ cp minikube/env.template minikube/env
    $ vi env
    ```
1. prepare minikube -- [/docs/minikube/macOS/01_prepare_minikube.ipynb](/docs/minikube/macOS/01_prepare_minikube.ipynb).
1. start pods on minikube -- [/docs/minikube/macOS/02_start_pods.ipynb](/docs/minikube/macOS/02_start_pods.ipynb).
1. start monitoring and logging on minikube -- [/docs/minikube/macOS/03_start_monitoring_and_logging.ipynb](/docs/minikube/macOS/03_start_monitoring_and_logging.ipynb).


### minikube on Ubuntu

1. setup environment variables

    ```bash
    $ cp minikube/env.template minikube/env
    $ vi env
    ```
1. prepare minikube -- [/docs/minikube/Ubuntu/01_prepare_minikube.ipynb](/docs/minikube/Ubuntu/01_prepare_minikube.ipynb).
1. start pods on minikube -- [/docs/minikube/Ubuntu/02_start_pods.ipynb](/docs/minikube/Ubuntu/02_start_pods.ipynb).
1. start monitoring and logging on minikube -- [/docs/minikube/Ubuntu/03_start_monitoring_and_logging.ipynb](/docs/minikube/Ubuntu/03_start_monitoring_and_logging.ipynb).


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

### Support components
* [RoboticBase/fiware-ambassador-auth](https://github.com/RoboticBase/fiware-ambassador-auth)
    * A REST API component working with [Ambassador](https://www.getambassador.io/) on Kubernetes in order to authorize and authanticate the client.
    * Bearar Authenticaton and Basic Authentication are supported.

## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2018 [TIS Inc.](https://www.tis.co.jp/)
