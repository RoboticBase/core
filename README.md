# RoboticBase-core

This repository is a core components of "RoboticBase". The latest version (0.4.2) conforms to [FIWARE Release 7.6](https://github.com/FIWARE/catalogue/releases/tag/FIWARE_7.6).

## Description
"RoboticBase" is a robot management platform based on [FIWARE](http://www.fiware.org/) which enables you to manage and operate many kinds of robots and IoT devices as interactions of contexts.

"RoboticBase" allows robots to collaborate with IoT devices, Open Data, human beings and so on. You can connect a robot to "RoboticBase" using the open APIs of the robot, and operate the robot through those APIs. In turn, "RoboticBase" has an ability to manage ROS. If you connect a ROS robot to "RoboticBase", you can operate the robot directly without restrictions.  
For example, you can deploy a ROS program to the robot and access the raw data of the robot through "RoboticBase".

![roboticbase-core-architecture.png](/docs/images/roboticbase-core-architecture.png)


|FIWARE components|summary|version|
|:--|:--|:--|
|[FIWARE orion](https://catalogue-server.fiware.org/enablers/publishsubscribe-context-broker-orion-context-broker)|Publish/Subscribe Context Broker|2.2.0|
|[FIWARE cygnus](https://catalogue-server.fiware.org/enablers/cygnus)|Data collection and Persistence Agent|1.10.0|
|[FIWARE sth-comet](https://catalogue-server.fiware.org/enablers/sth-comet)|An Agent to manage historical raw and aggregated time series context|2.5.0|
|[FIWARE IDAS](https://catalogue-server.fiware.org/enablers/backend-device-management-idas)|Backend Device Management Agent|[iotagent-ul](https://fiware-iotagent-ul.readthedocs.io/en/latest/) 1.8.0 <br/> [iotagent-json](https://fiware-iotagent-json.readthedocs.io/en/latest/) 1.9.0|

|Other components|summary|version|
|:--|:--|:--|
|[kubernetes](https://kubernetes.io/)|Container Orchestration Platform|1.13 or higher|
|[ambassador](https://www.getambassador.io/)|API Gateway|0.53.1|
|[auth](https://github.com/RoboticBase/fiware-ambassador-auth)|Authorization and Authentication component working with ambassador|0.3.0|
|[RabbitMQ](https://www.rabbitmq.com/)|Distributed Message Queue|3.7.14|
|[MongoDB](https://www.mongodb.com/)|Document-oriented NoSQL Database|4.1.10|
|[Prometheus](https://prometheus.io/)|Monitoring and Alerting toolkit|2.9.1|
|[Grafana](https://grafana.com/)|Analytics and Alerting platform for time series metrics|6.1.4|
|[Elasticsearch](https://www.elastic.co/products/elasticsearch)|Distributed search and analytics engine|6.3.0|
|[fluentd](https://www.fluentd.org/)|Data collector for unified logging layer|2.4.0|
|[Kibana](https://www.elastic.co/products/kibana)|Visualize the Elasticsearch data|6.3.2|

## An experiment to prove our concept
We and University of Aizu have been performed an experiment to guide a visitor by collaborating with heterogeneous robots, IoT devices and people through this Robot Platform on Nov. 6th - 8th , 2018.

Please see this repository [ogcaizu/ogc-poc1](https://github.com/ogcaizu/ogc-poc1).

[![video](http://img.youtube.com/vi/D9NPxxYgPa0/0.jpg)](https://youtu.be/D9NPxxYgPa0)

## Requirements

* kubernetes client PC
    * `azure cli` is required when you use Azure AKS.

||version|
|:--|:--|
|OS|macOS Sierra 10.12.6 or Ubuntu 16.04|
|azure cli|2.0.63|
|kubectl|1.14.1|
|helm|2.13.1|

* Azure AKS

||version|
|:--|:--|
|region|japaneast|
|kubernetes|1.13.5|

* minikube
    * when you use monitoring & logging, you have to give **4 cpu & 8192 MB memories** to minikube.

||version|
|:--|:--|
|VirtualBox|5.2.28 r130011|
|minikube|1.0.0|
|kubernetes|1.14.1|

## getting started
### jupyter notebook (english)
1. install python3

1. install jupyter notebook

    ```bash
    $ cd docs/en-jupyter_notebook/
    $ ./setup_jupyter_notebook.sh
    ```
1. start jupyter notebook

    ```bash
    $ ./start_jupyter_notebook.sh
    ```

1. execute the commands in order according to the following documents:
    * when using Azure AKS
        * 01 prepare Microsoft Azure AKS -- [01_prepare_aks.ipynb](/docs/en-jupyter_notebook/azure_aks/01_prepare_aks.ipynb).
        * 02 start base pods on Azure AKS -- [02_start_base_pods.ipynb](/docs/en-jupyter_notebook/azure_aks/02_start_base_pods.ipynb).
        * 03 start auth pods for API Gateway on Azure AKS -- [03_start_auth_pods.ipynb](/docs/en-jupyter_notebook/azure_aks/03_start_auth_pods.ipynb).
        * 04 start FIWARE pods on Azure AKS -- [04_start_fiware_pods.ipynb](/docs/en-jupyter_notebook/azure_aks/04_start_fiware_pods.ipynb).
        * 05 start monitoring and logging on Azure AKS -- [05_start_monitoring_and_logging.ipynb](/docs/en-jupyter_notebook/azure_aks/05_start_monitoring_and_logging.ipynb).
        * 06 backup mongodb data to Azure Blog Storage -- [06_backup_mongodb.ipynb](/docs/en-jupyter_notebook/azure_aks/06_backup_mongodb.ipynb).
    * when using minikube
        * 01 prepare minikube -- [01_prepare_minikube.ipynb](/docs/en-jupyter_notebook/minikube/01_prepare_minikube.ipynb).
        * 02 start base pods on minikube -- [02_start_base_pods.ipynb](/docs/en-jupyter_notebook/minikube/02_start_base_pods.ipynb).
        * 03 start auth pods for API Gateway on minikube -- [03_start_auth_pods.ipynb](/docs/en-jupyter_notebook/minikube/03_start_auth_pods.ipynb).
        * 04 start FIWARE pods on minikube -- [04_start_fiware_pods.ipynb](/docs/en-jupyter_notebook/minikube/04_start_fiware_pods.ipynb).
        * 05 start monitoring and logging on minikube -- [05_start_monitoring_and_logging.ipynb](/docs/en-jupyter_notebook/minikube/05_start_monitoring_and_logging.ipynb).

### markdown (japanese)
1. ターミナルを開き、Markdownのドキュメントに従ってコマンドを実行してください
    * Azure AKSを用いる場合
        * 01 Microsoft Azure AKSの準備 -- [01_prepare_aks.md](/docs/ja-markdown/azure_aks/01_prepare_aks.md).
        * 02 AKS上にベースとなるPODを起動 -- [02_start_base_pods.md](/docs/ja-markdown/azure_aks/02_start_base_pods.md).
        * 03 AKS上にAPI Gateway認証用PODを起動 -- [03_start_auth_pods.md](/docs/ja-markdown/azure_aks/03_start_auth_pods.md).
        * 04 AKS上にFIWAREのPODを起動 -- [04_start_fiware_pods.md](/docs/ja-markdown/azure_aks/04_start_fiware_pods.md).
        * 05 モニタリングとロギング -- [05_start_monitoring_and_logging.md](/docs/ja-markdown/azure_aks/05_start_monitoring_and_logging.md).
        * 06 mongodbのデータをAzure Blog Storageへバックアップ -- [06_backup_mongodb.md](/docs/ja-markdown/azure_aks/06_backup_mongodb.md).
    * minikubeを用いる場合
        * 01 minikubeの準備 -- [01_prepare_minikube.md](/docs/ja-markdown/minikube/01_prepare_minikube.md).
        * 02 minikube上にベースとなるPODを起動 -- [02_start_base_pods.md](/docs/ja-markdown/minikube/02_start_base_pods.md).
        * 03 minikube上にAPI Gateway認証用PODを起動 -- [03_start_auth_pods.md](/docs/ja-markdown/minikube/03_start_auth_pods.md).
        * 04 minikube上にFIWAREのPODを起動 -- [04_start_fiware_pods.md](/docs/ja-markdown/minikube/04_start_fiware_pods.md).
        * 05 モニタリングとロギング -- [05_start_monitoring_and_logging.md](/docs/ja-markdown/minikube/05_start_monitoring_and_logging.md).

## Related Repositories
### FIWARE components
* [telefonicaid/fiware-orion](https://github.com/telefonicaid/fiware-orion)
    * Orion is a FIWARE's reference implementation of the Publish/Subscribe Context Broker.
* [telefonicaid/iotagent-ul](https://github.com/telefonicaid/iotagent-ul)
    * IotAgent-UL is a bridge that can be used to communicate devices using the Ultralight 2.0 protocol and Orion.
        * Ultralight 2.0 is a lightweight text based protocol aimed to constrained devices and communications where the bandwidth and device memory may be limited resources.
* [telefonicaid/fiware-cygnus](https://github.com/telefonicaid/fiware-cygnus)
    * Cygnus is a connector in charge of persisting certain sources of data in certain configured third-party storages, creating a historical view of such data.
* [telefonicaid/fiware-sth-comet](https://github.com/telefonicaid/fiware-sth-comet)
    * A component of the FIWARE ecosystem in charge of managing historical and aggregated time series context information.

### Support components
* [RoboticBase/fiware-ambassador-auth](https://github.com/RoboticBase/fiware-ambassador-auth)
    * A REST API component working with [Ambassador](https://www.getambassador.io/) on Kubernetes in order to authorize and authanticate the client.
    * Bearar Authenticaton and Basic Authentication are supported.

## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2018-2019 [TIS Inc.](https://www.tis.co.jp/)
