# RoboticBase-core

This repository is a core components of "RoboticBase". The latest version (0.5) conforms to [FIWARE Release 7.8.1](https://github.com/FIWARE/catalogue/releases/tag/FIWARE_7.8.1).

## Description
"RoboticBase" is a robot management platform based on [FIWARE](http://www.fiware.org/) which enables you to manage and operate many kinds of robots and IoT devices as interactions of contexts.

"RoboticBase" allows robots to collaborate with IoT devices, Open Data, human beings and so on. You can connect a robot to "RoboticBase" using the open APIs of the robot, and operate the robot through those APIs. In turn, "RoboticBase" has an ability to manage ROS. If you connect a ROS robot to "RoboticBase", you can operate the robot directly without restrictions.  
For example, you can deploy a ROS program to the robot and access the raw data of the robot through "RoboticBase".

![roboticbase-core-architecture.png](/docs/images/roboticbase-core-architecture.png)


|FIWARE components|summary|version|
|:--|:--|:--|
|[FIWARE orion](https://fiware-orion.readthedocs.io/en/master/)|Publish/Subscribe Context Broker|2.3.0|
|[FIWARE cygnus](https://fiware-cygnus.readthedocs.io/en/latest/)|Data collection and Persistence Agent|1.17.1|
|[FIWARE sth-comet](https://fiware-sth-comet.readthedocs.io/en/latest/)|An Agent to manage historical raw and aggregated time series context|2.7.0|
|[FIWARE iotagent-ul](https://fiware-iotagent-ul.readthedocs.io/en/latest/)|Backend Device Management Agent|1.12.0|
|[FIWARE iotagent-json](https://fiware-iotagent-json.readthedocs.io/en/latest/)|Backend Device Management Agent|1.13.0|

|Other components|summary|version|
|:--|:--|:--|
|[kubernetes](https://kubernetes.io/)|Container Orchestration Platform|1.14 or higher|
|[ambassador](https://www.getambassador.io/)|API Gateway|1.1.1|
|[auth](https://github.com/RoboticBase/fiware-ambassador-auth)|Authorization and Authentication component working with ambassador|0.3.0|
|[RabbitMQ](https://www.rabbitmq.com/)|Distributed Message Queue|3.8.2|
|[MongoDB](https://www.mongodb.com/)|Document-oriented NoSQL Database|4.2.3|
|[Prometheus](https://prometheus.io/)|Monitoring and Alerting toolkit|2.16.0|
|[Grafana](https://grafana.com/)|Analytics and Alerting platform for time series metrics|6.6.1|
|[Elasticsearch](https://www.elastic.co/products/elasticsearch)|Distributed search and analytics engine|7.6.0|
|[fluentd](https://www.fluentd.org/)|Data collector for unified logging layer|2.9.0|
|[Kibana](https://www.elastic.co/products/kibana)|Visualize the Elasticsearch data|7.6.0|

## An experiment to prove our concept
We and University of Aizu have been performed an experiment to guide a visitor by collaborating with heterogeneous robots, IoT devices and people through this Robot Platform.

### PoC1 on Nov. 6th - 8th , 2018.
Please see this repository [ogcaizu/ogc-poc1](https://github.com/ogcaizu/ogc-poc1).

[![video](http://img.youtube.com/vi/D9NPxxYgPa0/0.jpg)](https://youtu.be/D9NPxxYgPa0)

### PoC2 on Nov. 26th - 29th , 2019.
Please see this repository [RoboticBase/uoa-poc2](https://github.com/RoboticBase/uoa-poc2).

[![short version](https://img.youtube.com/vi/R09vPSEbg1g/0.jpg)](https://www.youtube.com/watch?v=R09vPSEbg1g)
[![long version](https://img.youtube.com/vi/pR10cp93KX4/0.jpg)](https://www.youtube.com/watch?v=pR10cp93KX4)

## Requirements

* kubernetes client PC
    * [minikube](https://github.com/kubernetes/minikube) and [Oracle VM VirtualBox](https://www.virtualbox.org/) is required when you use minikube.
    * [azure cli](https://github.com/Azure/azure-cli) is required when you use [Azure AKS](https://azure.microsoft.com/en-us/services/kubernetes-service/).
    * [kubectl](https://kubernetes.io/docs/reference/kubectl/overview/) and [helm](https://helm.sh/) is required to handle minikube and Azure AKS.
    * [pipenv](https://pipenv.pypa.io/en/latest/) is required to setup ansible and other libraries.
    * [openssl](https://www.openssl.org/) is required to generate tls cert files.

||version|
|:--|:--|
|OS|macOS Mojave 10.14.6<br/>Ubuntu 16.04|
|pyenv|1.2.16|
|pipenv|2018.11.26|
|kubectl|1.17.3|
|helm|3.1.1|

* for Azure AKS

||version|
|:--|:--|
|openssl|2.6.5|
|azure cli|2.1.0|when you use Azure AKS|

* for minikube
    * when you use monitoring & logging, you have to give **4 cpu & 8192 MB memories** to minikube.

||version|
|:--|:--|
|VirtualBox|6.1.2 r135662|
|minikube|1.7.3|

## getting started
### install ansible by using pipenv
1. install ansible and other libraries by using pipenv

    ```
    $ cd ansible
    $ pipenv install
    ```
1. start pipenv shell

    ```
    $ pipenv shell
    ```

### start RoboticBase/core on minikube
1. change the password of `iotagent` mqtt user defined in the following yml file:
    * [group\_vars/all.yml](ansible/group_vars/all.yml)
1. change the values defined in the following yml files if necessary:
    * [inventories/minikube/group\_vars/minikube.yml](ansible/inventories/minikube/group_vars/minikube.yml)
    * [inventories/minikube/host\_vars/localhost.yml](ansible/inventories/minikube/host_vars/localhost.yml)
1. execute an ansible playbook by using following command:

    ```
    $ ansible-playbook -i inventories/minikube --extra-vars="ansible_python_interpreter=$(which python)" minikube.yml
    ```

#### start RoboticBase/core on Azure AKS
1. change the password of `iotagent` mqtt user defined in the following yml file:
    * `group_vars/all.yml`
1. change the `dns.domain` and `dns.email` to your own domain and email in the following yml file:
    * `inventories/aks/group_vars/aks.yml`
1. change the values defined in the following yml files if necessary:
    * `group_vars/all.yml`
    * `inventories/aks/host_vars/azure.yml`
    * `inventories/aks/group_vars/aks.yml`
1. execute a shell script to create azure credentials

    ```
    $ ./generate_azure_credentials.sh
    ```
1. execute an ansible playbook by using following command:

    ```
    $ ansible-playbook -i inventories/aks --extra-vars="ansible_python_interpreter=$(which python)" aks.yml
    ```
## Related Repositories
### FIWARE components
* [telefonicaid/fiware-orion](https://github.com/telefonicaid/fiware-orion)
    * Orion is a FIWARE's reference implementation of the Publish/Subscribe Context Broker.
* [telefonicaid/iotagent-ul](https://github.com/telefonicaid/iotagent-ul)
    * IotAgent-UL is a bridge that can be used to communicate devices using the Ultralight 2.0 protocol and Orion.
        * Ultralight 2.0 is a lightweight text based protocol aimed to constrained devices and communications where the bandwidth and device memory may be limited resources.
* [telefonicaid/iotagent-json](https://github.com/telefonicaid/iotagent-json)
    * IotAgent-JSON is a bridge that can be used to communicate devices using the JSON and Orion.
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
Copyright (c) 2018-2020 [TIS Inc.](https://www.tis.co.jp/)
