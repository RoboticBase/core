# container-centric fiware demonstaration

This repository construct a container-centric [FIWARE](http://www.fiware.org/) demonstration on [Kubernetes](https://kubernetes.io/).

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

### Minikube

1. [prepare Microsoft Azure AKS](/docs/minikube/1_prepare_minikube.md)
1. [start containers on Kubernetes](/docs/minikube/2_start_containers.md)
1. [configure fiware](/docs/minikube/3_configure_fiware.md)
1. [operate 'turtlesim'](/docs/minikube/4_operate_turtlesim.md)
1. [operate 'gopigo'](/docs/minikube/5_operate_gopigo.md) (if gopigo is available)

## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2018 [TIS Inc.](https://www.tis.co.jp/)
