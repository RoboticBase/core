# 1. prepare Microsoft Azure AKS

Prepare [Microsoft Azure AKS](https://azure.microsoft.com/en-us/services/container-service/) by following steps:

1. [create DNS zone](#create-dns-zone-of-examplecom)
1. [start private registry](#start-private-registry-on-azure-container-registry)
1. [start kubernetes](#start-kubernetes-on-azure-aks)
1. [install helm](#install-helm)

**In the following document, replace "example.com" with your domain and replace "tenant.onmicrosoft.com" with your tentant.**

## create DNS zone of "example.com"

```bash
mac:$ az group create --name dns-zone --location japaneast
```

```bash
mac:$ az network dns zone create --resource-group dns-zone --name "example.com"
```

```bash
mac:$ az network dns zone show --resource-group dns-zone --name "example.com" | jq ".nameServers"
[
  "ns1-XX.azure-dns.com.",
  "ns2-XX.azure-dns.net.",
  "ns3-XX.azure-dns.org.",
  "ns4-XX.azure-dns.info."
]
```

## start private registry on Azure Container Registry

```bash
mac:$ az login --tenant tenant.onmicrosoft.com
```

```bash
mac:$ az group create --name fiware-demo --location centralus
```

```bash
mac:$ az acr create --resource-group fiware-demo --name fiwareacr --sku Basic
mac:$ export REPOSITORY=$(az acr show --resource-group fiware-demo --name fiwareacr | jq '.loginServer' -r); echo ${REPOSITORY}
```

```bash
mac:$ az acr login --name fiwareacr
```

## start kubernetes on Azure AKS

```bash
mac:$ az provider register -n Microsoft.Compute
mac:$ az provider register -n Microsoft.Storage
mac:$ az provider register -n Microsoft.Network
mac:$ az provider register -n Microsoft.ContainerService
```

```bash
mac:$ az provider show -n Microsoft.Compute | jq '.registrationState' -r
Registered
mac:$ az provider show -n Microsoft.Storage | jq '.registrationState' -r
Registered
mac:$ az provider show -n Microsoft.Network | jq '.registrationState' -r
Registered
mac:$ az provider show -n Microsoft.ContainerService | jq '.registrationState' -r
Registered
```

```bash
mac:$ az aks create --resource-group fiware-demo --name fiwareaks --node-count 4 --ssh-key-value $HOME/.ssh/azure.pub
```

```bash
mac:$ az aks get-credentials --resource-group fiware-demo --name fiwareaks
```

```bash
mac:$ kubectl get nodes
NAME                       STATUS    ROLES     AGE       VERSION
aks-nodepool1-27506152-0   Ready     agent     2m        v1.9.6
aks-nodepool1-27506152-1   Ready     agent     2m        v1.9.6
aks-nodepool1-27506152-2   Ready     agent     2m        v1.9.6
aks-nodepool1-27506152-3   Ready     agent     2m        v1.9.6
```

```bash
mac:$ CLIENT_ID=$(az aks show --resource-group fiware-demo --name fiwareaks --query "servicePrincipalProfile.clientId" --output tsv);echo ${CLIENT_ID}
mac:$ ACR_ID=$(az acr show --name fiwareacr --resource-group fiware-demo --query "id" --output tsv); echo ${ACR_ID}
mac:$ az role assignment create --assignee ${CLIENT_ID} --role Reader --scope ${ACR_ID}
```

## install helm
```bash
mac:$ curl --output ~/Downloads/helm-v2.8.2-darwin-amd64.tar.gz https://storage.googleapis.com/kubernetes-helm/helm-v2.8.2-darwin-amd64.tar.gz
mac:$ tar xfz ~/Downloads/helm-v2.8.2-darwin-amd64.tar.gz -C /tmp
mac:$ sudo mv /tmp/darwin-amd64/helm /usr/local/bin
```

```bash
mac:$ helm update
mac:$ helm init
```

```bash
mac:$ kubectl get pods --all-namespaces | grep tiller
kube-system   tiller-deploy-865dd6c794-cdzd9          1/1       Running   0          6m
```

```bash
mac:$ helm version
Client: &version.Version{SemVer:"v2.8.2", GitCommit:"a80231648a1473929271764b920a8e346f6de844", GitTreeState:"clean"}
Server: &version.Version{SemVer:"v2.8.2", GitCommit:"a80231648a1473929271764b920a8e346f6de844", GitTreeState:"clean"}
```
