# RoboticBase Coreインストールマニュアル #1

## 構築環境(2019年4月26日現在)
### macOS
- macOS Sierra 10.12.6
- azure-cli 2.0.63
- git 2.14.3
- kubectl 1.14.1
- helm v2.13.1
 

### Ubuntu
- Ubuntu 16.04.5 LTS
- apt-transport-https 1.2.29ubuntu0.1
- lsb-release 9.20160110ubuntu0.2
- software-properties-common 0.96.20.8
- dirmngr 2.1.11-6ubuntu2.1
- azure-cli 2.0.63
- git 2.7.4-0ubuntu1.6
- kubectl 1.14.1
- helm v2.13.1

# Azure Kubernetes Service(AKS)の準備

## Azure CLIのインストール
### macOS
1. Azure CLIのインストール

    ```
    $ brew update && brew install azure-cli
    ```

1. Azure CLIのバージョンを確認

    ```
    $ az --version
    ```

    - 実行結果（例）

        ```
        azure-cli                         2.0.63
        ```

### Ubuntu
1. 前提条件のパッケージをインストール

    ```
    $ sudo apt-get install -y apt-transport-https lsb-release software-properties-common dirmngr
    ```

1. Azure CLI用のリポジトリを追加

    ```
    $ AZ_REPO=$(lsb_release -cs)
    $ echo "deb [arch=amd64] https://packages.microsoft.com/repos/azure-cli/ $AZ_REPO main" | sudo tee /etc/apt/sources.list.d/azure-cli.list
    ```

1. Azure CLIリポジトリの公開鍵を登録

    ```
    $ sudo apt-key --keyring /etc/apt/trusted.gpg.d/Microsoft.gpg adv --keyserver packages.microsoft.com --recv-keys xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    ```

1. パッケージリスト更新

    ```
    $ sudo apt-get update -y
    ```

1. Azure CLIのインストール

    ```
    $ sudo apt-get install azure-cli
    ```

1. Azure CLIのインストール確認

    ```
    $ dpkg -l | grep azure-cli
    ```

1. Azure CLIのバージョンを確認

    ```
    $ az --version
    ```

    - 実行結果（例）

        ```
        azure-cli                         2.0.63
        ```


## gitのインストール
### macOS
1. gitのインストール

    ```
    $ brew update && brew install git
    ```

### Ubuntu
1. gitのインストール

    ```
    $ sudo apt-get install -y git
    ```

1. gitのインストール確認

    ```
    $ dpkg -l | grep git
    ```

    - 実行結果（例）

        ```
        ii  git                                        1:2.7.4-0ubuntu1.6                                  amd64        fast, scalable, distributed revision control system
        ```

## jqのインストール
### macOS
1. jqのインストール

    ```
    $ brew update && brew install jq
    ```

### Ubuntu
1. jqのインストール

    ```
    $ sudo apt-get install -y jq
    ```

## kubectlのインストール
### macOS
1. kubectlのインストール

    ```
    $ brew update && brew install kubernetes-cli
    ```

### Ubuntu
1. kubernetesリポジトリの公開鍵登録

    ```
    $ curl -s https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
    ```

1. kubernetesのリポジトリ登録

    ```
    $ sudo apt-add-repository "deb http://apt.kubernetes.io/ kubernetes-xenial main"
    ```

1. パッケージリストの更新

    ```
    $ sudo apt-get update -y
    ```

1. kubectlのインストール

    ```
    $ sudo apt-get install -y kubectl
    ```

1. kubectlのインストール確認

    ```
    $ dpkg -s kubectl
    ```

    - 実行結果（例）

        ```
        Package: kubectl
        Status: install ok installed
        Priority: optional
        Section: misc
        Installed-Size: 38349
        Maintainer: Kubernetes Authors <kubernetes-dev+release@googlegroups.com>
        Architecture: amd64
        Version: 1.13.3-00
        Description: Kubernetes Command Line Tool
        The Kubernetes command line tool for interacting with the Kubernetes API.
        Homepage: https://kubernetes.io
        ```

## Helmのインストール
### macOS
1. Helmのインストール

    ```
    $ brew update && brew install kubernetes-helm
    ```

### Ubuntu
1. Helmのインストール

    ```
    $ cd /tmp
    $ curl -LO https://storage.googleapis.com/kubernetes-helm/helm-v2.5.0-linux-amd64.tar.gz
    $ sudo tar xvf helm-v2.5.0-linux-amd64.tar.gz
    $ sudo cp linux-amd64/helm /usr/bin
    $ sudo rm -rf linux-amd64
    $ rm helm-v2.5.0-linux-amd64.tar.gz
    ```


1. Helmのバージョン確認

    ```
    $ helm version --client
    ```

    - 実行結果（例）

        ```
        Client: &version.Version{SemVer:"v2.5.0", GitCommit:"012cb0ac1a1b2f888144ef5a67b8dab6c2d45be6", GitTreeState:"clean"}
        ```

## RoboticBase/coreの取得
1. ベースファイルの取得

    ```
    $ cd ${HOME}
    $ git clone https://github.com/RoboticBase/core
    ```

## 環境変数の設定
1. 環境変数の設定

    ```
    $ export CORE_ROOT=$HOME/core
    $ cd $CORE_ROOT;pwd
    ```

    - 実行結果（例）

        ```
        /home/fiware/core
        ```

1. 環境ファイルのコピー

    ```
    $ cd $CORE_ROOT/docs/environments/azure_aks
    $ cp env.template env
    ```

1. 環境ファイルの設定

    ```bash
    #!/bin/bash

    export TENANT="example.onmicrosoft.com"; echo "TENANT=${TENANT}"
    export DOMAIN="example.com"; echo "DOMAIN=${DOMAIN}"
    export EMAIL="nobody@example.com"; echo "EMAIL=${EMAIL}"
    export SSH_KEY="$HOME/.ssh/azure.pub"; echo "SSH_KEY=${SSH_KEY}"
    export NODE_COUNT=4; echo "NODE_COUNT=${NODE_COUNT}"
    export MQTT__iotagent="password_of_iotagent";echo "MQTT__iotagent=${MQTT__iotagent}"

    export REGION="japaneast"; echo "REGION=${REGION}"
    export DNS_ZONE_RG="dns-zone"; echo "DNS_ZONE_RG=${DNS_ZONE_RG}"
    export AKS_RG="rbcore"; echo "AKS_RG=${AKS_RG}"
    export ACR_NAME="rbcacr"; echo "ACR_NAME=${ACR_NAME}"
    export AKS_NAME="rbcaks"; echo "AKS_NAME=${AKS_NAME}"

    export REPOSITORY="<<REPOSITORY>>"; echo "REPOSITORY=${REPOSITORY}"
    ```

    ※利用環境によって`TENANT`や`DOMAIN`、`EMAIL`などを変更してください  
    　また`DNS_ZONE`や`AKS_RG`などには、今後作成するリソースの名前を記載してください

1. プロジェクトルートに移動

    ```
    $ cd $CORE_ROOT
    ```

1. 環境設定の読み込み

    ```
    $ source $CORE_ROOT/docs/environments/azure_aks/env
    ```


## AKSのログイン

1. テナントIDを指定してのAKSログイン

    ```
    $ az login --tenant ${TENANT}
    ```

    ※コマンド実施後、設定端末のブラウザにて発行されたURLに移動し、コードを入力してください

    ```
    To sign in, use a web browser to open the page https://microsoft.com/devicelogin and enter the code GUYE26E84 to authenticate.
    ```

    ※その後、対象のアカウントを選択、一定時間経過後、ターミナル側に下記のID一覧が表示されます

    ```json
    [
      {
        "cloudName": "AzureCloud",
        "id": "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx",
        "isDefault": true,
        "name": "Microsoft Azure",
        "state": "Enabled",
        "tenantId": "example.onmicrosoft.com",
        "user": {
          "name": "nobody@example.com",
          "type": "user"
        }
      }
    ]
    ```

1. ログインの確認

    ```
    $ az account show
    ```

    - 実行結果（例）

        ```json
        {
            "environmentName": "AzureCloud",
            "id": "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx",
            "isDefault": true,
            "name": "Microsoft Azure",
            "state": "Enabled",
            "tenantId": "example.onmicrosoft.com",
            "user": {
                "name": "nobody@example.com",
                "type": "user"
            }
        }
        ```


## DNSゾーンの作成
1. DNS用のリソースグループ作成

    ```
    $ az group create --name ${DNS_ZONE_RG} --location ${REGION}
    ```

    - 実行結果（例）

        ```json
        {
            "id": "/subscriptions/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx/resourceGroups/dns-zone",
            "location": "japaneast",
            "managedBy": null,
            "name": "dns-zone",
            "properties": {
                "provisioningState": "Succeeded"
            },
            "tags": null,
            "type": null
        }
        ```

1. DNSゾーンの作成

    ```
    $ az network dns zone create --resource-group ${DNS_ZONE_RG} --name "${DOMAIN}"
    ```

    - 実行結果（例）

        ```json
        {
            "etag": "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx",
            "id": "/subscriptions/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx/resourceGroups/dns-zone/providers/Microsoft.Network/dnszones/example.com",
            "location": "global",
            "maxNumberOfRecordSets": 5000,
            "name": "example.com",
            "nameServers": [
                "ns1-xx.azure-dns.com.",
                "ns2-xx.azure-dns.net.",
                "ns3-xx.azure-dns.org.",
                "ns4-xx.azure-dns.info."
            ],
            "numberOfRecordSets": 2,
            "registrationVirtualNetworks": null,
            "resolutionVirtualNetworks": null,
            "resourceGroup": "dns-zone",
            "tags": {},
            "type": "Microsoft.Network/dnszones",
            "zoneType": "Public"
        }
        ```

1. DNSゾーンの作成確認

    ```
    $ az network dns zone show --resource-group ${DNS_ZONE_RG} --name "${DOMAIN}" | jq ".nameServers"
    ```

    - 実行結果（例）

        ```json
        [
            "ns1-xx.azure-dns.com.",
            "ns2-xx.azure-dns.net.",
            "ns3-xx.azure-dns.org.",
            "ns4-xx.azure-dns.info."
        ]
        ```


## AKS用のリソースグループ作成

1. AKS用のリソースグループ作成

    ```
    $ az group create --name ${AKS_RG} --location ${REGION}
    ```

    - 実行結果（例）

        ```json
        {
            "id": "/subscriptions/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx/rbcore",
            "location": "japaneast",
            "managedBy": null,
            "name": "rbcore",
            "properties": {
                "provisioningState": "Succeeded"
            },
            "tags": null,
            "type": null
        }
        ```

## プライベートコンテナレジストリとしてAzure Container Registryを起動

1. Azure Container Registryの作成

    ```
    $ az acr create --resource-group ${AKS_RG} --name ${ACR_NAME} --sku Basic
    ```

    - 実行結果（例）

        ```json
        {
            "adminUserEnabled": false,
            "creationDate": "2019-02-14T01:07:34.738571+00:00",
            "id": "/subscriptions/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx/resourceGroups/rbcore/providers/Microsoft.ContainerRegistry/registries/rbcacr",
            "location": "japaneast",
            "loginServer": "rbcacr.azurecr.io",
            "name": "rbcacr",
            "networkRuleSet": null,
            "provisioningState": "Succeeded",
            "resourceGroup": "rbcore",
            "sku": {
                "name": "Basic",
                "tier": "Basic"
            },
            "status": null,
            "storageAccount": null,
            "tags": {},
            "type": "Microsoft.ContainerRegistry/registries"
        }
        ```

1. REPOSITORYの環境変数設定

    ```
    $ export REPOSITORY=$(az acr show --resource-group ${AKS_RG} --name ${ACR_NAME} | jq '.loginServer' -r)
    ```

1. 環境ファイルのREPOSITORY書き換え
  * macOS

    ```
    $ sed -i '' -e "s/<<REPOSITORY>>/${REPOSITORY}/" ${CORE_ROOT}/docs/environments/azure_aks/env
    ```
  * Ubuntu

    ```
    $ sed -i -e "s/<<REPOSITORY>>/${REPOSITORY}/" ${CORE_ROOT}/docs/environments/azure_aks/env
    ```

1. REPOSITORYの環境変数確認

    ```
    $ echo ${REPOSITORY}
    ```

    - 実行結果（例）

        ```
        rbcacr.azurecr.io
        ```


## AKS作成時に利用する公開鍵の作成

1. Azure認証用のRSA鍵ペアを作成

    ```
    $ ssh-keygen -t rsa -f ~/.ssh/azure
    ```

    - 実行結果（例）

        ```
        Generating public/private rsa key pair.
        Enter passphrase (empty for no passphrase): 
        Enter same passphrase again: 
        Your identification has been saved in /home/fiware/.ssh/azure.
        Your public key has been saved in /home/fiware/.ssh/azure.pub.
        The key fingerprint is:
        SHA256:xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx fiware@FIWARE-PC
        The key's randomart image is:
        +---[RSA 2048]----+
        |o++++.  .        |
        |.o+ ...o .       |
        | . + +.o+        |
        |    =.o...       |
        |  ....* S .      |
        |o.o= O o +       |
        |o=o E = . .      |
        |o .* + +         |
        | .  = . oo.      |
        +----[SHA256]-----+
        ```

1. Azure認証用のRSA鍵ペアの作成確認

    ```
    $ ls -la /home/fiware/.ssh/
    ```

    - 実行結果（例）

        ```
        合計 20
        drwx------  2 fiware fiware 4096  2月 18 17:15 .
        drwxr-xr-x 24 fiware fiware 4096  2月 18 16:26 ..
        -rw-------  1 fiware fiware 1679  2月 18 17:15 azure
        -rw-r--r--  1 fiware fiware  398  2月 18 17:15 azure.pub
        -rw-r--r--  1 fiware fiware  222  2月 13 13:31 known_hosts
        ```


## AKSの起動準備

1. プロバイダの登録

    ```
    $ az provider register -n Microsoft.Compute
    $ az provider register -n Microsoft.Storage
    $ az provider register -n Microsoft.Network
    $ az provider register -n Microsoft.ContainerService
    ```

1. プロバイダの登録確認

    ```
    $ az provider show -n Microsoft.Compute | jq '.registrationState' -r
    $ az provider show -n Microsoft.Storage | jq '.registrationState' -r
    $ az provider show -n Microsoft.Network | jq '.registrationState' -r
    $ az provider show -n Microsoft.ContainerService | jq '.registrationState' -r
    ```

    - 実行結果（例）

        ```
        Registered
        ```

## AKSの起動

1. AKSで利用可能なkubernetesのバージョン確認

    ```
    $ az aks get-versions --location ${REGION} --output table
    ```

    - 実行結果（例）

        ```
        KubernetesVersion    Upgrades
        \-------------------  -----------------------
        1.13.5               None available
        1.12.7               1.13.5
        1.12.6               1.12.7, 1.13.5
        1.11.9               1.12.6, 1.12.7
        1.11.8               1.11.9, 1.12.6, 1.12.7
        1.10.13              1.11.8, 1.11.9
        1.10.12              1.10.13, 1.11.8, 1.11.9
        1.9.11               1.10.12, 1.10.13
        1.9.10               1.9.11, 1.10.12, 1.10.13
        ```


1. NODEの仮想環境を `Standard_D2s_v3 `に指定

   ```
    $ export NODE_VM_SIZE="Standard_D2s_v3"
    ```

1. NODEのOS容量を64GBに指定

    ```
    $ export NODE_OSDISK_SIZE_GB=64
    ```

1. AKSのバージョンを `1.13.5` に指定

    ```
    $ export AKS_VERSION="1.13.5"
    ```

1. AKSの起動

    ```
    $ az aks create --resource-group ${AKS_RG} --name ${AKS_NAME} --node-count ${NODE_COUNT} --node-vm-size ${NODE_VM_SIZE} --node-osdisk-size ${NODE_OSDISK_SIZE_GB} --ssh-key-value ${SSH_KEY} --kubernetes-version ${AKS_VERSION}
    ```

    - 実行結果（例）

      ```json
      {
        "aadProfile": null,
        "addonProfiles": null,
        "agentPoolProfiles": [
          {
            "count": 4,
            "maxPods": 110,
            "name": "nodepool1",
            "osDiskSizeGb": 64,
            "osType": "Linux",
            "storageProfile": "ManagedDisks",
            "vmSize": "Standard_D2s_v3",
            "vnetSubnetId": null
          }
        ],
        "dnsPrefix": "rbcaks-rbcore-38ac45",
        "enableRbac": true,
        "fqdn": "rbcaks-rbcore-38ac45-ed59dae0.hcp.japaneast.azmk8s.io",
        "id": "/subscriptions/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx/resourcegroups/rbcore/providers/Microsoft.ContainerService/managedClusters/rbcaks",
        "kubernetesVersion": "1.13.5",
        "linuxProfile": {
          "adminUsername": "azureuser",
          "ssh": {
            "publicKeys": [
              {
                "keyData": "ssh-rsa xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx fiware@FIWARE-PC\n"
              }
            ]
          }
        },
        "location": "japaneast",
        "name": "rbcaks",
        "networkProfile": {
          "dnsServiceIp": "10.0.0.10",
          "dockerBridgeCidr": "172.17.0.1/16",
          "networkPlugin": "kubenet",
          "networkPolicy": null,
          "podCidr": "10.244.0.0/16",
          "serviceCidr": "10.0.0.0/16"
        },
        "nodeResourceGroup": "MC_rbcore_rbcaks_japaneast",
        "provisioningState": "Succeeded",
        "resourceGroup": "rbcore",
        "servicePrincipalProfile": {
          "clientId": "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx",
          "secret": null
        },
        "tags": null,
        "type": "Microsoft.ContainerService/ManagedClusters"
      }
      ```


## AKSの資格情報取得

1. Kubernetesクラスタのアクセス認証情報取得

    ```
    $ az aks get-credentials --resource-group ${AKS_RG} --name ${AKS_NAME} --overwrite-existing
    ```

    - 実行結果（例）

        ```
        Merged "rbcaks" as current context in /home/fiware/.kube/config
        ```

1. クライアントのID取得

    ```
    $ CLIENT_ID=$(az aks show --resource-group ${AKS_RG} --name ${AKS_NAME} --query "servicePrincipalProfile.clientId" --output tsv)
    ```

1. ACRのID取得

    ```
    $ ACR_ID=$(az acr show --resource-group ${AKS_RG} --name ${ACR_NAME} --query "id" --output tsv)
    $ az role assignment create --assignee ${CLIENT_ID} --role Reader --scope ${ACR_ID}
    ```

    - 実行結果（例）

        ```json
        {
            "canDelegate": null,
            "id": "/subscriptions/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx/resourceGroups/rbcore/providers/Microsoft.ContainerRegistry/registries/rbcacr/providers/Microsoft.Authorization/roleAssignments/0ee39ef3-e228-4af1-b49d-fb856e01a8d3",
            "name": "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx",
            "principalId": "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx",
            "resourceGroup": "rbcore",
            "roleDefinitionId": "/subscriptions/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx/providers/Microsoft.Authorization/roleDefinitions/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx",
            "scope": "/subscriptions/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx/resourceGroups/rbcore/providers/Microsoft.ContainerRegistry/registries/rbcacr",
            "type": "Microsoft.Authorization/roleAssignments"
        }
        ```


## ノードの確認

1. ノードの確認

    ```
    $ kubectl get nodes
    ```

    - 実行結果（例）

        ```
        NAME                       STATUS   ROLES   AGE   VERSION
        aks-nodepool1-35549331-0   Ready    agent   20h   v1.13.5
        aks-nodepool1-35549331-1   Ready    agent   20h   v1.13.5
        aks-nodepool1-35549331-2   Ready    agent   20h   v1.13.5
        aks-nodepool1-35549331-3   Ready    agent   20h   v1.13.5
        ```


## Role-based access control(RBAC)の設定

1. RBAC用のリソース定義確認

    ```
    $ cd $CORE_ROOT/rbac
    $ ls -la 
    ```

    - 実行結果（例）

        ```
        合計 20
        drwxr-xr-x  2 root root 4096  2月 13 14:52 .
        drwxr-xr-x 17 root root 4096  2月 13 14:52 ..
        -rw-r--r--  1 root root  466  2月 13 14:52 dashboard-rbac.yaml
        -rw-r--r--  1 root root  501  2月 13 14:52 default-rbac.yaml
        -rw-r--r--  1 root root  354  2月 13 14:52 tiller-rbac.yaml
        ```

1. RBACによるアクセス制限追加

    ```
    $ kubectl apply -f dashboard-rbac.yaml
    ```

    - 実行結果（例）

        ```
        clusterrolebinding.rbac.authorization.k8s.io/rook-operator created
        ```

1. kubernetes-dashboardのserviceaccounts確認

    ```
    $ kubectl get serviceaccounts -n kube-system | grep kubernetes-dashboard
    ```

    - 実行結果（例）

        ```
        kubernetes-dashboard                 1         61m
        ```

1. tiller-rbacの作成

    ```
    $ kubectl apply -f tiller-rbac.yaml
    ```

    - 実行結果（例）

        ```
        clusterrolebinding.rbac.authorization.k8s.io/tiller created
        ```

1. tillerのserviceaccounts確認

    ```
    $ kubectl get serviceaccounts -n kube-system | grep tiller
    ```

    - 実行結果（例）

        ```
        tiller                               1         79s
        ```

1. default-rbacの作成

    ```
    $ kubectl apply -f default-rbac.yaml
    ```

    - 実行結果（例）

        ```
        clusterrole.rbac.authorization.k8s.io/default-read created
        clusterrolebinding.rbac.authorization.k8s.io/default created
        ```

1. default-readのclusterroles確認

    ```
    $ kubectl get clusterroles | grep default-read
    ```

    - 実行結果（例）

        ```
        default-read                                                           2m20s
        ```

1. defaultのclusterrolebindings確認

    ```
    $ kubectl get clusterrolebindings | grep default
    ```

    - 実行結果（例）

        ```
        default                                                13m
        ```

## Helmの初期化
1. Helmのサービスアカウント初期化

    ```
    $ helm init --service-account tiller
    ```

    - 実行結果（例）

        ```
        $HELM_HOME has been configured at /home/fiware/.helm.
        Warning: Tiller is already installed in the cluster.
        (Use --client-only to suppress this message, or --upgrade to upgrade Tiller to the current version.)
        Happy Helming!
        ```

1. Helmのリポジトリアップデート

    ```
    $ helm repo update
    ```

    - 実行結果（例）

        ```
        Hang tight while we grab the latest from your chart repositories...
        ...Skip local chart repository
        ...Successfully got an update from the "stable" chart repository
        Update Complete. ? Happy Helming!?
        ```

1. Tillerのpods確認

    ```
    $ kubectl get pods --all-namespaces | grep tiller
    ```

    - 実行結果（例）

        ```
        kube-system   tiller-deploy-67f9f6f798-vjkxq          1/1     Running   0          13m
        ```

1. Helmのバージョン確認

    ```
    $ helm version
    ```

    - 実行結果（例）

        ```
        Client: &version.Version{SemVer:"v2.13.1", GitCommit:"618447cbf203d147601b4b9bd7f8c37a5d39fbb4", GitTreeState:"clean"}
        Server: &version.Version{SemVer:"v2.13.1", GitCommit:"618447cbf203d147601b4b9bd7f8c37a5d39fbb4", GitTreeState:"clean"}
        ```
