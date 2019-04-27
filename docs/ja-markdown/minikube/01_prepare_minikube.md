# RoboticBase Coreインストールマニュアル #1

## 構築環境(2019年4月26日現在)
### macOS
- macOS Sierra 10.12.6
- virtualbox 5.2.28r130011
- docker-ce 18.09.2
- kubectl 1.14.1
- helm v2.13.1
- minikube 1.0.0
- mosquitto-clients 1.6.0

### Ubuntu
- Ubuntu 16.04.6 LTS
- virtualbox 5.2.28r130011
- docker-ce 18.09.5~3-0~ubuntu-xenial
- kubectl 1.14.1
- helm v2.13.1
- minikube 1.0.0
- mosquitto-clients 1.5.7-0mosquitto1~xenial1


## VirtualBoxのインストール
### macOS
1. VirtualBoxのインストール

    ```
    $ curl -Lo ~/Downloads/VirtualBox-5.2.28-130011-OSX.dmg https://download.virtualbox.org/virtualbox/5.2.28/VirtualBox-5.2.28-130011-OSX.dmg
    $ open ~/Downloads/VirtualBox-5.2.28-130011-OSX.dmg
    $ open /Applications/VirtualBox.app
    ```

1. VirtualBoxのバージョン確認

    ```
    $ vboxmanage -v
    ```

    - 実行結果（例）
        
        ```
        5.2.28r130011
        ```


### Ubuntu
1. virtualboxのリポジトリの公開鍵登録

    ```
    $ wget -q https://www.virtualbox.org/download/oracle_vbox_2016.asc -O- | sudo apt-key add -
    $ wget -q https://www.virtualbox.org/download/oracle_vbox.asc -O- | sudo apt-key add -
    ```

1. virtualboxのリポジトリ登録

    ```
    $ sudo add-apt-repository "deb http://download.virtualbox.org/virtualbox/debian xenial contrib"
    ```

1. パッケージリストの更新

    ```
    $ sudo apt-get update
    ```

1. VirtualBoxのインストール

    ```
    $ sudo apt-get install -y virtualbox-5.2
    ```

1. VirtualBoxのバージョン確認

    ```
    $ vboxmanage -v
    ```

    - 実行結果（例）
        
        ```
        5.2.28r130011
        ```

## docker-ceのインストール
### macOS
1. Docker CE for Macを公式サイトからダウンロード

    ```
    $ wget https://download.docker.com/mac/stable/Docker.dmg
    ```

1. `Docker.dmg` をダブルクリックし、 `Docker.app` をApplicationsフォルダへ移動
1. `Docker.app` をダブルクリックしてDockerを起動

### Ubuntu
1. 前提ファイルのインストール

    ```
    $ sudo apt-get install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
    ```

1. docker-ceリポジトリの公開鍵を登録

    ```
    $ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
    ```

1. docker-ceリポジトリを登録

    ```
    $ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
    ```

1. パッケージリストの更新

    ```
    $ sudo apt-get update
    ```

1. docker-ceのインストール

    ```
    $ sudo apt-get install -y docker-ce docker-ce-cli containerd.io
    ```

1. docker-ceのインストール確認

    ```
    $ dpkg -l | grep docker
    ```

    - 実行結果（例）

        ```
        ii  docker-ce                                  5:18.09.5~3-0~ubuntu-xenial                  amd64        Docker: the open-source application container engine
        ii  docker-ce-cli                              5:18.09.5~3-0~ubuntu-xenial                  amd64        Docker CLI: the open-source application container engine
        ```

1. dockerコマンドの実行権限を付与

    ```
    $ sudo gpasswd -a $USER docker
    ```

1. dockerの再起動

    ```
    $ sudo systemctl restart docker
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

## envsubstのインストール
### macOS
1. envsubstのインストール

    ```
    $ brew update && brew install gettext
    $ brew link --force gettext
    ```

### Ubuntu
1. envsubstのインストール

    ```
    $ sudo apt-get install -y gettext-base
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
    $ sudo apt-get update
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
        Installed-Size: 42122
        Maintainer: Kubernetes Authors <kubernetes-dev+release@googlegroups.com>
        Architecture: amd64
        Version: 1.14.1-00
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
    $ curl -LO https://storage.googleapis.com/kubernetes-helm/helm-v2.13.1-linux-amd64.tar.gz
    $ sudo tar xvf helm-v2.13.1-linux-amd64.tar.gz
    $ sudo cp linux-amd64/helm /usr/bin
    $ sudo rm -rf linux-amd64
    $ rm helm-v2.13.1-linux-amd64.tar.gz
    $ cd ${HOME}
    ```

## minikubeのインストール
### macOS
1. minikubeのインストール

    ```
    $ brew cask install minikube
    ```

1. minikubeのバージョン確認

    ```
    $ minikube version
    ```

    - 実行結果（例）

        ```
        minikube version: v1.0.0
        ```

### Ubuntu
1. minikubeのインストール

    ```
    $ curl -Lo minikube https://storage.googleapis.com/minikube/releases/v1.0.0/minikube-linux-amd64 && chmod +x minikube
    $ sudo mv minikube /usr/local/bin
    ```

1. minikubeのバージョン確認

    ```
    $ minikube version
    ```

    - 実行結果（例）

        ```
        minikube version: v1.0.0
        ```

## MQTT clientのインストール
### macOS
1. `mosquitto` のインストール

    ```
    $ brew update && brew install mosquitto
    ```

### Ubuntu
1. `mosquitto` のリポジトリ登録

    ```
    $ sudo add-apt-repository ppa:mosquitto-dev/mosquitto-ppa
    ```

1. パッケージリストの更新

    ```
    $ sudo apt-get update -y
    ```

1. `mosquitto-clients` のインストール

    ```
    $ sudo apt-get install mosquitto mosquitto-clients
    ```

1. `mosquitto-clients `のインストール確認

    ```
    $ dpkg -l | grep mosquitto
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
    $ cd $CORE_ROOT/docs/environments/minikube
    $ cp env.template env
    ```

1. 環境ファイルの設定

    ```
    $ vi env
    ```

    ```bash
    #!/bin/bash
    export MINIKUBE_NAME="minikube"; echo "MINIKUBE_NAME=${MINIKUBE_NAME}"

    export MQTT__iotagent="password_of_iotagent";echo "MQTT__iotagent=${MQTT__iotagent}"

    export HOST_IPADDR="<<HOST_IPADDR>>"; echo "HOST_IPADDR=${HOST_IPADDR}"
    export REPOSITORY="<<REPOSITORY>>"; echo "REPOSITORY=${REPOSITORY}"
    ```

    ※ `MQTT__iotagent` の値(MQTT Brokerの `iotagent` ユーザのパスワード）を変更してください

1. プロジェクトルートに移動

    ```
    $ cd $CORE_ROOT
    ```

1. 環境設定の読み込み

    ```
    $ source $CORE_ROOT/docs/environments/minikube/env
    ```

## minikubeの起動
1. minikubeの起動

    ```
    $ export CPU_CORE_NUM="4"
    $ export MEMORY_MB=8192
    $ export K8S_VERSION="v1.14.1"
    $ minikube start --cpus ${CPU_CORE_NUM} --memory ${MEMORY_MB} --kubernetes-version ${K8S_VERSION} --bootstrapper=kubeadm --extra-config=kubelet.authentication-token-webhook=true --extra-config=kubelet.authorization-mode=Webhook --extra-config=scheduler.address=0.0.0.0 --extra-config=controller-manager.address=0.0.0.0 --profile ${MINIKUBE_NAME}
    ```

    - 実行結果（例）

        ```
        😄  minikube v1.0.0 on darwin (amd64)
        🤹  Downloading Kubernetes v1.14.1 images in the background ...
        🔥  Creating virtualbox VM (CPUs=4, Memory=8192MB, Disk=20000MB) ...
        💿  Downloading Minikube ISO ...
         142.88 MB / 142.88 MB [============================================] 100.00% 0s
        📶  "minikube" IP address is 192.168.99.100
        🐳  Configuring Docker as the container runtime ...
        🐳  Version of container runtime is 18.06.2-ce
        ⌛  Waiting for image downloads to complete ...
        ✨  Preparing Kubernetes environment ...
            ▪ kubelet.authentication-token-webhook=true
            ▪ kubelet.authorization-mode=Webhook
            ▪ scheduler.address=0.0.0.0
            ▪ controller-manager.address=0.0.0.0
        🚜  Pulling images required by Kubernetes v1.14.1 ...
        🚀  Launching Kubernetes v1.14.1 using kubeadm ... 
        ⌛  Waiting for pods: apiserver proxy etcd scheduler controller dns
        🔑  Configuring cluster permissions ...
        🤔  Verifying component health .....
        💗  kubectl is now configured to use "minikube"
        🏄  Done! Thank you for using minikube!
        ```

1. Kubernetesのバージョン確認

    ```
    $ kubectl version
    ```

    - 実行結果（例）

        ```
        Client Version: version.Info{Major:"1", Minor:"14", GitVersion:"v1.14.1", GitCommit:"b7394102d6ef778017f2ca4046abbaa23b88c290", GitTreeState:"clean", BuildDate:"2019-04-19T22:15:46Z", GoVersion:"go1.12.4", Compiler:"gc", Platform:"darwin/amd64"}
        Server Version: version.Info{Major:"1", Minor:"14", GitVersion:"v1.14.1", GitCommit:"b7394102d6ef778017f2ca4046abbaa23b88c290", GitTreeState:"clean", BuildDate:"2019-04-08T17:02:58Z", GoVersion:"go1.12.1", Compiler:"gc", Platform:"linux/amd64"}
        ```

## minikubeのネットワーク設定確認
### macOS
1. ネットワーク名の確認

    ```
    $ NWNAME=$(VBoxManage showvminfo ${MINIKUBE_NAME} | grep "Host-only Interface" | awk 'match($0, /vboxnet[0-9]+/){print substr($0,RSTART,RLENGTH)}');echo ${NWNAME}
    ```

    - 実行結果（例）

        ```
        vboxnet0
        ```

1. IPアドレスの確認

    ```
    $ export HOST_IPADDR=$(ifconfig ${NWNAME} | awk '/inet / {print $2}')
    $ sed -i '' -e "s/<<HOST_IPADDR>>/${HOST_IPADDR}/" ${CORE_ROOT}/docs/environments/minikube/env
    $ echo ${HOST_IPADDR}
    ```

    - 実行結果（例）

        ```
        192.168.99.1
        ```

1. ネットマスクの確認

    ```
    $ NETMASK_HEX=$(ifconfig ${NWNAME} | awk '/netmask / {print $4}')
    $ export NETMASK=$(echo "${NETMASK_HEX:2}" | perl -pe '$_ = unpack("B32", pack("H*", $_)); s/0+$//g; $_ = length')
    $ echo ${NETMASK}
    ```

    - 実行結果（例）

        ```
        24
        ```

### Ubuntu
1. ipcalcのインストール

    ```
    $ sudo apt install ipcalc
    ```

1. langの変更

    ```
    $ export LANG=C
    ```

1. ネットワーク名の確認

    ```
    $ NWNAME=$(VBoxManage showvminfo ${MINIKUBE_NAME} | grep "Host-only Interface" | awk 'match($0, /vboxnet[0-9]+/){print substr($0,RSTART,RLENGTH)}');echo ${NWNAME}
    ```

    - 実行結果（例）

        ```
        vboxnet0
        ```

1. IPアドレスの確認

    ```
    $ export HOST_IPADDR=$(ifconfig ${NWNAME}  | awk '/inet / {print $2}' | cut -d: -f2)
    $ sed -i -e "s/<<HOST_IPADDR>>/${HOST_IPADDR}/" ${CORE_ROOT}/docs/environments/minikube/env
    $ echo ${HOST_IPADDR}
    ```

    - 実行結果（例）

        ```
        192.168.99.1
        ```

1. ネットマスクの確認

    ```
    $ NETMASK_IP=$(ifconfig ${NWNAME} | awk '/Mask/ {print $4}' | cut -d: -f2)
    $ export NETMASK=$(ipcalc ${HOST_IPADDR} ${NETMASK_IP} | awk '/Netmask: / {print $4}');echo ${NETMASK}
    ```

    - 実行結果（例）

        ```
        24
        ```


## minikubeのコンフィグレーション編集

1. minikubeのコンフィグレーション編集

    ```
    $ if [ -f ${HOME}/.minikube/machines/${MINIKUBE_NAME}/config.json.org ]; then cp ${HOME}/.minikube/machines/${MINIKUBE_NAME}/config.json.org ${HOME}/.minikube/machines/${MINIKUBE_NAME}/config.json; else cp ${HOME}/.minikube/machines/${MINIKUBE_NAME}/config.json ${HOME}/.minikube/machines/${MINIKUBE_NAME}/config.json.org; fi
    $ cat ${HOME}/.minikube/machines/${MINIKUBE_NAME}/config.json | perl -pse 's/"InsecureRegistry": \[/"InsecureRegistry": [\n                "$h\/$m",/g;' -- -h=${HOST_IPADDR} -m=${NETMASK} > /tmp/config.json
    $ mv /tmp/config.json ${HOME}/.minikube/machines/${MINIKUBE_NAME}/config.json
    $ diff -u ${HOME}/.minikube/machines/${MINIKUBE_NAME}/config.json.org ${HOME}/.minikube/machines/${MINIKUBE_NAME}/config.json
    ```

    - 実行結果（例）

        ```diff
        --- /home/fiware/.minikube/machines/minikube/config.json.org    2019-03-05 16:49:53.169617372 +0900
        +++ /home/fiware/.minikube/machines/minikube/config.json        2019-03-05 16:49:53.173617453 +0900
        @@ -41,6 +41,7 @@
                    "Env": null,
                    "Ipv6": false,
                    "InsecureRegistry": [
        +               "192.168.99.1/24",
                        "10.96.0.0/12"
                    ],
                    "Labels": null,
        ```


## minikubeの再起動

1. minikubeの再起動

    ```
    $ minikube stop --profile ${MINIKUBE_NAME}
    $ minikube start --cpus ${CPU_CORE_NUM} --memory ${MEMORY_MB} --kubernetes-version ${K8S_VERSION} --bootstrapper=kubeadm --extra-config=kubelet.authentication-token-webhook=true --extra-config=kubelet.authorization-mode=Webhook --extra-config=scheduler.address=0.0.0.0 --extra-config=controller-manager.address=0.0.0.0 --profile ${MINIKUBE_NAME}
    ```

    - 実行結果（例）

        ```
        ✋  Stopping "minikube" in virtualbox ...
        🛑  "minikube" stopped.
        😄  minikube v1.0.0 on darwin (amd64)
        🤹  Downloading Kubernetes v1.14.1 images in the background ...
        💡  Tip: Use 'minikube start -p <name>' to create a new cluster, or 'minikube delete' to delete this one.
        🔄  Restarting existing virtualbox VM for "minikube" ...
        ⌛  Waiting for SSH access ...
        📶  "minikube" IP address is 192.168.99.100
        🐳  Configuring Docker as the container runtime ...
        🐳  Version of container runtime is 18.06.2-ce
        ⌛  Waiting for image downloads to complete ...
        ✨  Preparing Kubernetes environment ...
            ▪ kubelet.authentication-token-webhook=true
            ▪ kubelet.authorization-mode=Webhook
            ▪ scheduler.address=0.0.0.0
            ▪ controller-manager.address=0.0.0.0
        🚜  Pulling images required by Kubernetes v1.14.1 ...
        🔄  Relaunching Kubernetes v1.14.1 using kubeadm ... 
        ⌛  Waiting for pods: apiserver proxy etcd scheduler controller dns
        📯  Updating kube-proxy configuration ...
        🤔  Verifying component health ......
        💗  kubectl is now configured to use "minikube"
        🏄  Done! Thank you for using minikube!
        ```

1. ノードの確認

    ```
    $ kubectl get nodes
    ```

    - 実行結果（例）

        ```
        NAME       STATUS   ROLES    AGE   VERSION
        minikube   Ready    master   10m   v1.14.1
        ```


## hostOSでローカルレジストリを起動
### macOS
1. ローカルdockerデーモンを受け付けるように、InsecureRegistryを編集
    * `Docker for mac > Preference > Daemon > Advanced` から、Dockerデーモンの設定を次のように編集する

    ```diff
     {
       "debug" : true,
    -  "experimental" : true
    +  "experimental" : true,
    +  "insecure-registries" : [
    +    "192.168.99.0/24"
    +  ]
     }
    ```
    ※ `-` の行を削除し、 `+` の行を追加

### Ubuntu
1. ローカルdockerデーモンを受け付けるように、InsecureRegistryを編集

    ```
    $ cat << __EOL__ | sudo tee /etc/docker/daemon.json
    { "insecure-registries":["192.168.99.0/24"] }
    __EOL__
    ```

1. 編集結果を確認
    ```
    $ cat /etc/docker/daemon.json
    ```

    - 実行結果（例）

        ```json
        { "insecure-registries":["192.168.99.0/24"] }
        ```

1. systemctlの設定ファイル再読込

    ```
    $ sudo systemctl daemon-reload
    ```

1. dockerの再起動

    ```
    $ sudo systemctl restart docker
    ```

1. dockerの起動確認

    ```
    $ sudo systemctl status docker
    ```

    - 実行結果（例）

        ```
        ● docker.service - Docker Application Container Engine
        Loaded: loaded (/lib/systemd/system/docker.service; enabled; vendor preset: enabled)
        Active: active (running) since 火 2019-03-05 17:45:22 JST; 29s ago
            Docs: https://docs.docker.com
        Main PID: 14538 (dockerd)
            Tasks: 17
        Memory: 34.7M
            CPU: 291ms
        CGroup: /system.slice/docker.service
                mq14538 /usr/bin/dockerd -H fd:// --containerd=/run/containerd/containerd.sock

        3月 05 17:45:22 roboticbase-pc dockerd[14538]: time="2019-03-05T17:45:22.407075302+09:00" level=warning msg="Your kern
        3月 05 17:45:22 roboticbase-pc dockerd[14538]: time="2019-03-05T17:45:22.407103943+09:00" level=warning msg="Your kern
        3月 05 17:45:22 roboticbase-pc dockerd[14538]: time="2019-03-05T17:45:22.407540237+09:00" level=info msg="Loading cont
        3月 05 17:45:22 roboticbase-pc dockerd[14538]: time="2019-03-05T17:45:22.671625976+09:00" level=info msg="Default brid
        3月 05 17:45:22 roboticbase-pc dockerd[14538]: time="2019-03-05T17:45:22.812144133+09:00" level=info msg="Loading cont
        3月 05 17:45:22 roboticbase-pc dockerd[14538]: time="2019-03-05T17:45:22.863976008+09:00" level=warning msg="Not using
        3月 05 17:45:22 roboticbase-pc dockerd[14538]: time="2019-03-05T17:45:22.864291080+09:00" level=info msg="Docker daemo
        3月 05 17:45:22 roboticbase-pc dockerd[14538]: time="2019-03-05T17:45:22.864360582+09:00" level=info msg="Daemon has c
        3月 05 17:45:22 roboticbase-pc dockerd[14538]: time="2019-03-05T17:45:22.887574824+09:00" level=info msg="API listen o
        ```


## ローカルレジストリのスタート
### macOS
1. ローカルレジストリのスタート

    ```
    $ docker run --name registry -p 5000:5000 -d registry:2.7.1
    ```

    - 実行結果（例）

        ```
        Unable to find image 'registry:2.7.1' locally
        2.7.1: Pulling from library/registry
        c87736221ed0: Pull complete
        1cc8e0bb44df: Pull complete
        54d33bcb37f5: Pull complete
        e8afc091c171: Pull complete
        b4541f6d3db6: Pull complete
        Digest: sha256:3b00e5438ebd8835bcfa7bf5246445a6b57b9a50473e89c02ecc8e575be3ebb5
        Status: Downloaded newer image for registry:2.7.1
        a1eb1d13205879387870fe55f52e145844d6a1754fb7d4bc5de2a8297bf5d091
        ```

1. リポジトリの確認

    ```
    $ export REPOSITORY=${HOST_IPADDR}:5000
    $ sed -i '' -e "s/<<REPOSITORY>>/${REPOSITORY}/" ${CORE_ROOT}/docs/environments/minikube/env
    $ echo ${REPOSITORY}
    ```

    - 実行結果（例）

        ```
        192.168.99.1:5000
        ```

### Ubuntu

1. ローカルレジストリのスタート

    ```
    $ docker run --name registry -p 5000:5000 -d registry:2.7.1
    ```

    - 実行結果（例）

        ```
        Unable to find image 'registry:2.7.1' locally
        2.7.1: Pulling from library/registry
        c87736221ed0: Pull complete
        1cc8e0bb44df: Pull complete
        54d33bcb37f5: Pull complete
        e8afc091c171: Pull complete
        b4541f6d3db6: Pull complete
        Digest: sha256:3b00e5438ebd8835bcfa7bf5246445a6b57b9a50473e89c02ecc8e575be3ebb5
        Status: Downloaded newer image for registry:2.7.1
        a1eb1d13205879387870fe55f52e145844d6a1754fb7d4bc5de2a8297bf5d091
        ```

1. リポジトリの確認

    ```
    $ export REPOSITORY=${HOST_IPADDR}:5000
    $ sed -i -e "s/<<REPOSITORY>>/${REPOSITORY}/" ${CORE_ROOT}/docs/environments/minikube/env
    $ echo ${REPOSITORY}
    ```

    - 実行結果（例）

        ```
        192.168.99.1:5000
        ```


## Role-based access control(RBAC)の設定

1. tiller-rbacの作成

    ```
    $ kubectl apply -f rbac/tiller-rbac.yaml
    ```

    - 実行結果（例）

        ```
        serviceaccount/tiller created
        clusterrolebinding.rbac.authorization.k8s.io/tiller created
        ```

1. tillerのserviceaccounts確認

    ```
    $ kubectl get serviceaccounts -n kube-system | grep tiller
    ```

    - 実行結果（例）

        ```
        tiller                               1         11m
        ```

1. default-rbacの作成

    ```
    $ kubectl apply -f rbac/default-rbac.yaml
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
        default-read                                                           63s
        ```
      
1. defaultのclusterrolebindings確認

    ```
    $ kubectl get clusterrolebindings | grep default
    ```

    - 実行結果（例）

        ```
        default                                                36s
        ```

## Helmの初期化
1. Helmのサービスアカウント初期化

    ```
    $ helm init --service-account tiller
    ```

    - 実行結果（例）

        ```
        $HELM_HOME has been configured at /Users/nmatsui/.helm.

        Tiller (the Helm server-side component) has been installed into your Kubernetes Cluster.

        Please note: by default, Tiller is deployed with an insecure 'allow unauthenticated users' policy.
        To prevent this, run `helm init` with the --tiller-tls-verify flag.
        For more information on securing your installation see: https://docs.helm.sh/using_helm/#securing-your-helm-installation
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
        Update Complete. ⎈ Happy Helming!⎈
        ```

1. Tillerのpods確認

    ```
    $ kubectl get pods --all-namespaces | grep tiller
    ```

    - 実行結果（例）

        ```
        kube-system   tiller-deploy-95d654d46-m5qgs      1/1     Running   0          54s
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
