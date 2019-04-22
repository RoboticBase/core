# RoboticBase Coreインストールマニュアル #1

## 構築環境(2019年3月6日現在)
- Ubuntu 16.04.5 LTS
- git 2.7.4-0ubuntu1.6
- virtualbox 5.2.12r12259
- kubectl 1.13.4
- kubeadm 1.13.4
- kubelet 1.13.4
- minikube 0.34.1
- ipcalc 0.41-5
- docker-ce 18.09.3~3-0~ubuntu-xenial
- docker-ce-cli 18.09.3~3-0~ubuntu-xenial
- helm v2.10.0

# minikubeの準備

## 環境変数の設定

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

1. ベースファイルの取得

    ```
    $ cd ${HOME}
    $ git clone https://github.com/RoboticBase/core
    ```

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
    $ cd $CORE_ROOT/docs/minikube
    $ cp env.template env
    ```

1. プロジェクトルートに移動

    ```
    $ cd $CORE_ROOT
    ```

1. 環境設定の読み込み

    ```
    $ source $CORE_ROOT/docs/minikube/env
    ```

1. VirtualBoxのインストール

    ```
    $ curl -Lo VirtualBox-5.2.12-122591-Linux_amd64.run https://download.virtualbox.org/virtualbox/5.2.12/VirtualBox-5.2.12-122591-Linux_amd64.run && chmod +x VirtualBox-5.2.12-122591-Linux_amd64.run && sudo ./VirtualBox-5.2.12-122591-Linux_amd64.run && rm VirtualBox-5.2.12-122591-Linux_amd64.run
    ```

    - 実行結果（例）

        ```
        % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
                                    Dload  Upload   Total   Spent    Left  Speed
        100 84.5M  100 84.5M    0     0  7579k      0  0:00:11  0:00:11 --:--:-- 8532k
        Verifying archive integrity... All good.
        Uncompressing VirtualBox for Linux installation.............
        VirtualBox Version 5.2.12 r122591 (2018-05-09T10:52:30Z) installer
        Installing VirtualBox to /opt/VirtualBox
        Python found: python, installing bindings...
        
        VirtualBox has been installed successfully.
        
        You will find useful information about using VirtualBox in the user manual
        /opt/VirtualBox/UserManual.pdf
        and in the user FAQ
        http://www.virtualbox.org/wiki/User_FAQ
        
        We hope that you enjoy using VirtualBox.
        ```

1. VirtualBoxのバージョン確認

    ```
    $ vboxmanage -v
    ```

    - 実行結果（例）
        
        ```
        5.2.12r122591 
        ```


## minikubeのインストール

1. minikubeのインストール

    ```
    $ curl -Lo minikube https://storage.googleapis.com/minikube/releases/v0.34.1/minikube-darwin-amd64 \　&& chmod +x minikube
    $ sudo mv minikube /usr/local/bin
    ````

1. minikubeの起動

    ```
    $ minikube start --cpus 4 --memory 16384 --kubernetes-version v1.12.5 --bootstrapper=kubeadm --extra-config=kubelet.authentication-token-webhook=true --extra-config=kubelet.authorization-mode=Webhook --extra-config=scheduler.address=0.0.0.0 --extra-config=controller-manager.address=0.0.0.0 --profile ${MINIKUBE_NAME}
    ```

    - 実行結果（例）

        ```
        o   minikube v0.34.1 on linux (amd64)
        >   Creating virtualbox VM (CPUs=4, Memory=8192MB, Disk=20000MB) ...
        @   Downloading Minikube ISO ...
        184.30 MB / 184.30 MB [============================================] 100.00% 0s
        -   "minikube" IP address is 192.168.99.100
        -   Configuring Docker as the container runtime ...
        -   Preparing Kubernetes environment ...
            - kubelet.authentication-token-webhook=true
            - kubelet.authorization-mode=Webhook
            - scheduler.address=0.0.0.0
            - controller-manager.address=0.0.0.0
        @   Downloading kubeadm v1.12.5
        @   Downloading kubelet v1.12.5
        -   Pulling images required by Kubernetes v1.12.5 ...
        -   Launching Kubernetes v1.12.5 using kubeadm ...
        -   Configuring cluster permissions ...
        -   Verifying component health .....
        +   kubectl is now configured to use "minikube"
        i   For best results, install kubectl: https://kubernetes.io/docs/tasks/tools/install-kubectl/
        =   Done! Thank you for using minikube!
        ```

1. kubelet,kubeadm,kubectlのインストール

    ```
    $ sudo apt-get update && sudo apt-get install -y apt-transport-https
    $ curl -s https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
    $ sudo sh -c 'cat <<EOF >/etc/apt/sources.list.d/kubernetes.list
    deb http://apt.kubernetes.io/ kubernetes-xenial main
    EOF'
    $ sudo apt-get update
    $ sudo apt-get install -y kubelet kubeadm kubectl
    ```

1. kubectlのインストール確認

    ```
    $ kubectl version
    ```

    - 実行結果（例）

        ```
        Client Version: version.Info{Major:"1", Minor:"13", GitVersion:"v1.13.4", GitCommit:"c27b913fddd1a6c480c229191a087698aa92f0b1", GitTreeState:"clean", BuildDate:"2019-02-28T13:37:52Z", GoVersion:"go1.11.5", Compiler:"gc", Platform:"linux/amd64"}
        Server Version: version.Info{Major:"1", Minor:"12", GitVersion:"v1.12.5", GitCommit:"51dd616cdd25d6ee22c83a858773b607328a18ec", GitTreeState:"clean", BuildDate:"2019-01-16T18:14:49Z", GoVersion:"go1.10.7", Compiler:"gc", Platform:"linux/amd64"}
        ```

1. kubeadmのインストール確認

    ```
    $ kubeadm version
    ```

    - 実行結果（例）

        ```
        kubeadm version: &version.Info{Major:"1", Minor:"13", GitVersion:"v1.13.4", GitCommit:"c27b913fddd1a6c480c229191a087698aa92f0b1", GitTreeState:"clean", BuildDate:"2019-02-28T13:35:32Z", GoVersion:"go1.11.5", Compiler:"gc", Platform:"linux/amd64"}
        ```

1. kubeletのインストール確認

    ```
    $ kubelet --version
    ```

    - 実行結果（例）

        ```
        Kubernetes v1.13.4
        ```


## minikubeのネットワーク設定確認

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
    $ sed -i -e "s/<<HOST_IPADDR>>/${HOST_IPADDR}/" ${CORE_ROOT}/docs/minikube/env
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

        ```
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
    $ minikube start --cpus 4 --memory 16384 --kubernetes-version v1.12.5 --bootstrapper=kubeadm --extra-config=kubelet.authentication-token-webhook=true --extra-config=kubelet.authorization-mode=Webhook --extra-config=scheduler.address=0.0.0.0 --extra-config=controller-manager.address=0.0.0.0 --profile ${MINIKUBE_NAME}
    ```

    - 実行結果（例）

        ```
        o   minikube v0.34.1 on linux (amd64)
        i   Tip: Use 'minikube start -p <name>' to create a new cluster, or 'minikube delete' to delete this one.
        :   Restarting existing virtualbox VM for "minikube" ...
        :   Waiting for SSH access ...
        -   "minikube" IP address is 192.168.99.100
        -   Configuring Docker as the container runtime ...
        -   Preparing Kubernetes environment ...
            - kubelet.authentication-token-webhook=true
            - kubelet.authorization-mode=Webhook
            - scheduler.address=0.0.0.0
            - controller-manager.address=0.0.0.0
        -   Pulling images required by Kubernetes v1.12.5 ...
        :   Relaunching Kubernetes v1.12.5 using kubeadm ...
        :   Waiting for kube-proxy to come back up ...
        -   Verifying component health ......
        +   kubectl is now configured to use "minikube"
        =   Done! Thank you for using minikube!
        ```

1. ノードの確認

    ```
    $ kubectl get nodes
    ```

    - 実行結果（例）

        ```
        NAME       STATUS   ROLES    AGE   VERSION
        minikube   Ready    master   22m   v1.12.5
        ```


## docker-ceのインストール

1. 前提ファイルのインストール

    ```
    $ sudo apt-get install -y apt-transport-https ca-certificates curl software-properties-common
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
    $ sudo apt-get install -y docker-ce
    ```

1. docker-ceのインストール確認

    ```
    $ dpkg -l | grep docker
    ```

    - 実行結果（例）

        ```
        ii  docker-ce                                  5:18.09.3~3-0~ubuntu-xenial                         amd64        Docker: the open-source application container engine
        ii  docker-ce-cli                              5:18.09.3~3-0~ubuntu-xenial                         amd64        Docker CLI: the open-source application container engine
        ```

1. dockerコマンドの実行権限を付与

    ```
    $ sudo gpasswd -a $USER docker
    ```

1. dockerの再起動

    ```
    $ sudo systemctl restart docker
    $ exit
    ```


## hostOSでローカルレジストリを起動

1. ローカルdockerデーモンを受け付けるように、InsecureRegistryを編集

    ```
    $ sudo vi /etc/docker/daemon.json
    { "insecure-registries":["192.168.99.0/24"] }
    :wq
    $ cat /etc/docker/daemon.json
    ```

    - 実行結果（例）

        ```
        { "insecure-registries":["192.168.99.0/24"] }
        ```

1. systemctlの設定ファイル再読込

    ```
    $ systemctl daemon-reload
    ```

    - 実行結果（例）

        ```
        ==== AUTHENTICATING FOR org.freedesktop.systemd1.reload-daemon ===
        Authentication is required to reload the systemd state.
        Authenticating as: ros-terminal,,, (fiware)
        Password:
        ==== AUTHENTICATION COMPLETE ===
        ```

1. dockerの再起動

    ```
    $ systemctl restart docker
    ```

    - 実行結果（例）

        ```
        ==== AUTHENTICATING FOR org.freedesktop.systemd1.manage-units ===
        Authentication is required to restart 'docker.service'.
        Authenticating as: ros-terminal,,, (fiware)
        Password:
        ==== AUTHENTICATION COMPLETE ===
        ```

1. dockerの起動確認

    ```
    $ systemctl status docker
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

1. ローカルレジストリのスタート

    ```
    $ docker run --name registry -p 5000:5000 -d registry:2.6
    ```

    - 実行結果（例）

        ```
        Unable to find image 'registry:2.6' locally
        2.6: Pulling from library/registry
        169185f82c45: Pull complete
        046e2d030894: Pull complete
        cae47d055bbf: Pull complete
        8ee3a92fedf2: Pull complete
        7629f7023b7e: Pull complete
        Digest: sha256:e97930f598b89aa90a4a5d398da11e491c9ade43d8e63d901ccdbd08a93c5e92
        Status: Downloaded newer image for registry:2.6
        b6f99a7c9d783303857ba74fef88d7e1f34194e4cadb357cdd3b5fb6cf51a815
        ```

1. リポジトリの確認

    ```
    $ export REPOSITORY=${HOST_IPADDR}:5000
    $ sed -i -e "s/<<REPOSITORY>>/${REPOSITORY}/" ${CORE_ROOT}/docs/minikube/env
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


## Helmのインストール

1. Helmのインストール

    ```
    $ cd /tmp
    $ curl -LO https://storage.googleapis.com/kubernetes-helm/helm-v2.10.0-linux-amd64.tar.gz
    $ sudo tar xvf helm-v2.10.0-linux-amd64.tar.gz
    $ sudo cp linux-amd64/helm /usr/bin
    $ sudo rm -rf linux-amd64
    $ rm helm-v2.10.0-linux-amd64.tar.gz
    ```

1. Helmのバージョン確認

    ```
    $ helm version --client
    ```

    - 実行結果（例）

        ```
        Client: &version.Version{SemVer:"v2.10.0", GitCommit:"9ad53aac42165a5fadc6c87be0dea6b115f93090", GitTreeState:"clean"}
        ```

1. Helmのサービスアカウント初期化

    ```
    $ helm init --service-account tiller
    ```

    - 実行結果（例）

        ```
        Creating /home/fiware/.helm
        Creating /home/fiware/.helm/repository
        Creating /home/fiware/.helm/repository/cache
        Creating /home/fiware/.helm/repository/local
        Creating /home/fiware/.helm/plugins
        Creating /home/fiware/.helm/starters
        Creating /home/fiware/.helm/cache/archive
        Creating /home/fiware/.helm/repository/repositories.yaml
        Adding stable repo with URL: https://kubernetes-charts.storage.googleapis.com
        Adding local repo with URL: http://127.0.0.1:8879/charts
        $HELM_HOME has been configured at /home/fiware/.helm.

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
        Update Complete. ? Happy Helming!?
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
        Client: &version.Version{SemVer:"v2.10.0", GitCommit:"9ad53aac42165a5fadc6c87be0dea6b115f93090", GitTreeState:"clean"}
        Server: &version.Version{SemVer:"v2.10.0", GitCommit:"9ad53aac42165a5fadc6c87be0dea6b115f93090", GitTreeState:"clean"}
        ```
