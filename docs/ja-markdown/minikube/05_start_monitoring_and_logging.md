# RoboticBase Coreインストールマニュアル #5


## 構築環境(2019年7月18日現在)


# 3. minikubeでモニターリング＆ロギングの開始

## 環境変数設定

1. 環境変数の設定

    ```
    $ export CORE_ROOT="${HOME}/core"
    $ cd ${CORE_ROOT};pwd
    ```

    - 実行結果（例）

        ```
        /home/fiware/core
        ```

1. 環境設定の読み込み

    ```
    $ source ${CORE_ROOT}/docs/environments/minikube/env
    ```

## コマンドのエイリアスを設定
1. エイリアスの設定

    ```
    $ if [ "$(uname)" == 'Darwin' ]; then
      alias openbrowser='open'
    elif [ "$(expr substr $(uname -s) 1 5)" == 'Linux' ]; then
      alias openbrowser='xdg-open'
    else
      echo "Your platform ($(uname -a)) is not supported."
      exit 1
    fi
    ```

## minikubeでfiware cygnus(elasticsearch sink)を起動

1. cygnus-elasticsearch-serviceの作成

    ```
    $ kubectl apply -f cygnus/cygnus-elasticsearch-service.yaml
    ```

    - 実行結果（例）

        ```
        service/cygnus-elasticsearch created
        ```

1. cygnus-elasticsearch-deploymentの作成

    ```
    $ kubectl apply -f cygnus/cygnus-elasticsearch-deployment.yaml
    ```

    - 実行結果（例）

        ```
        deployment.apps/cygnus-elasticsearch created
        ```

1. cygnus-elasticsearchのpods状態確認

    ```
    $ kubectl get pods -l app=cygnus-elasticsearch
    ```

    - 実行結果（例）

        ```
        NAME                                    READY   STATUS    RESTARTS   AGE
        cygnus-elasticsearch-8575567db7-bx8w7   1/1     Running   0          95s
        cygnus-elasticsearch-8575567db7-c2xv7   1/1     Running   0          95s
        cygnus-elasticsearch-8575567db7-gjpth   1/1     Running   0          95s
        ```

1. cygnus-elasticsearchのservices状態確認

    ```
    $ kubectl get services -l app=cygnus-elasticsearch
    ```

    - 実行結果（例）

        ```
        NAME                   TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)             AGE
        cygnus-elasticsearch   ClusterIP   10.108.153.95   <none>        5050/TCP,8081/TCP   2m12s
        ```


## prometheusとgrafanaの設定

1. prometheus-operatorのインストール

    ```
    $ helm install stable/prometheus-operator --name po --namespace monitoring -f monitoring/prometheus-operator-minikube.yaml
    ```

    - 実行結果（例）

        ```
        NAME:   po
        LAST DEPLOYED: Sat Jul  6 14:44:32 2019
        NAMESPACE: monitoring
        STATUS: DEPLOYED

        RESOURCES:
        ==> v1/Alertmanager
        NAME                                 AGE
        po-prometheus-operator-alertmanager  1s

        ==> v1/ClusterRole
        NAME                                   AGE
        po-grafana-clusterrole                 2s
        po-prometheus-operator-alertmanager    2s
        po-prometheus-operator-operator        2s
        po-prometheus-operator-operator-psp    2s
        po-prometheus-operator-prometheus      2s
        po-prometheus-operator-prometheus-psp  2s
        psp-po-kube-state-metrics              2s

        ==> v1/ClusterRoleBinding
        NAME                                   AGE
        po-grafana-clusterrolebinding          2s
        po-prometheus-operator-alertmanager    2s
        po-prometheus-operator-operator        2s
        po-prometheus-operator-operator-psp    2s
        po-prometheus-operator-prometheus      2s
        po-prometheus-operator-prometheus-psp  2s
        psp-po-kube-state-metrics              2s

        ==> v1/ConfigMap
        NAME                                                      DATA  AGE
        po-grafana                                                1     2s
        po-grafana-config-dashboards                              1     2s
        po-grafana-test                                           1     2s
        po-prometheus-operator-etcd                               1     2s
        po-prometheus-operator-grafana-datasource                 1     2s
        po-prometheus-operator-k8s-cluster-rsrc-use               1     2s
        po-prometheus-operator-k8s-coredns                        1     2s
        po-prometheus-operator-k8s-node-rsrc-use                  1     2s
        po-prometheus-operator-k8s-resources-cluster              1     2s
        po-prometheus-operator-k8s-resources-namespace            1     2s
        po-prometheus-operator-k8s-resources-pod                  1     2s
        po-prometheus-operator-k8s-resources-workload             1     2s
        po-prometheus-operator-k8s-resources-workloads-namespace  1     2s
        po-prometheus-operator-nodes                              1     2s
        po-prometheus-operator-persistentvolumesusage             1     2s
        po-prometheus-operator-pods                               1     2s
        po-prometheus-operator-statefulset                        1     2s

        ==> v1/Deployment
        NAME                             READY  UP-TO-DATE  AVAILABLE  AGE
        po-kube-state-metrics            0/1    1           0          1s
        po-prometheus-operator-operator  0/1    1           0          1s

        ==> v1/Pod(related)
        NAME                                              READY  STATUS             RESTARTS  AGE
        po-grafana-594865f85-rqdcs                        0/2    Init:0/1           0         1s
        po-kube-state-metrics-844bd6548c-xdzb4            0/1    ContainerCreating  0         1s
        po-prometheus-node-exporter-9gpwj                 0/1    ContainerCreating  0         1s
        po-prometheus-operator-operator-6984bb6db6-8cwft  0/1    ContainerCreating  0         1s

        ==> v1/Prometheus
        NAME                               AGE
        po-prometheus-operator-prometheus  1s

        ==> v1/PrometheusRule
        NAME                                                         AGE
        po-prometheus-operator-alertmanager.rules                    1s
        po-prometheus-operator-etcd                                  1s
        po-prometheus-operator-general.rules                         1s
        po-prometheus-operator-k8s.rules                             1s
        po-prometheus-operator-kube-apiserver.rules                  1s
        po-prometheus-operator-kube-prometheus-node-alerting.rules   1s
        po-prometheus-operator-kube-prometheus-node-recording.rules  1s
        po-prometheus-operator-kube-scheduler.rules                  1s
        po-prometheus-operator-kubernetes-absent                     1s
        po-prometheus-operator-kubernetes-apps                       1s
        po-prometheus-operator-kubernetes-resources                  1s
        po-prometheus-operator-kubernetes-storage                    1s
        po-prometheus-operator-kubernetes-system                     1s
        po-prometheus-operator-node-network                          1s
        po-prometheus-operator-node-time                             1s
        po-prometheus-operator-node.rules                            1s
        po-prometheus-operator-prometheus-operator                   1s
        po-prometheus-operator-prometheus.rules                      1s

        ==> v1/Role
        NAME             AGE
        po-grafana-test  2s

        ==> v1/RoleBinding
        NAME             AGE
        po-grafana-test  2s

        ==> v1/Secret
        NAME                                              TYPE    DATA  AGE
        alertmanager-po-prometheus-operator-alertmanager  Opaque  1     2s
        po-grafana                                        Opaque  3     2s

        ==> v1/Service
        NAME                                            TYPE       CLUSTER-IP      EXTERNAL-IP  PORT(S)    AGE
        po-grafana                                      ClusterIP  10.106.85.127   <none>       80/TCP     2s
        po-kube-state-metrics                           ClusterIP  10.106.251.12   <none>       8080/TCP   2s
        po-prometheus-node-exporter                     ClusterIP  10.97.49.161    <none>       9100/TCP   2s
        po-prometheus-operator-alertmanager             ClusterIP  10.97.209.186   <none>       9093/TCP   2s
        po-prometheus-operator-coredns                  ClusterIP  None            <none>       9153/TCP   2s
        po-prometheus-operator-kube-controller-manager  ClusterIP  None            <none>       10252/TCP  2s
        po-prometheus-operator-kube-etcd                ClusterIP  None            <none>       2379/TCP   2s
        po-prometheus-operator-kube-scheduler           ClusterIP  None            <none>       10251/TCP  2s
        po-prometheus-operator-operator                 ClusterIP  10.108.31.165   <none>       8080/TCP   1s
        po-prometheus-operator-prometheus               ClusterIP  10.101.223.201  <none>       9090/TCP   1s

        ==> v1/ServiceAccount
        NAME                                 SECRETS  AGE
        po-grafana                           1        2s
        po-grafana-test                      1        2s
        po-kube-state-metrics                1        2s
        po-prometheus-node-exporter          1        2s
        po-prometheus-operator-alertmanager  1        2s
        po-prometheus-operator-operator      1        2s
        po-prometheus-operator-prometheus    1        2s

        ==> v1/ServiceMonitor
        NAME                                            AGE
        po-prometheus-operator-alertmanager             1s
        po-prometheus-operator-apiserver                1s
        po-prometheus-operator-coredns                  1s
        po-prometheus-operator-grafana                  1s
        po-prometheus-operator-kube-controller-manager  1s
        po-prometheus-operator-kube-etcd                1s
        po-prometheus-operator-kube-scheduler           1s
        po-prometheus-operator-kube-state-metrics       1s
        po-prometheus-operator-kubelet                  1s
        po-prometheus-operator-node-exporter            1s
        po-prometheus-operator-operator                 1s
        po-prometheus-operator-prometheus               1s

        ==> v1beta1/ClusterRole
        NAME                             AGE
        po-kube-state-metrics            2s
        psp-po-prometheus-node-exporter  2s

        ==> v1beta1/ClusterRoleBinding
        NAME                             AGE
        po-kube-state-metrics            2s
        psp-po-prometheus-node-exporter  2s

        ==> v1beta1/DaemonSet
        NAME                         DESIRED  CURRENT  READY  UP-TO-DATE  AVAILABLE  NODE SELECTOR  AGE
        po-prometheus-node-exporter  1        1        0      1           0          <none>         1s

        ==> v1beta1/PodSecurityPolicy
        NAME                                 PRIV   CAPS      SELINUX           RUNASUSER  FSGROUP    SUPGROUP  READONLYROOTFS  VOLUMES
        po-grafana                           false  RunAsAny  RunAsAny          RunAsAny   RunAsAny   false     configMap,emptyDir,projected,secret,downwardAPI,persistentVolumeClaim
        po-grafana-test                      false  RunAsAny  RunAsAny          RunAsAny   RunAsAny   false     configMap,downwardAPI,emptyDir,projected,secret
        po-kube-state-metrics                false  RunAsAny  MustRunAsNonRoot  MustRunAs  MustRunAs  false     secret
        po-prometheus-node-exporter          false  RunAsAny  RunAsAny          MustRunAs  MustRunAs  false     configMap,emptyDir,projected,secret,downwardAPI,persistentVolumeClaim,hostPath
        po-prometheus-operator-alertmanager  false  RunAsAny  RunAsAny          MustRunAs  MustRunAs  false     configMap,emptyDir,projected,secret,downwardAPI,persistentVolumeClaim
        po-prometheus-operator-operator      false  RunAsAny  RunAsAny          MustRunAs  MustRunAs  false     configMap,emptyDir,projected,secret,downwardAPI,persistentVolumeClaim
        po-prometheus-operator-prometheus    false  RunAsAny  RunAsAny          MustRunAs  MustRunAs  false     configMap,emptyDir,projected,secret,downwardAPI,persistentVolumeClaim

        ==> v1beta1/Role
        NAME        AGE
        po-grafana  2s

        ==> v1beta1/RoleBinding
        NAME        AGE
        po-grafana  2s

        ==> v1beta2/Deployment
        NAME        READY  UP-TO-DATE  AVAILABLE  AGE
        po-grafana  0/1    1           0          1s


        NOTES:
        The Prometheus Operator has been installed. Check its status by running:
          kubectl --namespace monitoring get pods -l "release=po"

        Visit https://github.com/coreos/prometheus-operator for instructions on how
        to create & configure Alertmanager and Prometheus instances using the Operator.

        ```

1. prometheus-operatorのpods状態確認

    ```
    $ kubectl --namespace monitoring get pods -l "app=prometheus-operator-operator,release=po"
    ```

    - 実行結果（例）

        ```
        NAME                                               READY   STATUS    RESTARTS   AGE
        po-prometheus-operator-operator-7cf7c5cc97-78h9g   1/1     Running   0          2m28s
        ```

1. monitoringネームスペースのdaemonsets状態確認

    ```
    $ kubectl get daemonsets --namespace monitoring
    ```

    - 実行結果（例）

        ```
        NAME                          DESIRED   CURRENT   READY   UP-TO-DATE   AVAILABLE   NODE SELECTOR   AGE
        po-prometheus-node-exporter   1         1         1       1            1           <none>          3m47s
        ```

1. monitoringネームスペースのdeployments状態確認

    ```
    $ kubectl get deployments --namespace monitoring
    ```

    - 実行結果（例）

        ```
        NAME                              READY   UP-TO-DATE   AVAILABLE   AGE
        po-grafana                        1/1     1            1           4m27s
        po-kube-state-metrics             1/1     1            1           4m27s
        po-prometheus-operator-operator   1/1     1            1           4m27s
        ```

1. monitoringネームスペースのstatefulsets状態確認

    ```
    $ kubectl get statefulsets --namespace monitoring
    ```

    - 実行結果（例）

        ```
        NAME                                               READY   AGE
        alertmanager-po-prometheus-operator-alertmanager   1/1     4m42s
        prometheus-po-prometheus-operator-prometheus       1/1     4m32s
        ```

1. monitoringネームスペースのpods状態確認

    ```
    $ kubectl get pods --namespace monitoring
    ```

    - 実行結果（例）

        ```
        NAME                                                 READY   STATUS    RESTARTS   AGE
        alertmanager-po-prometheus-operator-alertmanager-0   2/2     Running   0          5m10s
        po-grafana-594865f85-rqdcs                           2/2     Running   0          5m39s
        po-kube-state-metrics-844bd6548c-xdzb4               1/1     Running   0          5m39s
        po-prometheus-node-exporter-9gpwj                    1/1     Running   0          5m39s
        po-prometheus-operator-operator-6984bb6db6-8cwft     1/1     Running   0          5m39s
        prometheus-po-prometheus-operator-prometheus-0       3/3     Running   1          5m
        ```

1. monitoringネームスペースのservices状態確認

    ```
    $ kubectl get services --namespace monitoring
    ```

    - 実行結果（例）

        ```
        NAME                                  TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)             AGE
        alertmanager-operated                 ClusterIP   None             <none>        9093/TCP,6783/TCP   5m46s
        po-grafana                            ClusterIP   10.106.85.127    <none>        80/TCP              6m16s
        po-kube-state-metrics                 ClusterIP   10.106.251.12    <none>        8080/TCP            6m16s
        po-prometheus-node-exporter           ClusterIP   10.97.49.161     <none>        9100/TCP            6m16s
        po-prometheus-operator-alertmanager   ClusterIP   10.97.209.186    <none>        9093/TCP            6m16s
        po-prometheus-operator-operator       ClusterIP   10.108.31.165    <none>        8080/TCP            6m15s
        po-prometheus-operator-prometheus     ClusterIP   10.101.223.201   <none>        9090/TCP            6m15s
        prometheus-operated                   ClusterIP   None             <none>        9090/TCP            5m36s
        ```

## prometheusルールの編集

1. general.rulesの編集

    ```
    $ kubectl edit --namespace=monitoring prometheusrules po-prometheus-operator-general.rules
    ```

    ※先頭に-の付いている部分を削除してください

    ```diff
           for: 10m
           labels:
             severity: warning
    -    - alert: Watchdog
    -      annotations:
    -        message: |
    -          This is an alert meant to ensure that the entire alerting pipeline is functional.
    -          This alert is always firing, therefore it should always be firing in Alertmanager
    -          and always fire against a receiver. There are integrations with various notification
    -          mechanisms that send a notification when this alert is not firing. For example the
    -          "DeadMansSnitch" integration in PagerDuty.
    -      expr: vector(1)
    -      labels:
    -        severity: none
    ```

## prometheusの確認

1. 別ターミナルを開き、prometheusのポートフォワーディングを開始

    ```
    $ kubectl --namespace monitoring port-forward $(kubectl get pod --namespace monitoring -l prometheus=kube-prometheus -l app=prometheus -o template --template "{{(index .items 0).metadata.name}}") 9090:9090
    ```

    - 実行結果（例）

        ```
        Forwarding from 127.0.0.1:9090 -> 9090
        Forwarding from [::1]:9090 -> 9090
        ```

1. ブラウザでprometheusにアクセス

    ```
    $ openbrowser http://localhost:9090
    ```

1. prometheusのWEB管理画面が表示されたことを確認

    ![prometheus001](images/prometheus/prometheus001.png)

1. 「Alerts」を選択

    ![prometheus002](images/prometheus/prometheus002.png)

1. CPUやMemoryのリソース使用量警告以外のすべてのアラートが「0 active」と表示されることを確認

    ![prometheus003-1](images/prometheus/prometheus003-1.png)
    ![prometheus003-2](images/prometheus/prometheus003-2.png)

1. メニューから「Status」「Targets」をクリック

    ![prometheus004](images/prometheus/prometheus004.png)

1. すべての「State」が「UP」になっていることを確認

    ![prometheus005](images/prometheus/prometheus005.png)

1. ブラウザを終了

1. Ctrl-Cでport-forwardingを終了し、別ターミナル閉じる


## grafanaのパスワード変更とダッシュボード確認

1. 別ターミナルを開き、grafanaのポートフォワーディングを開始

    ```
    $ kubectl --namespace monitoring port-forward $(kubectl get pod --namespace monitoring -l app=grafana,release=po -o template --template "{{(index .items 0).metadata.name}}") 3000:3000
    ```

    - 実行結果（例）

        ```
        Forwarding from 127.0.0.1:3000 -> 3000
        Forwarding from [::1]:3000 -> 3000
        ```

1. ブラウザでgrafanaにアクセス

    ```
    $ openbrowser http://localhost:3000/login
    ```

1. grafanaのログイン画面が表示されたことを確認

    ![grafana001](images/grafana/grafana001.png)

1. 「email or username」に「admin」、「password」に「prom-operator」を入力し「Log In」をクリック

    ![grafana002](images/grafana/grafana002.png)

1. 左下の「Preferences」より、adminユーザのプロファイル画面を表示する

    ![grafana003](images/grafana/grafana003.png)
    ![grafana004](images/grafana/grafana004.png)

1. 「Change Password」よりパスワードの変更画面を表示し、パスワードを変更する

    ![grafana005](images/grafana/grafana005.png)
    ![grafana006](images/grafana/grafana006.png)

1. ホーム画面に遷移し、左上の「Home」をクリックしてprometheus-operatorがインストールしたダッシュボードのリストを表示する

    ![grafana007](images/grafana/grafana007.png)
    ![grafana008](images/grafana/grafana008.png)

1. 「Kubernetes / Compute Resouces / Cluster」をクリックし、AKSクラスタのリソース使用状況が表示されることを確認する

    ![grafana009](images/grafana/grafana009.png)

1. ブラウザを終了

1. Ctrl-Cでport-forwardingを終了し、別ターミナル閉じる


## Elasticsearchの設定

1. elasticsearch-minikube-serviceの作成

    ```
    $ kubectl apply -f logging/elasticsearch-minikube-service.yaml
    ```

    - 実行結果（例）

        ```
        service/elasticsearch-logging created
        ```

1. elasticsearch-minikube-deploymentの作成

    ```
    $ kubectl apply -f logging/elasticsearch-minikube-deployment.yaml
    ```

    - 実行結果（例）

        ```
        serviceaccount/elasticsearch-logging created
        clusterrole.rbac.authorization.k8s.io/elasticsearch-logging created
        clusterrolebinding.rbac.authorization.k8s.io/elasticsearch-logging created
        statefulset.apps/elasticsearch-logging created
        ```

1. elasticsearch-loggingのstatefulsets状態確認

    ```
    $ kubectl get statefulsets --namespace monitoring -l k8s-app=elasticsearch-logging
    ```

    - 実行結果（例）

        ```
        NAME                    READY   AGE
        elasticsearch-logging   2/2     114s
        ```

1. elasticsearch-loggingのpods状態確認

    ```
    $ kubectl get pods --namespace monitoring -l k8s-app=elasticsearch-logging
    ```

    - 実行結果（例）

        ```
        NAME                      READY   STATUS    RESTARTS   AGE
        elasticsearch-logging-0   1/1     Running   0          3m49s
        elasticsearch-logging-1   1/1     Running   0          59s
        ```

1. elasticsearch-loggingのservices状態確認

    ```
    $ kubectl get services --namespace monitoring -l k8s-app=elasticsearch-logging
    ```

    - 実行結果（例）

        ```
        NAME                    TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)    AGE
        elasticsearch-logging   ClusterIP   10.96.35.140   <none>        9200/TCP   4m23s
        ```

1. elasticsearchのクラスタ設定の変更

    ```
    $ kubectl exec -it elasticsearch-logging-0 --namespace monitoring -- curl -H "Content-Type: application/json" -X PUT http://elasticsearch-logging:9200/_cluster/settings -d '{"transient": {"cluster.routing.allocation.enable":"all"}}'
    ```

    - 実行結果（例）

        ```
        {"acknowledged":true,"persistent":{},"transient":{"cluster":{"routing":{"allocation":{"enable":"all"}}}}}
        ```


## fluentdの設定

1. fluentd-es-configmapの作成

    ```
    $ kubectl apply -f logging/fluentd-es-configmap.yaml
    ```

    - 実行結果（例）

        ```
        configmap/fluentd-es-config-v0.2.0 created
        ```

1. fluentd-es-dsの作成

    ```
    $ kubectl apply -f logging/fluentd-es-ds.yaml
    ```

    - 実行結果（例）

        ```
        serviceaccount/fluentd-es created
        clusterrole.rbac.authorization.k8s.io/fluentd-es created
        clusterrolebinding.rbac.authorization.k8s.io/fluentd-es created
        daemonset.apps/fluentd-es-v2.4.0 created
        ```

1. fluentd-esのdaemonsets状態確認

    ```
    $ kubectl get daemonsets --namespace monitoring -l k8s-app=fluentd-es
    ```

    - 実行結果（例）

        ```
        NAME                DESIRED   CURRENT   READY   UP-TO-DATE   AVAILABLE   NODE SELECTOR   AGE
        fluentd-es-v2.4.0   1         1         1       1            1           <none>          1d
        ```

1. fluentd-esのpods状態確認

    ```
    $ kubectl get pods --namespace monitoring -l k8s-app=fluentd-es
    ```

    - 実行結果（例）

        ```
        NAME                      READY   STATUS    RESTARTS   AGE
        fluentd-es-v2.4.0-p28rf   1/1     Running   0          69s
        ```


## kibanaの設定

1. kibana-serviceの作成

    ```
    $ kubectl apply -f logging/kibana-service.yaml
    ```

    - 実行結果（例）

        ```
        service/kibana-logging created
        ```

1. kibana-deploymentの作成

    ```
    $ kubectl apply -f logging/kibana-deployment.yaml
    ```

    - 実行結果（例）

        ```
        deployment.apps/kibana-logging created
        ```

1. kubana-loggingのpods状態確認

    ```
    $ kubectl get pods --namespace monitoring -l k8s-app=kibana-logging
    ```

    - 実行結果（例）

        ```
        NAME                              READY   STATUS    RESTARTS   AGE
        kibana-logging-76ff7dbb49-lc46n   1/1     Running   0          2m20s
        ```


## curatorの設定

1. curator-configmapの作成

    ```
    $ kubectl apply -f logging/curator-configmap.yaml
    ```

    - 実行結果（例）

        ```
        configmap/curator-config created
        ```

1. curator-cronjobの作成

    ```
    $ kubectl apply -f logging/curator-cronjob.yaml
    ```

    - 実行結果（例）

        ```
        cronjob.batch/elasticsearch-curator created
        ```

1. elasticsearch-curatorのcronjobs状態確認

    ```
    $ kubectl get cronjobs --namespace monitoring -l k8s-app=elasticsearch-curator
    ```

    - 実行結果（例）

        ```
        NAME                    SCHEDULE     SUSPEND   ACTIVE   LAST SCHEDULE   AGE
        elasticsearch-curator   0 18 * * *   False     0        <none>          12s
        ```

## KibanaにIndex Patternsの設定

1. 別ターミナルでKibanaのポートフォワーディングを開始

    ```
    $ kubectl --namespace monitoring port-forward $(kubectl get pod -l k8s-app=kibana-logging --namespace monitoring -o template --template "{{(index .items 0).metadata.name}}") 5601:5601
    ```

    - 実行結果（例）

        ```
        Forwarding from 127.0.0.1:5601 -> 5601
        Forwarding from [::1]:5601 -> 5601
        ```

1. ブラウザでkibanaにアクセス

    ```
    $ openbrowser http://localhost:5601/
    ```

1. KibanaのWEB管理画面が表示されたことを確認

    ![kibana001](images/kibana/kibana001.png)

1. 「Management」をクリック

    ![kibana002](images/kibana/kibana002.png)

1. 「Index Patterns」をクリック

    ![kibana003](images/kibana/kibana003.png)

1. 「Index pattern」のテキストボックスに「logstash-*」を入力

    ![kibana004](images/kibana/kibana004.png)

1. 「Success!」のメッセージが表示されたことを確認し「Next step」をクリック

    ![kibana005](images/kibana/kibana005.png)

1. 「Time Filter field name」のテキストボックスで「@timestamp」を選択し「Create Index pattern」をクリック

    ![kibana006](images/kibana/kibana006.png)

1. 「logstash-*」が作成されたことを確認

    ![kibana007](images/kibana/kibana007.png)

1. 「Discover」をクリックすると、Kubernetesやコンテナのログが表示される

    ![kibana008](images/kibana/kibana008.png)

1. ブラウザを終了

1. Ctrl-Cでport-forwardingを終了し、別ターミナル閉じる


## grafanaにelasticsearch dashboardの追加

1. 別ターミナルでgrafanaのポートフォワーディングを開始

    ```
    $ kubectl --namespace monitoring port-forward $(kubectl get pod --namespace monitoring -l app=grafana,release=po -o template --template "{{(index .items 0).metadata.name}}") 3000:3000
    ```

    - 実行結果（例）

        ```
        Forwarding from 127.0.0.1:3000 -> 3000
        Forwarding from [::1]:3000 -> 3000
        ```

1. ブラウザでgrafanaにアクセス

    ```
    $ openbrowser http://localhost:3000/login
    ```

1. grafanaのWEB管理画面が表示されたことを確認

    ![grafana2-001](images/grafana2/grafana2-001.png)

1. 「歯車」「Data Sources」をクリック

    ![grafana2-002](images/grafana2/grafana2-002.png)

1. 「Add data source」をクリック

    ![grafana2-003](images/grafana2/grafana2-003.png)

1. 「ElasticSearch」をクリック

    ![grafana2-004](images/grafana2/grafana2-004.png)

1. 下記の設定値を入力し、「Save & Test」をクリック

    Name : elasticsearch  
    URL : http://elasticsearch-logging:9200  
    Access : Server(Default)  
    Index name : logstash-*  
    Time field name : @timestamp  
    Version : 6.0+

    ![grafana2-005](images/grafana2/grafana2-005.png)

1. 「Datasource Updated」と表示されることを確認

    ![grafana2-006](images/grafana2/grafana2-006.png)

1. 「+」「Import」をクリック

    ![grafana2-007](images/grafana2/grafana2-007.png)

1. 「Upload .json File」をクリック

    ![grafana2-008](images/grafana2/grafana2-008.png)

1. 「monitoring/dashboard\_elasticsearch.json」を選択し「開く」をクリック

    ![grafana2-009](images/grafana2/grafana2-009.png)

1. 「Import」をクリック

    ![grafana2-010](images/grafana2/grafana2-010.png)

1. ElasticSeartchのログ件数のグラフとログ内容のテーブルが表示されることを確認

    ![grafana2-011](images/grafana2/grafana2-011.png)

1. ブラウザを終了

1. Ctrl-Cでport-forwardingを終了し、別ターミナル閉じる
