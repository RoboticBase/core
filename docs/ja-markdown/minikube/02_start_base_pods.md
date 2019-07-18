# RoboticBase Coreインストールマニュアル #2

## 構築環境(2019年7月18日現在)
# minikubeでAPI GatewayやMessage Broker、KeyValue DBのpodsを開始


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
      alias randomstr32='cat /dev/urandom | LC_CTYPE=C tr -dc 'a-zA-Z0-9' | head -c 32'
    elif [ "$(expr substr $(uname -s) 1 5)" == 'Linux' ]; then
      alias randomstr32='cat /dev/urandom 2>/dev/null | head -n 40 | tr -cd 'a-zA-Z0-9' | head -c 32'
    else
      echo "Your platform ($(uname -a)) is not supported."
      exit 1
    fi
    ```

## minikubeにRabbitMQの設定

1. rabbitmq-rbacの作成

    ```
    $ kubectl apply -f rabbitmq/rabbitmq-rbac.yaml
    ```

    - 実行結果（例）

        ```
        serviceaccount/rabbitmq created
        role.rbac.authorization.k8s.io/endpoint-reader created
        rolebinding.rbac.authorization.k8s.io/endpoint-reader created
        ```

1. rabbitmq-minikube-servicesの作成

    ```
    $ kubectl apply -f rabbitmq/rabbitmq-minikube-services.yaml
    ```

    - 実行結果（例）

        ```
        service/rabbitmq-amqp created
        service/rabbitmq-mqtt created
        service/rabbitmq created
        ```

1. rabbitmq-minikube-statefulsetの作成

    ```
    $ kubectl apply -f rabbitmq/rabbitmq-minikube-statefulset.yaml
    ```

    - 実行結果（例）

        ```
        configmap/rabbitmq-config created
        statefulset.apps/rabbitmq created
        ```

1. rabbitmqのpods状態確認

    ```
    $ kubectl get pods -l app=rabbitmq
    ```

    - 実行結果（例）

        ```
        NAME         READY   STATUS    RESTARTS   AGE
        rabbitmq-0   1/1     Running   0          3m
        rabbitmq-1   1/1     Running   0          2m
        rabbitmq-2   1/1     Running   0          1m
        ```

1. rabbitmqのクラスタ状態確認

    ```
    $ kubectl exec rabbitmq-0 -- rabbitmqctl cluster_status
    ```

    - 実行結果（例）

        ```
        Cluster status of node rabbit@rabbitmq-0.rabbitmq.default.svc.cluster.local ...
        [{nodes,[{disc,['rabbit@rabbitmq-0.rabbitmq.default.svc.cluster.local',
                        'rabbit@rabbitmq-1.rabbitmq.default.svc.cluster.local',
                        'rabbit@rabbitmq-2.rabbitmq.default.svc.cluster.local']}]},
        {running_nodes,['rabbit@rabbitmq-2.rabbitmq.default.svc.cluster.local',
                        'rabbit@rabbitmq-1.rabbitmq.default.svc.cluster.local',
                        'rabbit@rabbitmq-0.rabbitmq.default.svc.cluster.local']},
        {cluster_name,<<"rabbit@rabbitmq-0.rabbitmq.default.svc.cluster.local">>},
        {partitions,[]},
        {alarms,[{'rabbit@rabbitmq-2.rabbitmq.default.svc.cluster.local',[]},
                {'rabbit@rabbitmq-1.rabbitmq.default.svc.cluster.local',[]},
                {'rabbit@rabbitmq-0.rabbitmq.default.svc.cluster.local',[]}]}]
        ```


## ゲストパスワードの変更
1. ゲストパスワードの変更

    ```
    $ kubectl exec rabbitmq-0 -- rabbitmqctl change_password guest $(randomstr32)
    ```

    - 実行結果（例）

        ```
        Changing password for user "guest" ...
        ```

## RabbitMQのユーザ登録

1. RabbitMQのユーザ登録

    ```
    $ for e in $(env); do
    if [[ "${e}" =~ ^MQTT__([[:alnum:]_-]+)=([[:alnum:]_-]+)$ ]]; then
        username=${BASH_REMATCH[1]}
        password=${BASH_REMATCH[2]}
        
        kubectl exec rabbitmq-0 -- rabbitmqctl add_user ${username} ${password}
        kubectl exec rabbitmq-0 -- rabbitmqctl set_permissions -p / ${username} ".*" ".*" ".*"
    fi
    done
    ```

    - 実行結果（例）

        ```
        Adding user "iotagent" ...
        Setting permissions for user "iotagent" in vhost "/" ...
        ```

1. RabbitMQのユーザ状態確認

    ```
    $ kubectl exec rabbitmq-0 -- rabbitmqctl list_users
    ```

    - 実行結果（例）

        ```
        Listing users ...
        user    tags
        guest   [administrator]
        iotagent        []
        ```

1. RabbitMQのservices状態確認

    ```
    $ kubectl get services -l app=rabbitmq
    ```

    - 実行結果（例）

        ```
        NAME            TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)              AGE
        rabbitmq        ClusterIP   None            <none>        5672/TCP             7m3s
        rabbitmq-amqp   ClusterIP   10.110.239.41   <none>        15672/TCP,5672/TCP   7m4s
        rabbitmq-mqtt   NodePort    10.106.119.89   <none>        1883:30376/TCP       7m4s
        ```


## RabbitMQのポート確認

1. RabbitMQのポート確認

    ```
    $ export MQTT_PORT=$(kubectl describe service rabbitmq-mqtt | grep "NodePort:" | awk '{print $3}' | awk -F'/' '{print $1}');echo ${MQTT_PORT}
    ```

    - 実行結果（例）

        ```
        30376
        ```

1. VirtualBoxnのNAT設定

    ```
    $ VBoxManage controlvm "${MINIKUBE_NAME}" natpf1 "mqtt,tcp,0.0.0.0,1883,,${MQTT_PORT}"
    $ VBoxManage showvminfo "${MINIKUBE_NAME}"| grep ${MQTT_PORT}
    ```

    - 実行結果（例）

        ```
        NIC 1 Rule(0):   name = mqtt, protocol = tcp, host ip = 0.0.0.0, host port = 1883, guest ip = , guest port = 30376
        ```


## MQTTの疎通確認
1. mqttの疎通確認

    ```
    $ mosquitto_pub -h ${HOST_IPADDR} -p 1883  -d -u iotagent -P ${MQTT__iotagent} -t /test -m "test"
    ```

    - 実行結果（例）

        ```
        Client mosqpub/25312-ros-termin sending CONNECT
        Client mosqpub/25312-ros-termin received CONNACK
        Client mosqpub/25312-ros-termin sending PUBLISH (d0, q0, r0, m1, '/test', ... (4 bytes))
        Client mosqpub/25312-ros-termin sending DISCONNECT
        ```


## minikubeにmongodbの設定

1. mongodbのインストール

    ```
    $ helm install --name mongodb -f mongodb/mongodb-replicaset-values-minikube.yaml stable/mongodb-replicaset
    ```

    - 実行結果（例）

        ```
        NAME:   mongodb
        LAST DEPLOYED: Wed Mar  6 11:32:17 2019
        NAMESPACE: default
        STATUS: DEPLOYED

        RESOURCES:
        ==> v1/ConfigMap
        NAME             DATA  AGE
        mongodb-init     1     2s
        mongodb-mongodb  1     2s
        mongodb-tests    1     2s

        ==> v1/Service
        NAME            TYPE       CLUSTER-IP  EXTERNAL-IP  PORT(S)    AGE
        mongodb-client  ClusterIP  None        <none>       27017/TCP  2s
        mongodb         ClusterIP  None        <none>       27017/TCP  1s

        ==> v1/StatefulSet
        NAME     DESIRED  CURRENT  AGE
        mongodb  3        1        1s

        ==> v1/Pod(related)
        NAME       READY  STATUS   RESTARTS  AGE
        mongodb-0  0/1    Pending  0         1s

        NOTES:
        1. After the statefulset is created completely, one can check which instance is primary by running:

            $ for ((i = 0; i < 3; ++i)); do kubectl exec --namespace default mongodb-$i -- sh -c 'mongo --eval="printjson(rs.isMaster())"'; done

        2. One can insert a key into the primary instance of the mongodb replica set by running the following:
            MASTER_POD_NAME must be replaced with the name of the master found from the previous step.

            $ kubectl exec --namespace default MASTER_POD_NAME -- mongo --eval="printjson(db.test.insert({key1: 'value1'}))"

        3. One can fetch the keys stored in the primary or any of the slave nodes in the following manner.
            POD_NAME must be replaced by the name of the pod being queried.

            $ kubectl exec --namespace default POD_NAME -- mongo --eval="rs.slaveOk(); db.test.find().forEach(printjson)"
        ```

1. mongodbのstatefulsets状態確認

    ```
    $  kubectl get statefulsets -l release=mongodb -l app=mongodb-replicaset
    ```

    - 実行結果（例）

        ```
        NAME      READY   AGE
        mongodb   3/3     3m1s
        ```

1. mongodbのpods状態確認

    ```
    $ kubectl get pods -l release=mongodb -l app=mongodb-replicaset
    ```

    - 実行結果（例）

        ```
        NAME        READY   STATUS    RESTARTS   AGE
        mongodb-0   1/1     Running   0          4m47s
        mongodb-1   1/1     Running   0          2m40s
        mongodb-2   1/1     Running   0          111s
        ```

1. mongodbのservices状態確認

    ```
    $ kubectl get services -l release=mongodb -l app=mongodb-replicaset
    ```

    - 実行結果（例）

        ```
        NAME             TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)     AGE
        mongodb          ClusterIP   None         <none>        27017/TCP   4m57s
        mongodb-client   ClusterIP   None         <none>        27017/TCP   4m58s
        ```

1. mongodbの状態確認

    ```
    $ kubectl exec mongodb-0 -c mongodb-replicaset -- mongo --eval 'printjson(rs.status().members.map(function(e) {return {name: e.name, stateStr:e.stateStr};}))'
    ```

    - 実行結果（例）

        ```
        MongoDB shell version v4.1.13
        connecting to: mongodb://127.0.0.1:27017/?compressors=disabled&gssapiServiceName=mongodb
        Implicit session: session { "id" : UUID("21eaa109-63dd-4b65-88b4-e6ba9f8bb338") }
        MongoDB server version: 4.1.13
        [
            {
                "name" : "mongodb-0.mongodb.default.svc.cluster.local:27017",
                "stateStr" : "PRIMARY"
            },
            {
                "name" : "mongodb-1.mongodb.default.svc.cluster.local:27017",
                "stateStr" : "SECONDARY"
            },
            {
                "name" : "mongodb-2.mongodb.default.svc.cluster.local:27017",
                "stateStr" : "SECONDARY"
            }
        ]
        ```

## minikubeのambassador設定

1. minikubeにambassador-minikube-serviceの作成

    ```
    $ kubectl apply -f ambassador/ambassador-minikube-service.yaml
    ```

    - 実行結果（例）

        ```
        service/ambassador created
        ```

1. minikubeにambassador-deploymentの作成

    ```
    $ kubectl apply -f ambassador/ambassador-deployment.yaml
    ```

    - 実行結果（例）

        ```
        service/ambassador-admin created
        clusterrole.rbac.authorization.k8s.io/ambassador created
        serviceaccount/ambassador created
        clusterrolebinding.rbac.authorization.k8s.io/ambassador created
        customresourcedefinition.apiextensions.k8s.io/authservices.getambassador.io created
        customresourcedefinition.apiextensions.k8s.io/consulresolvers.getambassador.io created
        customresourcedefinition.apiextensions.k8s.io/kubernetesendpointresolvers.getambassador.io created
        customresourcedefinition.apiextensions.k8s.io/kubernetesserviceresolvers.getambassador.io created
        customresourcedefinition.apiextensions.k8s.io/mappings.getambassador.io created
        customresourcedefinition.apiextensions.k8s.io/modules.getambassador.io created
        customresourcedefinition.apiextensions.k8s.io/ratelimitservices.getambassador.io created
        customresourcedefinition.apiextensions.k8s.io/tcpmappings.getambassador.io created
        customresourcedefinition.apiextensions.k8s.io/tlscontexts.getambassador.io created
        customresourcedefinition.apiextensions.k8s.io/tracingservices.getambassador.io created
        deployment.apps/ambassador created
        ```

1. ambassadorのpods状態確認

    ```
    $ kubectl get pods -l app=ambassador
    ```

    - 実行結果（例）

        ```
        NAME                          READY   STATUS    RESTARTS   AGE
        ambassador-69dcd7cb7c-2xhns   1/1     Running   0          2m
        ambassador-69dcd7cb7c-7w6w8   1/1     Running   0          2m
        ambassador-69dcd7cb7c-gskqv   1/1     Running   0          2m
        ```

1. ambassadorのservices状態確認

    ```
    $ kubectl get services -l app=ambassador
    ```

    - 実行結果（例）

        ```
        NAME         TYPE       CLUSTER-IP     EXTERNAL-IP   PORT(S)        AGE
        ambassador   NodePort   10.96.42.125   <none>        80:32053/TCP   2m41s
        ```

## VirtualBoxのNAT設定

1. ambassadorのポート確認

    ```
    $ HTTP_PORT=$(kubectl describe service ambassador | grep "NodePort:" | awk '{print $3}' | awk -F'/' '{print $1}');echo ${HTTP_PORT}
    ```

    - 実行結果（例）

        ```
        32053
        ```

1. VirtualBoxのNAT設定

    ```
    $ VBoxManage controlvm "${MINIKUBE_NAME}" natpf1 "http,tcp,0.0.0.0,8080,,${HTTP_PORT}"
    $ VBoxManage showvminfo "${MINIKUBE_NAME}" | grep ${HTTP_PORT}
    ```

    - 実行結果（例）

        ```
        NIC 1 Rule(0):   name = http, protocol = tcp, host ip = 0.0.0.0, host port = 8080, guest ip = , guest port = 32053
        ```

1. VirtualBoxのNAT接続確認

    ```
    $ curl -i http://${HOST_IPADDR}:8080
    ```

    - 実行結果（例）

        ```
        HTTP/1.1 404 Not Found
        date: Wed, 06 Mar 2019 04:22:04 GMT
        server: envoy
        content-length: 0
        ```
