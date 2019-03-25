# RoboticBase Coreインストールマニュアル #2

## 構築環境(2019年3月6日現在)
- mosquitto-clients 1.4.8-1ubuntu0.16.04.6
- mongodb v4.0.6
- jq 1.5+dfsg-1ubuntu0.1

# 2. minikubeでpodsの開始


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
    $ source ${CORE_ROOT}/docs/minikube/env
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

1. rabbitmqのcluster_status状態確認

    ```
    kubectl exec rabbitmq-0 -- rabbitmqctl cluster_status
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
    $ kubectl exec rabbitmq-0 -- rabbitmqctl change_password guest $(cat /dev/urandom 2>/dev/null | head -n 40 | tr -cd 'a-zA-Z0-9' | head -c 32)
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

1. RabbitMQのlist_users状態確認

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


## MQTTブローカーの設定

1. mosquitto-clientsのインストール

    ```
    $ sudo apt-get install -y mosquitto-clients
    ```

1. mosquitto-clientsのインストール確認

    ```
    $ dpkg -l | grep  mosquitto-clients
    ```

    - 実行結果（例）

        ```
        ii  mosquitto-clients                          1.4.8-1ubuntu0.16.04.6                              amd64        Mosquitto command line MQTT clients
        ```

1. MQTTブローカーの接続確認

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

1. mongodbのPersistentVolumeClaims状態確認

    ```
    $ kubectl get PersistentVolumeClaims -l release=mongodb -l app=mongodb-replicaset
    ```

    - 実行結果（例）

        ```
        No resources found.
        ```

1. mongodbのstatefulsets状態確認

    ```
    $  kubectl get statefulsets -l release=mongodb -l app=mongodb-replicaset
    ```

    - 実行結果（例）

        ```
        NAME      DESIRED   CURRENT   AGE
        mongodb   3         3         3m9s
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
        printjson(rs.status().members.map(function(e) {return {name: e.name, stateStr:e.stateStr};}))'
        MongoDB shell version v4.0.6
        connecting to: mongodb://127.0.0.1:27017/?gssapiServiceName=mongodb
        Implicit session: session { "id" : UUID("2c9140b7-fb2b-4027-96d6-e0b27aee756e") }
        MongoDB server version: 4.0.6
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

1. secrets/auth-tokens.jsonの作成

    ```
    cat << __EOS__ > secrets/auth-tokens.json
    [
        {
            "host": ".*",
            "settings": {
                "bearer_tokens": [
                    {
                        "token": "$(cat /dev/urandom 2>/dev/null | head -n 40 | tr -cd 'a-zA-Z0-9' | head -c 32)",
                        "allowed_paths": ["^/orion/.*$", "^/idas/.*$"]
                    }, {
                        "token": "$(cat /dev/urandom 2>/dev/null | head -n 40 | tr -cd 'a-zA-Z0-9' | head -c 32)",
                        "allowed_paths": ["^/visualizer/positions/$"]
                    }
                ],
                "basic_auths": [
                    {
                        "username": "user1",
                        "password": "$(cat /dev/urandom 2>/dev/null | head -n 40 | tr -cd 'a-zA-Z0-9' | head -c 16)",
                        "allowed_paths": ["/controller/web/"]
                    }, {
                        "username": "visualizer",
                        "password": "$(cat /dev/urandom 2>/dev/null | head -n 40 | tr -cd 'a-zA-Z0-9' | head -c 16)",
                        "allowed_paths": ["/visualizer/locus/"]
                    }
                ],
                "no_auths": {
                    "allowed_paths": ["^.*/static/.*$"]
                }
            }
        }
    ]
    __EOS__
    $ cat secrets/auth-tokens.json
    ```

    - 実行結果（例）

        ```
        [
            {
                "host": ".*",
                "settings": {
                    "bearer_tokens": [
                        {
                            "token": "5Z9KpEAE5z3XR7ZsV5cGGeefZUOJFLv0",
                            "allowed_paths": ["^/orion/.*$", "^/idas/.*$"]
                        }, {
                            "token": "u6vuTzRay9XbeXgfgK5ncfL5aQrzoY1t",
                            "allowed_paths": ["^/visualizer/positions/$"]
                        }
                    ],
                    "basic_auths": [
                        {
                            "username": "user1",
                            "password": "Tat7WDFTv86117Vn",
                            "allowed_paths": ["/controller/web/"]
                        }, {
                            "username": "visualizer",
                            "password": "RTDdlnvp68jhKcuT",
                            "allowed_paths": ["/visualizer/locus/"]
                        }
                    ],
                    "no_auths": {
                        "allowed_paths": ["^.*/static/.*$"]
                    }
                }
            }
        ]
        ```

1. secrets/auth-tokens.jsonの登録

    ```
    $ kubectl create secret generic auth-tokens --from-file=./secrets/auth-tokens.json
    ```

    - 実行結果（例）

        ```
        secret/auth-tokens created
        ```

1. auth-serviceの作成

    ```
    $ kubectl apply -f auth/auth-service.yaml
    ```

    - 実行結果（例）

        ```
        service/auth created
        ```

1. auth-deploymentの作成

    ```
    $ kubectl apply -f auth/auth-deployment.yaml
    ```

    - 実行結果（例）

        ```
        deployment.apps/auth created
        ```

1. authのpods状態確認

    ```
    $ kubectl get pods -l app=auth
    ```

    - 実行結果（例）

        ```
        NAME                    READY   STATUS    RESTARTS   AGE
        auth-67ff6bd94b-nqjjd   1/1     Running   0          44s
        auth-67ff6bd94b-wxwq4   1/1     Running   0          44s
        auth-67ff6bd94b-z8jkj   1/1     Running   0          44s
        ```

1. authのservices状態確認

    ```
    $ kubectl get services -l app=auth
    ```

    - 実行結果（例）

        ```
        NAME   TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)    AGE
        auth   ClusterIP   10.102.160.70   <none>        3000/TCP   68s
        ```

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
        clusterrole.rbac.authorization.k8s.io/ambassador created
        serviceaccount/ambassador created
        clusterrolebinding.rbac.authorization.k8s.io/ambassador created
        deployment.apps/ambassador created
        ```

1. ambassadorのpods状態確認

    ```
    $ kubectl get pods -l app=ambassador
    ```

    - 実行結果（例）

        ```
        NAME                          READY   STATUS    RESTARTS   AGE
        ambassador-69dcd7cb7c-2xhns   2/2     Running   0          2m
        ambassador-69dcd7cb7c-7w6w8   2/2     Running   0          2m
        ambassador-69dcd7cb7c-gskqv   2/2     Running   0          2m
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


## minikubeにfiware orionの設定

1. orion-minikube-serviceの作成

    ```
    $ kubectl apply -f orion/orion-minikube-service.yaml
    ```

    - 実行結果（例）

        ```
        service/orion created
        ```

1. orion-deploymentの作成

    ```
    $ kubectl apply -f orion/orion-deployment.yaml
    ```

    - 実行結果（例）

        ```
        deployment.apps/orion created
        ```

1. orionのpods状態確認

    ```
    $ kubectl get pods -l app=orion
    ```

    - 実行結果（例）

        ```
        NAME                     READY   STATUS    RESTARTS   AGE
        orion-584b686499-6wp4l   1/1     Running   0          92s
        orion-584b686499-7x778   1/1     Running   0          92s
        orion-584b686499-lcmpp   1/1     Running   0          92s
        ```

1. orionのservices確認

    ```
    $ kubectl get services -l app=orion
    ```

    - 実行結果（例）

        ```
        NAME    TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)    AGE
        orion   ClusterIP   10.106.7.244   <none>        1026/TCP   2m31s
        ```

1. orionの接続確認

    ```
    $ curl -i http://${HOST_IPADDR}:8080/orion/v2/entities/
    ```

    - 実行結果（例）

        ```
        HTTP/1.1 401 Unauthorized
        content-length: 60
        content-type: text/plain
        www-authenticate: Bearer realm="token_required"
        date: Wed, 06 Mar 2019 04:29:17 GMT
        server: envoy

        {"authorized":false,"error":"missing Header: authorization"}
        ```

1. jqのインストール

    ```
    $ sudo apt-get install -y jq
    ```

1. jqのインストール確認

    ```
    $ dpkg -l | grep jq
    ```

    - 実行結果（例）

        ```
        ii  jq                                         1.5+dfsg-1ubuntu0.1                                 amd64        lightweight and flexible command-line JSON processor
        fiware@roboticbase-pc:~/core$
        ```

1. secrets/auth-tokensを利用したorion接続確認

    ```
    $ TOKEN=$(cat ${CORE_ROOT}/secrets/auth-tokens.json | jq '.[0].settings.bearer_tokens[0].token' -r)
    $ curl -i -H "Authorization: bearer ${TOKEN}" http://${HOST_IPADDR}:8080/orion/v2/entities/
    ```

    - 実行結果（例）

        ```
        HTTP/1.1 200 OK
        content-length: 2
        content-type: application/json
        fiware-correlator: 55aa070a-3fc9-11e9-8ab3-0242ac110012
        date: Wed, 06 Mar 2019 04:35:56 GMT
        x-envoy-upstream-service-time: 3
        server: envoy

        []
        ```

    ※200以外のコードが出力された場合は、以下のコマンドを実行し、Ambassador全てのPodを再起動してください  

    ```
    kubectl delete pods -l app=ambassador
    ```

## minikubeにfiware idasの設定

1. iotagent-configのインストール

    ```
    $ env IOTA_PASSWORD=${MQTT__iotagent} envsubst < idas/rb-config.js > /tmp/rb-config.js
    $ kubectl create secret generic iotagent-config --from-file /tmp/rb-config.js
    $ rm /tmp/rb-config.js
    ```

    - 実行結果（例）

        ```
        secret/iotagent-config created
        ```

1. iotagent-ul-minikube-serviceの作成

    ```
    $ kubectl apply -f idas/iotagent-ul-minikube-service.yaml
    ```

    - 実行結果（例）

        ```
        service/iotagent-ul created
        ```

1. iotagent-ul-deploymentの作成

    ```
    $ kubectl apply -f idas/iotagent-ul-deployment.yaml
    ```

    - 実行結果（例）

        ```
        deployment.apps/iotagent-ul created
        ```

1. iotagent-ulのpods状態確認

    ```
    $ kubectl get pods -l app=iotagent-ul
    ```

    - 実行結果（例）

        ```
        NAME                           READY   STATUS    RESTARTS   AGE
        iotagent-ul-659d5f8598-6fxkc   1/1     Running   0          5m33s
        iotagent-ul-659d5f8598-grz6f   1/1     Running   0          5m33s
        iotagent-ul-659d5f8598-tqwtm   1/1     Running   0          5m33s
        ```

1. iotagent-ulのsevivces状態確認

    ```
    $ kubectl get services -l app=iotagent-ul
    ```

    - 実行結果（例）

        ```
        NAME          TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)             AGE
        iotagent-ul   ClusterIP   10.102.35.241   <none>        4041/TCP,7896/TCP   3m
        ```

1. idasにsecrets/auth-tokensを利用した接続確認

    ```
    $ TOKEN=$(cat ${CORE_ROOT}/secrets/auth-tokens.json | jq '.[0].settings.bearer_tokens[0].token' -r)
    $ curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: test" -H "Fiware-Servicepath: /*" http://${HOST_IPADDR}:8080/idas/ul20/manage/iot/services/
    ```

    - 実行結果（例）

        ```
        HTTP/1.1 200 OK
            content-length: 2
            content-type: application/json
            fiware-correlator: 55aa070a-3fc9-11e9-8ab3-0242ac110012
            date: Wed, 06 Mar 2019 04:35:56 GMT
            x-envoy-upstream-service-time: 3
            server: envoy

            []
        ```

## minikubeにfiware cygnusの設定

1. cygnus-mongoの作成

    ```
    $ kubectl apply -f cygnus/cygnus-mongo-service.yaml
    ```

    - 実行結果（例）

        ```
        service/cygnus-mongo created
        ```

1. cygnus-mongo-deploymentの作成

    ```
    $ kubectl apply -f cygnus/cygnus-mongo-deployment.yaml
    ```

    - 実行結果（例）

        ```
        deployment.apps/cygnus-mongo created
        ```

1. cygnus-mongoのpods状態確認

    ```
    $ kubectl get pods -l app=cygnus-mongo
    ```

    - 実行結果（例）

        ```
        NAME                           READY   STATUS    RESTARTS   AGE
        cygnus-mongo-ff75ccbfb-4d9h2   1/1     Running   0          2m45s
        cygnus-mongo-ff75ccbfb-q8tql   1/1     Running   0          2m44s
        cygnus-mongo-ff75ccbfb-w7ml5   1/1     Running   0          2m44s
        ```

1. cygnus-mongoのservices状態確認

    ```
    $ kubectl get services -l app=cygnus-mongo
    ```

    - 実行結果（例）

        ```
        NAME           TYPE        CLUSTER-IP    EXTERNAL-IP   PORT(S)             AGE
        cygnus-mongo   ClusterIP   10.99.98.62   <none>        5050/TCP,8081/TCP   2m26s
        ```






