# RoboticBase Coreインストールマニュアル #2

## 構築環境(2019年2月20日現在)

- docker-ce 18.09.3
- docker-ce-cli 18.09.3
- ca-certificates 20170717~16.0  
- mosquitto 1.5.7-0mosquitto1~xenial1  
- mosquitto-clients 1.5.7-0mosquitto1~xenial1

# Azure AKSでpodsの開始


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
    $ source ${CORE_ROOT}/docs/azure_aks/env
    ```

## Azureにログイン

1. テナントIDを指定してのAKSログイン

    ```
    $ az login --tenant ${TENANT}
    ```

    - 実行結果（例）

        ```
        fiware@FIWARE-PC:/tmp$ az login --tenant ${TENANT}
        To sign in, use a web browser to open the page https://microsoft.com/devicelogin and enter the code GVG5YS2HA to authenticate.
        [
        {
            "cloudName": "AzureCloud",
            "id": "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx",
            "isDefault": true,
            "name": "Microsoft Azure",
            "state": "Enabled",
            "tenantId": "example.onmicrosoft.com",
            "user": {
            "name": "example@example.com",
            "type": "user"
            }
        }
        ]
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

1. docker-ceリポジトリを登録する

    ```
    $ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
    ```

1. パッケージリストの更新

    ```
    $ sudo apt-get update -y
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

1. dockerサービスの再起動

    ```
    $ sudo systemctl restart docker
    $ exit
    ```

1. dockerサービスの自動起動を設定

    ```
    $ sudo systemctl enable docker
    ```

1. dockerサービスの状態確認

    ```
    $ systemctl status docker
    ```

    - 実行結果（例）

        ```
        ● docker.service - Docker Application Container Engine
        Loaded: loaded (/lib/systemd/system/docker.service; enabled; vendor preset: enabled)
        Active: active (running) since 月 2019-02-18 14:19:18 JST; 16min ago
            Docs: https://docs.docker.com
        Main PID: 9138 (dockerd)
            Tasks: 10
        Memory: 30.9M
            CPU: 537ms
        CGroup: /system.slice/docker.service
                mq9138 /usr/bin/dockerd -H fd://
        ```

## ワイルドカードTLS証明書の作成

1. dockerコンテナ起動用コマンドの作成

    ```
    $ echo "docker run -it -v ${CORE_ROOT}/secrets:/etc/letsencrypt certbot/certbot certonly --manual --domain *.${DOMAIN} --email ${EMAIL} --no-eff-email --agree-tos --manual-public-ip-logging-ok --preferred-challenges dns-01 --server https://acme-v02.api.letsencrypt.org/directory"
    ```

1. Let's encrypt用のTLS証明書作成

    ```
    $ docker run -it -v /home/fiware/core/secrets:/etc/letsencrypt certbot/certbot certonly --manual --domain *.api.example.com --email exampe@example.com --no-eff-email --agree-tos --manual-public-ip-logging-ok --preferred-challenges dns-01 --server https://acme-v02.api.letsencrypt.org/directory
    ```

    - 実行結果（例）

        ```
        Saving debug log to /var/log/letsencrypt/letsencrypt.log
        Plugins selected: Authenticator manual, Installer None
        Obtaining a new certificate
        Performing the following challenges:
        dns-01 challenge for example.com

        - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        Please deploy a DNS TXT record under the name
        _acme-challenge.example.com with the following value:

        XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

        Before continuing, verify the record is deployed.
        - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        Press Enter to Continue
        ```

1. 別ターミナルで表示されたDNS TXT recordを環境変数に設定

    ```
    $ export DNS_TXT="XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"

    ```

1. DNSゾーンにTXTレコードの登録

    ```
    $ az network dns record-set txt add-record --resource-group ${DNS_ZONE_RG} --zone-name "${DOMAIN}" --record-set-name "_acme-challenge" --value "${DNS_TXT}"
    ```

    - 実行結果（例）

        ```
        {
        "etag": "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx",
        "fqdn": "_acme-challenge.example.com.",
        "id": "/subscriptions/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx/resourceGroups/dns-zone/providers/Microsoft.Network/dnszones/example.com/TXT/_acme-challenge",
        "metadata": null,
        "name": "_acme-challenge",
        "provisioningState": "Succeeded",
        "resourceGroup": "dns-zone",
        "targetResource": {
            "id": null
        },
        "ttl": 3600,
        "txtRecords": [
            {
            "value": [
                "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
            ]
            }
        ],
        "type": "Microsoft.Network/dnszones/TXT"
        }
        ```

1. 元ターミナルでEnterキーを押下しLet's encryptの認証が成功したことを確認

    ```
    Waiting for verification...
    Cleaning up challenges

    IMPORTANT NOTES:
    - Congratulations! Your certificate and chain have been saved at:
      /etc/letsencrypt/live/example.com/fullchain.pem
      Your key file has been saved at:
      /etc/letsencrypt/live/example.com/privkey.pem
      Your cert will expire on 2019-05-21. To obtain a new or tweaked
      version of this certificate in the future, simply run certbot
      again. To non-interactively renew *all* of your certificates, run
      "certbot renew"
    - If you like Certbot, please consider supporting our work by:

      Donating to ISRG / Let's Encrypt:   https://letsencrypt.org/donate
      Donating to EFF:                    https://eff.org/donate-le
    ```

1. DNSリソースグループのDNS record削除

    ```
    $ az network dns record-set txt remove-record --resource-group ${DNS_ZONE_RG} --zone-name "${DOMAIN}" --record-set-name "_acme-challenge" --value "${DNS_TXT}"
    ```

1. secretsのユーザーグループ確認

    ```
    $ sudo ls -la secrets/
    ```

    - 実行結果（例）

        ```
        合計 44
        drwxrwxr-x  9 fiware fiware 4096  2月 20 16:32 .
        drwxrwxr-x 15 fiware fiware 4096  2月 18 16:27 ..
        -rw-rw-r--  1 fiware fiware 1200  2月 18 16:26 DST_Root_CA_X3.pem
        drwx------  3 root   root   4096  2月 20 15:13 accounts
        drwx------  3 root   root   4096  2月 20 15:24 archive
        -rw-rw-r--  1 fiware fiware 1163  2月 20 16:32 auth-tokens.json
        drwxr-xr-x  2 root   root   4096  2月 20 15:22 csr
        drwx------  2 root   root   4096  2月 20 15:22 keys
        drwx------  3 root   root   4096  2月 20 15:24 live
        drwxr-xr-x  2 root   root   4096  2月 20 15:24 renewal
        drwxr-xr-x  5 root   root   4096  2月 20 15:13 renewal-hooks
        ```

1. secretsのユーザーグループを現在のユーザに変更

    ```
    $ sudo chown -hR ${USER}:${USER} secrets/
    ```

1. secretsのユーザーグループ確認

    ```
    $ ls -la secrets/
    ```

    - 実行結果（例）

        ```
        合計 44
        drwxrwxr-x  9 fiware fiware 4096  2月 20 16:32 .
        drwxrwxr-x 15 fiware fiware 4096  2月 18 16:27 ..
        -rw-rw-r--  1 fiware fiware 1200  2月 18 16:26 DST_Root_CA_X3.pem
        drwx------  3 fiware fiware 4096  2月 20 15:13 accounts
        drwx------  3 fiware fiware 4096  2月 20 15:24 archive
        -rw-rw-r--  1 fiware fiware 1163  2月 20 16:32 auth-tokens.json
        drwxr-xr-x  2 fiware fiware 4096  2月 20 15:22 csr
        drwx------  2 fiware fiware 4096  2月 20 15:22 keys
        drwx------  3 fiware fiware 4096  2月 20 15:24 live
        drwxr-xr-x  2 fiware fiware 4096  2月 20 15:24 renewal
        drwxr-xr-x  5 fiware fiware 4096  2月 20 15:13 renewal-hooks
        ```


## AKSにRabbitMQの設定

1. secretsに証明書ファイルを登録 

    ```
    $ kubectl create secret generic rabbitmq-certifications --from-file=${CORE_ROOT}/secrets/live/${DOMAIN}/fullchain.pem --from-file=${CORE_ROOT}/secrets/live/${DOMAIN}/cert.pem --from-file=${CORE_ROOT}/secrets/live/${DOMAIN}/privkey.pem
    ```

    - 実行結果（例）

        ```
        secret/rabbitmq-certifications created
        ```

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

1. rabbitmq-azure-servicesの作成

    ```
    $ kubectl apply -f rabbitmq/rabbitmq-azure-services.yaml
    ```

    - 実行結果（例）

        ```
        service/rabbitmq-amqp created
        service/rabbitmq-mqtt created
        service/rabbitmq-mqtts created
        service/rabbitmq created
        ```

1. rabbitmq-azure-statefulsetの作成

    ```
    $ kubectl apply -f rabbitmq/rabbitmq-azure-statefulset.yaml
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
        rabbitmq-0   1/1     Running   0          3h21m
        rabbitmq-1   1/1     Running   0          3h20m
        rabbitmq-2   1/1     Running   0          3h19m
        ```

1. rabbitmqのcluster_status状態確認

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


## グローバルIPをRabbitMQのAレコードに登録

1. rabbitmqのservices状態確認

    ```
    $ kubectl get services -l app=rabbitmq -l service=mqtts
    ```

    - 実行結果（例）

        ```
        NAME             TYPE           CLUSTER-IP     EXTERNAL-IP    PORT(S)          AGE
        rabbitmq-mqtts   LoadBalancer   10.0.226.238   23.102.75.46   8883:31693/TCP   21m
        ```

1. RabbitMQのグローバルIPアドレスを取得

    ```
    $ MQTTS_IPADDR=$(kubectl get services -l app=rabbitmq -l service=mqtts -o jsonpath='{.items[0].status.loadBalancer.ingress[0].ip}')
    ```

1. mqttサブドメインのDNSレコードを追加

    ```
    $ az network dns record-set a add-record --resource-group ${DNS_ZONE_RG} --zone-name "${DOMAIN}" --record-set-name "mqtt" --ipv4-address "${MQTTS_IPADDR}"
    ```

    - 実行結果（例）

        ```
        {
            "arecords": [
                {
                "ipv4Address": "23.102.75.46"
                }
            ],
            "etag": "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx",
            "fqdn": "mqtt.example.com.",
            "id": "/subscriptions/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx/resourceGroups/dns-zone/providers/Microsoft.Network/dnszones/example.com/A/mqtt",
            "metadata": null,
            "name": "mqtt",
            "provisioningState": "Succeeded",
            "resourceGroup": "dns-zone",
            "targetResource": {
                "id": null
            },
            "ttl": 3600,
            "type": "Microsoft.Network/dnszones/A"
        }
        ```

1. mqttサブドメインの名前解決確認

    ```
    $ nslookup mqtt.${DOMAIN}
    ```

    - 実行結果（例）

        ```
        Server:         127.0.1.1
        Address:        127.0.1.1#53

        Non-authoritative answer:
        Name:   mqtt.example.com
        Address: 23.102.75.46
        ```

1. mosquitto_pubのリポジトリ登録

    ```
    $ sudo add-apt-repository ppa:mosquitto-dev/mosquitto-ppa
    ```

1. パッケージリストの更新

    ```
    $ sudo apt-get update -y
    ```

1. mosquitto_pubのインストール

    ```
    $ sudo apt-get install mosquitto mosquitto-clients
    ```

1. mosquitto_pubのインストール確認

    ```
    $ dpkg -l | grep mosquitto
    ```

1. mqttの疎通確認

    ```
    $ mosquitto_pub -h mqtt.${DOMAIN} -p 8883 --cafile ${CORE_ROOT}/secrets/DST_Root_CA_X3.pem -d -u iotagent -P ${MQTT__iotagent} -t /test -m "test"
    ```

    - 実行結果（例）

        ```
        Client mosqpub|4079-FIWARE-PC sending CONNECT
        Client mosqpub|4079-FIWARE-PC received CONNACK (0)
        Client mosqpub|4079-FIWARE-PC sending PUBLISH (d0, q0, r0, m1, '/test', ... (4 bytes))
        Client mosqpub|4079-FIWARE-PC sending DISCONNECT
        ```


## AKSにmongodbの設定

1. mongodbのインストール

    ```
    $ helm install --name mongodb -f mongodb/mongodb-replicaset-values-azure.yaml stable/mongodb-replicaset
    ```

    - 実行結果（例）

        ```
        NAME:   mongodb
        LAST DEPLOYED: Tue Feb 19 15:31:05 2019
        NAMESPACE: default
        STATUS: DEPLOYED

        RESOURCES:
        ==> v1/StatefulSet
        NAME     KIND
        mongodb  StatefulSet.v1.apps

        ==> v1/ConfigMap
        NAME             DATA  AGE
        mongodb-tests    1     1s
        mongodb-mongodb  1     1s
        mongodb-init     1     1s

        ==> v1/Service
        NAME            CLUSTER-IP  EXTERNAL-IP  PORT(S)    AGE
        mongodb         None        <none>       27017/TCP  1s
        mongodb-client  None        <none>       27017/TCP  1s

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
        NAME                STATUS   VOLUME                                     CAPACITY   ACCESS MODES   STORAGECLASS      AGE
        datadir-mongodb-0   Bound    pvc-efc8f4dc-340f-11e9-a6d0-3eb5d27c5279   30Gi       RWO            managed-premium   6m2s
        datadir-mongodb-1   Bound    pvc-3537f1be-3410-11e9-a6d0-3eb5d27c5279   30Gi       RWO            managed-premium   4m6s
        datadir-mongodb-2   Bound    pvc-6bf39873-3410-11e9-a6d0-3eb5d27c5279   30Gi       RWO            managed-premium   2m34s
        ```

1. mongodbのstatefulsets状態確認

    ```
    $ kubectl get statefulsets -l release=mongodb -l app=mongodb-replicaset
    ```

    - 実行結果（例）

        ```
        NAME      DESIRED   CURRENT   AGE
        mongodb   3         3         13m
        ```

1. mongodbのpods状態確認

    ```
    $ kubectl get pods -l release=mongodb -l app=mongodb-replicaset
    ```

    - 実行結果（例）

        ```
        NAME        READY   STATUS    RESTARTS   AGE
        mongodb-0   1/1     Running   0          16m
        mongodb-1   1/1     Running   0          14m
        mongodb-2   1/1     Running   0          13m
        ```

1. mongodbのservices状態確認

    ```
    $ kubectl get services -l release=mongodb -l app=mongodb-replicaset
    ```

    - 実行結果（例）

        ```
        NAME             TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)     AGE
        mongodb          ClusterIP   None         <none>        27017/TCP   18m
        mongodb-client   ClusterIP   None         <none>        27017/TCP   18m
        ```

1. mongodbの状態確認

    ```
    $ kubectl exec mongodb-0 -c mongodb-replicaset -- mongo --eval 'printjson(rs.status().members.map(function(e) {return {name: e.name, stateStr:e.stateStr};}))'
    ```

    - 実行結果（例）

        ```
        MongoDB shell version v4.0.6
        connecting to: mongodb://127.0.0.1:27017/?gssapiServiceName=mongodb
        Implicit session: session { "id" : UUID("19bbba43-0bdb-41af-87cf-9433ace6ef0c") }
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


## AKSのambassador設定

1. secret/ambassador-certsの登録

    ```
    $ kubectl create secret tls ambassador-certs --cert=${CORE_ROOT}/secrets/live/${DOMAIN}/fullchain.pem --key=${CORE_ROOT}/secrets/live/${DOMAIN}/privkey.pem
    ```

    - 実行結果（例）

        ```
        secret/ambassador-certs created
        ```

1. ambassador-azure-servicesの作成

    ```
    $ kubectl apply -f ambassador/ambassador-azure-services.yaml
    ```

    - 実行結果（例）

        ```
        service/ambassador created
        ```

1. ambassador-deploymentの作成

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
        fiware@FIWARE-PC:~/core$ kubectl get pods -l app=ambassador
        NAME                          READY   STATUS    RESTARTS   AGE
        ambassador-69dcd7cb7c-gw2mm   2/2     Running   0          104s
        ambassador-69dcd7cb7c-qwkdb   2/2     Running   0          104s
        ambassador-69dcd7cb7c-tfrfv   2/2     Running   0          104s
        ```

1. グローバルIPが割り当て確認

    ```
    $ kubectl get services -l app=ambassador
    ```

    - 実行結果（例）

        ```
        NAME         TYPE           CLUSTER-IP    EXTERNAL-IP      PORT(S)                      AGE
        ambassador   LoadBalancer   10.0.255.79   104.41.182.148   443:31787/TCP,80:30655/TCP   4m43s
        ```


## ambassadorのDNSゾーンにAレコード登録

1. HTTPS_IPADDRの設定

    ```
    $ HTTPS_IPADDR=$(kubectl get services -l app=ambassador -o json | jq '.items[0].status.loadBalancer.ingress[0].ip' -r)
    ```

1. apiにDNSレコードを追加

    ```
    $ az network dns record-set a add-record --resource-group ${DNS_ZONE_RG} --zone-name "${DOMAIN}" --record-set-name "api" --ipv4-address "${HTTPS_IPADDR}"
    ```

    - 実行結果（例）

        ```
        {
            "arecords": [
                {
                    "ipv4Address": "104.41.182.148"
                }
            ],
            "etag": "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx",
            "fqdn": "api.example.com.",
            "id": "/subscriptions/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx/resourceGroups/dns-zone/providers/Microsoft.Network/dnszones/example.com/A/api",
            "metadata": null,
            "name": "api",
            "provisioningState": "Succeeded",
            "resourceGroup": "dns-zone",
            "targetResource": {
                "id": null
            },
            "ttl": 3600,
            "type": "Microsoft.Network/dnszones/A"
        }
        ```

1. apiドメインの名前解決確認

    ```
    $ nslookup api.${DOMAIN}
    ```

    - 実行結果（例）

        ```
        Server:         127.0.1.1
        Address:        127.0.1.1#53

        Non-authoritative answer:
        Name:   api.example.com
        Address: 104.41.182.148
        ```

1. apiドメインの確認

    ```
    $ curl -i https://api.${DOMAIN}
    ```

    - 実行結果（例）

        ```
        HTTP/1.1 404 Not Found
        date: Thu, 21 Feb 2019 00:32:29 GMT
        server: envoy
        content-length: 0
        ```


## AKS上にて認証/承認サービスを作成

1. secrets/auth-tokens.jsonの作成

    ```
    $ cat << __EOS__ > secrets/auth-tokens.json
    [
        {
            "host": "api\\\\..+$",
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
    ```

1. secrets/auth-tokens.jsonを登録

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
        auth-67ff6bd94b-gm6gc   1/1     Running   0          94s
        auth-67ff6bd94b-w8gmn   1/1     Running   0          94s
        auth-67ff6bd94b-xbjcn   1/1     Running   0          94s
        ```

1. authのservices状態確認

    ```
    $ kubectl get services -l app=auth
    ```

    - 実行結果（例）

        ```
        NAME   TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)    AGE
        auth   ClusterIP   10.0.81.23   <none>        3000/TCP   6m44s
        ```


## AKSにfiware orionの設定

1. orion-serviceの作成

    ```
    $ kubectl apply -f orion/orion-service.yaml
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
        orion-584b686499-jdrcg   1/1     Running   0          85s
        orion-584b686499-k9px6   1/1     Running   0          85s
        orion-584b686499-txncc   1/1     Running   0          85s
        ```

1. orionのservices確認

    ```
    $ kubectl get services -l app=orion
    ```

    - 実行結果（例）

        ```
        NAME    TYPE        CLUSTER-IP    EXTERNAL-IP   PORT(S)    AGE
        orion   ClusterIP   10.0.113.89   <none>        1026/TCP   3m32s
        ```

1. orionの接続確認

    ```
    $ curl -i https://api.${DOMAIN}/orion/v2/entities/
    ```

    - 実行結果（例）

        ```
        curl -i https://api.${DOMAIN}/orion/v2/entities/
        HTTP/1.1 401 Unauthorized
        content-length: 60
        content-type: text/plain
        www-authenticate: Bearer realm="token_required"
        date: Thu, 21 Feb 2019 00:23:54 GMT
        server: envoy
        ```

1. 認証トークンの環境設定

    ```
    $ TOKEN=$(cat ${CORE_ROOT}/secrets/auth-tokens.json | jq '.[0].settings.bearer_tokens[0].token' -r)
    ```

1. orionの認証確認

    ```
    $ curl -i -H "Authorization: bearer ${TOKEN}" https://api.${DOMAIN}/orion/v2/entities/
    ```

    - 実行結果（例）

        ```
        fiware@FIWARE-PC:~/core$ curl -i -H "Authorization: bearer ${TOKEN}" https://api.${DOMAIN}/orion/v2/entities/
        HTTP/1.1 200 OK
        content-length: 2
        content-type: application/json
        fiware-correlator: 25a8864a-35a0-11e9-8318-5a27bdec5d3b
        date: Thu, 21 Feb 2019 06:15:54 GMT
        x-envoy-upstream-service-time: 2
        server: envoy

        []
        ```

    ※200以外のコードが出力された場合は、以下のコマンドを実行し、Ambassador全てのPodを再起動してください  

    ```
    $ kubectl delete pods -l app=ambassador
    ```

## AKSにfiware idasの設定  

1. iotagent-configのインストール

    ```
    $ env IOTA_PASSWORD=${MQTT__iotagent} envsubst < idas/rb-config.js > /
    $ kubectl create secret generic iotagent-config --from-file /tmp/rb-config.js
    $ rm /tmp/rb-config.js
    ```

    - 実行結果（例）

        ```
        secret/iotagent-config created
        ```

1. iotagent-ul-serviceの作成

    ```
    $ kubectl apply -f idas/iotagent-ul-service.yaml
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
        iotagent-ul-659d5f8598-r4j6x   1/1     Running   0          8m5s
        iotagent-ul-659d5f8598-tj5sz   1/1     Running   0          8m5s
        iotagent-ul-659d5f8598-z57cm   1/1     Running   0          8m5s
        ```

1. iotagent-ulのservices状態確認

    ```
    $ kubectl get services -l app=iotagent-ul
    ```

    - 実行結果（例）

        ```
        NAME          TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)             AGE
        iotagent-ul   ClusterIP   10.0.81.53   <none>        4041/TCP,7896/TCP   11m
        ```

1. idasにsecrets/auth-tokensを利用した接続確認

    ```
    $ TOKEN=$(cat ${CORE_ROOT}/secrets/auth-tokens.json | jq '.[0].settings.bearer_tokens[0].token' -r)
    $ curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: test" -H "Fiware-Servicepath: /*" https://api.${DOMAIN}/idas/ul20/manage/iot/services/
    ```

    - 実行結果（例）

        ```
        HTTP/1.1 200 OK
        x-powered-by: Express
        fiware-correlator: d7569174-8a31-4229-8905-42b6407309dc
        content-type: application/json; charset=utf-8
        content-length: 25
        etag: W/"19-WMYe0U6ocKhQjp+oaVnMHLdbylc"
        date: Thu, 21 Feb 2019 06:52:38 GMT
        x-envoy-upstream-service-time: 23
        server: envoy

        {"count":0,"services":[]}
        ```

    ※200以外のコードが出力された場合は、以下のコマンドを実行し、Ambassador全てのPodを再起動してください  

    ```
    $ kubectl delete pods -l app=ambassador
    ```

## AKSにfiware cygnusの設定

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
        cygnus-mongo-ff75ccbfb-p4f9t   1/1     Running   0          2m23s
        cygnus-mongo-ff75ccbfb-sg2mb   1/1     Running   0          2m23s
        cygnus-mongo-ff75ccbfb-v7ft4   1/1     Running   0          2m23s
        ```

1. cygnus-mongoのservices状態確認

    ```
    $ kubectl get services -l app=cygnus-mongo
    ```

    - 実行結果（例）

        ```
        NAME           TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)             AGE
        cygnus-mongo   ClusterIP   10.0.252.152   <none>        5050/TCP,8081/TCP   4m44s
        ```
