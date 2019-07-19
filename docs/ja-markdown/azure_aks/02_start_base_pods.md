# RoboticBase Coreインストールマニュアル #2

## 構築環境(2019年7月18日現在)
# Azure AKSでAPI GatewayやMessage Broker、KeyValue DBのpodsを開始


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
    $ source ${CORE_ROOT}/docs/environments/azure_aks/env
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

## ワイルドカードTLS証明書の作成

1. dockerコンテナ起動用コマンドの作成

    ```
    $ echo "docker run -it -v ${CORE_ROOT}/secrets:/etc/letsencrypt certbot/certbot certonly --manual --domain *.${DOMAIN} --email ${EMAIL} --no-eff-email --agree-tos --manual-public-ip-logging-ok --preferred-challenges dns-01 --server https://acme-v02.api.letsencrypt.org/directory"
    ```

1. 別ターミナルを開いて生成されたコマンドを実行し、Let's encrypt用のTLS証明書作成

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

1. 別ターミナルで表示されたDNS TXT recordを、元のターミナル上で環境変数に設定

    ```
    $ export DNS_TXT="XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"

    ```

1. 元のターミナル上で、DNSゾーンにTXTレコードの登録

    ```
    $ az network dns record-set txt add-record --resource-group ${DNS_ZONE_RG} --zone-name "${DOMAIN}" --record-set-name "_acme-challenge" --value "${DNS_TXT}"
    ```

    - 実行結果（例）

        ```json
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

1. 別ターミナルでEnterキーを押下し、Let's encryptの認証が成功したことを確認

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

1.  別ターミナルを閉じる

1. 元のターミナル上でDNSリソースグループのDNS record削除

    ```
    $ az network dns record-set txt remove-record --resource-group ${DNS_ZONE_RG} --zone-name "${DOMAIN}" --record-set-name "_acme-challenge" --value "${DNS_TXT}"
    ```

1. Ubuntuの場合、生成されたTLS証明書のユーザーを変更
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


## AKSでRabbitMQを起動

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

## RabbitMQのゲストパスワードの変更
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

1. rabbitmqのservices状態確認

    ```
    $ kubectl get services -l app=rabbitmq -l service=mqtts
    ```

    - 実行結果（例）

        ```
        NAME             TYPE           CLUSTER-IP     EXTERNAL-IP   PORT(S)          AGE
        rabbitmq-mqtts   LoadBalancer   10.0.226.238   XX.XX.XX.XX   8883:31693/TCP   21m
        ```

## RabbitMQのグローバルIPをDNSに登録

1. RabbitMQのグローバルIPアドレスを取得

    ```
    $ MQTTS_IPADDR=$(kubectl get services -l app=rabbitmq -l service=mqtts -o jsonpath='{.items[0].status.loadBalancer.ingress[0].ip}')
    ```

1. RabbitMQのサブドメイン（ `mqtt` ）をDNSに追加

    ```
    $ az network dns record-set a add-record --resource-group ${DNS_ZONE_RG} --zone-name "${DOMAIN}" --record-set-name "mqtt" --ipv4-address "${MQTTS_IPADDR}"
    ```

    - 実行結果（例）

        ```json
        {
            "arecords": [
                {
                "ipv4Address": "XX.XX.XX.XX"
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
        Address: XX.XX.XX.XX
        ```

## MQTTSの疎通確認
1. mqttsの疎通確認

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


## AKSでmongodbを起動

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
        NAME      READY   AGE
        mongodb   3/3     5m32s
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
        MongoDB shell version v4.1.13
        connecting to: mongodb://127.0.0.1:27017/?compressors=disabled&gssapiServiceName=mongodb
        Implicit session: session { "id" : UUID("2508d7c2-8857-4228-8f1d-0fd1e6859b08") }
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


## AKSでambassadorを起動

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
        fiware@FIWARE-PC:~/core$ kubectl get pods -l app=ambassador
        NAME                          READY   STATUS    RESTARTS   AGE
        ambassador-69dcd7cb7c-gw2mm   1/1     Running   0          104s
        ambassador-69dcd7cb7c-qwkdb   1/1     Running   0          104s
        ambassador-69dcd7cb7c-tfrfv   1/1     Running   0          104s
        ```

1. ambassadorのservices状態確認

    ```
    $ kubectl get services -l app=ambassador
    ```

    - 実行結果（例）

        ```
        NAME         TYPE           CLUSTER-IP    EXTERNAL-IP   PORT(S)                      AGE
        ambassador   LoadBalancer   10.0.255.79   XX.XX.XX.XX   443:31787/TCP,80:30655/TCP   4m43s
        ```


## ambassadorのグローバルIPをDNSに登録

1. ambassadorのグローバルIPアドレスを取得

    ```
    $ HTTPS_IPADDR=$(kubectl get services -l app=ambassador -o json | jq '.items[0].status.loadBalancer.ingress[0].ip' -r)
    ```

1. ambassadorのサブドメイン（ `api` ）をDNSレコードを追加

    ```
    $ az network dns record-set a add-record --resource-group ${DNS_ZONE_RG} --zone-name "${DOMAIN}" --record-set-name "api" --ipv4-address "${HTTPS_IPADDR}"
    ```

    - 実行結果（例）

        ```json
        {
            "arecords": [
                {
                    "ipv4Address": "XX.XX.XX.XX"
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
        Address: XX.XX.XX.XX
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
