# RoboticBase Coreインストールマニュアル #4

## 構築環境(2019年7月18日現在)
# Azure AKSでFIWAREのpodsを開始


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

## AKSでfiware orionを起動

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

        {"authorized":false,"error":"missing Header: authorization"}
        ```

1. 認証トークンを取得

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

## AKSでfiware idasを起動
**IoTデバイスとMQTTで交換するメッセージ形式として `Ultralight 2.0` を使用するか `json` を使用するかを選択してください。**

* `Ultralight 2.0` を使用する場合： "選択肢1: iotagent-ulを起動" の手順を実施
* `json` を使用する場合： "選択肢2: iotagent-jsonを起動" の手順を実施

### 選択肢1: iotagent-ulを起動

1. iotagent-ulの設定をsecretに登録

    ```
    $ env IOTA_PASSWORD=${MQTT__iotagent} envsubst < idas/rb-config-ul.js > /tmp/rb-config-ul.js
    $ kubectl create secret generic iotagent-ul-config --from-file /tmp/rb-config-ul.js
    $ rm /tmp/rb-config-ul.js
    ```

    - 実行結果（例）

        ```
        secret/iotagent-ul-config created
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
    $ env IOTAGENT_UL_NGSI_VERSION="v1" envsubst < idas/iotagent-ul-deployment.yaml | kubectl apply -f -
    ```

    - 実行結果（例）

        ```
        deployment.apps/iotagent-ul created
        ```
    * **orionとの接続でNGSIv2を利用したい場合は、 `IOTAGENT_UL_NGSI_VERSION` を "v2" に変更してください**

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

1. idasの接続確認

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

### 選択肢2: iotagent-jsonを起動

1. iotagent-jsonの設定をsecretに登録

    ```
    $ env IOTA_PASSWORD=${MQTT__iotagent} envsubst < idas/rb-config-json.js > /tmp/rb-config-json.js
    $ kubectl create secret generic iotagent-json-config --from-file /tmp/rb-config-json.js
    $ rm /tmp/rb-config-json.js
    ```

    - 実行結果（例）

        ```
        secret/iotagent-json-config created
        ```

1. iotagent-json-serviceの作成

    ```
    $ kubectl apply -f idas/iotagent-json-service.yaml
    ```

    - 実行結果（例）

        ```
        service/iotagent-json created
        ```

1. iotagent-json-deploymentの作成

    ```
    $ env IOTAGENT_JSON_NGSI_VERSION="v1" envsubst < idas/iotagent-json-deployment.yaml | kubectl apply -f -
    ```

    - 実行結果（例）

        ```
        deployment.apps/iotagent-json created
        ```
    * **orionとの接続でNGSIv2を利用したい場合は、 `IOTAGENT_JSON_NGSI_VERSION` を "v2" に変更してください**

1. iotagent-jsonのpods状態確認

    ```
    $ kubectl get pods -l app=iotagent-json
    ```

    - 実行結果（例）

        ```
        NAME                             READY   STATUS    RESTARTS   AGE
        iotagent-json-65879f88b4-2h2hx   1/1     Running   0          53s
        iotagent-json-65879f88b4-r9sv2   1/1     Running   0          53s
        iotagent-json-65879f88b4-xx9dl   1/1     Running   0          53s
        ```

1. iotagent-jsonのservices状態確認

    ```
    $ kubectl get services -l app=iotagent-json
    ```

    - 実行結果（例）

        ```
        NAME            TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)             AGE
        iotagent-json   ClusterIP   10.111.43.11   <none>        4041/TCP,7896/TCP   2m
        ```

1. idasの接続確認

    ```
    $ TOKEN=$(cat ${CORE_ROOT}/secrets/auth-tokens.json | jq '.[0].settings.bearer_tokens[0].token' -r)
    $ curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: test" -H "Fiware-Servicepath: /*" https://api.${DOMAIN}/idas/json/manage/iot/services/
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

## AKSでfiware cygnus(mongodb sink)を起動

1. cygnus-mongo-serviceの作成

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

## AKSでfiware sth-cometを起動

1. comet-serviceの作成

    ```
    $ kubectl apply -f comet/comet-service.yaml
    ```

    - 実行結果（例）

        ```
        service/comet created
        ```

1. comet-deploymentの作成

    ```
    $ kubectl apply -f comet/comet-deployment.yaml
    ```

    - 実行結果（例）

        ```
        deployment.apps/comet created
        ```

1. sth-cometのpods状態確認

    ```
    $ kubectl get pods -l app=comet
    ```

    - 実行結果（例）

        ```
        NAME                    READY   STATUS    RESTARTS   AGE
        comet-7fb8b9554-bfrwl   1/1     Running   0          26s
        comet-7fb8b9554-hbmkx   1/1     Running   0          26s
        comet-7fb8b9554-ms94m   1/1     Running   0          26s
        ```

1. sth-cometのservices確認

    ```
    $ kubectl get services -l app=comet
    ```

    - 実行結果（例）

        ```
        NAME    TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)    AGE
        comet   ClusterIP   10.0.120.142   <none>        8666/TCP   67s
        ```
1. sth-cometの接続確認

    ```
    $ TOKEN=$(cat ${PJ_ROOT}/secrets/auth-tokens.json | jq '.[0].settings.bearer_tokens[0].token' -r)
    $ curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: test" -H "Fiware-Servicepath: /*" https://api.${DOMAIN}/comet/STH/v1/contextEntities/
    ```

    - 実行結果（例）

        ```
        HTTP/1.1 404 Not Found
        fiware-correlator: 7db06a49-57a2-465f-a4ef-714d8e773637
        content-type: application/json; charset=utf-8
        cache-control: no-cache
        content-length: 38
        vary: accept-encoding
        date: Tue, 18 Jun 2019 03:55:17 GMT
        x-envoy-upstream-service-time: 4
        server: envoy

        {"statusCode":404,"error":"Not Found"}
        ```
