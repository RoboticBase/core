# RoboticBase Coreインストールマニュアル #4

## 構築環境(2019年4月26日現在)
# minikubeでFIWAREのpodsを開始


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

## minikubeでfiware orionを起動

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

1. 認証トークンを取得

    ```
    $ TOKEN=$(cat ${CORE_ROOT}/secrets/auth-tokens.json | jq '.[0].settings.bearer_tokens[0].token' -r)
    ```

1. orionの認証確認

    ```
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
    $ kubectl delete pods -l app=ambassador
    ```

## minikubeでfiware idas(iotagent-ul)を起動

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

## minikubeでfiware cygnus(mongodb sink)を起動

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

## minikubeでfiware sth-cometを起動

1. comet-minikube-serviceの作成

    ```
    $ kubectl apply -f comet/comet-minikube-service.yaml
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
    $ curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: test" -H "Fiware-Servicepath: /*" http://${HOST_IPADDR}:8080/comet/STH/v1/contextEntities/
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
