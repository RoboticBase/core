# RoboticBase Coreインストールマニュアル #3

## 構築環境(2019年4月26日現在)
# minikubeで認証認可用のpodsを開始


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

## minikubeのauth設定
1. secrets/auth-tokens.jsonの作成
    * macOS

        ```
        $ cat << __EOS__ > secrets/auth-tokens.json
        [
            {
                "host": ".*",
                "settings": {
                    "bearer_tokens": [
                        {
                            "token": "$(cat /dev/urandom | LC_CTYPE=C tr -dc 'a-zA-Z0-9' | head -c 32)",
                            "allowed_paths": ["^/orion/.*$", "^/idas/.*$"]
                        }
                    ],
                    "basic_auths": [],
                    "no_auths": {
                        "allowed_paths": []
                    }
                }
            }
        ]
        __EOS__
        ```
    * Ubuntu

        ```
        $ cat << __EOS__ > secrets/auth-tokens.json
        [
            {
                "host": ".*",
                "settings": {
                    "bearer_tokens": [
                        {
                            "token": "$(cat /dev/urandom 2>/dev/null | head -n 40 | tr -cd 'a-zA-Z0-9' | head -c 32)",
                            "allowed_paths": ["^/orion/.*$", "^/idas/.*$"]
                        }
                    ],
                    "basic_auths": [],
                    "no_auths": {
                        "allowed_paths": []
                    }
                }
            }
        ]
        __EOS__
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
