# RoboticBase Coreインストールマニュアル #3

## 構築環境(2019年7月18日現在)
# Azure AKSで認証認可用のpodsを開始


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
      alias randomstr8='cat /dev/urandom | LC_CTYPE=C tr -dc 'a-zA-Z0-9' | head -c 8'
      alias randomstr16='cat /dev/urandom | LC_CTYPE=C tr -dc 'a-zA-Z0-9' | head -c 16'
      alias randomstr32='cat /dev/urandom | LC_CTYPE=C tr -dc 'a-zA-Z0-9' | head -c 32'
    elif [ "$(expr substr $(uname -s) 1 5)" == 'Linux' ]; then
      alias randomstr8='cat /dev/urandom 2>/dev/null | head -n 40 | tr -cd 'a-zA-Z0-9' | head -c 8'
      alias randomstr16='cat /dev/urandom 2>/dev/null | head -n 40 | tr -cd 'a-zA-Z0-9' | head -c 16'
      alias randomstr32='cat /dev/urandom 2>/dev/null | head -n 40 | tr -cd 'a-zA-Z0-9' | head -c 32'
    else
      echo "Your platform ($(uname -a)) is not supported."
      exit 1
    fi
    ```

## AKSのauth設定

1. secrets/auth-tokens.jsonの作成

    ```
    $ cat << __EOS__ > secrets/auth-tokens.json
    [
        {
            "host": "api\\\\..+$",
            "settings": {
                "bearer_tokens": [
                    {
                        "token": "$(randomstr32)",
                        "allowed_paths": ["^/orion/.*$", "^/idas/.*$", "^/comet/.*$"]
                    }
                ],
                "basic_auths": [],
                "no_auths": {
                    "allowed_paths": []
                }
            }
        }, {
            "host": "kibana\\\\..+$",
            "settings": {
                "bearer_tokens": [],
                "basic_auths": [
                    {
                        "username": "$(randomstr8)",
                        "password": "$(randomstr16)",
                        "allowed_paths": ["^.*$"]
                    }
                ],
                "no_auths": {
                    "allowed_paths": []
                }
            }
        }, {
            "host": "grafana\\\\..+$",
            "settings": {
                "bearer_tokens": [],
                "basic_auths": [],
                "no_auths": {
                    "allowed_paths": ["^.*$"]
                }
            }
        }
    ]
    __EOS__
    $ cat ./secrets/auth-tokens.json
    ```

    - 実行結果（例）

        ```json
        [
          {
            "host": "api\\..+$",
            "settings": {
              "bearer_tokens": [
                {
                  "token": "nrWtb8sS0MmwlkhHXv0DC6orPMpFFbni",
                  "allowed_paths": ["^/orion/.*$", "^/idas/.*$", "^/comet/.*$"]
                }
              ],
              "basic_auths": [],
              "no_auths": {}
            }
          },
          {
            "host": "kibana\\..+$",
            "settings": {
              "bearer_tokens": [],
              "basic_auths": [
                {
                  "username": "yW7FvSGD",
                  "password": "6BoTFE5xfUlX3ssV",
                  "allowed_paths": ["^.*$"]
                }
              ],
              "no_auths": {
                "allowed_paths": []
              }
            }
          },
          {
            "host": "grafana\\..+$",
            "settings": {
              "bearer_tokens": [],
              "basic_auths": [],
              "no_auths": {
                "allowed_paths": ["^.*$"]
              }
            }
          }
        ]
        ```

## AKSにauthの設定
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
