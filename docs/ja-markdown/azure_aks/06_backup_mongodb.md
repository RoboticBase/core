# RoboticBase Coreインストールマニュアル #6

## 環境構築(2019年4月26日現在)

## 環境変数設定

1. 環境変数の設定

    ```
    $ export CORE_ROOT=${HOME}/core
    ```

1. 環境設定の読み込み

    ```
    $ source ${CORE_ROOT}/docs/environments/azure_aks/env
    ```

## AzureにStorage Accountを作成

1. AzureにStorage Accountを作成

    ```
    $ az storage account create --name ${STORAGE_ACCOUNT} --resource-group ${AKS_RG} --location ${REGION} --sku Standard_LRS --encryption blob
    ```
    - 実行結果（例）

        ```json
        {
          "accessTier": null,
          "creationTime": "2019-06-09T06:37:54.259956+00:00",
          "customDomain": null,
          "enableAzureFilesAadIntegration": null,
          "enableHttpsTrafficOnly": false,
          "encryption": {
            "keySource": "Microsoft.Storage",
            "keyVaultProperties": null,
            "services": {
              "blob": {
                "enabled": true,
                "lastEnabledTime": "2019-06-09T06:37:54.338055+00:00"
              },
              "file": {
                "enabled": true,
                "lastEnabledTime": "2019-06-09T06:37:54.338055+00:00"
              },
              "queue": null,
              "table": null
            }
          },
          "failoverInProgress": null,
          "geoReplicationStats": null,
          "id": "/subscriptions/xxxxxxxx-xxxx-xxxx-xxxxx-xxxxxxxxxxxx/resourceGroups/rb-nmatsui/providers/Microsoft.Storage/storageAccounts/staccountnm",
          "identity": null,
          "isHnsEnabled": null,
          "kind": "Storage",
          "lastGeoFailoverTime": null,
          "location": "japaneast",
          "name": "rbstorageaccount",
          "networkRuleSet": {
            "bypass": "AzureServices",
            "defaultAction": "Allow",
            "ipRules": [],
            "virtualNetworkRules": []
          },
          "primaryEndpoints": {
            "blob": "https://rbstorageaccount.blob.core.windows.net/",
            "dfs": null,
            "file": "https://rbstorageaccount.file.core.windows.net/",
            "queue": "https://rbstorageaccount.queue.core.windows.net/",
            "table": "https://rbstorageaccount.table.core.windows.net/",
            "web": null
          },
          "primaryLocation": "japaneast",
          "provisioningState": "Succeeded",
          "resourceGroup": "rbcore",
          "secondaryEndpoints": null,
          "secondaryLocation": null,
          "sku": {
            "capabilities": null,
            "kind": null,
            "locations": null,
            "name": "Standard_LRS",
            "resourceType": null,
            "restrictions": null,
            "tier": "Standard"
          },
          "statusOfPrimary": "available",
          "statusOfSecondary": null,
          "tags": {},
          "type": "Microsoft.Storage/storageAccounts"
        }
        ```
1. 作成したStorage Accountの1つ目のAccess Keyを取得

    ```
    $ export ACCOUNT_KEY=$(az storage account keys list --account-name ${STORAGE_ACCOUNT} --resource-group ${AKS_RG} --output json | jq .[0].value -r)
    ```

## Storage Containerを作成

1. 作成したStorage AccountにContainerを作成

    ```
    $ az storage container create --name ${STORAGE_CONTAINER} --account-name ${STORAGE_ACCOUNT} --account-key ${ACCOUNT_KEY}
    ```
    - 実行結果（例）

        ```json
        {
          "created": true
        }
        ```
1. Containerが作成されていることを確認

    ```
    $ az storage blob list --container-name ${STORAGE_CONTAINER} --account-name ${STORAGE_ACCOUNT} --account-key ${ACCOUNT_KEY}
    ```
    - 実行結果（例）

        ```json
        []
        ```

## backup job を手動起動
環境が正しく設定されていることを確認する

1. バックアップジョブを起動

    ```
    $ envsubst < mongodb/simple-dump-job.yaml | kubectl apply -f -
    ```
    - 実行結果（例）

        ```
        job.batch/simple-dump-job created
        ```
1. jobの状態確認

    ```
    $ kubectl get jobs -l job=simple-dump-job
    ```
    - 実行結果（例）

        ```
        NAME              COMPLETIONS   DURATION   AGE
        simple-dump-job   1/1           5s         27s
        ```
1. Podの状態確認

    ```
    $ kubectl get pods -l job=simple-dump-job
    ```
    - 実行結果（例）

        ```
        NAME                    READY   STATUS      RESTARTS   AGE
        simple-dump-job-7td2x   0/1     Completed   0          45s
        ```
1. jobがCOMPLETIONSが1/1になり、PodがCompletedしていればバックアップジョブの起動に成功している
1. バックアップジョブのログ確認

    ```
    $ kubectl logs $(kubectl get pods -l job=simple-dump-job -o template --template "{{(index .items 0).metadata.name}}")
    ```
    - 実行結果（例）

        ```
        2019/06/07 08:48:32 [   INFO] __main__ - start main
        2019/06/07 08:48:32 [   INFO] src.mongodb - start dump, endpoint=rs0/mongodb-client:27017
        2019/06/07 08:48:32 [   INFO] src.mongodb - exec dump command, dump_dir=/opt/mongodump_20190607174832JST
        2019/06/07 08:48:33 [   INFO] src.mongodb - 
        2019/06/07 08:48:33 [   INFO] src.mongodb - 2019-06-07T08:48:33.758+0000    writing admin.system.version to 
        2019-06-07T08:48:33.760+0000    done dumping admin.system.version (1 document)
        2019-06-07T08:48:33.760+0000    writing iotagentul.groups to 
        2019-06-07T08:48:33.760+0000    writing orion.entities to 
        2019-06-07T08:48:33.761+0000    done dumping iotagentul.groups (0 documents)
        2019-06-07T08:48:33.761+0000    done dumping orion.entities (0 documents)

        2019/06/07 08:48:33 [   INFO] src.mongodb - compress dump dir, dump_dir=/opt/mongodump_20190607174832JST
        2019/06/07 08:48:33 [   INFO] src.mongodb - finish dump, dump_file=/opt/mongodump_20190607174832JST.tar.gz
        2019/06/07 08:48:33 [   INFO] src.azure_blob_storage - start upload, container=mongodump, file=/opt/mongodump_20190607174832JST.tar.gz
        2019/06/07 08:48:33 [   INFO] azure.storage.common.storageclient - Client-Request-ID=083ca8f2-8901-11e9-b537-a6ad4145284e Outgoing request: Method=PUT, Path=/mongodump/mongodump_20190607174832JST.tar.gz, Query={'timeout': None}, Headers={'x-ms-blob-type': 'BlockBlob', 'x-ms-lease-id': None, 'If-Modified-Since': None, 'If-Unmodified-Since': None, 'If-Match': None, 'If-None-Match': None, 'Content-Length': '700', 'x-ms-version': '2018-11-09', 'User-Agent': 'Azure-Storage/2.0.0-2.0.1 (Python CPython 3.7.3; Linux 4.15.0-1045-azure)', 'x-ms-client-request-id': '083ca8f2-8901-11e9-b537-a6ad4145284e', 'x-ms-date': 'Fri, 07 Jun 2019 08:48:33 GMT', 'Authorization': 'REDACTED'}.
        2019/06/07 08:48:33 [   INFO] azure.storage.common.storageclient - Client-Request-ID=083ca8f2-8901-11e9-b537-a6ad4145284e Receiving Response: Server-Timestamp=Fri, 07 Jun 2019 08:48:33 GMT, Server-Request-ID=6829bef5-e01e-003f-760d-1d20d7000000, HTTP Status Code=201, Message=Created, Headers={'content-length': '0', 'content-md5': 'jLwedUl693uDUHgfnlyUyg==', 'last-modified': 'Fri, 07 Jun 2019 08:48:33 GMT', 'etag': '"0x8D6EB24EC81FEAC"', 'server': 'Windows-Azure-Blob/1.0 Microsoft-HTTPAPI/2.0', 'x-ms-request-id': '6829bef5-e01e-003f-760d-1d20d7000000', 'x-ms-version': '2018-11-09', 'x-ms-request-server-encrypted': 'true', 'date': 'Fri, 07 Jun 2019 08:48:33 GMT'}.
        2019/06/07 08:48:33 [   INFO] src.azure_blob_storage - finish upload, last_modified=2019-06-07 08:48:33+00:00
        2019/06/07 08:48:33 [   INFO] __main__ - finish main
        ```
1. Azure Blob Storageの指定したContainerにダンプファイルがアップロードされていることを確認

    ```
    $ az storage blob list --container-name ${STORAGE_CONTAINER} --account-name ${STORAGE_ACCOUNT} --account-key ${ACCOUNT_KEY} -o table
    ```
    - 実行結果（例）

        ```
        Name                                Blob Type    Blob Tier    Length    Content Type              Last Modified              Snapshot
        ----------------------------------  -----------  -----------  --------  ------------------------  -------------------------  ----------
        mongodump_20190607174832JST.tar.gz  BlockBlob                 700       application/octet-stream  2019-06-07T08:48:33+00:00
        ```
1. 手動起動したバックアップジョブを削除

    ```
    $ kubectl delete job simple-dump-job
    ```
    - 実行結果（例）

        ```
        job.batch "simple-dump-job" deleted
        ```

## cronjobを起動

1. cronjobを起動

    ```
    $ envsubst < mongodb/simple-dump-cronjob.yaml | kubectl apply -f -
    ```
    - 実行結果（例）

        ```
        cronjob.batch/simple-dump-cornjob created
        ```
1. cronjobを確認

    ```
    $ kubectl get cronjobs -l job=simple-dump-cronjob
    ```
    - 実行結果（例）

        ```
        NAME                  SCHEDULE         SUSPEND   ACTIVE   LAST SCHEDULE   AGE
        simple-dump-cornjob   0 0-23/3 * * *   False     0        <none>          12s
        ```
    - 上記の設定では、毎日3時間ごと（00:00, 03:00, 06:00, 09:00, 12:00, 15:00, 18:00, 21:00）にバックアップジョブが自動起動する
