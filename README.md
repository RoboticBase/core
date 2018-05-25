# container-centric fiware demonstaration for Microsoft Azure AKS
This repository construct a container-centric [FIWARE](http://www.fiware.org/) demonstration on [Kubernetes](https://kubernetes.io/) using [Microsoft Azure AKS](https://azure.microsoft.com/en-us/services/container-service/).

## Requirements

||version|
|:--|:--|
|azure cli|2.0.31|
|kubectl|1.10.1|
|helm|2.8.2|
|envsubst|0.19.8.1|

||version|
|:--|:--|
|kubernetes|1.9.6|

## create DNS zone of "cloudconductor.jp"

```bash
mac:$ az group create --name dns-zone --location japaneast
```

```bash
mac:$ az network dns zone create --resource-group dns-zone --name "cloudconductor.jp"
```

```bash
mac:$ az network dns zone show --resource-group dns-zone --name "cloudconductor.jp" | jq ".nameServers"
[
  "ns1-09.azure-dns.com.",
  "ns2-09.azure-dns.net.",
  "ns3-09.azure-dns.org.",
  "ns4-09.azure-dns.info."
]
```

## start private registry on Azure Container Registry

```bash
mac:$ az login --tenant tisstc01.onmicrosoft.com
```

```bash
mac:$ az group create --name fiware-demo --location centralus
```

```bash
mac:$ az acr create --resource-group fiware-demo --name fiwareacr --sku Basic
mac:$ export REPOSITORY=$(az acr show --resource-group fiware-demo --name fiwareacr | jq '.loginServer' -r); echo ${REPOSITORY}
fiwareacr.azurecr.io
```

```bash
mac:$ az acr login --name fiwareacr
```

## start kubernetes on Azure AKS

```bash
mac:$ az provider register -n Microsoft.Compute
mac:$ az provider register -n Microsoft.Storage
mac:$ az provider register -n Microsoft.Network
mac:$ az provider register -n Microsoft.ContainerService
```

```bash
mac:$ az provider show -n Microsoft.Compute | jq '.registrationState' -r
Registered
mac:$ az provider show -n Microsoft.Storage | jq '.registrationState' -r
Registered
mac:$ az provider show -n Microsoft.Network | jq '.registrationState' -r
Registered
mac:$ az provider show -n Microsoft.ContainerService | jq '.registrationState' -r
Registered
```

```bash
mac:$ az aks create --resource-group fiware-demo --name fiwareaks --node-count 4 --ssh-key-value $HOME/.ssh/azure.pub
```

```bash
mac:$ az aks get-credentials --resource-group fiware-demo --name fiwareaks
```

```bash
mac:$ kubectl get nodes
NAME                       STATUS    ROLES     AGE       VERSION
aks-nodepool1-27506152-0   Ready     agent     2m        v1.9.6
aks-nodepool1-27506152-1   Ready     agent     2m        v1.9.6
aks-nodepool1-27506152-2   Ready     agent     2m        v1.9.6
aks-nodepool1-27506152-3   Ready     agent     2m        v1.9.6
```

```bash
mac:$ CLIENT_ID=$(az aks show --resource-group fiware-demo --name fiwareaks --query "servicePrincipalProfile.clientId" --output tsv);echo ${CLIENT_ID}
mac:$ ACR_ID=$(az acr show --name fiwareacr --resource-group fiware-demo --query "id" --output tsv); echo ${ACR_ID}
mac:$ az role assignment create --assignee ${CLIENT_ID} --role Reader --scope ${ACR_ID}
```

## install helm
```bash
mac:$ curl --output ~/Downloads/helm-v2.8.2-darwin-amd64.tar.gz https://storage.googleapis.com/kubernetes-helm/helm-v2.8.2-darwin-amd64.tar.gz
mac:$ tar xfz ~/Downloads/helm-v2.8.2-darwin-amd64.tar.gz -C /tmp
mac:$ sudo mv /tmp/darwin-amd64/helm /usr/local/bin
```

```bash
mac:$ helm update
mac:$ helm init
```

```bash
mac:$ kubectl get pods --all-namespaces | grep tiller
kube-system   tiller-deploy-865dd6c794-cdzd9          1/1       Running   0          6m
```

```bash
mac:$ helm version
Client: &version.Version{SemVer:"v2.8.2", GitCommit:"a80231648a1473929271764b920a8e346f6de844", GitTreeState:"clean"}
Server: &version.Version{SemVer:"v2.8.2", GitCommit:"a80231648a1473929271764b920a8e346f6de844", GitTreeState:"clean"}
```

## start etcd cluster on AKS

```bash
mac:$ helm install stable/etcd-operator --name fiware-etcd --set rbac.create=false
```

```bash
mac:$ kubectl get pods | grep fiware-etcd
fiware-etcd-etcd-operator-etcd-backup-operator-d49598cb6-v92zl    1/1       Running   0          51s
fiware-etcd-etcd-operator-etcd-operator-d69bdfb64-m6bjz           1/1       Running   0          51s
fiware-etcd-etcd-operator-etcd-restore-operator-5c65cd4469t9xrp   1/1       Running   0          51s
```

```bash
mac:$ kubectl apply -f etcd/etcd-cluster.yaml
```

```bash
mac:$ kubectl get pods -l app=etcd
NAME                READY     STATUS    RESTARTS   AGE
etcd-cluster-0000   1/1       Running   0          2m
etcd-cluster-0001   1/1       Running   0          2m
etcd-cluster-0002   1/1       Running   0          2m
```

```bash
mac:$ kubectl get services -l app=etcd
NAME                  TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)             AGE
etcd-cluster          ClusterIP   None           <none>        2379/TCP,2380/TCP   3m
etcd-cluster-client   ClusterIP   10.0.228.101   <none>        2379/TCP            3m
```

```bash
mac:$ kubectl run --rm -it etcdclient --image quay.io/coreos/etcd --restart=Never -- etcdctl --peers http://etcd-cluster-client:2379 member list
ab42ec96726a2edc: name=etcd-cluster-0001 peerURLs=http://etcd-cluster-0001.etcd-cluster.default.svc:2380 clientURLs=http://etcd-cluster-0001.etcd-cluster.default.svc:2379 isLeader=false
d3dfab2ca43ad0a7: name=etcd-cluster-0000 peerURLs=http://etcd-cluster-0000.etcd-cluster.default.svc:2380 clientURLs=http://etcd-cluster-0000.etcd-cluster.default.svc:2379 isLeader=true
d7c603bd213157b2: name=etcd-cluster-0002 peerURLs=http://etcd-cluster-0002.etcd-cluster.default.svc:2380 clientURLs=http://etcd-cluster-0002.etcd-cluster.default.svc:2379 isLeader=false
```

## start vernemq cluster on AKS

* create usernames & passwords of vernemq
```bash
mac:$ mkdir -p secrets
mac:$ touch secrets/vmq.passwd
mac:$ docker run --rm -v $(pwd)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd iotagent
mac:$ docker run --rm -v $(pwd)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd raspberrypi
mac:$ docker run --rm -v $(pwd)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd turtlesim
mac:$ docker run --rm -v $(pwd)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd gopigo
```

```bash
mac:$ docker run -it -v $(pwd)/secrets:/etc/letsencrypt certbot/certbot certonly --manual --domain mqtt.cloudconductor.jp --email nobuyuki.matsui@gmail.com --agree-tos --manual-public-ip-logging-ok --preferred-challenges dns
```

* Another terminal
```bash
mac-another:$ az network dns record-set txt add-record --resource-group dns-zone --zone-name "cloudconductor.jp" --record-set-name "_acme-challenge.mqtt" --value "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
mac-another:$ az network dns record-set list --resource-group dns-zone --zone-name "cloudconductor.jp" | jq '.[] | {"fqdn": .fqdn, "type": .type}'
{
  "fqdn": "cloudconductor.jp.",
  "type": "Microsoft.Network/dnszones/NS"
}
{
  "fqdn": "cloudconductor.jp.",
  "type": "Microsoft.Network/dnszones/SOA"
}
{
  "fqdn": "_acme-challenge.mqtt.cloudconductor.jp.",
  "type": "Microsoft.Network/dnszones/TXT"
}
```

* Press 'ENTER' at original terminal when `_acme-challenge.mqtt.cloudconductor.jp.` txt record is created.


* After completion of certbot
```bash
mac-another:$ az network dns record-set txt remove-record --resource-group dns-zone --zone-name "cloudconductor.jp" --record-set-name "_acme-challenge.mqtt" --value "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
```

```bash
mac:$ cat secrets/DST_Root_CA_X3.pem secrets/archive/mqtt.cloudconductor.jp/chain1.pem > secrets/ca.crt
mac:$ cp secrets/archive/mqtt.cloudconductor.jp/fullchain1.pem secrets/server.crt
mac:$ cp secrets/archive/mqtt.cloudconductor.jp/privkey1.pem secrets/server.key
```

```bash
mac:$ kubectl create secret generic vernemq-passwd --from-file=./secrets/vmq.passwd
mac:$ kubectl create secret generic vernemq-certifications --from-file=./secrets/ca.crt --from-file=./secrets/server.crt --from-file=./secrets/server.key
```

```bash
mac:$ kubectl get secrets
NAME                                                          TYPE                                  DATA      AGE
default-token-2klq6                                           kubernetes.io/service-account-token   3         19m
fiware-etcd-etcd-operator-etcd-backup-operator-token-ghsq2    kubernetes.io/service-account-token   3         5m
fiware-etcd-etcd-operator-etcd-operator-token-qm2xp           kubernetes.io/service-account-token   3         5m
fiware-etcd-etcd-operator-etcd-restore-operator-token-4wzkm   kubernetes.io/service-account-token   3         5m
vernemq-certifications                                        Opaque                                3         3s
vernemq-passwd                                                Opaque                                1         11s
```

```bash
mac:$ kubectl apply -f vernemq/vernemq-cluster-azure.yaml
```

```bash
mac:$ kubectl get pods -l app=vernemq
NAME        READY     STATUS    RESTARTS   AGE
vernemq-0   1/1       Running   0          1m
vernemq-1   1/1       Running   0          1m
vernemq-2   1/1       Running   0          33s
```

```bash
mac:$ kubectl get services -l app=mqtts
NAME      TYPE           CLUSTER-IP     EXTERNAL-IP       PORT(S)          AGE
mqtts     LoadBalancer   10.0.170.212   WWW.XXX.YYY.ZZZ   8883:30187/TCP   3m
```

```bash
mac:$ kubectl exec vernemq-0 -- vmq-admin cluster show
+---------------------------------------------------+-------+
|                       Node                        |Running|
+---------------------------------------------------+-------+
|VerneMQ@vernemq-0.vernemq.default.svc.cluster.local| true  |
|VerneMQ@vernemq-1.vernemq.default.svc.cluster.local| true  |
|VerneMQ@vernemq-2.vernemq.default.svc.cluster.local| true  |
+---------------------------------------------------+-------+
```

```bash
mac:$ az network dns record-set a add-record --resource-group dns-zone --zone-name "cloudconductor.jp" --record-set-name "mqtt" --ipv4-address "WWW.XXX.YYY.ZZZ"
```

* XXXXXXXXXXXX is the password of "iotagent"
```text
mac:$ mosquitto_sub -h mqtt.cloudconductor.jp -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
```

## start mondodb cluster on AKS

```bash
mac:$ kubectl apply -f mongodb/mongodb-cluster-azure.yaml
```

```bash
mac:$ kubectl get PersistentVolumeClaims -l app=mongodb
NAME                              STATUS    VOLUME                                     CAPACITY   ACCESS MODES   STORAGECLASS      AGE
mongodb-storage-claim-mongodb-0   Bound     pvc-aafd0623-5f1c-11e8-92d3-0a58ac1f13ee   30Gi       RWO            managed-premium   8m
mongodb-storage-claim-mongodb-1   Bound     pvc-121ca1c4-5f1d-11e8-92d3-0a58ac1f13ee   30Gi       RWO            managed-premium   5m
mongodb-storage-claim-mongodb-2   Bound     pvc-72a215e7-5f1d-11e8-92d3-0a58ac1f13ee   30Gi       RWO            managed-premium   2m
```

```bash
mac:$ kubectl get pods -l app=mongodb
NAME        READY     STATUS    RESTARTS   AGE
mongodb-0   2/2       Running   0          9m
mongodb-1   2/2       Running   0          6m
mongodb-2   2/2       Running   0          4m
```

```bash
mac:$ kubectl get services -l app=mongodb
NAME      TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)     AGE
mongodb   ClusterIP   None         <none>        27017/TCP   10m
```

```bash
mac:$ kubectl exec mongodb-0 -c mongodb -- mongo --eval 'printjson(rs.status().members.map(function(e) {return {name: e.name, stateStr:e.stateStr};}))'
MongoDB shell version v3.6.5
connecting to: mongodb://127.0.0.1:27017
MongoDB server version: 3.6.5
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

## start ambassador on AKS

```bash
mac:$ docker run -it -v $(pwd)/secrets:/etc/letsencrypt certbot/certbot certonly --manual --domain api.cloudconductor.jp --email nobuyuki.matsui@gmail.com --agree-tos --manual-public-ip-logging-ok --preferred-challenges dns
```

* Another terminal
```bash
mac-another:$ az network dns record-set txt add-record --resource-group dns-zone --zone-name "cloudconductor.jp" --record-set-name "_acme-challenge.api" --value "YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY"
mac-another:$ az network dns record-set list --resource-group dns-zone --zone-name "cloudconductor.jp" | jq '.[] | {"fqdn": .fqdn, "type": .type}'
{
  "fqdn": "cloudconductor.jp.",
  "type": "Microsoft.Network/dnszones/NS"
}
{
  "fqdn": "cloudconductor.jp.",
  "type": "Microsoft.Network/dnszones/SOA"
}
{
  "fqdn": "_acme-challenge.api.cloudconductor.jp.",
  "type": "Microsoft.Network/dnszones/TXT"
}
{
  "fqdn": "mqtt.cloudconductor.jp.",
  "type": "Microsoft.Network/dnszones/A"
}
```

* Press 'ENTER' at original terminal when `_acme-challenge.api.cloudconductor.jp.` txt record is created.

* After completion of certbot
```bash
mac-another:$ az network dns record-set txt remove-record --resource-group dns-zone --zone-name "cloudconductor.jp" --record-set-name "_acme-challenge.api" --value "YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY"
```

```bash
mac:$ kubectl create secret tls ambassador-certs --cert=$(pwd)/secrets/live/api.cloudconductor.jp/fullchain.pem --key=$(pwd)/secrets/live/api.cloudconductor.jp/privkey.pem
```

```bash
mac:$ kubectl apply -f ambassador/ambassador-azure.yaml
```

```bash
mac:$ kubectl get pods -l service=ambassador
NAME                          READY     STATUS    RESTARTS   AGE
ambassador-79768bd968-7n4vl   2/2       Running   0          50s
ambassador-79768bd968-cpl5j   2/2       Running   0          50s
ambassador-79768bd968-h2ct2   2/2       Running   0          50s
```

```bash
mac:$ kubectl get services -l service=ambassador
NAME         TYPE           CLUSTER-IP    EXTERNAL-IP       PORT(S)                      AGE
ambassador   LoadBalancer   10.0.191.59   www.xxx.yyy.zzz   443:30357/TCP,80:32755/TCP   4m
```

```bash
mac:$ az network dns record-set a add-record --resource-group dns-zone --zone-name "cloudconductor.jp" --record-set-name "api" --ipv4-address "www.xxx.yyy.zzz"
```

```bash
mac:$ curl -i https://api.cloudconductor.jp
HTTP/1.1 404 Not Found
date: Fri, 25 May 2018 00:47:41 GMT
server: envoy
content-length: 0
```

* create random string
```bash
mac:$ cat /dev/urandom | LC_CTYPE=C tr -dc 'a-zA-Z0-9' | head -c 32
```

* create `secrets/auth-tokens.json` like below:
```json:secrets/auth-tokens.json
{
  "bearer_tokens": [
      {
          "token": "iRGTsKKHwgjf4rR2XMSN3oE9Dhm6ym3O",
          "allowed_paths": ["^/orion/.*$", "^/idas/.*$"]
      }, {
          "token": "4Xc1GFa2D8zkZRbkdygm902oGYeUAJno",
          "allowed_paths": ["^/idas/.*$"]
      }
  ],
  "basic_auths": [
      {
          "username": "user1",
          "password": "P@ssw0rd",
          "allowed_paths": ["/controller/web/"]
      }, {
          "username": "user2",
          "password": "P@ssw0rd",
          "allowed_paths": ["/controller/web/"]
      }
  ]
}
```

```bash
mac:$ kubectl create secret generic auth-tokens --from-file=./secrets/auth-tokens.json
```

```bash
mac:$ kubectl get secrets
NAME                                                          TYPE                                  DATA      AGE
ambassador-certs                                              kubernetes.io/tls                     2         4m
auth-tokens                                                   Opaque                                1         1s
default-token-2klq6                                           kubernetes.io/service-account-token   3         28m
fiware-etcd-etcd-operator-etcd-backup-operator-token-ghsq2    kubernetes.io/service-account-token   3         14m
fiware-etcd-etcd-operator-etcd-operator-token-qm2xp           kubernetes.io/service-account-token   3         14m
fiware-etcd-etcd-operator-etcd-restore-operator-token-4wzkm   kubernetes.io/service-account-token   3         14m
vernemq-certifications                                        Opaque                                3         8m
vernemq-passwd                                                Opaque                                1         8m
```

```bash
mac:$ az acr login --name fiwareacr
mac:$ docker build -t ${REPOSITORY}/tech-sketch/fiware-bearer-auth:0.1.0 ./ambassador/fiware-bearer-auth
mac:$ docker push ${REPOSITORY}/tech-sketch/fiware-bearer-auth:0.1.0
```

```bash
mac:$ az acr repository list --name fiwareacr --output table
Result
------------------------------
tech-sketch/fiware-bearer-auth
```

```bash
mac:$ envsubst < ambassador/bearer-auth.yaml | kubectl apply -f -
```

```bash
mac:$ kubectl get pods -l pod=bearer-auth
NAME                           READY     STATUS    RESTARTS   AGE
bearer-auth-6fffdbd9c9-7kkpr   1/1       Running   0          56s
bearer-auth-6fffdbd9c9-qxw6m   1/1       Running   0          56s
bearer-auth-6fffdbd9c9-sdn5b   1/1       Running   0          56s
```

```bash
mac:$ kubectl get services -l service=bearer-auth
NAME          TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)    AGE
bearer-auth   ClusterIP   10.0.129.102   <none>        3000/TCP   2m
```

## start orion on AKS

```bash
mac:$ kubectl apply -f orion/orion.yaml
```

```bash
mac:$ kubectl get pods -l app=orion
NAME                     READY     STATUS    RESTARTS   AGE
orion-54f5cdcb5d-d2pt5   1/1       Running   0          56s
orion-54f5cdcb5d-hv274   1/1       Running   0          56s
orion-54f5cdcb5d-xbnx2   1/1       Running   0          56s
```

```bash
mac:$ kubectl get services -l app=orion
NAME      TYPE        CLUSTER-IP    EXTERNAL-IP   PORT(S)    AGE
orion     ClusterIP   10.0.44.126   <none>        1026/TCP   1m
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" https://api.cloudconductor.jp/orion/v2/entities/
HTTP/1.1 200 OK
content-length: 2
content-type: application/json
fiware-correlator: 4731eb48-4dc1-11e8-b1a2-0a580af4010a
date: Wed, 02 May 2018 04:28:35 GMT
x-envoy-upstream-service-time: 5
server: envoy

[]
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" https://api.cloudconductor.jp/orion/v2/subscriptions/
HTTP/1.1 200 OK
content-length: 2
content-type: application/json
fiware-correlator: 5a4ecc6e-4dc1-11e8-b1a2-0a580af4010a
date: Wed, 02 May 2018 04:29:07 GMT
x-envoy-upstream-service-time: 2
server: envoy

[]
```

## start idas on AKS

```bash
mac:$ az acr login --name fiwareacr
mac:$ docker build -t ${REPOSITORY}/tech-sketch/fiware-mqtt-msgfilter:0.1.0 idas/fiware-mqtt-msgfilter/
mac:$ docker push ${REPOSITORY}/tech-sketch/fiware-mqtt-msgfilter:0.1.0
```

```bash
mac:$ az acr repository list --name fiwareacr --output table
Result
---------------------------------
tech-sketch/fiware-bearer-auth
tech-sketch/fiware-mqtt-msgfilter
```

```bash
mac:$ envsubst < idas/mqtt-msgfilter.yaml | kubectl apply -f -
```

```bash
mac:$ kubectl get pods -l pod=mqtt-msgfilter
NAME                              READY     STATUS    RESTARTS   AGE
mqtt-msgfilter-6f76445596-cmbqz   1/1       Running   0          26s
mqtt-msgfilter-6f76445596-wrhzb   1/1       Running   0          26s
mqtt-msgfilter-6f76445596-znnvg   1/1       Running   0          26s
```

```bash
mac:$ kubectl get services -l service=mqtt-msgfilter
NAME             TYPE        CLUSTER-IP    EXTERNAL-IP   PORT(S)    AGE
mqtt-msgfilter   ClusterIP   10.0.133.42   <none>        5001/TCP   43s
```

* replace `<<password_of_iotagent>>` to the password of "iotagent"
```bash
mac:$ env IOTAGENT_PASSWORD=XXXXXXXXXXXX envsubst < idas/iotagent-ul/config.js.template > idas/iotagent-ul/config.js
```

```bash
mac:$ az acr login --name fiwareacr
mac:$ docker build -t ${REPOSITORY}/tech-sketch/iotagent-ul:1.6.0 idas/iotagent-ul/
mac:$ docker push ${REPOSITORY}/tech-sketch/iotagent-ul:1.6.0
```

```bash
mac:$ az acr repository list --name fiwareacr --output table
Result
---------------------------------
tech-sketch/fiware-bearer-auth
tech-sketch/fiware-mqtt-msgfilter
tech-sketch/iotagent-ul
```

```bash
mac:$ envsubst < idas/iotagent-ul.yaml | kubectl apply -f -
```

```bash
mac:$ kubectl get pods -l app=iotagent-ul
NAME                           READY     STATUS    RESTARTS   AGE
iotagent-ul-79685b64bf-8krps   1/1       Running   0          3m
iotagent-ul-79685b64bf-m6nlg   1/1       Running   0          3m
iotagent-ul-79685b64bf-mjpbl   1/1       Running   0          3m
```

```bash
mac:$ kubectl get services -l app=iotagent-ul
NAME          TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)             AGE
iotagent-ul   ClusterIP   10.0.180.155   <none>        4041/TCP,7896/TCP   43s
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" https://api.cloudconductor.jp/idas/ul20/manage/iot/services/
HTTP/1.1 200 OK
x-powered-by: Express
fiware-correlator: c114fc5e-b4a2-40f6-b7fe-1d68369784e5
content-type: application/json; charset=utf-8
content-length: 25
etag: W/"19-WMYe0U6ocKhQjp+oaVnMHLdbylc"
date: Wed, 02 May 2018 06:16:18 GMT
x-envoy-upstream-service-time: 9
server: envoy

{"count":0,"services":[]}
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.cloudconductor.jp/idas/ul20/manage/iot/devices/
HTTP/1.1 200 OK
x-powered-by: Express
fiware-correlator: 1d1ee2f1-83e4-454e-8ef5-a10fd49630ab
content-type: application/json; charset=utf-8
content-length: 24
etag: W/"18-90KiBjq8YRQpT/NsVf7vo89XXWw"
date: Wed, 02 May 2018 06:16:58 GMT
x-envoy-upstream-service-time: 8
server: envoy

{"count":0,"devices":[]}
```

## start cygnus on AKS

```bash
mac:$ az acr login --name fiwareacr
mac:$ docker build -t ${REPOSITORY}/tech-sketch/cygnus-ngsi:1.8.0 ./cygnus/fiware-cygnus/
mac:$ docker push ${REPOSITORY}/tech-sketch/cygnus-ngsi:1.8.0
```

```bash
mac:$ az acr repository list --name fiwareacr --output table
Result
---------------------------------
tech-sketch/cygnus-ngsi
tech-sketch/fiware-bearer-auth
tech-sketch/fiware-mqtt-msgfilter
tech-sketch/iotagent-ul
```

```bash
mac:$ envsubst < cygnus/cygnus.yaml | kubectl apply -f -
```

```bash
mac:$ kubectl get pods -l app=cygnus
NAME                      READY     STATUS    RESTARTS   AGE
cygnus-5c68fb6578-fdmtg   1/1       Running   0          44s
cygnus-5c68fb6578-stmds   1/1       Running   0          44s
cygnus-5c68fb6578-z85lp   1/1       Running   0          44s
```

```bash
mac:$ kubectl get services -l app=cygnus
NAME      TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)             AGE
cygnus    ClusterIP   10.103.255.240   <none>        5050/TCP,8081/TCP   1m
```

## start cmd-proxy for 'turtlesim' on AKS

```bash
mac:$ az acr login --name fiwareacr
mac:$ docker build --build-arg PORT=8888 -t ${REPOSITORY}/tech-sketch/fiware-cmd-proxy:0.1.0 ./controller/fiware-cmd-proxy/
mac:$ docker push ${REPOSITORY}/tech-sketch/fiware-cmd-proxy:0.1.0
```

```bash
mac:$ az acr repository list --name fiwareacr --output table
Result
---------------------------------
tech-sketch/cygnus-ngsi
tech-sketch/fiware-bearer-auth
tech-sketch/fiware-cmd-proxy
tech-sketch/fiware-mqtt-msgfilter
tech-sketch/iotagent-ul
```

* create three 'cmd-proxy' pods and a 'cmd-proxy' service to control 'turtlesim'.
```bash
mac:$ env FIWARE_SERVICE=demo1 FIWARE_SERVICEPATH=/ ROBOT_ID=turtlesim ROBOT_TYPE=demo1 envsubst < controller/fiware-cmd-proxy.yaml | kubectl apply -f -
```

```bash
mac:$ kubectl get pods -l pod=cmd-proxy
NAME                         READY     STATUS    RESTARTS   AGE
cmd-proxy-58c756cbcd-q7jzz   1/1       Running   0          5s
cmd-proxy-58c756cbcd-rs75j   1/1       Running   0          5s
cmd-proxy-58c756cbcd-v9ptr   1/1       Running   0          5s
```

```bash
mac:$ kubectl get services -l service=cmd-proxy
NAME        TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)    AGE
cmd-proxy   ClusterIP   10.0.208.226   <none>        8888/TCP   34s
```

## register "demo1" service

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.cloudconductor.jp/idas/ul20/manage/iot/services/ -X POST -d @- <<__EOS__
{
  "services": [
    {
      "apikey": "demo1",
      "cbroker": "http://orion:1026",
      "resource": "/iot/d",
      "entity_type": "demo1"
    }
  ]
}
__EOS__
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" https://api.cloudconductor.jp/idas/ul20/manage/iot/services/ | jq .
{
  "count": 1,
  "services": [
    {
      "_id": "5aea5264d95cfc000124890b",
      "subservice": "/",
      "service": "demo1",
      "apikey": "demo1",
      "resource": "/iot/d",
      "__v": 0,
      "attributes": [],
      "lazy": [],
      "commands": [],
      "entity_type": "demo1",
      "internal_attributes": [],
      "static_attributes": []
    }
  ]
}
```

## register "gamepad" device

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.cloudconductor.jp/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
{
  "devices": [
    {
      "device_id": "gamepad",
      "entity_name": "gamepad",
      "entity_type": "demo1",
      "timezone": "Asia/Tokyo",
      "protocol": "UL20",
      "attributes": [
        {
          "name": "button",
          "type": "string"
        }
      ],
      "transport": "MQTT"
    }
  ]
}
__EOS__
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.cloudconductor.jp/idas/ul20/manage/iot/devices/gamepad/ | jq .
{
  "device_id": "gamepad",
  "service": "demo1",
  "service_path": "/",
  "entity_name": "gamepad",
  "entity_type": "demo1",
  "transport": "MQTT",
  "attributes": [
    {
      "object_id": "button",
      "name": "button",
      "type": "string"
    }
  ],
  "lazy": [],
  "commands": [],
  "static_attributes": [],
  "protocol": "UL20"
}
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.cloudconductor.jp/orion/v2/entities/gamepad/ | jq .
{
  "id": "gamepad",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": " ",
    "metadata": {}
  },
  "button": {
    "type": "string",
    "value": " ",
    "metadata": {}
  }
}
```

## test publishing an attribute from gamepad to orion through MQTT and idas

* XXXXXXXXXXXX is the password of "iotagent"
```text
mac:$ mosquitto_sub -h mqtt.cloudconductor.jp -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
```

* YYYYYYYYYYYY is the password of "raspberrypi"
```bash
raspberrypi:raspi_gamepad$ env MQTT_HOST=mqtt.cloudconductor.jp RASPI_RASSWORD=YYYYYYYYYYYY envsubst < conf/pxkwcr-azure.yaml.template > conf/pxkwcr-azure.yaml
raspberrypi:raspi_gamepad$ ./main.py pxkwcr-azure
2018/05/06 11:46:04 [   INFO] __main__ - run script using pxkwcr.yaml
2018/05/06 11:46:04 [   INFO] src.controller - initialized FUJIWORK PXKWCR Controller
2018/05/06 11:46:04 [   INFO] src.controller - start publishing...
...
```

* press 'circle' button of gamepad
```bash
raspberrypi:raspi_gamepad$ ./main.py
...
2018/05/06 11:46:15 [   INFO] src.controller - published "2018-05-06T02:46:14.876494+0000|button|circle" to "/demo1/gamepad/attrs"
2018/05/06 11:46:15 [   INFO] src.controller - connected mqtt broker[mqtt.cloudconductor.jp:8883], response_code=0
...
```

```bash
mac:$ mosquitto_sub -h mqtt.cloudconductor.jp -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
...
Client mosqsub|60435-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/gamepad/attrs', ... (45 bytes))
2018-05-06T02:46:14.876494+0000|button|circle
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.cloudconductor.jp/orion/v2/entities/gamepad/ | jq .
{
  "id": "gamepad",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-05-06T02:46:14.876494+0000",
    "metadata": {}
  },
  "button": {
    "type": "string",
    "value": "circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-06T02:46:14.876494+0000"
      }
    }
  }
}
```

## register "turtlesim" device

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.cloudconductor.jp/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
{
  "devices": [
    {
      "device_id": "turtlesim",
      "entity_name": "turtlesim",
      "entity_type": "demo1",
      "timezone": "Asia/Tokyo",
      "protocol": "UL20",
      "attributes": [
        {
          "name": "temperature",
          "type": "float32"
        }
      ],
      "commands": [
        {
          "name": "move",
          "type": "string"
        }
      ],
      "transport": "MQTT"
    }
  ]
}
__EOS__
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.cloudconductor.jp/idas/ul20/manage/iot/devices/turtlesim/ | jq .
{
  "device_id": "turtlesim",
  "service": "demo1",
  "service_path": "/",
  "entity_name": "turtlesim",
  "entity_type": "demo1",
  "transport": "MQTT",
  "attributes": [
    {
      "object_id": "temperature",
      "name": "temperature",
      "type": "float32"
    }
  ],
  "lazy": [],
  "commands": [
    {
      "object_id": "move",
      "name": "move",
      "type": "string"
    }
  ],
  "static_attributes": [],
  "protocol": "UL20"
}
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.cloudconductor.jp/orion/v2/entities/turtlesim/ | jq .
{
  "id": "turtlesim",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": " ",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": " ",
    "metadata": {}
  },
  "move_status": {
    "type": "commandStatus",
    "value": "UNKNOWN",
    "metadata": {}
  },
  "temperature": {
    "type": "float32",
    "value": " ",
    "metadata": {}
  },
  "move": {
    "type": "string",
    "value": "",
    "metadata": {}
  }
}
```

## test publishing an attribute from turtlesim to orion through MQTT and idas, subscribing a cmd from orion to turtlesim and publishing a cmdexe from turtlesim to orion through MQTT and idas

* XXXXXXXXXXXX is the password of "iotagent"
```bash
mac:$ mosquitto_sub -h mqtt.cloudconductor.jp -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
```

* start X on ros server and login X using RDP

* open terminal1 and start `roscore`
```bash
ros-terminal1:ros_ws$ source devel/setup.bash
ros-terminal1:ros_ws$ roscore
...
```

* open terminal2 and start `turtlesim`
```bash
ros-terminal2:ros_ws$ source devel/setup.bash
ros-terminal2:ros_ws$ rosrun turtlesim turtlesim_node
...
```

* open terminal3 and start `turtlesim_operator`
```bash
ros-terminal3:ros_ws$ env MQTT_HOST=mqtt.cloudconductor.jp TURTLESIM_PASSWORD=Turtles1mP@ssw0rd envsubst < src/turtlesim_operator/config/params-azure.yaml.template > src/turtlesim_operator/config/params.yaml
ros-terminal3:ros_ws$ source devel/setup.bash
ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
... logging to /home/ubuntu/.ros/log/872b8e06-5fbf-11e8-bdde-02dff3ffcd9e/roslaunch-ubuntu-xenial-12540.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu-xenial:42084/

SUMMARY
========

PARAMETERS
...
running rosparam delete /command_sender/
running rosparam delete /attribute_receiver/
process[command_sender-1]: started with pid [12567]
process[attribute_receiver-2]: started with pid [12568]
[INFO] [1527215493.418127]: [__main__:main] Start node : command_sender_node.py [mode=production]
[INFO] [1527215493.420951]: [__main__:main] Start node : attribute_receiver_node.py
[INFO] [1527215493.426882]: [turtlesim_operator.command_sender:CommandSender.connect] Connect mqtt broker
[INFO] [1527215493.432872]: [turtlesim_operator.attribute_receiver:AttributeReceiver.connect] Connect mqtt broker
[INFO] [1527215494.574014]: [turtlesim_operator.attribute_receiver:AttributeReceiver.start] AttributeReceiver start : attribute_receiver_node.py
[INFO] [1527215494.575131]: [turtlesim_operator.command_sender:CommandSender.start] CommandSender start : command_sender_node.py
[INFO] [1527215494.878739]: [turtlesim_operator.attribute_receiver:AttributeReceiver._on_connect] mqtt connect status=0
[INFO] [1527215494.880586]: [turtlesim_operator.command_sender:CommandSender._on_connect] mqtt connect status=0
...
```

* send 'circle' cmd to 'turtlesim' entity
```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" -H "Content-Type: application/json" https://api.cloudconductor.jp/orion/v1/updateContext -d @-<<__EOS__ | jq .
{
  "contextElements": [
    {
      "id": "turtlesim",
      "isPattern": "false",
      "type": "demo1",
      "attributes": [
        {
          "name": "move",
          "type": "string",
          "value": "circle"
        }
      ]
    }
  ],
  "updateAction": "UPDATE"
}
__EOS__
```

```bash
mac:$ mosquitto_sub -h mqtt.cloudconductor.jp -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
Client mosqsub|22161-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/cmd', ... (21 bytes))
turtlesim@move|circle
Client mosqsub|22161-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/cmdexe', ... (30 bytes))<Paste>
...
```

```bash
ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
...
[INFO] [1527219554.229942]: [turtlesim_operator.command_sender:CommandSender._on_message] received message from mqtt: turtlesim@move|circle
[INFO] [1527219554.232153]: [turtlesim_operator.command_sender:CommandSender._do_circle] do circle
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.cloudconductor.jp/orion/v2/entities/turtlesim/ | jq .
{
  "id": "turtlesim",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-05-25T03:39:14.00Z",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": "executed circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T03:39:14.339Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "OK",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T03:39:14.339Z"
      }
    }
  },
  "temperature": {
    "type": "float32",
    "value": " ",
    "metadata": {}
  },
  "move": {
    "type": "string",
    "value": "",
    "metadata": {}
  }
}
```

* open terminal4 and publish `templerature` to rostopic
```bash
ros-terminal4:ros_ws$ source devel/setup.bash
ros-terminal4:ros_ws$ rostopic pub -1 /turtle1/temperature std_msgs/Float32 -- 25.3
```

```bash
ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
...
[INFO] [1527221663.972928]: [turtlesim_operator.attribute_receiver:AttributeReceiver._on_receive] received message from ros : 25.2999992371
...
```

```bash
mac:$ mosquitto_sub -h mqtt.cloudconductor.jp -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
Client mosqsub|22161-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/attrs', ... (57 bytes))
2018-05-25T04:14:23.973467+0000|temperature|25.2999992371
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.cloudconductor.jp/orion/v2/entities/turtlesim/ | jq .
{
  "id": "turtlesim",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-05-25T04:14:23.973467+0000",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": " ",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T03:39:14.339Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "UNKNOWN",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T03:39:14.339Z"
      }
    }
  },
  "temperature": {
    "type": "float32",
    "value": "12.1000003815",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T04:30:16.726545+0000"
      }
    }
  },
  "move": {
    "type": "string",
    "value": "",
    "metadata": {}
  }
}
```

## register "gopigo" device (if gopigo is available)

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.cloudconductor.jp/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
{
  "devices": [
    {
      "device_id": "gopigo",
      "entity_name": "gopigo",
      "entity_type": "demo1",
      "timezone": "Asia/Tokyo",
      "protocol": "UL20",
      "commands": [
        {
          "name": "move",
          "type": "string"
        }
      ],
      "transport": "MQTT"
    }
  ]
}
__EOS__
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.cloudconductor.jp/idas/ul20/manage/iot/devices/gopigo/ | jq .
{
  "device_id": "gopigo",
  "service": "demo1",
  "service_path": "/",
  "entity_name": "gopigo",
  "entity_type": "demo1",
  "transport": "MQTT",
  "attributes": [],
  "lazy": [],
  "commands": [
    {
      "object_id": "move",
      "name": "move",
      "type": "string"
    }
  ],
  "static_attributes": [],
  "protocol": "UL20"
}
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.cloudconductor.jp/orion/v2/entities/gopigo/ | jq .
{
  "id": "gopigo",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": " ",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": " ",
    "metadata": {}
  },
  "move_status": {
    "type": "commandStatus",
    "value": "UNKNOWN",
    "metadata": {}
  },
  "move": {
    "type": "string",
    "value": "",
    "metadata": {}
  }
}
```

## test subscribing a cmd from orion to gopigo and publishing a cmdexe from gopigo to orion through MQTT and idas (if gopigo is available)

* XXXXXXXXXXXX is the password of "iotagent"
```bash
mac:$ mosquitto_sub -h mqtt.cloudconductor.jp -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
```

* ssh to gopigo on terminal1 and start `roscore`
```bash
ros-terminal1:gopigo_ws$ source devel/setup.bash
ros-terminal1:gopigo_ws$ roscore
...
```

* ssh to gopigo on terminal2 and start `ros_gopigo`
* ZZZZZZZZZZZZ is the password of "gopigo"
```bash
ros-terminal2:gopigo_ws$ env MQTT_HOST=mqtt.cloudconductor.jp GOPIGO_PASSWORD=ZZZZZZZZZZZZ envsubst < src/ros_gopigo/config/params-azure.yaml.template > src/ros_gopigo/config/params.yaml
ros-terminal2:gopigo_ws$ source devel/setup.bash
ros-terminal2:gopigo_ws$ roslaunch ros_gopigo ros_gopigo.launch
... logging to /home/ubuntu/.ros/log/2ed86872-5f2f-11e8-9dff-84afec5283f0/roslaunch-ubuntu-4727.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu:38978/

SUMMARY
========

PARAMETERS
...
running rosparam delete /gopigo_node/
ERROR: parameter [/gopigo_node] is not set
running rosparam delete /fiware2gopigo_node/
ERROR: parameter [/fiware2gopigo_node] is not set
process[gopigo_node-1]: started with pid [4754]
process[fiware2gopigo_node-2]: started with pid [4755]
[INFO] [1527152610.456460]: [__main__:main] Start node : ros_gopigo
[INFO] [1527152610.477046]: [__main__:main] Start node : fiware2gopigo
[INFO] [1527152610.514471]: [ros_gopigo.gopigo_impl:Gopigo.start] Gopigo start: ros_gopigo
[INFO] [1527152610.544036]: [ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo.connect] Connect to MQTT broker
[INFO] [1527152611.204234]: [ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo.start] Fiware2Gopigo start: fiware2gopigo
[INFO] [1527152611.399812]: [ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo._on_connect] connected to MQTT Broker, status: 0
...
```

* send 'circle' cmd to 'gopigo' entity
```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" -H "Content-Type: application/json" https://api.cloudconductor.jp/orion/v1/updateContext -d @-<<__EOS__ | jq .
{
  "contextElements": [
    {
      "id": "gopigo",
      "isPattern": "false",
      "type": "demo1",
      "attributes": [
        {
          "name": "move",
          "type": "string",
          "value": "circle"
        }
      ]
    }
  ],
  "updateAction": "UPDATE"
}
__EOS__
```

```bash
mac:$ mosquitto_sub -h mqtt.cloudconductor.jp -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
Client mosqsub|77956-Nobuyukin received PUBLISH (d0, q0, r0, m0, '/demo1/gopigo/cmd', ... (21 bytes))
gopigo@move|circle
Client mosqsub|77956-Nobuyukin received PUBLISH (d0, q0, r0, m0, '/demo1/gopigo/cmdexe', ... (28 bytes))
gopigo@move|executed circle
...
```

```bash
ros-terminal2:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
...
[INFO] [1527152726.487569]: [ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo._on_message] received message from mqtt: gopigo@move|circle
[INFO] [1527152726.499365]: [ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo._do_circle] do circle
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.cloudconductor.jp/orion/v2/entities/gopigo/ | jq .
{
  "id": "gopigo",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-05-24T09:05:26.00Z",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": "executed circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-24T09:05:26.799Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "OK",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-24T09:05:26.799Z"
      }
    }
  },
  "move": {
    "type": "string",
    "value": "",
    "metadata": {}
  }
}
```

## register cygnus

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.cloudconductor.jp/orion/v2/subscriptions/ -X POST -d @- <<__EOS__
{
  "subject": {
    "entities": [{
      "idPattern": "gamepad.*",
      "type": "demo1"
    }]
  },
  "notification": {
    "http": {
      "url": "http://cygnus:5050/notify"
    },
    "attrs": ["button"],
    "attrsFormat": "legacy"
  }
}
__EOS__
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.cloudconductor.jp/orion/v2/subscriptions/ -X POST -d @- <<__EOS__
{
  "subject": {
    "entities": [{
      "idPattern": "turtlesim.*",
      "type": "demo1"
    }]
  },
  "notification": {
    "http": {
      "url": "http://cygnus:5050/notify"
    },
    "attrs": ["temperature"],
    "attrsFormat": "legacy"
  }
}
__EOS__
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" https://api.cloudconductor.jp/orion/v2/subscriptions/ | jq .
[
  {
    "id": "5b06745991cc31d4197bd440",
    "status": "active",
    "subject": {
      "entities": [
        {
          "idPattern": "gamepad.*",
          "type": "demo1"
        }
      ],
      "condition": {
        "attrs": []
      }
    },
    "notification": {
      "timesSent": 1,
      "lastNotification": "2018-05-24T08:14:17.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      }
    }
  },
  {
    "id": "5b079158a7bb4515a7bbf6cc",
    "status": "active",
    "subject": {
      "entities": [
        {
          "idPattern": "turtlesim.*",
          "type": "demo1"
        }
      ],
      "condition": {
        "attrs": []
      }
    },
    "notification": {
      "timesSent": 1,
      "lastNotification": "2018-05-25T04:30:16.00Z",
      "attrs": [
        "temperature"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      }
    }
  }
]
```

```bash
mac:$ kubectl exec mongodb-0 -c mongodb -- mongo --eval 'printjson(db.getMongo().getDBNames())'
MongoDB shell version v3.6.5
connecting to: mongodb://127.0.0.1:27017
MongoDB server version: 3.6.5
[
	"admin",
	"config",
	"iotagentul",
	"local",
	"orion",
	"orion-demo1",
	"sth_demo1"
]
```

```bash
mac:$ kubectl exec mongodb-0 -c mongodb -- mongo sth_demo1 --eval 'printjson(db.getCollectionNames())'
MongoDB shell version v3.6.5
connecting to: mongodb://127.0.0.1:27017/sth_demo1
MongoDB server version: 3.6.5
[ "sth_/_gamepad_demo1", "sth_/_turtlesim_demo1" ]
```

```bash
mac:$ kubectl exec mongodb-0 -c mongodb -- mongo sth_demo1 --eval 'db.getCollection("sth_/_gamepad_demo1").find()'
MongoDB shell version v3.6.5
connecting to: mongodb://127.0.0.1:27017/sth_demo1
MongoDB server version: 3.6.5
{ "_id" : ObjectId("5b06745ac7f7c0000aac9659"), "recvTime" : ISODate("2018-05-24T08:14:17.733Z"), "attrName" : "button", "attrType" : "string", "attrValue" : "circle" }
```

```bash
mac:$ kubectl exec mongodb-0 -c mongodb -- mongo sth_demo1 --eval 'db.getCollection("sth_/_turtlesim_demo1").find()'
MongoDB shell version v3.6.5
connecting to: mongodb://127.0.0.1:27017/sth_demo1
MongoDB server version: 3.6.5
{ "_id" : ObjectId("5b0791597a204b000a487367"), "recvTime" : ISODate("2018-05-25T04:30:16.598Z"), "attrName" : "temperature", "attrType" : "float32", "attrValue" : "12.1000003815" }
```

## register fiware-cmd-proxy

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.cloudconductor.jp/orion/v2/subscriptions/ -X POST -d @- <<__EOS__
{
  "subject": {
    "entities": [{
      "idPattern": "gamepad.*",
      "type": "demo1"
    }]
  },
  "notification": {
    "http": {
      "url": "http://cmd-proxy:8888/gamepad/"
    },
    "attrs": ["button"]
  }
}
__EOS__
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" https://api.cloudconductor.jp/orion/v2/subscriptions/ | jq .
[
  {
    "id": "5b06745991cc31d4197bd440",
    "status": "active",
    "subject": {
      "entities": [
        {
          "idPattern": "gamepad.*",
          "type": "demo1"
        }
      ],
      "condition": {
        "attrs": []
      }
    },
    "notification": {
      "timesSent": 2,
      "lastNotification": "2018-05-24T08:42:24.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      },
      "lastSuccess": "2018-05-24T08:42:24.00Z"
    }
  },
  {
    "id": "5b079158a7bb4515a7bbf6cc",
    "status": "active",
    "subject": {
      "entities": [
        {
          "idPattern": "turtlesim.*",
          "type": "demo1"
        }
      ],
      "condition": {
        "attrs": []
      }
    },
    "notification": {
      "timesSent": 1,
      "lastNotification": "2018-05-25T04:30:16.00Z",
      "attrs": [
        "temperature"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      }
    }
  },
  {
    "id": "5b06878991cc31d4197bd441",
    "status": "active",
    "subject": {
      "entities": [
        {
          "idPattern": "gamepad.*",
          "type": "demo1"
        }
      ],
      "condition": {
        "attrs": []
      }
    },
    "notification": {
      "timesSent": 1,
      "lastNotification": "2018-05-24T09:36:09.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "normalized",
      "http": {
        "url": "http://cmd-proxy:8888/gamepad/"
      },
      "lastSuccess": "2018-05-24T09:36:09.00Z"
    }
  }
]
```

## control 'turtlesim'

### confirm ros server
1. confirm that `rocore` is running on ros server
1. confirm that `turtlesim` is running on ros server
1. confirm that `turtlesim_operator` is running on ros server

### gamepad
1. confirm that `main.py` is running on raspberrypi
1. when you press a button of gamepad, 'turtle' moves according to the pressed button

### web controller
1. access to https://api.cloudconductor.jp/controller/web/
1. when you press a button of web controller, gopigo moves according to the pressed button

## use 'gopigo' instead of 'turtlesim' (if gopigo is available)

```bash
NAME                              DESIRED   CURRENT   UP-TO-DATE   AVAILABLE   AGE
deployment.extensions/cmd-proxy   3         3         3            3           11m

NAME                                         DESIRED   CURRENT   READY     AGE
replicaset.extensions/cmd-proxy-57756c46cd   3         3         3         11m

NAME                             READY     STATUS    RESTARTS   AGE
pod/cmd-proxy-57756c46cd-bxxv8   1/1       Running   0          11m
pod/cmd-proxy-57756c46cd-mfj7f   1/1       Running   0          11m
pod/cmd-proxy-57756c46cd-xqqvz   1/1       Running   0          11m
nmatsui@:container-centric-fiware-demonstration (feature/web_controller_using_basicauth *=)$
```

```bash
mac:$ env FIWARE_SERVICE=demo1 FIWARE_SERVICEPATH=/ ROBOT_ID=gopigo ROBOT_TYPE=demo1 envsubst < controller/fiware-cmd-proxy.yaml | kubectl apply -f -
```

```bash
$ kubectl get deployments,replicasets,pods -l pod=cmd-proxy
NAME                              DESIRED   CURRENT   UP-TO-DATE   AVAILABLE   AGE
deployment.extensions/cmd-proxy   3         3         3            3           14m

NAME                                         DESIRED   CURRENT   READY     AGE
replicaset.extensions/cmd-proxy-57756c46cd   0         0         0         14m
replicaset.extensions/cmd-proxy-74766cf8     3         3         3         19s

NAME                           READY     STATUS    RESTARTS   AGE
pod/cmd-proxy-74766cf8-5xnq7   1/1       Running   0          16s
pod/cmd-proxy-74766cf8-fpcvf   1/1       Running   0          12s
pod/cmd-proxy-74766cf8-v8g25   1/1       Running   0          19s
```

## control 'gopigo'

### confirm gopigo
1. confirm that `rocore` is running on ros server
1. confirm that `ros_gopigo` is running on ros server

### gamepad
1. confirm that `main.py` is running on raspberrypi
1. when you press a button of gamepad, 'turtle' moves according to the pressed button

### web controller
1. access to https://api.cloudconductor.jp/controller/web/
1. when you press a button of web controller, gopigo moves according to the pressed button
