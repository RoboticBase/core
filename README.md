# fiware-demo1
This repository construct a [FIWARE](http://www.fiware.org/) platform on [Kubernetes](https://kubernetes.io/), and interconnect REST Service, IoT device and Robot through FIWARE platform.

## Requirements

||version|
|:--|:--|
|azure cli|2.0.31|
|kubectl|1.10.1|
|helm|2.8.2|

||version|
|:--|:--|
|kubernetes|1.8.11|

## start private registry on Azure Container Registry

```bash
mac:fiware-demo1$ az login
```

```bash
mac:fiware-demo1$ az group create --name fiware-demo --location centralus
```

```bash
mac:fiware-demo1$ az acr create --resource-group fiware-demo --name fiwareacr --sku Basic
```

## start kubernetes on Azure AKS

```bash
mac:fiware-demo1$ az provider register -n Microsoft.ContainerService
```

```bash
mac:fiware-demo1$ az aks create --resource-group fiware-demo --name fiwareaks --node-count 4 --ssh-key-value $HOME/.ssh/azure.pub
```

```bash
mac:fiware-demo1$ az aks get-credentials --resource-group fiware-demo --name fiwareaks
```

```bash
mac:fiware-demo1$ kubectl get nodes
NAME                       STATUS    ROLES     AGE       VERSION
aks-nodepool1-27506152-0   Ready     agent     2m        v1.9.6
aks-nodepool1-27506152-1   Ready     agent     2m        v1.9.6
aks-nodepool1-27506152-2   Ready     agent     2m        v1.9.6
aks-nodepool1-27506152-3   Ready     agent     2m        v1.9.6
```

```bash
mac:fiware-demo1$ CLIENT_ID=$(az aks show --resource-group fiware-demo --name fiwareaks --query "servicePrincipalProfile.clientId" --output tsv);echo ${CLIENT_ID}
mac:fiware-demo1$ ACR_ID=$(az acr show --name fiwareacr --resource-group fiware-demo --query "id" --output tsv); echo ${ACR_ID}
mac:fiware-demo1$ az role assignment create --assignee ${CLIENT_ID} --role Reader --scope ${ACR_ID}
```

## install helm
```bash
mac:fiware-demo1$ curl --output ~/Downloads/helm-v2.8.2-darwin-amd64.tar.gz https://storage.googleapis.com/kubernetes-helm/helm-v2.8.2-darwin-amd64.tar.gz
mac:fiware-demo1$ tar xfz ~/Downloads/helm-v2.8.2-darwin-amd64.tar.gz -C /tmp
mac:fiware-demo1$ sudo mv /tmp/darwin-amd64/helm /usr/local/bin
```

```bash
mac:fiware-demo1$ helm update
mac:fiware-demo1$ helm init
```

```bash
mac:fiware-demo1$ kubectl get pods --all-namespaces | grep tiller
kube-system   tiller-deploy-865dd6c794-cdzd9          1/1       Running   0          6m
```

```bash
mac:fiware-demo1$ helm version
Client: &version.Version{SemVer:"v2.8.2", GitCommit:"a80231648a1473929271764b920a8e346f6de844", GitTreeState:"clean"}
Server: &version.Version{SemVer:"v2.8.2", GitCommit:"a80231648a1473929271764b920a8e346f6de844", GitTreeState:"clean"}
```

## start etcd cluster on AKS

```bash
mac:fiware-demo1$ helm install stable/etcd-operator --name fiware-etcd --set rbac.create=false
```

```bash
mac:fiware-demo1$ kubectl get pods | grep fiware-etcd
fiware-etcd-etcd-operator-etcd-backup-operator-d49598cb6-v92zl    1/1       Running   0          51s
fiware-etcd-etcd-operator-etcd-operator-d69bdfb64-m6bjz           1/1       Running   0          51s
fiware-etcd-etcd-operator-etcd-restore-operator-5c65cd4469t9xrp   1/1       Running   0          51s
```

```bash
mac:$ kubectl apply -f etcd/etcd-cluster.yaml
```

```bash
mac:fiware-demo1$ kubectl get pods -l app=etcd
NAME                READY     STATUS    RESTARTS   AGE
etcd-cluster-0000   1/1       Running   0          2m
etcd-cluster-0001   1/1       Running   0          2m
etcd-cluster-0002   1/1       Running   0          2m
```

```bash
mac:fiware-demo1$ kubectl get services -l app=etcd
NAME                  TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)             AGE
etcd-cluster          ClusterIP   None           <none>        2379/TCP,2380/TCP   3m
etcd-cluster-client   ClusterIP   10.0.228.101   <none>        2379/TCP            3m
```

```text
mac:fiware-demo1$ kubectl run --rm -i --tty fun --image quay.io/coreos/etcd --restart=Never -- /bin/sh
/ # etcdctl --peers http://etcd-cluster-client:2379 member list
217daa653b4d7b56: name=etcd-cluster-0002 peerURLs=http://etcd-cluster-0002.etcd-cluster.default.svc:2380 clientURLs=http://etcd-cluster-0002.etcd-cluster.default.svc:2379 isLeader=true
41066d74e036e064: name=etcd-cluster-0001 peerURLs=http://etcd-cluster-0001.etcd-cluster.default.svc:2380 clientURLs=http://etcd-cluster-0001.etcd-cluster.default.svc:2379 isLeader=false
4e063fdd65c779b5: name=etcd-cluster-0000 peerURLs=http://etcd-cluster-0000.etcd-cluster.default.svc:2380 clientURLs=http://etcd-cluster-0000.etcd-cluster.default.svc:2379 isLeader=false
/ # exit
```

## start vernemq cluster on AKS

* create usernames & passwords of vernemq
```bash
mac:fiware-demo1$ mkdir -p secrets
mac:fiware-demo1$ touch secrets/vmq.passwd
mac:fiware-demo1$ docker run --rm -v $(PWD)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd iotagent
mac:fiware-demo1$ docker run --rm -v $(PWD)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd raspberrypi
mac:fiware-demo1$ docker run --rm -v $(PWD)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd turtlesim
```

```bash
mac:fiware-demo1$ docker run -it -v $(PWD)/secrets:/etc/letsencrypt certbot/certbot certonly --manual --domain mqtt.nmatsui.work --email nobuyuki.matsui@gmail.com --agree-tos --manual-public-ip-logging-ok --preferred-challenges dns
```

* Another terminal
```bash
mac-another:$ az network dns record-set txt add-record --resource-group nmatsui_dns --zone-name nmatsui.work --record-set-name "_acme-challenge.mqtt" --value "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
```

* After completion of certbot
```bash
mac-another:$ az network dns record-set txt remove-record --resource-group nmatsui_dns --zone-name nmatsui.work --record-set-name "_acme-challenge.mqtt" --value "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
```

```bash
mac:fiware-demo1$ cat secrets/DST_Root_CA_X3.pem secrets/archive/mqtt.nmatsui.work/chain1.pem > secrets/ca.crt
mac:fiware-demo1$ cp secrets/archive/mqtt.nmatsui.work/fullchain1.pem secrets/server.crt
mac:fiware-demo1$ cp secrets/archive/mqtt.nmatsui.work/privkey1.pem secrets/server.key
```

```bash
mac:fiware-demo1$ kubectl create secret generic vernemq-passwd --from-file=./secrets/vmq.passwd
mac:fiware-demo1$ kubectl create secret generic vernemq-certifications --from-file=./secrets/ca.crt --from-file=./secrets/server.crt --from-file=./secrets/server.key
```

```bash
mac:fiware-demo1$ kubectl get secrets
NAME                                                          TYPE                                  DATA      AGE
default-token-2klq6                                           kubernetes.io/service-account-token   3         19m
fiware-etcd-etcd-operator-etcd-backup-operator-token-ghsq2    kubernetes.io/service-account-token   3         5m
fiware-etcd-etcd-operator-etcd-operator-token-qm2xp           kubernetes.io/service-account-token   3         5m
fiware-etcd-etcd-operator-etcd-restore-operator-token-4wzkm   kubernetes.io/service-account-token   3         5m
vernemq-certifications                                        Opaque                                3         3s
vernemq-passwd                                                Opaque                                1         11s
```

```bash
mac:fiware-demo1$ kubectl apply -f vernemq/vernemq-cluster.yaml
```

```bash
mac:fiware-demo1$ kubectl get pods -l app=vernemq
NAME        READY     STATUS    RESTARTS   AGE
vernemq-0   1/1       Running   0          1m
vernemq-1   1/1       Running   0          1m
vernemq-2   1/1       Running   0          33s
```

```bash
mac:fiware-demo1$ kubectl exec vernemq-0 -- vmq-admin cluster show
+---------------------------------------------------+-------+
|                       Node                        |Running|
+---------------------------------------------------+-------+
|VerneMQ@vernemq-0.vernemq.default.svc.cluster.local| true  |
|VerneMQ@vernemq-1.vernemq.default.svc.cluster.local| true  |
|VerneMQ@vernemq-2.vernemq.default.svc.cluster.local| true  |
+---------------------------------------------------+-------+
```

```bash
mac:fiware-demo1$ kubectl get services -l app=mqtts
NAME      TYPE           CLUSTER-IP     EXTERNAL-IP       PORT(S)          AGE
mqtts     LoadBalancer   10.0.170.212   WWW.XXX.YYY.ZZZ   8883:30187/TCP   3m
```

```bash
mac:fiware-demo1$ az network dns record-set a add-record --resource-group nmatsui_dns --zone-name nmatsui.work --record-set-name "mqtt" --ipv4-address "WWW.XXX.YYY.ZZZ"
```

## start ambassador on AKS

```bash
mac:fiware-demo1$ docker run -it -v $(PWD)/secrets:/etc/letsencrypt certbot/certbot certonly --manual --domain api.nmatsui.work --email nobuyuki.matsui@gmail.com --agree-tos --manual-public-ip-logging-ok --preferred-challenges dns
```

* Another terminal
```bash
mac-another:$ az network dns record-set txt add-record --resource-group nmatsui_dns --zone-name nmatsui.work --record-set-name "_acme-challenge.api" --value "YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY"
```

* After completion of certbot
```bash
mac-another:$ az network dns record-set txt remove-record --resource-group nmatsui_dns --zone-name nmatsui.work --record-set-name "_acme-challenge.api" --value "YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY"
```

```bash
mac:fiware-demo1$ kubectl create secret tls ambassador-certs --cert=$(PWD)/secrets/live/api.nmatsui.work/fullchain.pem --key=$(PWD)/secrets/live/api.nmatsui.work/privkey.pem
```

```bash
mac:fiware-demo1$ kubectl apply -f ambassador/ambassador.yaml
```

```bash
mac:fiware-demo1$ kubectl get pods -l service=ambassador
NAME                          READY     STATUS    RESTARTS   AGE
ambassador-79768bd968-7n4vl   2/2       Running   0          50s
ambassador-79768bd968-cpl5j   2/2       Running   0          50s
ambassador-79768bd968-h2ct2   2/2       Running   0          50s
```

```bash
mac:fiware-demo1$ kubectl get services -l service=ambassador
NAME         TYPE           CLUSTER-IP    EXTERNAL-IP       PORT(S)                      AGE
ambassador   LoadBalancer   10.0.191.59   www.xxx.yyy.zzz   443:30357/TCP,80:32755/TCP   4m
```

```bash
mac:fiware-demo1$ az network dns record-set a add-record --resource-group nmatsui_dns --zone-name nmatsui.work --record-set-name "api" --ipv4-address "www.xxx.yyy.zzz"
```

```bash
mac:fiware-demo1$ STR=$(cat /dev/urandom | LC_CTYPE=C tr -dc 'a-zA-Z0-9' | head -c 32); cat << __EOS__ > secrets/auth-tokens.json
{"${STR}": ["^/orion/.*$", "^/idas/.*$"]}
__EOS__
```

```bash
mac:fiware-demo1$ kubectl create secret generic auth-tokens --from-file=./secrets/auth-tokens.json
```

```bash
mac:fiware-demo1$ kubectl get secrets
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

* confirm that local docker dameon has already started
```bash
mac:fiware-demo1$ az acr login --name fiwareacr
```

```bash
mac:fiware-demo1$ docker build -t fiwareacr.azurecr.io/tech-sketch/fiware-bearer-auth:0.1.0 ./ambassador/fiware-bearer-auth
mac:fiware-demo1$ docker push fiwareacr.azurecr.io/tech-sketch/fiware-bearer-auth:0.1.0
```

```bash
mac:fiware-demo1$ az acr repository list --name fiwareacr --output table
Result
------------------------------
tech-sketch/fiware-bearer-auth
```

```bash
mac:fiware-demo1$ kubectl apply -f ambassador/bearer-auth.yaml
```

```bash
mac:fiware-demo1$ kubectl get pods -l pod=bearer-auth
NAME                           READY     STATUS    RESTARTS   AGE
bearer-auth-6fffdbd9c9-7kkpr   1/1       Running   0          56s
bearer-auth-6fffdbd9c9-qxw6m   1/1       Running   0          56s
bearer-auth-6fffdbd9c9-sdn5b   1/1       Running   0          56s
```

```bash
mac:fiware-demo1$ kubectl get services -l service=bearer-auth
NAME          TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)    AGE
bearer-auth   ClusterIP   10.0.129.102   <none>        3000/TCP   2m
```

## start orion on AKS

```bash
mac:fiware-demo1$ kubectl apply -f orion/orion-mongodb.yaml
```

```bash
mac:fiware-demo1$ kubectl get pods -l app=orion-mongodb
NAME              READY     STATUS    RESTARTS   AGE
orion-mongodb-0   2/2       Running   0          10m
orion-mongodb-1   2/2       Running   0          7m
orion-mongodb-2   2/2       Running   0          5m
```

```bash
mac:fiware-demo1$ kubectl get services -l app=orion-mongodb
NAME            TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)     AGE
orion-mongodb   ClusterIP   None         <none>        27017/TCP   10m
```

```bash
mac:fiware-demo1$ kubectl exec orion-mongodb-0 -c orion-mongodb -- mongo --eval 'printjson(rs.status().members.map(function(e) {return {name: e.name, stateStr:e.stateStr};}))'
MongoDB shell version v3.6.4
connecting to: mongodb://127.0.0.1:27017
MongoDB server version: 3.6.4
[
	{
		"name" : "orion-mongodb-0.orion-mongodb.default.svc.cluster.local:27017",
		"stateStr" : "PRIMARY"
	},
	{
		"name" : "orion-mongodb-1.orion-mongodb.default.svc.cluster.local:27017",
		"stateStr" : "SECONDARY"
	},
	{
		"name" : "orion-mongodb-2.orion-mongodb.default.svc.cluster.local:27017",
		"stateStr" : "SECONDARY"
	}
]
```

```bash
mac:fiware-demo1$ kubectl apply -f orion/orion.yaml
```

```bash
mac:fiware-demo1$ kubectl get pods -l app=orion
NAME                     READY     STATUS    RESTARTS   AGE
orion-54f5cdcb5d-d2pt5   1/1       Running   0          56s
orion-54f5cdcb5d-hv274   1/1       Running   0          56s
orion-54f5cdcb5d-xbnx2   1/1       Running   0          56s
```

```bash
mac:fiware-demo1$ kubectl get services -l app=orion
NAME      TYPE        CLUSTER-IP    EXTERNAL-IP   PORT(S)    AGE
orion     ClusterIP   10.0.44.126   <none>        1026/TCP   1m
```

```bash
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -i -H "Authorization: bearer ${TOKEN}" https://api.nmatsui.work/orion/v2/entities/
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -i -H "Authorization: bearer ${TOKEN}" https://api.nmatsui.work/orion/v2/subscriptions/
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
mac:fiware-demo1$ kubectl apply -f idas/idas-mongodb.yaml
```

```bash
mac:fiware-demo1$ kubectl get pods -l app=idas-mongodb
NAME             READY     STATUS    RESTARTS   AGE
idas-mongodb-0   2/2       Running   0          12m
idas-mongodb-1   2/2       Running   0          10m
idas-mongodb-2   2/2       Running   0          7m
```

```bash
mac:fiware-demo1$ kubectl get services -l app=idas-mongodb
NAME           TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)     AGE
idas-mongodb   ClusterIP   None         <none>        27017/TCP   12m
```

```bash
mac:fiware-demo1$ kubectl exec idas-mongodb-0 -c idas-mongodb -- mongo --eval 'printjson(rs.status().members.map(function(e) {return {name: e.name, stateStr:e.stateStr};}))'
MongoDB shell version v3.6.4
connecting to: mongodb://127.0.0.1:27017
MongoDB server version: 3.6.4
[
	{
		"name" : "idas-mongodb-0.idas-mongodb.default.svc.cluster.local:27017",
		"stateStr" : "PRIMARY"
	},
	{
		"name" : "idas-mongodb-1.idas-mongodb.default.svc.cluster.local:27017",
		"stateStr" : "SECONDARY"
	},
	{
		"name" : "idas-mongodb-2.idas-mongodb.default.svc.cluster.local:27017",
		"stateStr" : "SECONDARY"
	}
]
```

```bash
mac:fiware-demo1$ az acr login --name fiwareacr
mac:fiware-demo1$ docker build -t fiwareacr.azurecr.io/tech-sketch/fiware-mqtt-msgfilter:0.1.0 idas/fiware-mqtt-msgfilter/
mac:fiware-demo1$ docker push fiwareacr.azurecr.io/tech-sketch/fiware-mqtt-msgfilter:0.1.0
```

```bash
mac:fiware-demo1$ az acr repository list --name fiwareacr --output table
Result
---------------------------------
tech-sketch/fiware-bearer-auth
tech-sketch/fiware-mqtt-msgfilter
```

```bash
mac:fiware-demo1$ kubectl apply -f idas/mqtt-msgfilter.yaml
```

```bash
mac:fiware-demo1$ kubectl get pods -l pod=mqtt-msgfilter
NAME                              READY     STATUS    RESTARTS   AGE
mqtt-msgfilter-6f76445596-cmbqz   1/1       Running   0          26s
mqtt-msgfilter-6f76445596-wrhzb   1/1       Running   0          26s
mqtt-msgfilter-6f76445596-znnvg   1/1       Running   0          26s
```

```bash
mac:fiware-demo1$ kubectl get services -l service=mqtt-msgfilter
NAME             TYPE        CLUSTER-IP    EXTERNAL-IP   PORT(S)    AGE
mqtt-msgfilter   ClusterIP   10.0.133.42   <none>        5001/TCP   43s
```

* replace `<<password_of_iotagent>>` to the password of "iotagent"
```bash
mac:fiware-demo1$ sed -e 's/<<password_of_iotagent>>/XXXXXXXXXXXX/g' idas/iotagent-ul/config.js.template > idas/iotagent-ul/config.js
```

```bash
mac:fiware-demo1$ az acr login --name fiwareacr
mac:fiware-demo1$ docker build -t fiwareacr.azurecr.io/tech-sketch/iotagent-ul:1.6.0 idas/iotagent-ul/
mac:fiware-demo1$ docker push fiwareacr.azurecr.io/tech-sketch/iotagent-ul:1.6.0
```

```bash
mac:fiware-demo1$ az acr repository list --name fiwareacr --output table
Result
---------------------------------
tech-sketch/fiware-bearer-auth
tech-sketch/fiware-mqtt-msgfilter
tech-sketch/iotagent-ul
```

```bash
mac:fiware-demo1$ kubectl apply -f idas/iotagent-ul.yaml
```

```bash
mac:fiware-demo1$ kubectl get pods -l app=iotagent-ul
NAME                           READY     STATUS    RESTARTS   AGE
iotagent-ul-79685b64bf-8krps   1/1       Running   0          3m
iotagent-ul-79685b64bf-m6nlg   1/1       Running   0          3m
iotagent-ul-79685b64bf-mjpbl   1/1       Running   0          3m
```

```bash
mac:fiware-demo1$ kubectl get services -l app=iotagent-ul
NAME          TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)             AGE
iotagent-ul   ClusterIP   10.0.180.155   <none>        4041/TCP,7896/TCP   43s
```

```bash
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" https://api.nmatsui.work/idas/ul20/manage/iot/services/
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.nmatsui.work/idas/ul20/manage/iot/devices/
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

## start cmd-proxy on AKS
```bash
mac:fiware-demo1$ az acr login --name fiwareacr
```

```bash
mac:fiware-demo1$ docker build --build-arg PORT=8888 -t fiwareacr.azurecr.io/tech-sketch/fiware-cmd-proxy:0.1.0 ./controller/fiware-cmd-proxy/
mac:fiware-demo1$ docker push fiwareacr.azurecr.io/tech-sketch/fiware-cmd-proxy:0.1.0
```

```bash
mac:fiware-demo1$ az acr repository list --name fiwareacr --output table
Result
---------------------------------
tech-sketch/fiware-bearer-auth
tech-sketch/fiware-cmd-proxy
tech-sketch/fiware-mqtt-msgfilter
tech-sketch/iotagent-ul
```

```bash
mac:fiware-demo1$ kubectl apply -f controller/fiware-cmd-proxy.yaml
```

```bash
mac:fiware-demo1$ kubectl get pods -l pod=cmd-proxy
NAME                         READY     STATUS    RESTARTS   AGE
cmd-proxy-58c756cbcd-q7jzz   1/1       Running   0          5s
cmd-proxy-58c756cbcd-rs75j   1/1       Running   0          5s
cmd-proxy-58c756cbcd-v9ptr   1/1       Running   0          5s
```

```bash
mac:fiware-demo1$ kubectl get services -l service=cmd-proxy
NAME        TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)    AGE
cmd-proxy   ClusterIP   10.0.208.226   <none>        8888/TCP   34s
```

## register "demo1" service

```bash
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.nmatsui.work/idas/ul20/manage/iot/services/ -X POST -d @- <<__EOS__
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" https://api.nmatsui.work/idas/ul20/manage/iot/services/ | jq .
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.nmatsui.work/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.nmatsui.work/idas/ul20/manage/iot/devices/gamepad/ | jq .
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.nmatsui.work/orion/v2/entities/gamepad/ | jq .
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

```text
mac:fiware-demo1$ mosquitto_sub -h mqtt.nmatsui.work -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P <<password_of_iotagent>>
...
```

```bash
raspberrypi:raspi_gamepad$ ./main.py
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
2018/05/06 11:46:15 [   INFO] src.controller - connected mqtt broker[mqtt.nmatsui.work:8883], response_code=0
...
```

```bash
mac:fiware-demo1$ mosquitto_sub -h mqtt.nmatsui.work -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P <<password_of_iotagent>>
...
...
Client mosqsub|60435-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/gamepad/attrs', ... (45 bytes))
2018-05-06T02:46:14.876494+0000|button|circle
...
```

```bash
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.nmatsui.work/orion/v2/entities/gamepad/ | jq .
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.nmatsui.work/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
{
  "devices": [
    {
      "device_id": "turtlesim",
      "entity_name": "turtlesim",
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.nmatsui.work/idas/ul20/manage/iot/devices/turtlesim/ | jq .
{
  "device_id": "turtlesim",
  "service": "demo1",
  "service_path": "/",
  "entity_name": "turtlesim",
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.nmatsui.work/orion/v2/entities/turtlesim/ | jq .
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
  "move": {
    "type": "string",
    "value": "",
    "metadata": {}
  }
}
```

## test subscribing a cmd from orion to ros and publishing a cmdexe from ros to orion through MQTT and idas

```bash
mac:fiware-demo1$ mosquitto_sub -h mqtt.nmatsui.work -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P <<password_of_iotagent>>
...
```

* start X on ros server and login X using RDP

```bash
ros-terminal1:ros_ws$ source devel/setup.bash
ros-terminal1:ros_ws$ roscore
...
```

```bash
ros-terminal2:ros_ws$ source devel/setup.bash
ros-terminal2:ros_ws$ rosrun turtlesim turtlesim_node
...
```

```bash
ros-terminal3:ros_ws$ source devel/setup.bash
ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
...
[INFO] [1525575494.645546]: [turtlesim_operator.command_sender:CommandSender._on_connect] mqtt connect status=0
[INFO] [1525575494.648536]: [turtlesim_operator.attribute_receiver:AttributeReceiver._on_connect] mqtt connect status=0
...
```

* send 'circle' cmd to 'turtlesim' entity
```bash
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" -H "Content-Type: application/json" https://api.nmatsui.work/orion/v1/updateContext -d @-<<__EOS__ | jq .
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
mac:fiware-demo1$ mosquitto_sub -h mqtt.nmatsui.work -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P <<password_of_iotagent>>
...
Client mosqsub|77956-Nobuyukin received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/cmd', ... (21 bytes))
turtlesim@move|circle
Client mosqsub|77956-Nobuyukin received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/cmdexe', ... (28 bytes))
turtlesim@move|MOVED: circle
...
```

```bash
ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
...
[INFO] [1525575568.112307]: [turtlesim_operator.command_sender:CommandSender._on_message] received message from mqtt: turtlesim@move|circle
[INFO] [1525575568.114085]: [turtlesim_operator.command_sender:CommandSender._do_circle] do circle
...
```

```bash
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.nmatsui.work/orion/v2/entities/turtlesim/ | jq .
{
  "id": "turtlesim",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-05-06T02:59:28.00Z",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": "MOVED: circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-06T02:59:28.233Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "OK",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-06T02:59:28.233Z"
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

## register fiware-cmd-proxy

```bash
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.nmatsui.work/orion/v2/subscriptions/ -X POST -d @- <<__EOS__
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" https://api.nmatsui.work/orion/v2/subscriptions/ | jq .
[
  {
    "id": "5aee70ef59a5a45b7935a8cc",
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
      "lastNotification": "2018-05-06T03:05:19.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "normalized",
      "http": {
        "url": "http://cmd-proxy:8888/gamepad/"
      }
    }
  }
]
```

## test gamepad and ros
1. start `raspi_damepad/main.py` on raspberrypi
1. start X on ros server
1. start `roscore` on ros
1. start `rosrun turtlesim turtlesim_node` on ros
1. start `roslaunch turtlesim_operator turtlesim_operator.launch` on ros
1. when you press a button of gamepad, "turtle" moves according to the pressed button
