# 2. start containers for container-centric fiware demonstration on AKS

Start pods & services on AKS by following steps:

1. [start etcd cluster](#start-etcd-cluster-on-aks)
1. [start vernemq cluster](#start-vernemq-cluster-on-aks)
1. [start mongodb cluster](#start-mondodb-cluster-on-aks)
1. [start ambassador](#start-ambassador-on-aks)
1. [start authorization & authentication servie](#start-authorization--authentication-service-on-aks)
1. [start fiware orion](#start-fiware-orion-on-aks)
1. [start duplicate message filter service for idas](#start-duplicate-message-filter-service-for-idas)
1. [start fiware IDAS(iotagent-ul)](#start-fiware-idasiotagent-ul-on-aks)
1. [start fiware cygnus](#start-fiware-cygnus-on-aks)
1. [start command pxory service](#start-command-proxy-service-on-aks)

**In the following document, replace "example.com" with your domain.**

## start etcd cluster on AKS

[etcd](https://github.com/coreos/etcd)

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
mac:$ export POD=$(kubectl get pods -l app=fiware-etcd-etcd-operator-etcd-operator --namespace default --output name)
mac:$ kubectl logs $POD --namespace=default
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

[vernemq](https://vernemq.com/)

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
mac:$ docker run -it -v $(pwd)/secrets:/etc/letsencrypt certbot/certbot certonly --manual --domain mqtt.example.com --email nobuyuki.matsui@gmail.com --agree-tos --manual-public-ip-logging-ok --preferred-challenges dns
```

* Another terminal
```bash
mac-another:$ az network dns record-set txt add-record --resource-group dns-zone --zone-name "example.com" --record-set-name "_acme-challenge.mqtt" --value "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
mac-another:$ az network dns record-set list --resource-group dns-zone --zone-name "example.com" | jq '.[] | {"fqdn": .fqdn, "type": .type}'
{
  "fqdn": "example.com.",
  "type": "Microsoft.Network/dnszones/NS"
}
{
  "fqdn": "example.com.",
  "type": "Microsoft.Network/dnszones/SOA"
}
{
  "fqdn": "_acme-challenge.mqtt.example.com.",
  "type": "Microsoft.Network/dnszones/TXT"
}
```

* Press 'ENTER' at original terminal when `_acme-challenge.mqtt.example.com.` txt record is created.

* After completion of certbot
```bash
mac-another:$ az network dns record-set txt remove-record --resource-group dns-zone --zone-name "example.com" --record-set-name "_acme-challenge.mqtt" --value "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
```

```bash
mac:$ cat secrets/DST_Root_CA_X3.pem secrets/archive/mqtt.example.com/chain1.pem > secrets/ca.crt
mac:$ cp secrets/archive/mqtt.example.com/fullchain1.pem secrets/server.crt
mac:$ cp secrets/archive/mqtt.example.com/privkey1.pem secrets/server.key
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
mac:$ az network dns record-set a add-record --resource-group dns-zone --zone-name "example.com" --record-set-name "mqtt" --ipv4-address "WWW.XXX.YYY.ZZZ"
```

* XXXXXXXXXXXX is the password of "iotagent"
```text
mac:$ mosquitto_sub -h mqtt.example.com -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
```

## start mondodb cluster on AKS

[mongodb](https://www.mongodb.com/)

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

[ambassador](https://www.getambassador.io/)

```bash
mac:$ docker run -it -v $(pwd)/secrets:/etc/letsencrypt certbot/certbot certonly --manual --domain api.example.com --email nobuyuki.matsui@gmail.com --agree-tos --manual-public-ip-logging-ok --preferred-challenges dns
```

* Another terminal
```bash
mac-another:$ az network dns record-set txt add-record --resource-group dns-zone --zone-name "example.com" --record-set-name "_acme-challenge.api" --value "YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY"
mac-another:$ az network dns record-set list --resource-group dns-zone --zone-name "example.com" | jq '.[] | {"fqdn": .fqdn, "type": .type}'
{
  "fqdn": "example.com.",
  "type": "Microsoft.Network/dnszones/NS"
}
{
  "fqdn": "example.com.",
  "type": "Microsoft.Network/dnszones/SOA"
}
{
  "fqdn": "_acme-challenge.api.example.com.",
  "type": "Microsoft.Network/dnszones/TXT"
}
{
  "fqdn": "mqtt.example.com.",
  "type": "Microsoft.Network/dnszones/A"
}
```

* Press 'ENTER' at original terminal when `_acme-challenge.api.example.com.` txt record is created.

* After completion of certbot
```bash
mac-another:$ az network dns record-set txt remove-record --resource-group dns-zone --zone-name "example.com" --record-set-name "_acme-challenge.api" --value "YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY"
```

```bash
mac:$ kubectl create secret tls ambassador-certs --cert=$(pwd)/secrets/live/api.example.com/fullchain.pem --key=$(pwd)/secrets/live/api.example.com/privkey.pem
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
mac:$ az network dns record-set a add-record --resource-group dns-zone --zone-name "example.com" --record-set-name "api" --ipv4-address "www.xxx.yyy.zzz"
```

```bash
mac:$ curl -i https://api.example.com
HTTP/1.1 404 Not Found
date: Fri, 25 May 2018 00:47:41 GMT
server: envoy
content-length: 0
```

## start authorization & authentication service on AKS
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
mac:$ kubectl apply -f ambassador/fiware-ambassador-auth.yaml
```

```bash
mac:$ kubectl get pods -l pod=ambassador-auth
NAME                           READY     STATUS    RESTARTS   AGE
ambassador-auth-6fffdbd9c9-7kkpr   1/1       Running   0          56s
ambassador-auth-6fffdbd9c9-qxw6m   1/1       Running   0          56s
ambassador-auth-6fffdbd9c9-sdn5b   1/1       Running   0          56s
```

```bash
mac:$ kubectl get services -l service=ambassador-auth
NAME          TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)    AGE
ambassador-auth   ClusterIP   10.0.129.102   <none>        3000/TCP   2m
```

## start fiware orion on AKS

[fiware orion](https://catalogue-server.fiware.org/enablers/publishsubscribe-context-broker-orion-context-broker)

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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" https://api.example.com/orion/v2/entities/
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" https://api.example.com/orion/v2/subscriptions/
HTTP/1.1 200 OK
content-length: 2
content-type: application/json
fiware-correlator: 5a4ecc6e-4dc1-11e8-b1a2-0a580af4010a
date: Wed, 02 May 2018 04:29:07 GMT
x-envoy-upstream-service-time: 2
server: envoy

[]
```

## start duplicate message filter service for idas

```bash
mac:$ kubectl apply -f idas/fiware-mqtt-msgfilter.yaml
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

## start fiware idas(iotagent-ul) on AKS

[fiware IDAS(iotagent-ul)](https://catalogue-server.fiware.org/enablers/backend-device-management-idas)

**In this demonstration, we use customized iotagent-ul in order to ignore duplicate MQTT messages.**

* XXXXXXXXXXXX is the password of "iotagent"
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" https://api.example.com/idas/ul20/manage/iot/services/
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/idas/ul20/manage/iot/devices/
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

## start fiware cygnus on AKS

[fiware cygnus](https://catalogue-server.fiware.org/enablers/cygnus)

**In this demonstration, we use re-configured cygnus in order to revoke unnecessary sinks.**

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

## start command proxy service on AKS

The 'command pxory service' connects 'gamepad' and 'web controller' to 'turtlesim' or 'gopigo'.

In this step, we configure the service as connecting to 'turtlesim'. If you want to start the service as connecting to 'gopigo', use `ROBOT_ID=gopigo` instead of `ROBOT_ID=turtlesim`.

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
