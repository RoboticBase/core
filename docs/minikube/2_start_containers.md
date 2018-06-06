# 2. start containers for container-centric fiware demonstration on minikube

Start pods & services on minikube by following steps:

1. [start etcd cluster](#start-etcd-cluster-on-minikube)
1. [start vernemq cluster](#start-vernemq-cluster-on-minikube)
1. [start mongodb cluster](#start-mondodb-cluster-on-minikube)
1. [start ambassador](#start-ambassador-on-minikube)
1. [start authorization & authentication servie](#start-authorization--authentication-service-on-minikube)
1. [start fiware orion](#start-fiware-orion-on-minikube)
1. [start duplicate message filter service for idas](#start-duplicate-message-filter-service-for-idas)
1. [start fiware IDAS(iotagent-ul)](#start-fiware-idasiotagent-ul-on-minikube)
1. [start fiware cygnus](#start-fiware-cygnus-on-minikube)
1. [start command pxory service](#start-command-proxy-service-on-minikube)

## start etcd cluster on minikube

[etcd](https://github.com/coreos/etcd)

```bash
mac:$ helm install stable/etcd-operator --name fiware-etcd --set rbac.create=false
```

```bash
mac:$ kubectl get pods | grep fiware-etcd
fiware-etcd-etcd-operator-etcd-backup-operator-d49598cb6-4pvdp    1/1       Running   0          54s
fiware-etcd-etcd-operator-etcd-operator-d69bdfb64-tb68b           1/1       Running   0          54s
fiware-etcd-etcd-operator-etcd-restore-operator-5c65cd4469s9jqx   1/1       Running   0          54s
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
etcd-cluster-0000   1/1       Running   0          1m
etcd-cluster-0001   1/1       Running   0          38s
etcd-cluster-0002   1/1       Running   0          28s
```

```bash
mac:$ kubectl get services -l app=etcd
NAME                  TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)             AGE
etcd-cluster          ClusterIP   None           <none>        2379/TCP,2380/TCP   1m
etcd-cluster-client   ClusterIP   10.98.99.185   <none>        2379/TCP            1m
```

```bash
mac:$ kubectl run --rm -it etcdclient --image quay.io/coreos/etcd --restart=Never -- etcdctl --peers http://etcd-cluster-client:2379 member list
a68edf98c8e508e0: name=etcd-cluster-0002 peerURLs=http://etcd-cluster-0002.etcd-cluster.default.svc:2380 clientURLs=http://etcd-cluster-0002.etcd-cluster.default.svc:2379 isLeader=false
beabc049e3548106: name=etcd-cluster-0001 peerURLs=http://etcd-cluster-0001.etcd-cluster.default.svc:2380 clientURLs=http://etcd-cluster-0001.etcd-cluster.default.svc:2379 isLeader=false
fbcaf341692bc2d9: name=etcd-cluster-0000 peerURLs=http://etcd-cluster-0000.etcd-cluster.default.svc:2380 clientURLs=http://etcd-cluster-0000.etcd-cluster.default.svc:2379 isLeader=true
```

## start vernemq cluster on minikube

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
mac:$ kubectl create secret generic vernemq-passwd --from-file=./secrets/vmq.passwd
```

```bash
mac:$ kubectl get secrets
NAME                                                          TYPE                                  DATA      AGE
default-token-9ffxs                                           kubernetes.io/service-account-token   3         45m
fiware-etcd-etcd-operator-etcd-backup-operator-token-lkxxl    kubernetes.io/service-account-token   3         5m
fiware-etcd-etcd-operator-etcd-operator-token-dvnc8           kubernetes.io/service-account-token   3         5m
fiware-etcd-etcd-operator-etcd-restore-operator-token-xgbl6   kubernetes.io/service-account-token   3         5m
vernemq-passwd                                                Opaque                                1         15s
```

```bash
mac:$ kubectl apply -f vernemq/vernemq-cluster-minikube.yaml
```

```bash
mac:$ kubectl get pods -l app=vernemq
NAME        READY     STATUS    RESTARTS   AGE
vernemq-0   1/1       Running   0          1m
vernemq-1   1/1       Running   0          24s
vernemq-2   1/1       Running   0          23s
```

```bash
mac:$ kubectl get services -l app=mqtt
NAME      TYPE       CLUSTER-IP     EXTERNAL-IP   PORT(S)          AGE
mqtt      NodePort   10.103.19.38   <none>        1883:32758/TCP   2m
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
mac:$ MQTT_PORT=$(kubectl describe service mqtt | grep "NodePort:" | awk '{print $3}' | awk -F'/' '{print $1}');echo ${MQTT_PORT}
mac:$ VBoxManage controlvm "minikube" natpf1 "mqtt,tcp,0.0.0.0,1883,,${MQTT_PORT}"
```

* XXXXXXXXXXXX is the password of "iotagent"
```text
mac:$ mosquitto_sub -h ${HOST_IPADDR} -p 1883 -d -t /# -u iotagent -P XXXXXXXXXXXX
...
```

## start mondodb cluster on minikube

[mongodb](https://www.mongodb.com/)

```bash
mac:$ kubectl apply -f mongodb/mongodb-cluster-minikube.yaml
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

## start ambassador on minikube

[ambassador](https://www.getambassador.io/)

```bash
mac:$ kubectl apply -f ambassador/ambassador-minikube.yaml
```

```bash
mac:$ kubectl get pods -l service=ambassador
NAME                          READY     STATUS    RESTARTS   AGE
ambassador-5cb767454c-26hbh   2/2       Running   0          57s
ambassador-5cb767454c-bhw7w   2/2       Running   0          57s
ambassador-5cb767454c-rmbxw   2/2       Running   0          57s
```

```bash
mac:$ kubectl get services -l service=ambassador
NAME         TYPE       CLUSTER-IP      EXTERNAL-IP   PORT(S)        AGE
ambassador   NodePort   10.102.193.74   <none>        80:32350/TCP   1m
```

```bash
mac:$ HTTP_PORT=$(kubectl describe service ambassador | grep "NodePort:" | awk '{print $3}' | awk -F'/' '{print $1}');echo ${HTTP_PORT}
mac:$ VBoxManage controlvm "minikube" natpf1 "http,tcp,0.0.0.0,8080,,${HTTP_PORT}"
```

```bash
mac:$ curl -i http://${HOST_IPADDR}:8080
HTTP/1.1 404 Not Found
date: Fri, 25 May 2018 09:57:09 GMT
server: envoy
content-length: 0
```

## start authorization & authentication service on minikube
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
ambassador-auth-69b5c77699-lxwch   1/1       Running   0          8s
ambassador-auth-69b5c77699-n6md2   1/1       Running   0          8s
ambassador-auth-69b5c77699-p2kql   1/1       Running   0          8s
```

```bash
mac:$ kubectl get services -l service=ambassador-auth
NAME          TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)    AGE
ambassador-auth   ClusterIP   10.107.210.87   <none>        3000/TCP   19s
```

## start fiware orion on minikube

[fiware orion](https://catalogue-server.fiware.org/enablers/publishsubscribe-context-broker-orion-context-broker)

```bash
mac:$ kubectl apply -f orion/orion.yaml
```

```bash
mac:$ kubectl get pods -l app=orion
NAME                    READY     STATUS    RESTARTS   AGE
orion-59f58dffc-gwcg8   1/1       Running   0          1m
orion-59f58dffc-jrrb8   1/1       Running   0          1m
orion-59f58dffc-kwp8w   1/1       Running   0          1m
```

```bash
mac:$ kubectl get services -l app=orion
NAME      TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)    AGE
orion     ClusterIP   10.100.201.136   <none>        1026/TCP   1m
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" http://${HOST_IPADDR}:8080/orion/v2/entities/
HTTP/1.1 200 OK
content-length: 2
content-type: application/json
fiware-correlator: c1d42066-6002-11e8-a713-0242ac110019
date: Fri, 25 May 2018 10:02:38 GMT
x-envoy-upstream-service-time: 2
server: envoy

[]
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" http://${HOST_IPADDR}:8080/orion/v2/subscriptions/
HTTP/1.1 200 OK
content-length: 2
content-type: application/json
fiware-correlator: d97b4f82-6002-11e8-b6b7-0242ac110019
date: Fri, 25 May 2018 10:03:18 GMT
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
mqtt-msgfilter-68445c5f8c-54m5d   1/1       Running   0          7s
mqtt-msgfilter-68445c5f8c-b8c9z   1/1       Running   0          7s
mqtt-msgfilter-68445c5f8c-sgwfh   1/1       Running   0          7s
```

```bash
mac:$ kubectl get services -l service=mqtt-msgfilter
NAME             TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)    AGE
mqtt-msgfilter   ClusterIP   10.106.214.152   <none>        5001/TCP   27s
```

## start fiware idas(iotagent-ul) on minikube

[fiware IDAS(iotagent-ul)](https://catalogue-server.fiware.org/enablers/backend-device-management-idas)

**In this demonstration, we use customized iotagent-ul in order to ignore duplicate MQTT messages.**

* XXXXXXXXXXXX is the password of "iotagent"
```bash
mac:$ env IOTAGENT_PASSWORD=XXXXXXXXXXXX envsubst < idas/iotagent-ul/config.js.template > idas/iotagent-ul/config.js
```

```bash
mac:$ docker build -t ${REPOSITORY}/tech-sketch/iotagent-ul:1.6.0 idas/iotagent-ul/
mac:$ docker push ${REPOSITORY}/tech-sketch/iotagent-ul:1.6.0
```

```bash
mac:$ envsubst < idas/iotagent-ul.yaml | kubectl apply -f -
```

```bash
mac:$ kubectl get pods -l app=iotagent-ul
NAME                           READY     STATUS    RESTARTS   AGE
iotagent-ul-8554cd456f-c9t6b   1/1       Running   0          10s
iotagent-ul-8554cd456f-mv9m4   1/1       Running   0          10s
iotagent-ul-8554cd456f-w8fxd   1/1       Running   0          10s
```

```bash
mac:$ kubectl get services -l app=iotagent-ul
NAME          TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)             AGE
iotagent-ul   ClusterIP   10.111.106.143   <none>        4041/TCP,7896/TCP   32s
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" http://${HOST_IPADDR}:8080/idas/ul20/manage/iot/services/
HTTP/1.1 200 OK
x-powered-by: Express
fiware-correlator: 73ca8ddb-6ba7-43da-b766-41b7be6f67ce
content-type: application/json; charset=utf-8
content-length: 25
etag: W/"19-WMYe0U6ocKhQjp+oaVnMHLdbylc"
date: Fri, 25 May 2018 10:06:18 GMT
x-envoy-upstream-service-time: 20
server: envoy

{"count":0,"services":[]}
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://${HOST_IPADDR}:8080/idas/ul20/manage/iot/devices/
HTTP/1.1 200 OK
x-powered-by: Express
fiware-correlator: b3ff3fbb-2e8b-4146-91d9-20063c91a731
content-type: application/json; charset=utf-8
content-length: 24
etag: W/"18-90KiBjq8YRQpT/NsVf7vo89XXWw"
date: Fri, 25 May 2018 10:06:49 GMT
x-envoy-upstream-service-time: 21
server: envoy

{"count":0,"devices":[]}
```

## start fiware cygnus on minikube

[fiware cygnus](https://catalogue-server.fiware.org/enablers/cygnus)

**In this demonstration, we use re-configured cygnus in order to revoke unnecessary sinks.**

```bash
mac:$ docker build -t ${REPOSITORY}/tech-sketch/cygnus-ngsi:1.8.0 ./cygnus/fiware-cygnus/
mac:$ docker push ${REPOSITORY}/tech-sketch/cygnus-ngsi:1.8.0
```

```bash
mac:$ envsubst < cygnus/cygnus.yaml | kubectl apply -f -
```

```bash
mac:$ kubectl get pods -l app=cygnus
NAME                      READY     STATUS    RESTARTS   AGE
cygnus-6b8ccf5474-9c6c2   1/1       Running   0          19s
cygnus-6b8ccf5474-lzrbz   1/1       Running   0          19s
cygnus-6b8ccf5474-v7p4r   1/1       Running   0          19s
```

```bash
mac:$ kubectl get services -l app=cygnus
NAME      TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)             AGE
cygnus    ClusterIP   10.111.106.17   <none>        5050/TCP,8081/TCP   40s
```

## start command proxy service on minikube

The 'command pxory service' connects 'gamepad' and 'web controller' to 'turtlesim' or 'gopigo'.

In this step, we configure the service as connecting to 'turtlesim'. If you want to start the service as connecting to 'gopigo', use `ROBOT_ID=gopigo` instead of `ROBOT_ID=turtlesim`.

* create three 'cmd-proxy' pods and a 'cmd-proxy' service to control 'turtlesim'.
```bash
mac:$ env FIWARE_SERVICE=demo1 FIWARE_SERVICEPATH=/ ROBOT_ID=turtlesim ROBOT_TYPE=demo1 envsubst < controller/fiware-cmd-proxy.yaml | kubectl apply -f -
```

```bash
mac:$ kubectl get pods -l pod=cmd-proxy
NAME                         READY     STATUS    RESTARTS   AGE
cmd-proxy-698f8db9c9-7lsrj   1/1       Running   0          29s
cmd-proxy-698f8db9c9-w5w5p   1/1       Running   0          29s
cmd-proxy-698f8db9c9-wtrw8   1/1       Running   0          29s
```

```bash
mac:$ kubectl get services -l service=cmd-proxy
NAME        TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)    AGE
cmd-proxy   ClusterIP   10.100.92.14   <none>        8888/TCP   49s
```
