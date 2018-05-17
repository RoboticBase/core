# fiware-demo1
This repository construct a [FIWARE](http://www.fiware.org/) platform on [Kubernetes](https://kubernetes.io/) using [minikube](https://github.com/kubernetes/minikube), and interconnect REST Service, IoT device and Robot through FIWARE platform.

## Requirements

||version|
|:--|:--|
|HostOS|macOS Sierra 10.12.6|
|VirtualBox|5.2.12 r122591|
|minikube|0.26.1|
|kubectl|1.10.1|
|helm|2.8.2|

||version|
|:--|:--|
|kubernetes|1.9.4|

## start minikube

```bash
mac:fiware-demo1$ minikube start --cpus 2 --memory 8192 --kubernetes-version v1.9.4 --bootstrapper=localkube
```

```bash
mac:fiware-demo1$ NWNAME=$(VBoxManage showvminfo minikube | grep "Host-only Interface" | awk 'match($0, /vboxnet[0-9]+/){print substr($0,RSTART,RLENGTH)}');echo ${NWNAME}
vboxnet0
mac:fiware-demo1$ HOST_IPADDR=$(ifconfig ${NWNAME} | awk '/inet / {print $2}');echo ${HOST_IPADDR}
192.168.99.1
mac:fiware-demo1$ route -n get ${HOST_IPADDR}
   route to: 192.168.99.1
destination: 192.168.99.0
       mask: 255.255.255.0
  interface: vboxnet0
      flags: <UP,DONE,CLONING>
 recvpipe  sendpipe  ssthresh  rtt,msec    rttvar  hopcount      mtu     expire
       0         0         0         0         0         0      1500     -3009
```

```bash
mac:fiware-demo1$ cp -p ~/.minikube/machines/minikube/config.json ~/.minikube/machines/minikube/config.json.org
mac:fiware-demo1$ vi /Users/nmatsui/.minikube/machines/minikube/config.json
mac:fiware-demo1$ diff -u ${HOME}/.minikube/machines/minikube/config.json.org ${HOME}/.minikube/machines/minikube/cofig.json
```

```diff
--- /Users/nmatsui/.minikube/machines/minikube/config.json.org	2018-05-15 09:21:13.000000000 +0900
+++ /Users/nmatsui/.minikube/machines/minikube/config.json	2018-05-15 09:29:29.000000000 +0900
@@ -41,6 +41,7 @@
             "Env": null,
             "Ipv6": false,
             "InsecureRegistry": [
+                "192.168.99.0/24",
                 "10.96.0.0/12"
             ],
             "Labels": null,
@@ -83,4 +84,4 @@
         }
     },
     "Name": "minikube"
-}
\ No newline at end of file
+}
```

```bash
mac:fiware-demo1$ minikube stop
mac:fiware-demo1$ minikube start --cpus 2 --memory 8192 --kubernetes-version v1.9.4 --bootstrapper=localkube
```

```bash
mac:fiware-demo1$ kubectl version
Client Version: version.Info{Major:"1", Minor:"10", GitVersion:"v1.10.1", GitCommit:"d4ab47518836c750f9949b9e0d387f20fb92260b", GitTreeState:"clean", BuildDate:"2018-04-12T14:26:04Z", GoVersion:"go1.9.3", Compiler:"gc", Platform:"darwin/amd64"}
Server Version: version.Info{Major:"", Minor:"", GitVersion:"v1.9.4", GitCommit:"bee2d1505c4fe820744d26d41ecd3fdd4a3d6546", GitTreeState:"clean", BuildDate:"2018-03-21T21:48:36Z", GoVersion:"go1.9.1", Compiler:"gc", Platform:"linux/amd64"}
```

```bash
mac:fiware-demo1$ kubectl get nodes
NAME       STATUS    ROLES     AGE       VERSION
minikube   Ready     <none>    10m       v1.9.4
```

## start local registry on hostOS (outside minikube)

* Docker for mac > Preference > Daemon > Advanced

```diff
mac:fiware-demo1$ diff -u /tmp/docker-mac-configuration.json.org /tmp/docker-mac-configuration.json
--- /tmp/docker-mac-configuration.json.org	2018-05-15 09:41:37.000000000 +0900
+++ /tmp/docker-mac-configuration.json	2018-05-15 09:42:39.000000000 +0900
@@ -1,4 +1,7 @@
 {
   "debug" : true,
-  "experimental" : true
+  "experimental" : true,
+  "insecure-registries" : [
+    "192.168.99.0/24"
+  ]
 }
```

```bash
mac:fiware-demo1$ docker run --name registry -p 5000:5000 -d registry:2.6
mac:fiware-demo1$ export REPOSITORY=${HOST_IPADDR}:5000;echo ${REPOSITORY}
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

## start etcd cluster on minikube

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
mac:fiware-demo1$ kubectl apply -f etcd/etcd-cluster.yaml
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
mac:fiware-demo1$ kubectl run --rm -it etcd-client --image quay.io/coreos/etcd --restart=Never -- etcdctl --peers http://etcd-cluster-client:2379 member list
324fbe655cac95f8: name=etcd-cluster-0000 peerURLs=http://etcd-cluster-0000.etcd-cluster.default.svc:2380 clientURLs=http://etcd-cluster-0000.etcd-cluster.default.svc:2379 isLeader=true
971de8afdc2639c4: name=etcd-cluster-0002 peerURLs=http://etcd-cluster-0002.etcd-cluster.default.svc:2380 clientURLs=http://etcd-cluster-0002.etcd-cluster.default.svc:2379 isLeader=false
ef8519feb19ec20c: name=etcd-cluster-0001 peerURLs=http://etcd-cluster-0001.etcd-cluster.default.svc:2380 clientURLs=http://etcd-cluster-0001.etcd-cluster.default.svc:2379 isLeader=false
```

## start vernemq cluster on minikube

* create usernames & passwords of vernemq
```bash
mac:fiware-demo1$ mkdir -p secrets
mac:fiware-demo1$ touch secrets/vmq.passwd
mac:fiware-demo1$ docker run --rm -v $(pwd)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd iotagent
mac:fiware-demo1$ docker run --rm -v $(pwd)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd raspberrypi
mac:fiware-demo1$ docker run --rm -v $(pwd)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd turtlesim
```

```bash
mac:fiware-demo1$ kubectl create secret generic vernemq-passwd --from-file=./secrets/vmq.passwd
```

```bash
mac:fiware-demo1$ kubectl get secrets
NAME                                                          TYPE                                  DATA      AGE
default-token-r4sz4                                           kubernetes.io/service-account-token   3         34m
fiware-etcd-etcd-operator-etcd-backup-operator-token-jw8dt    kubernetes.io/service-account-token   3         4m
fiware-etcd-etcd-operator-etcd-operator-token-p9kln           kubernetes.io/service-account-token   3         4m
fiware-etcd-etcd-operator-etcd-restore-operator-token-mbn6k   kubernetes.io/service-account-token   3         4m
vernemq-passwd                                                Opaque                                1         9s
```

```bash
mac:fiware-demo1$ kubectl apply -f vernemq/vernemq-cluster-minikube.yaml
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
mac:fiware-demo1$ kubectl get services -l app=mqtt
NAME      TYPE           CLUSTER-IP       EXTERNAL-IP   PORT(S)          AGE
mqtt      LoadBalancer   10.102.204.163   <pending>     1883:31136/TCP   36s
```

```bash
mac:fiware-demo1$ VBoxManage controlvm "minikube" natpf1 "mqtt,tcp,0.0.0.0,1883,,31136"
```

## start ambassador on minikube

```bash
mac:fiware-demo1$ kubectl apply -f ambassador/ambassador-minikube.yaml
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
NAME         TYPE           CLUSTER-IP      EXTERNAL-IP   PORT(S)        AGE
ambassador   LoadBalancer   10.105.25.105   <pending>     80:31952/TCP   3m
```

```bash
mac:fiware-demo1$ VBoxManage controlvm "minikube" natpf1 "ambassador,tcp,0.0.0.0,8080,,31952"
```

* create random string
```bash
mac:fiware-demo1$ cat /dev/urandom | LC_CTYPE=C tr -dc 'a-zA-Z0-9' | head -c 32
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
mac:fiware-demo1$ kubectl create secret generic auth-tokens --from-file=./secrets/auth-tokens.json
```

```bash
mac:fiware-demo1$ kubectl get secrets
NAME                                                          TYPE                                  DATA      AGE
auth-tokens                                                   Opaque                                1         4s
default-token-r4sz4                                           kubernetes.io/service-account-token   3         1h
fiware-etcd-etcd-operator-etcd-backup-operator-token-jw8dt    kubernetes.io/service-account-token   3         50m
fiware-etcd-etcd-operator-etcd-operator-token-p9kln           kubernetes.io/service-account-token   3         50m
fiware-etcd-etcd-operator-etcd-restore-operator-token-mbn6k   kubernetes.io/service-account-token   3         50m
vernemq-passwd                                                Opaque
```

```bash
mac:fiware-demo1$ docker build -t ${HOST_IPADDR}:5000/tech-sketch/fiware-bearer-auth:0.1.0 ./ambassador/fiware-bearer-auth
mac:fiware-demo1$ docker push ${HOST_IPADDR}:5000/tech-sketch/fiware-bearer-auth:0.1.0
```

```bash
mac:fiware-demo1$ sed -e "s/<<LOCAL_REPOSITORY>>/${HOST_IPADDR}:5000/g" ambassador/bearer-auth-minikube.yaml | kubectl apply -f -
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

## start orion on minikube

```bash
mac:fiware-demo1$ kubectl apply -f orion/orion-mongodb-minikube.yaml
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" http://127.0.0.1:8080/orion/v2/entities/
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" http://127.0.0.1:8080/orion/v2/subscriptions/
HTTP/1.1 200 OK
content-length: 2
content-type: application/json
fiware-correlator: 5a4ecc6e-4dc1-11e8-b1a2-0a580af4010a
date: Wed, 02 May 2018 04:29:07 GMT
x-envoy-upstream-service-time: 2
server: envoy

[]
```

## start idas on minikube

```bash
mac:fiware-demo1$ kubectl apply -f idas/idas-mongodb-minikube.yaml
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
mac:fiware-demo1$ docker build -t ${HOST_IPADDR}:5000/tech-sketch/fiware-mqtt-msgfilter:0.1.0 idas/fiware-mqtt-msgfilter/
mac:fiware-demo1$ docker push ${HOST_IPADDR}:5000/tech-sketch/fiware-mqtt-msgfilter:0.1.0
```

```bash
mac:fiware-demo1$ sed -e "s/<<LOCAL_REPOSITORY>>/${HOST_IPADDR}:5000/g" idas/mqtt-msgfilter-minikube.yaml | kubectl apply -f -
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
mac:fiware-demo1$ docker build -t ${HOST_IPADDR}:5000/tech-sketch/iotagent-ul:1.6.0 idas/iotagent-ul/
mac:fiware-demo1$ docker push ${HOST_IPADDR}:5000/tech-sketch/iotagent-ul:1.6.0
```

```bash
mac:fiware-demo1$ sed -e "s/<<LOCAL_REPOSITORY>>/${HOST_IPADDR}:5000/g" idas/iotagent-ul-minikube.yaml| kubectl apply -f -
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" http://127.0.0.1:8080/idas/ul20/manage/iot/services/
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://127.0.0.1:8080/idas/ul20/manage/iot/devices/
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

## start cygnus on minikube

```bash
mac:fiware-demo1$ kubectl apply -f cygnus/cygnus-mongodb-minikube.yaml
```

```bash
mac:fiware-demo1$ kubectl get pods -l app=cygnus-mongodb
NAME               READY     STATUS    RESTARTS   AGE
cygnus-mongodb-0   2/2       Running   0          29s
cygnus-mongodb-1   2/2       Running   0          20s
cygnus-mongodb-2   2/2       Running   0          12s
```

```bash
mac:fiware-demo1$ kubectl get services -l app=cygnus-mongodb
NAME             TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)     AGE
cygnus-mongodb   ClusterIP   None         <none>        27017/TCP   55s
```

```bash
mac:fiware-demo1$ kubectl exec cygnus-mongodb-0 -c cygnus-mongodb -- mongo --eval 'printjson(rs.status().members.map(function(e) {return {name: e.name, stateStr:e.stateStr};}))'
MongoDB shell version v3.6.4
connecting to: mongodb://127.0.0.1:27017
MongoDB server version: 3.6.4
[
	{
		"name" : "cygnus-mongodb-0.cygnus-mongodb.default.svc.cluster.local:27017",
		"stateStr" : "PRIMARY"
	},
	{
		"name" : "cygnus-mongodb-1.cygnus-mongodb.default.svc.cluster.local:27017",
		"stateStr" : "SECONDARY"
	},
	{
		"name" : "cygnus-mongodb-2.cygnus-mongodb.default.svc.cluster.local:27017",
		"stateStr" : "SECONDARY"
	}
]
```

```bash
mac:fiware-demo1$ docker build -t ${HOST_IPADDR}:5000/tech-sketch/cygnus-ngsi:1.8.0 cygnus/fiware-cygnus/
mac:fiware-demo1$ docker push ${HOST_IPADDR}:5000/tech-sketch/cygnus-ngsi:1.8.0
```

```bash
mac:fiware-demo1$ sed -e "s/<<LOCAL_REPOSITORY>>/${HOST_IPADDR}:5000/g" cygnus/cygnus-minikube.yaml | kubectl apply -f -
```

```bash
mac:fiware-demo1$ kubectl get pods -l app=cygnus
NAME                      READY     STATUS    RESTARTS   AGE
cygnus-5c68fb6578-fdmtg   1/1       Running   0          44s
cygnus-5c68fb6578-stmds   1/1       Running   0          44s
cygnus-5c68fb6578-z85lp   1/1       Running   0          44s
```

```bash
mac:fiware-demo1$ kubectl get services -l app=cygnus
NAME      TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)             AGE
cygnus    ClusterIP   10.103.255.240   <none>        5050/TCP,8081/TCP   1m
```

## start cmd-proxy on miikube

```bash
mac:fiware-demo1$ docker build --build-arg PORT=8888 -t ${HOST_IPADDR}:5000/tech-sketch/fiware-cmd-proxy:0.1.0 ./controller/fiware-cmd-proxy/
mac:fiware-demo1$ docker push ${HOST_IPADDR}:5000/tech-sketch/fiware-cmd-proxy:0.1.0
```

```bash
mac:fiware-demo1$ sed -e "s/<<LOCAL_REPOSITORY>>/${HOST_IPADDR}:5000/g" controller/fiware-cmd-proxy-minikube.yaml| kubectl apply -f -
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" http://127.0.0.1:8080/idas/ul20/manage/iot/services/ -X POST -d @- <<__EOS__
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" http://127.0.0.1:8080/idas/ul20/manage/iot/services/ | jq .
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" http://127.0.0.1:8080/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://127.0.0.1:8080/idas/ul20/manage/iot/devices/gamepad/ | jq .
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://127.0.0.1:8080/orion/v2/entities/gamepad/ | jq .
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
mac:fiware-demo1$ mosquitto_sub -h 127.0.0.1 -p 1883 -d -t /# -u iotagent -P XXXXXXXXXXXX
...
```

* replace `<<password_of_raspberrypi>>` to the password of "raspberrypi"
```bash
raspberrypi:raspi_gamepad$ cat conf/pxkwcr-minikube.yaml.template | sed -e "s/<<password_of_raspberrypi>>/YYYYYYYYYYYY/g" > conf/pxkwcr-minikube.yaml
raspberrypi:raspi_gamepad$ ./main.py pxkwcr-minikube
2018/05/06 11:46:04 [   INFO] __main__ - run script using pxkwcr-minikube.yaml
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
mac:fiware-demo1$ mosquitto_sub -h 127.0.0.1 -p 1883 -d -t /# -u iotagent -P XXXXXXXXXXXX
...
...
Client mosqsub|60435-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/gamepad/attrs', ... (45 bytes))
2018-05-06T02:46:14.876494+0000|button|circle
...
```

```bash
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://127.0.0.1:8080/orion/v2/entities/gamepad/ | jq .
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" http://127.0.0.1:8080/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://127.0.0.1:8080/idas/ul20/manage/iot/devices/turtlesim/ | jq .
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://127.0.0.1:8080/orion/v2/entities/turtlesim/ | jq .
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

* XXXXXXXXXXXX is the password of "iotagent"
```text
mac:fiware-demo1$ mosquitto_sub -h 127.0.0.1 -p 1883 -d -t /# -u iotagent -P XXXXXXXXXXXX
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

* replace `<<password_of_turtlesim>>` to the password of "turtlesim"
```bash
ros-terminal3:ros_ws$ cat src/turtlesim_operator/config/params-minikube.yaml.template | sed -e "s/<<password_of_turtlesim>>/ZZZZZZZZZZZZ/g" > src/turtlesim_operator/config/params.yaml
ros-terminal3:ros_ws$ source devel/setup.bash
ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
...
[INFO] [1525575494.645546]: [turtlesim_operator.command_sender:CommandSender._on_connect] mqtt connect status=0
[INFO] [1525575494.648536]: [turtlesim_operator.attribute_receiver:AttributeReceiver._on_connect] mqtt connect status=0
...
```

* send 'circle' cmd to 'turtlesim' entity
```bash
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" -H "Content-Type: application/json" http://127.0.0.1:8080/orion/v1/updateContext -d @-<<__EOS__ | jq .
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
mac:fiware-demo1$ mosquitto_sub -h 127.0.0.1 -p 1883 -d -t /# -u iotagent -P XXXXXXXXXXXX
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://127.0.0.1:8080/orion/v2/entities/turtlesim/ | jq .
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" http://127.0.0.1:8080/orion/v2/subscriptions/ -X POST -d @- <<__EOS__
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" http://127.0.0.1:8080/orion/v2/subscriptions/ | jq .
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

## register cygnus

```bash
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" http://127.0.0.1:8080/orion/v2/subscriptions/ -X POST -d @- <<__EOS__
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
mac:fiware-demo1$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" http://127.0.0.1:8080/orion/v2/subscriptions/ | jq .
[
  {
    "id": "5afa4e2b3b474a846e5a86ff",
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
      "timesSent": 99,
      "lastNotification": "2018-05-15T06:00:02.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "normalized",
      "http": {
        "url": "http://cmd-proxy:8888/gamepad/"
      },
      "lastSuccess": "2018-05-15T06:00:02.00Z"
    }
  },
  {
    "id": "5afb5e6dd7e508de16c8faee",
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
      "lastNotification": "2018-05-15T22:25:49.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "normalized",
      "http": {
        "url": "http://cygnus:5050/notify"
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
