# fiware-demo1
This repository construct a [FIWARE](http://www.fiware.org/) platform on [Kubernetes](https://kubernetes.io/), and interconnect REST Service, IoT device and Robot through FIWARE platform.

## Requirements

||version|
|:--|:--|
|azure cli|2.0.31|
|kubectl|1.10.1|

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
aks-nodepool1-27506152-0   Ready     agent     5m        v1.8.11
aks-nodepool1-27506152-1   Ready     agent     5m        v1.8.11
aks-nodepool1-27506152-2   Ready     agent     5m        v1.8.11
aks-nodepool1-27506152-3   Ready     agent     5m        v1.8.11
```

```bash
mac:$ CLIENT_ID=$(az aks show --resource-group fiware-demo --name fiwareaks --query "servicePrincipalProfile.clientId" --output tsv);echo ${CLIENT_ID}
mac:$ ACR_ID=$(az acr show --name fiwareacr --resource-group fiware-demo --query "id" --output tsv); echo ${ACR_ID}
mac:$ az role assignment create --assignee ${CLIENT_ID} --role Reader --scope ${ACR_ID}
```

## start vernemq cluster on AKS

* create usernames & passwords of vernemq
```bash
mac:$ mkdir -p secrets
mac:$ touch secrets/vmq.passwd
mac:$ docker run --rm -v $(PWD)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd iotagent
mac:$ docker run --rm -v $(PWD)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd raspberrypi
mac:$ docker run --rm -v $(PWD)/secrets:/mnt -it erlio/docker-vernemq vmq-passwd /mnt/vmq.passwd turtlesim
```

```bash
mac:$ docker run -it -v $(PWD)/secrets:/etc/letsencrypt certbot/certbot certonly --manual --domain mqtt.nmatsui.work --email nobuyuki.matsui@gmail.com --agree-tos --manual-public-ip-logging-ok --preferred-challenges dns
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
mac:$ cat secrets/DST_Root_CA_X3.pem secrets/archive/mqtt.nmatsui.work/chain1.pem > secrets/ca.crt
mac:$ cp secrets/archive/mqtt.nmatsui.work/fullchain1.pem secrets/server.crt
mac:$ cp secrets/archive/mqtt.nmatsui.work/privkey1.pem secrets/server.key
```

```bash
mac:$ kubectl create secret generic vernemq-passwd --from-file=./secrets/vmq.passwd
mac:$ kubectl create secret generic vernemq-certifications --from-file=./secrets/ca.crt --from-file=./secrets/server.crt --from-file=./secrets/server.key
```

```bash
mac:$ kubectl get secrets
NAME                     TYPE                                  DATA      AGE
default-token-klh8w      kubernetes.io/service-account-token   3         14m
vernemq-certifications   Opaque                                3         48s
vernemq-passwd           Opaque                                1         1m
```

```bash
mac:$ kubectl apply -f vernemq/vernemq-cluster.yaml
```

```bash
mac:$ kubectl get pods -l app=vernemq
NAME        READY     STATUS    RESTARTS   AGE
vernemq-0   1/1       Running   0          1m
vernemq-1   1/1       Running   0          1m
vernemq-2   1/1       Running   0          33s
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
mac:$ kubectl get services -l app=mqtts
NAME      TYPE           CLUSTER-IP     EXTERNAL-IP       PORT(S)          AGE
mqtts     LoadBalancer   10.0.170.212   WWW.XXX.YYY.ZZZ   8883:30187/TCP   3m
```

```bash
mac:$ az network dns record-set a add-record --resource-group nmatsui_dns --zone-name nmatsui.work --record-set-name "mqtt" --ipv4-address "WWW.XXX.YYY.ZZZ"
```

## start ambassador on AKS

```bash
mac:$ docker run -it -v $(PWD)/secrets:/etc/letsencrypt certbot/certbot certonly --manual --domain api.nmatsui.work --email nobuyuki.matsui@gmail.com --agree-tos --manual-public-ip-logging-ok --preferred-challenges dns
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
mac:$ kubectl create secret tls ambassador-certs --cert=$(PWD)/secrets/live/api.nmatsui.work/fullchain.pem --key=$(PWD)/secrets/live/api.nmatsui.work/privkey.pem
```

```bash
mac:$ kubectl apply -f ambassador/ambassador.yaml
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
mac:$ az network dns record-set a add-record --resource-group nmatsui_dns --zone-name nmatsui.work --record-set-name "api" --ipv4-address "www.xxx.yyy.zzz"
```

```bash
mac:$ STR=$(cat /dev/urandom | LC_CTYPE=C tr -dc 'a-zA-Z0-9' | head -c 32); cat << __EOS__ > secrets/auth-tokens.json
{"${STR}": ["^/orion/.*$", "^/idas/.*$"]}
__EOS__
```

```bash
mac:$ kubectl create secret generic auth-tokens --from-file=./secrets/auth-tokens.json
```

```bash
mac:$ kubectl get secrets
NAME                     TYPE                                  DATA      AGE
ambassador-certs         kubernetes.io/tls                     2         1h
auth-tokens              Opaque                                1         21s
default-token-klh8w      kubernetes.io/service-account-token   3         2h
vernemq-certifications   Opaque                                3         2h
vernemq-passwd           Opaque                                1         2h
```

```bash
mac:$ az acr login --name fiwareacr
```

```bash
mac:$ docker build -t fiwareacr.azurecr.io/tech-sketch/fiware-bearer-auth:0.1.0 ./ambassador/fiware-bearer-auth
mac:$ docker push fiwareacr.azurecr.io/tech-sketch/fiware-bearer-auth:0.1.0
```

```bash
mac:$ az acr repository list --name fiwareacr --output table
Result
------------------------------
tech-sketch/fiware-bearer-auth
```

```bash
mac:$ kubectl apply -f ambassador/bearer-auth.yaml
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
mac:$ kubectl apply -f orion/orion-mongodb.yaml
```

```bash
mac:$ kubectl get pods -l app=orion-mongodb
NAME              READY     STATUS    RESTARTS   AGE
orion-mongodb-0   2/2       Running   0          10m
orion-mongodb-1   2/2       Running   0          7m
orion-mongodb-2   2/2       Running   0          5m
```

```bash
mac:$ kubectl get services -l app=orion-mongodb
NAME            TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)     AGE
orion-mongodb   ClusterIP   None         <none>        27017/TCP   10m
```

```bash
mac:$ kubectl exec orion-mongodb-0 -c orion-mongodb -- mongo --eval 'printjson(rs.status().members.map(function(e) {return {name: e.name, stateStr:e.stateStr};}))'
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -i -H "Authorization: bearer ${TOKEN}" https://api.nmatsui.work/orion/v2/entities/
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -i -H "Authorization: bearer ${TOKEN}" https://api.nmatsui.work/orion/v2/subscriptions/
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
mac:$ kubectl apply -f idas/idas-mongodb.yaml
```

```bash
mac:$ kubectl get pods -l app=idas-mongodb
NAME             READY     STATUS    RESTARTS   AGE
idas-mongodb-0   2/2       Running   0          12m
idas-mongodb-1   2/2       Running   0          10m
idas-mongodb-2   2/2       Running   0          7m
```

```bash
mac:$ kubectl get services -l app=idas-mongodb
NAME           TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)     AGE
idas-mongodb   ClusterIP   None         <none>        27017/TCP   12m
```

```bash
mac:$ kubectl exec idas-mongodb-0 -c idas-mongodb -- mongo --eval 'printjson(rs.status().members.map(function(e) {return {name: e.name, stateStr:e.stateStr};}))'
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

* replace `<<password_of_iotagent>>` to the password of "iotagent"
```bash
mac:$ sed -e 's/<<password_of_iotagent>>/XXXXXXXXXXXX/g' idas/iotagent-ul/config.js.template > idas/iotagent-ul/config.js
```

```bash
mac:$ az acr login --name fiwareacr
```

```bash
mac:$ GIT_REV_IOTA_UL="1.6.0"
mac:$ docker build --build-arg GIT_REV_IOTA=${GIT_REV_IOTA_UL} -t fiwareacr.azurecr.io/tech-sketch/iotagent-ul:${GIT_REV_IOTA_UL} idas/iotagent-ul/
mac:$ docker push fiwareacr.azurecr.io/tech-sketch/iotagent-ul:${GIT_REV_IOTA_UL}
```

```bash
mac:$ az acr repository list --name fiwareacr --output table
Result
------------------------------
tech-sketch/fiware-bearer-auth
tech-sketch/iotagent-ul
```

```bash
mac:$ cat idas/iotagent-ul.yaml | sed -e "s/<<GIT_REV_IOTA>>/${GIT_REV_IOTA_UL}/g" | kubectl apply -f -
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" https://api.nmatsui.work/idas/ul20/manage/iot/services/
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -i -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.nmatsui.work/idas/ul20/manage/iot/devices/
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

## register "gamepad" as an IoT device of "demo1" service

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.nmatsui.work/idas/ul20/manage/iot/services/ -X POST -d @- <<__EOS__
{
  "services": [
    {
      "apikey": "gamepad",
      "cbroker": "http://orion:1026",
      "resource": "/iot/d",
      "entity_type": "gamepad"
    }
  ]
}
__EOS__
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" https://api.nmatsui.work/idas/ul20/manage/iot/services/ | jq .
{
  "count": 1,
  "services": [
    {
      "_id": "5ae95ec67e6ec400019acd28",
      "subservice": "/",
      "service": "demo1",
      "apikey": "gamepad",
      "resource": "/iot/d",
      "__v": 0,
      "attributes": [],
      "lazy": [],
      "commands": [],
      "entity_type": "gamepad",
      "internal_attributes": [],
      "static_attributes": []
    }
  ]
}
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.nmatsui.work/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
{
  "devices": [
    {
      "device_id": "gamepad-1",
      "entity_name": "gamepad-1",
      "entity_type": "gamepad",
      "timezone": "Asia/Tokyo",
      "protocol": "UL20",
      "attributes": [
        {
          "name": "button",
          "type": "string"
        },
        {
          "name": "hat",
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.nmatsui.work/idas/ul20/manage/iot/devices/ | jq .
{
  "count": 1,
  "devices": [
    {
      "device_id": "gamepad-1",
      "service": "demo1",
      "service_path": "/",
      "entity_name": "gamepad-1",
      "entity_type": "gamepad",
      "transport": "MQTT",
      "attributes": [
        {
          "object_id": "button",
          "name": "button",
          "type": "string"
        },
        {
          "object_id": "hat",
          "name": "hat",
          "type": "string"
        }
      ],
      "lazy": [],
      "commands": [],
      "static_attributes": [],
      "protocol": "UL20"
    }
  ]
}
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.nmatsui.work/orion/v2/entities/gamepad-1/ | jq .
{
  "id": "gamepad-1",
  "type": "gamepad",
  "TimeInstant": {
    "type": "ISO8601",
    "value": " ",
    "metadata": {}
  },
  "button": {
    "type": "string",
    "value": " ",
    "metadata": {}
  },
  "hat": {
    "type": "string",
    "value": " ",
    "metadata": {}
  }
}
```

## register "ros" as an IoT device of "demo1" service

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.nmatsui.work/idas/ul20/manage/iot/services/ -X POST -d @- <<__EOS__
{
  "services": [
    {
      "apikey": "ros",
      "cbroker": "http://orion:1026",
      "resource": "/iot/d",
      "entity_type": "ros"
    }
  ]
}
__EOS__
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" https://api.nmatsui.work/idas/ul20/manage/iot/services/ | jq .
{
  "count": 2,
  "services": [
    {
      "_id": "5ae95ec67e6ec400019acd28",
      "subservice": "/",
      "service": "demo1",
      "apikey": "gamepad",
      "resource": "/iot/d",
      "__v": 0,
      "attributes": [],
      "lazy": [],
      "commands": [],
      "entity_type": "gamepad",
      "internal_attributes": [],
      "static_attributes": []
    },
    {
      "_id": "5ae969ba7e6ec400019acd2a",
      "subservice": "/",
      "service": "demo1",
      "apikey": "ros",
      "resource": "/iot/d",
      "__v": 0,
      "attributes": [],
      "lazy": [],
      "commands": [],
      "entity_type": "ros",
      "internal_attributes": [],
      "static_attributes": []
    }
  ]
}
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.nmatsui.work/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
{
  "devices": [
    {
      "device_id": "turtlesim",
      "entity_name": "turtlesim",
      "entity_type": "ros",
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.nmatsui.work/idas/ul20/manage/iot/devices/ | jq .
{
  "count": 2,
  "devices": [
    {
      "device_id": "gamepad-1",
      "service": "demo1",
      "service_path": "/",
      "entity_name": "gamepad-1",
      "entity_type": "gamepad",
      "transport": "MQTT",
      "attributes": [
        {
          "object_id": "button",
          "name": "button",
          "type": "string"
        },
        {
          "object_id": "hat",
          "name": "hat",
          "type": "string"
        }
      ],
      "lazy": [],
      "commands": [],
      "static_attributes": [],
      "protocol": "UL20"
    },
    {
      "device_id": "turtlesim",
      "service": "demo1",
      "service_path": "/",
      "entity_name": "turtlesim",
      "entity_type": "ros",
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
  ]
}
```

```bash
$ TOKEN=$(cat secrets/auth-tokens.json | jq '.|keys[0]' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.nmatsui.work/orion/v2/entities/turtlesim/ | jq .
{
  "id": "turtlesim",
  "type": "ros",
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
