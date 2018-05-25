# 1. prepare minikube

Prepare [minikube](https://github.com/kubernetes/minikube) by following steps:

## start minikube

```bash
mac:$ minikube start --cpus 2 --memory 8192 --kubernetes-version v1.9.4 --bootstrapper=localkube
```

## check network configuration of minikube

```bash
mac:$ NWNAME=$(VBoxManage showvminfo minikube | grep "Host-only Interface" | awk 'match($0, /vboxnet[0-9]+/){print substr($0,RSTART,RLENGTH)}');echo ${NWNAME}
mac:$ HOST_IPADDR=$(ifconfig ${NWNAME} | awk '/inet / {print $2}');echo ${HOST_IPADDR}
mac:$ NETMASK=$(ifconfig ${NWNAME} | awk '/netmask / {print $4}');echo ${NETMASK}
```

## edit configuration of minikube

Edit `InsecureRegistry` of minikube in order to access local registory.

```bash
mac:$ cp -p ~/.minikube/machines/minikube/config.json ~/.minikube/machines/minikube/config.json.org
mac:$ vi ~/.minikube/machines/minikube/config.json
mac:$ diff -u ~/.minikube/machines/minikube/config.json.org ~/.minikube/machines/minikube/config.json
```

```diff
--- /Users/nmatsui/.minikube/machines/minikube/config.json.org	2018-05-25 17:05:38.000000000 +0900
+++ /Users/nmatsui/.minikube/machines/minikube/config.json	2018-05-25 17:20:11.000000000 +0900
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

## restart minikube

```bash
mac:$ minikube stop
mac:$ minikube start --cpus 2 --memory 8192 --kubernetes-version v1.9.4 --bootstrapper=localkube
```

```bash
mac:$ kubectl get nodes
NAME       STATUS    ROLES     AGE       VERSION
minikube   Ready     master    18m       v1.9.4
```

## start local registry on hostOS

Edit `InsecureRegistry` of local docker daemon in order to accept minikube.

** Docker for mac > Preference > Daemon > Advanced **

```diff
diff -u /tmp/docker-mac-configuration.json.org /tmp/docker-mac-configuration.json
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
mac:$ docker run --name registry -p 5000:5000 -d registry:2.6
mac:$ export REPOSITORY=${HOST_IPADDR}:5000;echo ${REPOSITORY}
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
kube-system   tiller-deploy-865dd6c794-9z42z          1/1       Running   0          31s
```

```bash
mac:$ helm version
Client: &version.Version{SemVer:"v2.8.2", GitCommit:"a80231648a1473929271764b920a8e346f6de844", GitTreeState:"clean"}
Server: &version.Version{SemVer:"v2.8.2", GitCommit:"a80231648a1473929271764b920a8e346f6de844", GitTreeState:"clean"}
```
