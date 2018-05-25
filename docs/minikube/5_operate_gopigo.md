# 5. operate 'gopigo' using 'gamepad' and 'web controller' (if gopigo is available)

Operate 'gopigo' using 'gamepad' and 'web controller' through fiware on minikube by following steps:

1. [reconfigure 'cmd-proxy' to connect 'gopigo'](#reconfigure-cmd-proxy-to-connect-gopigo)
1. [start 'gopigo'](#start-gopigo)
1. [operate 'gopigo' using 'gamepad'](#operate-gopigo-using-gamepad)
1. [operate 'gopigo' using 'web controller'](#operate-gopigo-using-web-controller)

## reconfigure 'cmd-proxy' to connect 'gopigo'

```bash
mac:$ env FIWARE_SERVICE=demo1 FIWARE_SERVICEPATH=/ ROBOT_ID=gopigo ROBOT_TYPE=demo1 envsubst < controller/fiware-cmd-proxy.yaml | kubectl apply -f -
```

```bash
mac:$ kubectl get deployments,replicasets,pods -l pod=cmd-proxy
NAME                              DESIRED   CURRENT   UP-TO-DATE   AVAILABLE   AGE
deployment.extensions/cmd-proxy   3         3         3            3           1h

NAME                                         DESIRED   CURRENT   READY     AGE
replicaset.extensions/cmd-proxy-576bd95dcc   3         3         3         22s
replicaset.extensions/cmd-proxy-698f8db9c9   0         0         0         1h

NAME                             READY     STATUS    RESTARTS   AGE
pod/cmd-proxy-576bd95dcc-f4gfl   1/1       Running   0          21s
pod/cmd-proxy-576bd95dcc-hhnsj   1/1       Running   0          19s
pod/cmd-proxy-576bd95dcc-hrwtp   1/1       Running   0          22s
```

## start 'gopigo'
1. ssh to gopigo on terminal1 and start `roscore`

    ```bash
    ros-terminal1:gopigo_ws$ source devel/setup.bash
    ros-terminal1:gopigo_ws$ roscore
    ```
1. ssh to gopigo on terminal2 and start `ros_gopigo` (ZZZZZZZZZZZZ is the password of "gopigo", and "HHHHHHHHHH" is the `hostname` of host pc)
    ```bash
    ros-terminal2:gopigo_ws$ env MQTT_HOST=HHHHHHHHHH GOPIGO_PASSWORD=ZZZZZZZZZZZZ envsubst < src/ros_gopigo/config/params-minikube.yaml.template > src/ros_gopigo/config/params.yaml
    ros-terminal2:gopigo_ws$ source devel/setup.bash
    ros-terminal2:gopigo_ws$ roslaunch ros_gopigo ros_gopigo.launch
    ```

## operate 'gopigo' using 'gamepad'
1. ssh raspberrypi and start `main.py` (YYYYYYYYYYYY is the password of "raspberrypi", and "HHHHHHHHHH" is the `hostname` of host pc)

    ```bash
    raspberrypi:raspi_gamepad$ env MQTT_HOST="HHHHHHHHHH" RASPI_RASSWORD=YYYYYYYYYYYY envsubst < conf/pxkwcr-minikube.yaml.template > conf/pxkwcr-minikube.yaml
    raspberrypi:raspi_gamepad$ ./main.py pxkwcr-azure
    ```
1. when you press a button of gamepad, 'gopigo' moves according to the pressed button

## operate 'gopigo' using 'web controller'
1. access to 'web controller( http://127.0.0.1:8080/controller/web/ )'
1. login 'web controller' using user/passwrod which is defined by `auth-token.json`
1. when you press a button of web controller, 'gopigo' moves according to the pressed button
