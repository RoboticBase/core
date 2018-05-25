# 4. operate 'turtlesim' using 'gamepad' and 'web controller'

Operate 'turtlesim' using 'gamepad' and 'web controller' through fiware on minikube by following steps:

1. [reconfigure 'cmd-proxy' to connect 'turtlesim'](#reconfigure-cmd-proxy-to-connect-turtlesim)
1. [start 'turtlesim'](#start-turtlesim)
1. [operate 'turtlesim' using 'gamepad'](#operate-turtlesim-using-gamepad)
1. [operate 'turtlesim' using 'web controller'](#operate-turtlesim-using-web-controller)

## reconfigure 'cmd-proxy' to connect 'turtlesim'

```bash
mac:$ env FIWARE_SERVICE=demo1 FIWARE_SERVICEPATH=/ ROBOT_ID=turtlesim ROBOT_TYPE=demo1 envsubst < controller/fiware-cmd-proxy.yaml | kubectl apply -f -
```

```bash
mac:$ kubectl get deployments,replicasets,pods -l pod=cmd-proxy
NAME                              DESIRED   CURRENT   UP-TO-DATE   AVAILABLE   AGE
deployment.extensions/cmd-proxy   3         3         3            3           1h

NAME                                         DESIRED   CURRENT   READY     AGE
replicaset.extensions/cmd-proxy-698f8db9c9   3         3         3         1h

NAME                             READY     STATUS    RESTARTS   AGE
pod/cmd-proxy-698f8db9c9-7lsrj   1/1       Running   0          1h
pod/cmd-proxy-698f8db9c9-w5w5p   1/1       Running   0          1h
pod/cmd-proxy-698f8db9c9-wtrw8   1/1       Running   0          1h
```

## start 'turtlesim'
1. start X on ros server
1. login X using RDP
1. open terminal1 and start `roscore`

    ```bash
    ros-terminal1:ros_ws$ source devel/setup.bash
    ros-terminal1:ros_ws$ roscore
    ```
1. open terminal2 and start `turtlesim`

    ```bash
    ros-terminal2:ros_ws$ source devel/setup.bash
    ros-terminal2:ros_ws$ rosrun turtlesim turtlesim_node
    ```
1. open terminal3 and start `turtlesim_operator` (ZZZZZZZZZZZZ is the password of "turtlesim", and "HHHHHHHHHH" is the `hostname` of host pc)

    ```bash
    ros-terminal3:ros_ws$ env MQTT_HOST=HHHHHHHHHH TURTLESIM_PASSWORD=ZZZZZZZZZZZZ envsubst < src/turtlesim_operator/config/params-minikube.yaml.template > src/turtlesim_operator/config/params.yaml
    ros-terminal3:ros_ws$ source devel/setup.bash
    ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
    ```

## operate 'turtlesim' using 'gamepad'
1. ssh raspberrypi and start `main.py` (YYYYYYYYYYYY is the password of "raspberrypi", and "HHHHHHHHHH" is the `hostname` of host pc)

    ```bash
    raspberrypi:raspi_gamepad$ env MQTT_HOST="HHHHHHHHHH" RASPI_RASSWORD=YYYYYYYYYYYY envsubst < conf/pxkwcr-minikube.yaml.template > conf/pxkwcr-minikube.yaml
    raspberrypi:raspi_gamepad$ ./main.py pxkwcr-azure
    ```
1. when you press a button of gamepad, 'turtle' moves according to the pressed button

## operate 'turtlesim' using 'web controller'
1. access to 'web controller( http://127.0.0.1:8080/controller/web/ )'
1. login 'web controller' using user/passwrod which is defined by `auth-token.json`
1. when you press a button of web controller, 'turtle' moves according to the pressed button
