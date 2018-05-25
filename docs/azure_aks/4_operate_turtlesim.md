# 4. operate 'turtlesim' using 'gamepad' and 'web controller'

Operate 'turtlesim' using 'gamepad' and 'web controller' through fiware on AKS.

**In the following document, replace "example.com" with your domain.**

## reconfigure 'cmd-proxy' to connect 'turtlesim'

```bash
mac:$ env FIWARE_SERVICE=demo1 FIWARE_SERVICEPATH=/ ROBOT_ID=turtlesim ROBOT_TYPE=demo1 envsubst < controller/fiware-cmd-proxy.yaml | kubectl apply -f -
```

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
1. open terminal3 and start `turtlesim_operator` (ZZZZZZZZZZZZ is the password of "turtlesim")

    ```bash
    ros-terminal3:ros_ws$ env MQTT_HOST=mqtt.example.com TURTLESIM_PASSWORD=ZZZZZZZZZZZZ envsubst < src/turtlesim_operator/config/params-azure.yaml.template > src/turtlesim_operator/config/params.yaml
    ros-terminal3:ros_ws$ source devel/setup.bash
    ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
    ```

## operate 'turtlesim' using 'gamepad'
1. ssh raspberrypi and start `main.py` (YYYYYYYYYYYY is the password of "raspberrypi")

    ```bash
    raspberrypi:raspi_gamepad$ env MQTT_HOST=mqtt.example.com RASPI_RASSWORD=YYYYYYYYYYYY envsubst < conf/pxkwcr-azure.yaml.template > conf/pxkwcr-azure.yaml
    raspberrypi:raspi_gamepad$ ./main.py pxkwcr-azure
    ```
1. when you press a button of gamepad, 'turtle' moves according to the pressed button

### operate 'turtlesim' using 'web controller'
1. access to 'web controller( https://api.example.com/controller/web/ )'
1. login 'web controller' using user/passwrod which is defined by `auth-token.json`
1. when you press a button of web controller, 'turtle' moves according to the pressed button
