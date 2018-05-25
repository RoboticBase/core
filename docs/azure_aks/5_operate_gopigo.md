# 5. operate 'gopigo' using 'gamepad' and 'web controller' (if gopigo is available)

Operate 'gopigo' using 'gamepad' and 'web controller' through fiware on AKS.

**In the following document, replace "example.com" with your domain.**

## reconfigure 'cmd-proxy' to connect 'gopigo'

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

## start 'gopigo'
1. ssh to gopigo on terminal1 and start `roscore`

    ```bash
    ros-terminal1:gopigo_ws$ source devel/setup.bash
    ros-terminal1:gopigo_ws$ roscore
    ```
1. ssh to gopigo on terminal2 and start `ros_gopigo` (ZZZZZZZZZZZZ is the password of "gopigo")
    ```bash
    ros-terminal2:gopigo_ws$ env MQTT_HOST=mqtt.example.com GOPIGO_PASSWORD=ZZZZZZZZZZZZ envsubst < src/ros_gopigo/config/params-azure.yaml.template > src/ros_gopigo/config/params.yaml
    ros-terminal2:gopigo_ws$ source devel/setup.bash
    ros-terminal2:gopigo_ws$ roslaunch ros_gopigo ros_gopigo.launch
    ```

## operate 'gopigo' using 'gamepad'
1. ssh raspberrypi and start `main.py` (YYYYYYYYYYYY is the password of "raspberrypi")

    ```bash
    raspberrypi:raspi_gamepad$ env MQTT_HOST=mqtt.example.com RASPI_RASSWORD=YYYYYYYYYYYY envsubst < conf/pxkwcr-azure.yaml.template > conf/pxkwcr-azure.yaml
    raspberrypi:raspi_gamepad$ ./main.py pxkwcr-azure
    ```
1. when you press a button of gamepad, 'gopigo' moves according to the pressed button

### operate 'gopigo' using 'web controller'
1. access to 'web controller( https://api.example.com/controller/web/ )'
1. login 'web controller' using user/passwrod which is defined by `auth-token.json`
1. when you press a button of web controller, 'gopigo' moves according to the pressed button
