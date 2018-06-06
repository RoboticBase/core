# 3. configure fiware on minikube

Configure fiware on minikube by following steps:

1. [register "demo1" service](#register-demo1-service)
1. [register "gamepad" device](#register-gamepad-device)
1. [test publishing 'button' attribute from 'gamepad' to 'orion'](#test-publishing-button-attribute-from-gamepad-to-orion)
1. [register "turtlesim" device](#register-turtlesim-device)
1. [test publishing 'temperature' attribute from 'turtlesim' to orion, and subscribing 'move' command from orion to 'turtlesim'](#test-publishing-temperature-attribute-from-turtlesim-to-orion-and-subscribing-move-command-from-orion-to-turtlesim)
1. [register "gopigo" device (if gopigo is available)](#register-gopigo-device-if-gopigo-is-available)
1. [test subscribing 'move' command from orion to 'gopigo' (if gopigo is available)](#test-subscribing-move-command-from-orion-to-gopigo-if-gopigo-is-available)
1. [register fiware cygnus as subscriber](#register-fiware-cygnus-as-subscriber)
1. [register cmd-proxy as subscriber](#register-cmd-proxy-as-subscriber)

**In the following document, replace "example.com" with your domain.**

## register "demo1" service

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" http://${HOST_IPADDR}:8080/idas/ul20/manage/iot/services/ -X POST -d @- <<__EOS__
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" http://${HOST_IPADDR}:8080/idas/ul20/manage/iot/services/ | jq .
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" http://${HOST_IPADDR}:8080/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://${HOST_IPADDR}:8080/idas/ul20/manage/iot/devices/gamepad/ | jq .
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://${HOST_IPADDR}:8080/orion/v2/entities/gamepad/ | jq .
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

## test publishing 'button' attribute from 'gamepad' to 'orion'

* XXXXXXXXXXXX is the password of "iotagent"
```text
mac:$ mosquitto_sub -h ${HOST_IPADDR} -p 1883 -d -t /# -u iotagent -P XXXXXXXXXXXX
...
```

* check HOSTNAME of host pc
```bash
mac:$ hostname
MacBook-Pro.local
```

* prepare `fiware-gamepad-controller` on Raspberry Pi
```bash
raspberrypi:$ git clone https://github.com/tech-sketch/fiware-gamepad-controller.git
raspberrypi:$ cd fiware-gamepad-controller
raspoberypi:$ pip install -r requirements/common.txt
```

* ssh raspberrypi and start `main.py` (YYYYYYYYYYYY is the password of "raspberrypi", and "HHHHHHHHHH" is the `hostname` of host pc)
```bash
raspberrypi:$ env MQTT_HOST="HHHHHHHHHH" RASPI_RASSWORD="YYYYYYYYYYYY" envsubst < conf/pxkwcr-minikube.yaml.template > conf/pxkwcr-minikube.yaml
raspberrypi:$ ./main.py pxkwcr-minikube
2018/05/25 19:25:33 [   INFO] __main__ - run script using pxkwcr-minikube.yaml
2018/05/25 19:25:34 [   INFO] src.controller - initialized FUJIWORK PXKWCR Controller
2018/05/25 19:25:34 [   INFO] src.controller - start publishing...
...
```

* press 'circle' button of gamepad
```bash
raspberrypi:raspi_gamepad$ ./main.py pxkwcr-minikube
...
2018/05/25 19:33:33 [   INFO] src.controller - published "2018-05-25T10:33:33.708776+0000|button|circle" to "/demo1/gamepad/attrs"
2018/05/25 19:33:33 [   INFO] src.controller - connected mqtt broker[MacBook-Pro.local:1883], response_code=0
...
```

```bash
mac:$ mosquitto_sub -h ${HOST_IPADDR} -p 1883 -d -t /# -u iotagent -P XXXXXXXXXXXX
...
...
Client mosqsub|86430-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/gamepad/attrs', ... (45 bytes))
2018-05-25T10:33:33.708776+0000|button|circle
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://${HOST_IPADDR}:8080/orion/v2/entities/gamepad/ | jq .
{
  "id": "gamepad",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-05-25T10:33:33.708776+0000",
    "metadata": {}
  },
  "button": {
    "type": "string",
    "value": "circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T10:33:33.708776+0000"
      }
    }
  }
}
```

## register "turtlesim" device

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" http://${HOST_IPADDR}:8080/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
{
  "devices": [
    {
      "device_id": "turtlesim",
      "entity_name": "turtlesim",
      "entity_type": "demo1",
      "timezone": "Asia/Tokyo",
      "protocol": "UL20",
      "attributes": [
        {
          "name": "temperature",
          "type": "float32"
        }
      ],
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://${HOST_IPADDR}:8080/idas/ul20/manage/iot/devices/turtlesim/ | jq .
{
  "device_id": "turtlesim",
  "service": "demo1",
  "service_path": "/",
  "entity_name": "turtlesim",
  "entity_type": "demo1",
  "transport": "MQTT",
  "attributes": [
    {
      "object_id": "temperature",
      "name": "temperature",
      "type": "float32"
    }
  ],
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://${HOST_IPADDR}:8080/orion/v2/entities/turtlesim/ | jq .
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
  "temperature": {
    "type": "float32",
    "value": " ",
    "metadata": {}
  },
  "move": {
    "type": "string",
    "value": "",
    "metadata": {}
  }
}
```

## test publishing 'temperature' attribute from 'turtlesim' to orion, and subscribing 'move' command from orion to 'turtlesim'

* XXXXXXXXXXXX is the password of "iotagent"
```bash
mac:$ mosquitto_sub -h ${HOST_IPADDR} -p 1883 -d -t /# -u iotagent -P XXXXXXXXXXXX
...
```

* start X on ros server and login X using RDP

* prepare `fiware-ros-turtlesim` on ROS test server
```bash
ros-terminal1:$ cd ~/ros_ws/src
ros-terminal1:$ git clone https://github.com/tech-sketch/fiware-ros-turtlesim.git
ros-terminal1:$ cd fiware-ros-turtlesim
ros-terminal1:$ pip install -r requirements/common.txt
```

* open terminal1 and start `roscore`
```bash
ros-terminal1:$ cd ~/ros_ws
ros-terminal1:$ source devel/setup.bash
ros-terminal1:$ roscore
...
```

* open terminal2 and start `turtlesim`
```bash
ros-terminal2:$ cd ~/ros_ws
ros-terminal2:$ source devel/setup.bash
ros-terminal2:$ rosrun turtlesim turtlesim_node
...
```

* check HOSTNAME of host pc
```bash
mac:$ hostname
MacBook-Pro.local
```

* open terminal3 and start `turtlesim_operator` (ZZZZZZZZZZZZ is the password of "turtlesim", and "HHHHHHHHHH" is the `hostname` of host pc)
```bash
ros-terminal3:$ cd ~/ros_ws
ros-terminal3:$ catkin_make
ros-terminal3:$ source devel/setup.bash
ros-terminal3:$ env MQTT_HOST="HHHHHHHHHH" TURTLESIM_PASSWORD="ZZZZZZZZZZZZ" envsubst < src/fiware-ros-turtlesim/config/params-minikube.yaml.template > src/fiware-ros-turtlesim/config/params.yaml
ros-terminal3:$ roslaunch fiware-ros-turtlesim fiware-ros-turtlesim.launch
... logging to /home/ubuntu/.ros/log/1bbff6ea-6008-11e8-bdde-02dff3ffcd9e/roslaunch-ubuntu-xenial-28173.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu-xenial:35724/

SUMMARY
========

PARAMETERS
...
running rosparam delete /command_sender/
ERROR: parameter [/command_sender] is not set
running rosparam delete /attribute_receiver/
ERROR: parameter [/attribute_receiver] is not set
process[command_sender-1]: started with pid [28200]
process[attribute_receiver-2]: started with pid [28201]
[INFO] [1527244874.614400]: [__main__:main] Start node : command_sender_node.py [mode=production]
[INFO] [1527244874.621494]: [__main__:main] Start node : attribute_receiver_node.py
[INFO] [1527244874.625126]: [fiware_ros_turtlesim.command_sender:CommandSender.connect] Connect mqtt broker
[INFO] [1527244874.632231]: [fiware_ros_turtlesim.attribute_receiver:AttributeReceiver.connect] Connect mqtt broker
[INFO] [1527244874.730438]: [fiware_ros_turtlesim.attribute_receiver:AttributeReceiver.start] AttributeReceiver start : attribute_receiver_node.py
[INFO] [1527244874.731459]: [fiware_ros_turtlesim.command_sender:CommandSender.start] CommandSender start : command_sender_node.py
[INFO] [1527244874.737303]: [fiware_ros_turtlesim.attribute_receiver:AttributeReceiver._on_connect] mqtt connect status=0
[INFO] [1527244874.742508]: [fiware_ros_turtlesim.command_sender:CommandSender._on_connect] mqtt connect status=0
...
```

* send 'circle' cmd to 'turtlesim' entity
```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" -H "Content-Type: application/json" http://${HOST_IPADDR}:8080/orion/v1/updateContext -d @-<<__EOS__ | jq .
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
mac:$ mosquitto_sub -h ${HOST_IPADDR} -p 1883 -d -t /# -u iotagent -P XXXXXXXXXXXX
...
Client mosqsub|86430-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/cmd', ... (21 bytes))
turtlesim@move|circle
Client mosqsub|86430-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/cmdexe', ... (30 bytes))
turtlesim@move|executed circle
...
```

```bash
ros-terminal3:$ roslaunch turtlesim_operator turtlesim_operator.launch
...
[INFO] [1527244943.572875]: [fiware_ros_turtlesim.command_sender:CommandSender._on_message] received message from mqtt: turtlesim@move|circle
[INFO] [1527244943.574072]: [fiware_ros_turtlesim.command_sender:CommandSender._do_circle] do circle
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://${HOST_IPADDR}:8080/orion/v2/entities/turtlesim/ | jq .
{
  "id": "turtlesim",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-05-25T10:42:23.00Z",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": "executed circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T10:42:23.668Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "OK",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T10:42:23.668Z"
      }
    }
  },
  "temperature": {
    "type": "float32",
    "value": " ",
    "metadata": {}
  },
  "move": {
    "type": "string",
    "value": "",
    "metadata": {}
  }
}
```

* open terminal4 and publish `templerature` to rostopic
```bash
ros-terminal4:$ cd ~/ros_ws
ros-terminal4:$ source devel/setup.bash
ros-terminal4:$ rostopic pub -1 /turtle1/temperature std_msgs/Float32 -- 25.3
```

```bash
ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
...
[INFO] [1527245105.261060]: [fiware_ros_turtlesim.attribute_receiver:AttributeReceiver._on_receive] received message from ros : 25.2999992371
...
```

```bash
mac:$ mosquitto_sub -h ${HOST_IPADDR} -p 1883 -d -t /# -u iotagent -P XXXXXXXXXXXX
...
Client mosqsub|86430-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/attrs', ... (57 bytes))
2018-05-25T10:45:05.262068+0000|temperature|25.2999992371
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://${HOST_IPADDR}:8080/orion/v2/entities/turtlesim/ | jq .
{
  "id": "turtlesim",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-05-25T10:45:05.262068+0000",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": "executed circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T10:42:23.668Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "OK",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T10:42:23.668Z"
      }
    }
  },
  "temperature": {
    "type": "float32",
    "value": "25.2999992371",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T10:45:05.262068+0000"
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

## register "gopigo" device (if gopigo is available)

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" http://${HOST_IPADDR}:8080/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
{
  "devices": [
    {
      "device_id": "gopigo",
      "entity_name": "gopigo",
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://${HOST_IPADDR}:8080/idas/ul20/manage/iot/devices/gopigo/ | jq .
{
  "device_id": "gopigo",
  "service": "demo1",
  "service_path": "/",
  "entity_name": "gopigo",
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://${HOST_IPADDR}:8080/orion/v2/entities/gopigo/ | jq .
{
  "id": "gopigo",
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

## test subscribing 'move' command from orion to 'gopigo' (if gopigo is available)

* XXXXXXXXXXXX is the password of "iotagent"
```bash
mac:$ mosquitto_sub -h ${HOST_IPADDR} -p 1883 -d -t /# -u iotagent -P XXXXXXXXXXXX
...
```

* prepare `fiware-ros-gopigo` on gopigo
```bash
ros-terminal1:$ cd ~/gopigo_ws/src
ros-terminal1:$ git clone https://github.com/tech-sketch/fiware-ros-gopigo.git
ros-terminal1:$ cd fiware-ros-gopigo
ros-terminal1:$ /bin/bash update_tools_for_ubuntu.sh
ros-terminal1:$ pip install -r requirements/common.txt
ros-terminal1:$ pip install -r requirements/gopigo.txt
```

* ssh to gopigo on terminal1 and start `roscore`
```bash
ros-terminal1:$ cd ~/gopigo_ws
ros-terminal1:$ source devel/setup.bash
ros-terminal1:$ roscore
...
```

* ssh to gopigo on terminal2 and start `ros_gopigo` (ZZZZZZZZZZZZ is the password of "gopigo", and "HHHHHHHHHH" is the `hostname` of host pc)
```bash
ros-terminal2:$ cd ~/gopigo_ws
ros-terminal2:$ catkin_make
ros-terminal2:$ source devel/setup.bash
ros-terminal2:$ env MQTT_HOST="HHHHHHHHHH GOPIGO_PASSWORD=ZZZZZZZZZZZZ envsubst < src/fiware-ros-gopigo/config/params-minikube.yaml.template > src/fiware-ros-gopigo/config/params.yaml
ros-terminal2:$ roslaunch fiware-ros-gopigo fiware-ros-gopigo.launch
... logging to /home/ubuntu/.ros/log/999a53e6-600b-11e8-81c1-84afec5283f0/roslaunch-ubuntu-1881.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu:36544/

SUMMARY
========

PARAMETERS
...
running rosparam delete /gopigo_node/
ERROR: parameter [/gopigo_node] is not set
running rosparam delete /fiware2gopigo_node/
ERROR: parameter [/fiware2gopigo_node] is not set
process[gopigo_node-1]: started with pid [1908]
process[fiware2gopigo_node-2]: started with pid [1909]
INFO: cannot create a symlink to latest log directory: [Errno 2] No such file or directory: '/home/ubuntu/.ros/log/latest'
[INFO] [1527246373.245011]: [__main__:main] Start node : fiware2gopigo
[INFO] [1527246373.245010]: [__main__:main] Start node : ros_gopigo
[INFO] [1527246373.300212]: [fiware_ros_gopigo.gopigo_impl:Gopigo.start] Gopigo start: ros_gopigo
[INFO] [1527246373.313067]: [fiware_ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo.connect] Connect to MQTT broker
[INFO] [1527246373.431902]: [fiware_ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo.start] Fiware2Gopigo start: fiware2gopigo
[INFO] [1527246373.457469]: [fiware_ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo._on_connect] connected to MQTT Broker, status: 0
...
```

* send 'circle' cmd to 'gopigo' entity
```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" -H "Content-Type: application/json" http://${HOST_IPADDR}:8080/orion/v1/updateContext -d @-<<__EOS__ | jq .
{
  "contextElements": [
    {
      "id": "gopigo",
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
mac:$ mosquitto_sub -h ${HOST_IPADDR} -p 1883 -d -t /# -u iotagent -P XXXXXXXXXXXX
...
Client mosqsub|86430-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/gopigo/cmd', ... (18 bytes))
gopigo@move|circle
Client mosqsub|86430-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/gopigo/cmdexe', ... (27 bytes))
gopigo@move|executed circle
...
```

```bash
ros-terminal2:$ roslaunch fiware_ros_gopigo fiware_ros_gopigo.launch
...
[INFO] [1527246429.544076]: [ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo._on_message] received message from mqtt: gopigo@move|circle
[INFO] [1527246429.556360]: [ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo._do_circle] do circle
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" http://${HOST_IPADDR}:8080/orion/v2/entities/gopigo/ | jq .
{
  "id": "gopigo",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-05-25T11:07:09.00Z",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": "executed circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T11:07:09.650Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "OK",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T11:07:09.650Z"
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

## register fiware cygnus as subscriber

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" http://${HOST_IPADDR}:8080/orion/v2/subscriptions/ -X POST -d @- <<__EOS__
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" http://${HOST_IPADDR}:8080/orion/v2/subscriptions/ -X POST -d @- <<__EOS__
{
  "subject": {
    "entities": [{
      "idPattern": "turtlesim.*",
      "type": "demo1"
    }]
  },
  "notification": {
    "http": {
      "url": "http://cygnus:5050/notify"
    },
    "attrs": ["temperature"],
    "attrsFormat": "legacy"
  }
}
__EOS__
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" http://${HOST_IPADDR}:8080/orion/v2/subscriptions/ | jq .
[
  {
    "id": "5b07ef2c3c78149942463e5b",
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
      "lastNotification": "2018-05-25T11:10:36.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      },
      "lastSuccess": "2018-05-25T11:10:36.00Z"
    }
  },
  {
    "id": "5b07ef348416d0af73568697",
    "status": "active",
    "subject": {
      "entities": [
        {
          "idPattern": "turtlesim.*",
          "type": "demo1"
        }
      ],
      "condition": {
        "attrs": []
      }
    },
    "notification": {
      "timesSent": 1,
      "lastNotification": "2018-05-25T11:10:44.00Z",
      "attrs": [
        "temperature"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      },
      "lastSuccess": "2018-05-25T11:10:44.00Z"
    }
  }
]
```

```bash
mac:$ kubectl exec mongodb-0 -c mongodb -- mongo --eval 'printjson(db.getMongo().getDBNames())'
MongoDB shell version v3.6.5
connecting to: mongodb://127.0.0.1:27017
MongoDB server version: 3.6.5
[
	"admin",
	"config",
	"iotagentul",
	"local",
	"orion",
	"orion-demo1",
	"sth_demo1"
]
```

```bash
mac:$ kubectl exec mongodb-0 -c mongodb -- mongo sth_demo1 --eval 'printjson(db.getCollectionNames())'
MongoDB shell version v3.6.5
connecting to: mongodb://127.0.0.1:27017/sth_demo1
MongoDB server version: 3.6.5
[ "sth_/_gamepad_demo1", "sth_/_turtlesim_demo1" ]
```

```bash
mac:$ kubectl exec mongodb-0 -c mongodb -- mongo sth_demo1 --eval 'db.getCollection("sth_/_gamepad_demo1").find()'
MongoDB shell version v3.6.5
connecting to: mongodb://127.0.0.1:27017/sth_demo1
MongoDB server version: 3.6.5
{ "_id" : ObjectId("5b07ef2c24aa9a000ad3d21c"), "recvTime" : ISODate("2018-05-25T11:10:36.290Z"), "attrName" : "button", "attrType" : "string", "attrValue" : "circle" }
```

```bash
mac:$ kubectl exec mongodb-0 -c mongodb -- mongo sth_demo1 --eval 'db.getCollection("sth_/_turtlesim_demo1").find()'
MongoDB shell version v3.6.5
connecting to: mongodb://127.0.0.1:27017/sth_demo1
MongoDB server version: 3.6.5
{ "_id" : ObjectId("5b07ef3524aa9a000ad3d21d"), "recvTime" : ISODate("2018-05-25T11:10:44.589Z"), "attrName" : "temperature", "attrType" : "float32", "attrValue" : "25.2999992371" }
```

## register cmd-proxy as subscriber

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" http://${HOST_IPADDR}:8080/orion/v2/subscriptions/ -X POST -d @- <<__EOS__
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" http://${HOST_IPADDR}:8080/orion/v2/subscriptions/ | jq .
[
  {
    "id": "5b07ef2c3c78149942463e5b",
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
      "lastNotification": "2018-05-25T11:10:36.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      },
      "lastSuccess": "2018-05-25T11:10:36.00Z"
    }
  },
  {
    "id": "5b07ef348416d0af73568697",
    "status": "active",
    "subject": {
      "entities": [
        {
          "idPattern": "turtlesim.*",
          "type": "demo1"
        }
      ],
      "condition": {
        "attrs": []
      }
    },
    "notification": {
      "timesSent": 3,
      "lastNotification": "2018-05-25T11:13:32.00Z",
      "attrs": [
        "temperature"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      },
      "lastSuccess": "2018-05-25T11:13:32.00Z"
    }
  },
  {
    "id": "5b07efdc8416d0af73568698",
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
      "lastNotification": "2018-05-25T11:13:32.00Z",
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
