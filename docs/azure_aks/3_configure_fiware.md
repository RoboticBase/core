# 3. configure fiware on AKS

Configure fiware on AKS by following steps:

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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.example.com/idas/ul20/manage/iot/services/ -X POST -d @- <<__EOS__
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /*" https://api.example.com/idas/ul20/manage/iot/services/ | jq .
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.example.com/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/idas/ul20/manage/iot/devices/gamepad/ | jq .
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/orion/v2/entities/gamepad/ | jq .
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
mac:$ mosquitto_sub -h mqtt.example.com -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
```

* prepare `fiware-gamepad-controller` on Raspberry Pi
```bash
raspberrypi:$ git clone https://github.com/tech-sketch/fiware-gamepad-controller.git
raspberrypi:$ cd fiware-gamepad-controller
raspoberypi:$ pip install -r requirements/common.txt
```

* scp `secrets/ca.crt` to raspberrypi
```bash
mac:$ scp ./secrets/ca.crt pi@raspberrypi.local:~/fiware-gamepad-controller/secrets/ca.crt
```

* ssh raspberrypi and start `main.py` (YYYYYYYYYYYY is the password of "raspberrypi")
```bash
raspberrypi:$ env MQTT_HOST="mqtt.example.com" RASPI_RASSWORD="YYYYYYYYYYYY" envsubst < conf/pxkwcr-azure.yaml.template > conf/pxkwcr-azure.yaml
raspberrypi:$ ./main.py pxkwcr-azure
2018/06/06 10:46:07 [   INFO] __main__ - run script using pxkwcr-azure.yaml
2018/06/06 10:46:07 [   INFO] src.controller - initialized FUJIWORK PXKWCR Controller
2018/06/06 10:46:07 [   INFO] src.controller - start publishing...
...
```

* press 'circle' button of gamepad
```bash
raspberrypi:raspi_gamepad$ ./main.py pxkwcr-azure
...
2018/06/06 10:49:00 [   INFO] src.controller - published "2018-06-06T01:49:00.649952+0000|button|circle" to "/demo1/gamepad/attrs"
2018/06/06 10:49:00 [   INFO] src.controller - connected mqtt broker[mqtt.cloudconductor.jp:8883], response_code=0
...
```

```bash
mac:$ mosquitto_sub -h mqtt.example.com -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
...
Client mosqsub|11667-Nobuyukin received PUBLISH (d0, q0, r0, m0, '/demo1/gamepad/attrs', ... (45 bytes))
2018-06-06T01:49:00.649952+0000|button|circle
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/orion/v2/entities/gamepad/ | jq .
{
  "id": "gamepad",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-06-06T01:49:00.649952+0000",
    "metadata": {}
  },
  "button": {
    "type": "string",
    "value": "circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-06-06T01:49:00.649952+0000"
      }
    }
  }
}
```

## register "turtlesim" device

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.example.com/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/idas/ul20/manage/iot/devices/turtlesim/ | jq .
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/orion/v2/entities/turtlesim/ | jq .
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
mac:$ mosquitto_sub -h mqtt.example.com -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
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

* scp `secrets/ca.crt` to ROS test server
```bash
mac:$ scp ./secrets/ca.crt ubuntu@turtlesim.local:~/ros_ws/src/fiware-ros-turtlesim/secrets/ca.crt
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

* open terminal3 and start `turtlesim_operator` (ZZZZZZZZZZZZ is the password of "turtlesim")
```bash
ros-terminal3:$ cd ~/ros_ws
ros-terminal3:$ catkin_make
ros-terminal3:$ source devel/setup.bash
ros-terminal3:$ env MQTT_HOST="mqtt.example.com" TURTLESIM_PASSWORD="ZZZZZZZZZZZZ" envsubst < src/fiware-ros-turtlesim/config/params-azure.yaml.template > src/fiware-ros-turtlesim/config/params.yaml
ros-terminal3:$ roslaunch fiware-ros-turtlesim fiware-ros-turtlesim.launch
... logging to /home/ubuntu/.ros/log/2e93ca42-692e-11e8-b04a-02dff3ffcd9e/roslaunch-ubuntu-xenial-12451.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu-xenial:38200/

SUMMARY
========

PARAMETERS
...
running rosparam delete /command_sender/
running rosparam delete /attribute_receiver/
process[command_sender-1]: started with pid [12478]
process[attribute_receiver-2]: started with pid [12479]
[INFO] [1528251177.864751]: [__main__:main] Start node : attribute_receiver_node.py
[INFO] [1528251177.866113]: [__main__:main] Start node : command_sender_node.py [mode=production]
[INFO] [1528251177.873212]: [fiware_ros_turtlesim.command_sender:CommandSender.connect] Connect mqtt broker
[INFO] [1528251177.873927]: [fiware_ros_turtlesim.attribute_receiver:AttributeReceiver.connect] Connect mqtt broker
[INFO] [1528251183.488578]: [fiware_ros_turtlesim.attribute_receiver:AttributeReceiver.start] AttributeReceiver start : attribute_receiver_node.py
[INFO] [1528251183.489523]: [fiware_ros_turtlesim.command_sender:CommandSender.start] CommandSender start : command_sender_node.py
[INFO] [1528251184.000669]: [fiware_ros_turtlesim.command_sender:CommandSender._on_connect] mqtt connect status=0
[INFO] [1528251184.001303]: [fiware_ros_turtlesim.attribute_receiver:AttributeReceiver._on_connect] mqtt connect status=0
...
```

* send 'circle' cmd to 'turtlesim' entity
```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" -H "Content-Type: application/json" https://api.example.com/orion/v1/updateContext -d @-<<__EOS__ | jq .
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
mac:$ mosquitto_sub -h mqtt.example.com -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
Client mosqsub|12427-Nobuyukin received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/cmd', ... (21 bytes))
turtlesim@move|circle
Client mosqsub|12427-Nobuyukin received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/cmdexe', ... (30 bytes))
turtlesim@move|executed circle
...
```

```bash
ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
...
[INFO] [1528251288.776914]: [fiware_ros_turtlesim.command_sender:CommandSender._on_message] received message from mqtt: turtlesim@move|circle
[INFO] [1528251288.778754]: [fiware_ros_turtlesim.command_sender:CommandSender._do_circle] do circle
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/orion/v2/entities/turtlesim/ | jq .
{
  "id": "turtlesim",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-06-06T02:14:48.00Z",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": "executed circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-06-06T02:14:48.902Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "OK",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-06-06T02:14:48.902Z"
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
[INFO] [1528251469.593141]: [fiware_ros_turtlesim.attribute_receiver:AttributeReceiver._on_receive] received message from ros : 25.2999992371
...
```

```bash
mac:$ mosquitto_sub -h mqtt.example.com -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
Client mosqsub|12427-Nobuyukin received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/attrs', ... (57 bytes))
2018-06-06T02:17:49.593979+0000|temperature|25.2999992371
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/orion/v2/entities/turtlesim/ | jq .
{
  "id": "turtlesim",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-06-06T02:17:49.593979+0000",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": "executed circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-06-06T02:14:48.902Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "OK",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-06-06T02:14:48.902Z"
      }
    }
  },
  "temperature": {
    "type": "float32",
    "value": "25.2999992371",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-06-06T02:17:49.593979+0000"
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.example.com/idas/ul20/manage/iot/devices/ -X POST -d @- <<__EOS__
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/idas/ul20/manage/iot/devices/gopigo/ | jq .
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/orion/v2/entities/gopigo/ | jq .
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
mac:$ mosquitto_sub -h mqtt.example.com -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
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

* scp `secrets/ca.crt` to gopigo
```bash
mac:$ scp ./secrets/ca.crt ubuntu@turtlesim.local:~/gopigo_ws/src/fiware-ros-gopigo/secrets/ca.crt
```

* ssh to gopigo on terminal1 and start `roscore`
```bash
ros-terminal1:$ cd ~/gopigo_ws
ros-terminal1:$ source devel/setup.bash
ros-terminal1:$ roscore
...
```

* ssh to gopigo on terminal2 and start `ros_gopigo` (ZZZZZZZZZZZZ is the password of "gopigo")
```bash
ros-terminal2:$ cd ~/gopigo_ws
ros-terminal2:$ catkin_make
ros-terminal2:$ source devel/setup.bash
ros-terminal2:$ env MQTT_HOST="mqtt.example.com" GOPIGO_PASSWORD="ZZZZZZZZZZZZ" envsubst < src/fiware-ros-gopigo/config/params-azure.yaml.template > src/fiware-ros-gopigo/config/params.yaml
ros-terminal2:$ roslaunch fiware-ros-gopigo fiware-ros-gopigo.launch
... logging to /home/ubuntu/.ros/log/2969f28a-6933-11e8-9a92-84afec5283f0/roslaunch-ubuntu-5805.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu:43469/

SUMMARY
========

PARAMETERS
...
running rosparam delete /gopigo_node/
ERROR: parameter [/gopigo_node] is not set
running rosparam delete /fiware2gopigo_node/
ERROR: parameter [/fiware2gopigo_node] is not set
process[gopigo_node-1]: started with pid [5832]
process[fiware2gopigo_node-2]: started with pid [5833]
[INFO] [1528253006.994686]: [__main__:main] Start node : ros_gopigo
[INFO] [1528253007.023473]: [__main__:main] Start node : fiware2gopigo
[INFO] [1528253007.067154]: [fiware_ros_gopigo.gopigo_impl:Gopigo.start] Gopigo start: ros_gopigo
[INFO] [1528253007.089022]: [fiware_ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo.connect] Connect to MQTT broker
[INFO] [1528253008.328665]: [fiware_ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo.start] Fiware2Gopigo start: fiware2gopigo
[INFO] [1528253008.518929]: [fiware_ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo._on_connect] connected to MQTT Broker, status: 0
```

* send 'circle' cmd to 'gopigo' entity
```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" -H "Content-Type: application/json" https://api.example.com/orion/v1/updateContext -d @-<<__EOS__ | jq .
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
mac:$ mosquitto_sub -h mqtt.example.com -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
Client mosqsub|12427-Nobuyukin received PUBLISH (d0, q0, r0, m0, '/demo1/gopigo/cmd', ... (18 bytes))
gopigo@move|circle
Client mosqsub|12427-Nobuyukin received PUBLISH (d0, q0, r0, m0, '/demo1/gopigo/cmdexe', ... (27 bytes))
gopigo@move|executed circle
...
```

```bash
ros-terminal2:$ roslaunch fiware_ros_gopigo fiware_ros_gopigo.launch
...
[INFO] [1528253171.803069]: [fiware_ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo._on_message] received message from mqtt: gopigo@move|circle
[INFO] [1528253171.815056]: [fiware_ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo._do_circle] do circle
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/orion/v2/entities/gopigo/ | jq .
{
  "id": "gopigo",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-06-06T02:46:12.00Z",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": "executed circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-06-06T02:46:12.020Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "OK",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-06-06T02:46:12.020Z"
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.example.com/orion/v2/subscriptions/ -X POST -d @- <<__EOS__
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.example.com/orion/v2/subscriptions/ -X POST -d @- <<__EOS__
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" https://api.example.com/orion/v2/subscriptions/ | jq .
[
  {
    "id": "5b174d6e7526578caaebd4ab",
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
      "lastNotification": "2018-06-06T02:56:46.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      }
    }
  },
  {
    "id": "5b174d80d142dab9c1a8db92",
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
      "lastNotification": "2018-06-06T02:57:04.00Z",
      "attrs": [
        "temperature"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      },
      "lastSuccess": "2018-06-06T02:57:04.00Z"
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
{ "_id" : ObjectId("5b174d6efc89b5000ad54284"), "recvTime" : ISODate("2018-06-06T02:56:46.156Z"), "attrName" : "button", "attrType" : "string", "attrValue" : "circle" }
```

```bash
mac:$ kubectl exec mongodb-0 -c mongodb -- mongo sth_demo1 --eval 'db.getCollection("sth_/_turtlesim_demo1").find()'
MongoDB shell version v3.6.5
connecting to: mongodb://127.0.0.1:27017/sth_demo1
MongoDB server version: 3.6.5
{ "_id" : ObjectId("5b174d815ff6fa000a96b6dc"), "recvTime" : ISODate("2018-06-06T02:57:04.721Z"), "attrName" : "temperature", "attrType" : "float32", "attrValue" : "25.2999992371" }
```

## register cmd-proxy as subscriber

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" -H "Content-Type: application/json" https://api.example.com/orion/v2/subscriptions/ -X POST -d @- <<__EOS__
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
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-ServicePath: /" https://api.example.com/orion/v2/subscriptions/ | jq .
[
  {
    "id": "5b174d6e7526578caaebd4ab",
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
      "lastNotification": "2018-06-06T02:56:46.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      },
      "lastSuccess": "2018-06-06T02:56:46.00Z"
    }
  },
  {
    "id": "5b174d80d142dab9c1a8db92",
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
      "lastNotification": "2018-06-06T02:57:04.00Z",
      "attrs": [
        "temperature"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      },
      "lastSuccess": "2018-06-06T02:57:04.00Z"
    }
  },
  {
    "id": "5b174ebcd142dab9c1a8db93",
    "status": "failed",
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
      "lastNotification": "2018-06-06T03:02:20.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "normalized",
      "http": {
        "url": "http://cmd-proxy:8888/gamepad/"
      },
      "lastFailure": "2018-06-06T03:02:20.00Z"
    }
  }
]
```
