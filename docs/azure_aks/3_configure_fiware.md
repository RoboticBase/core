# 3. configure fiware on AKS

Configure fiware on AKS like below:

1. register "demo1" service
1. register "gamepad" device
1. test publishing 'button' attribute from 'gamepad' to 'orion'
1. register "turtlesim" device
1. test publishing 'temperature' attribute from 'turtlesim' to orion, and subscribing 'move' command from orion to 'turtlesim'
1. register "gopigo" device (if gopigo is available)
1. test subscribing 'move' command from orion to 'gopigo' (if gopigo is available)
1. register cygnus
1. register fiware-cmd-proxy

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

* ssh raspberrypi and start `main.py` (YYYYYYYYYYYY is the password of "raspberrypi")
```bash
raspberrypi:raspi_gamepad$ env MQTT_HOST=mqtt.example.com RASPI_RASSWORD=YYYYYYYYYYYY envsubst < conf/pxkwcr-azure.yaml.template > conf/pxkwcr-azure.yaml
raspberrypi:raspi_gamepad$ ./main.py pxkwcr-azure
2018/05/06 11:46:04 [   INFO] __main__ - run script using pxkwcr.yaml
2018/05/06 11:46:04 [   INFO] src.controller - initialized FUJIWORK PXKWCR Controller
2018/05/06 11:46:04 [   INFO] src.controller - start publishing...
...
```

* press 'circle' button of gamepad
```bash
raspberrypi:raspi_gamepad$ ./main.py
...
2018/05/06 11:46:15 [   INFO] src.controller - published "2018-05-06T02:46:14.876494+0000|button|circle" to "/demo1/gamepad/attrs"
2018/05/06 11:46:15 [   INFO] src.controller - connected mqtt broker[mqtt.example.com:8883], response_code=0
...
```

```bash
mac:$ mosquitto_sub -h mqtt.example.com -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
...
Client mosqsub|60435-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/gamepad/attrs', ... (45 bytes))
2018-05-06T02:46:14.876494+0000|button|circle
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/orion/v2/entities/gamepad/ | jq .
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

* open terminal1 and start `roscore`
```bash
ros-terminal1:ros_ws$ source devel/setup.bash
ros-terminal1:ros_ws$ roscore
...
```

* open terminal2 and start `turtlesim`
```bash
ros-terminal2:ros_ws$ source devel/setup.bash
ros-terminal2:ros_ws$ rosrun turtlesim turtlesim_node
...
```

* open terminal3 and start `turtlesim_operator` (ZZZZZZZZZZZZ is the password of "gopigo")
```bash
ros-terminal3:ros_ws$ env MQTT_HOST=mqtt.example.com TURTLESIM_PASSWORD=ZZZZZZZZZZZZ envsubst < src/turtlesim_operator/config/params-azure.yaml.template > src/turtlesim_operator/config/params.yaml
ros-terminal3:ros_ws$ source devel/setup.bash
ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
... logging to /home/ubuntu/.ros/log/872b8e06-5fbf-11e8-bdde-02dff3ffcd9e/roslaunch-ubuntu-xenial-12540.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu-xenial:42084/

SUMMARY
========

PARAMETERS
...
running rosparam delete /command_sender/
running rosparam delete /attribute_receiver/
process[command_sender-1]: started with pid [12567]
process[attribute_receiver-2]: started with pid [12568]
[INFO] [1527215493.418127]: [__main__:main] Start node : command_sender_node.py [mode=production]
[INFO] [1527215493.420951]: [__main__:main] Start node : attribute_receiver_node.py
[INFO] [1527215493.426882]: [turtlesim_operator.command_sender:CommandSender.connect] Connect mqtt broker
[INFO] [1527215493.432872]: [turtlesim_operator.attribute_receiver:AttributeReceiver.connect] Connect mqtt broker
[INFO] [1527215494.574014]: [turtlesim_operator.attribute_receiver:AttributeReceiver.start] AttributeReceiver start : attribute_receiver_node.py
[INFO] [1527215494.575131]: [turtlesim_operator.command_sender:CommandSender.start] CommandSender start : command_sender_node.py
[INFO] [1527215494.878739]: [turtlesim_operator.attribute_receiver:AttributeReceiver._on_connect] mqtt connect status=0
[INFO] [1527215494.880586]: [turtlesim_operator.command_sender:CommandSender._on_connect] mqtt connect status=0
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
Client mosqsub|22161-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/cmd', ... (21 bytes))
turtlesim@move|circle
Client mosqsub|22161-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/cmdexe', ... (30 bytes))<Paste>
...
```

```bash
ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
...
[INFO] [1527219554.229942]: [turtlesim_operator.command_sender:CommandSender._on_message] received message from mqtt: turtlesim@move|circle
[INFO] [1527219554.232153]: [turtlesim_operator.command_sender:CommandSender._do_circle] do circle
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/orion/v2/entities/turtlesim/ | jq .
{
  "id": "turtlesim",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-05-25T03:39:14.00Z",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": "executed circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T03:39:14.339Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "OK",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T03:39:14.339Z"
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
ros-terminal4:ros_ws$ source devel/setup.bash
ros-terminal4:ros_ws$ rostopic pub -1 /turtle1/temperature std_msgs/Float32 -- 25.3
```

```bash
ros-terminal3:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
...
[INFO] [1527221663.972928]: [turtlesim_operator.attribute_receiver:AttributeReceiver._on_receive] received message from ros : 25.2999992371
...
```

```bash
mac:$ mosquitto_sub -h mqtt.example.com -p 8883 --cafile ./secrets/ca.crt -d -t /# -u iotagent -P XXXXXXXXXXXX
...
Client mosqsub|22161-MacBook-P received PUBLISH (d0, q0, r0, m0, '/demo1/turtlesim/attrs', ... (57 bytes))
2018-05-25T04:14:23.973467+0000|temperature|25.2999992371
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/orion/v2/entities/turtlesim/ | jq .
{
  "id": "turtlesim",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-05-25T04:14:23.973467+0000",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": " ",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T03:39:14.339Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "UNKNOWN",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T03:39:14.339Z"
      }
    }
  },
  "temperature": {
    "type": "float32",
    "value": "12.1000003815",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-25T04:30:16.726545+0000"
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

* ssh to gopigo on terminal1 and start `roscore`
```bash
ros-terminal1:gopigo_ws$ source devel/setup.bash
ros-terminal1:gopigo_ws$ roscore
...
```

* ssh to gopigo on terminal2 and start `ros_gopigo` (ZZZZZZZZZZZZ is the password of "gopigo")
```bash
ros-terminal2:gopigo_ws$ env MQTT_HOST=mqtt.example.com GOPIGO_PASSWORD=ZZZZZZZZZZZZ envsubst < src/ros_gopigo/config/params-azure.yaml.template > src/ros_gopigo/config/params.yaml
ros-terminal2:gopigo_ws$ source devel/setup.bash
ros-terminal2:gopigo_ws$ roslaunch ros_gopigo ros_gopigo.launch
... logging to /home/ubuntu/.ros/log/2ed86872-5f2f-11e8-9dff-84afec5283f0/roslaunch-ubuntu-4727.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu:38978/

SUMMARY
========

PARAMETERS
...
running rosparam delete /gopigo_node/
ERROR: parameter [/gopigo_node] is not set
running rosparam delete /fiware2gopigo_node/
ERROR: parameter [/fiware2gopigo_node] is not set
process[gopigo_node-1]: started with pid [4754]
process[fiware2gopigo_node-2]: started with pid [4755]
[INFO] [1527152610.456460]: [__main__:main] Start node : ros_gopigo
[INFO] [1527152610.477046]: [__main__:main] Start node : fiware2gopigo
[INFO] [1527152610.514471]: [ros_gopigo.gopigo_impl:Gopigo.start] Gopigo start: ros_gopigo
[INFO] [1527152610.544036]: [ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo.connect] Connect to MQTT broker
[INFO] [1527152611.204234]: [ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo.start] Fiware2Gopigo start: fiware2gopigo
[INFO] [1527152611.399812]: [ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo._on_connect] connected to MQTT Broker, status: 0
...
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
Client mosqsub|77956-Nobuyukin received PUBLISH (d0, q0, r0, m0, '/demo1/gopigo/cmd', ... (21 bytes))
gopigo@move|circle
Client mosqsub|77956-Nobuyukin received PUBLISH (d0, q0, r0, m0, '/demo1/gopigo/cmdexe', ... (28 bytes))
gopigo@move|executed circle
...
```

```bash
ros-terminal2:ros_ws$ roslaunch turtlesim_operator turtlesim_operator.launch
...
[INFO] [1527152726.487569]: [ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo._on_message] received message from mqtt: gopigo@move|circle
[INFO] [1527152726.499365]: [ros_gopigo.fiware2gopigo_impl:Fiware2Gopigo._do_circle] do circle
...
```

```bash
mac:$ TOKEN=$(cat secrets/auth-tokens.json | jq '.bearer_tokens[0].token' -r);curl -sS -H "Authorization: bearer ${TOKEN}" -H "Fiware-Service: demo1" -H "Fiware-Servicepath: /" https://api.example.com/orion/v2/entities/gopigo/ | jq .
{
  "id": "gopigo",
  "type": "demo1",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "2018-05-24T09:05:26.00Z",
    "metadata": {}
  },
  "move_info": {
    "type": "commandResult",
    "value": "executed circle",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-24T09:05:26.799Z"
      }
    }
  },
  "move_status": {
    "type": "commandStatus",
    "value": "OK",
    "metadata": {
      "TimeInstant": {
        "type": "ISO8601",
        "value": "2018-05-24T09:05:26.799Z"
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

## register cygnus

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
    "id": "5b06745991cc31d4197bd440",
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
      "lastNotification": "2018-05-24T08:14:17.00Z",
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
    "id": "5b079158a7bb4515a7bbf6cc",
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
      "lastNotification": "2018-05-25T04:30:16.00Z",
      "attrs": [
        "temperature"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      }
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
{ "_id" : ObjectId("5b06745ac7f7c0000aac9659"), "recvTime" : ISODate("2018-05-24T08:14:17.733Z"), "attrName" : "button", "attrType" : "string", "attrValue" : "circle" }
```

```bash
mac:$ kubectl exec mongodb-0 -c mongodb -- mongo sth_demo1 --eval 'db.getCollection("sth_/_turtlesim_demo1").find()'
MongoDB shell version v3.6.5
connecting to: mongodb://127.0.0.1:27017/sth_demo1
MongoDB server version: 3.6.5
{ "_id" : ObjectId("5b0791597a204b000a487367"), "recvTime" : ISODate("2018-05-25T04:30:16.598Z"), "attrName" : "temperature", "attrType" : "float32", "attrValue" : "12.1000003815" }
```

## register fiware-cmd-proxy

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
    "id": "5b06745991cc31d4197bd440",
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
      "timesSent": 2,
      "lastNotification": "2018-05-24T08:42:24.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      },
      "lastSuccess": "2018-05-24T08:42:24.00Z"
    }
  },
  {
    "id": "5b079158a7bb4515a7bbf6cc",
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
      "lastNotification": "2018-05-25T04:30:16.00Z",
      "attrs": [
        "temperature"
      ],
      "attrsFormat": "legacy",
      "http": {
        "url": "http://cygnus:5050/notify"
      }
    }
  },
  {
    "id": "5b06878991cc31d4197bd441",
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
      "lastNotification": "2018-05-24T09:36:09.00Z",
      "attrs": [
        "button"
      ],
      "attrsFormat": "normalized",
      "http": {
        "url": "http://cmd-proxy:8888/gamepad/"
      },
      "lastSuccess": "2018-05-24T09:36:09.00Z"
    }
  }
]
```
