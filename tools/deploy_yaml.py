#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import base64
import json
import os
import urllib.parse

import yaml

import requests


def parse_args():
    parser = argparse.ArgumentParser(description='Deploy component to kubernetes through mqtt-kube-operator')
    parser.add_argument('yaml_path', action='store', type=str, help='the path of yaml file you want to deploy')
    parser.add_argument('endpoint', action='store', type=str, help='the endpoint such as "https://api.example.com/"')
    parser.add_argument('token', action='store', type=str, help='the Bearer token to post endpoint')
    parser.add_argument('fiware_service', action='store', type=str, help='the value of "Fiware-Service" header')
    parser.add_argument('fiware_servicepath', action='store', type=str, help='the value of "Fiware-Servicepath" header')
    parser.add_argument('entity_type', action='store', type=str, help='orion entity type of mqtt-kube-operator"')
    parser.add_argument('entity_id', action='store', type=str, help='orion entity id of mqtt-kube-operator"')
    parser.add_argument('--delete', action='store_true', default=False, help='delete object if already deployed')
    return parser.parse_args()


def main(args):
    if not os.path.isfile(args.yaml_path):
        print(f'{args.yaml_path} does not exist')
        exit(1)

    command = 'delete' if args.delete else 'apply'
    print(f'{command} {args.yaml_path} to {args.endpoint}')
    with open(args.yaml_path) as f:
        data = json.dumps(yaml.load(f))


    url = urllib.parse.urljoin(args.endpoint, '/orion/v1/updateContext')
    headers = {
        'Content-Type': 'application/json',
        'Authorization': f'bearer {args.token}',
        'Fiware-Service': args.fiware_service,
        'Fiware-Servicepath': args.fiware_servicepath,
    }

    payload = {
        'contextElements': [
            {
                'id': args.entity_id,
                'isPattern': False,
                'type': args.entity_type,
                'attributes': [
                    {
                        'name': command,
                        'value': urllib.parse.quote(data),
                    }
                ]
            }
        ],
        'updateAction': 'UPDATE'
    }
    response = requests.post(url, json=payload, headers=headers)
    print(f'{json.dumps(response.json(), indent=4)}\n')


if __name__ == '__main__':
    main(parse_args())
