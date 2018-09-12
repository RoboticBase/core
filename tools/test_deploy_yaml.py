# -*- cofing: utf-8 -*-

import tempfile
from unittest.mock import patch
import json
import urllib.parse

import yaml

import pytest

from deploy_yaml import main


class MockArgs:
    def __init__(self, yaml_path='', endpoint='https://api.example.com/', token='dummytoken',
                 fiware_service='fiware_service', fiware_servicepath='fiware_servicepath',
                 entity_type='entity_type', entity_id='entity_id', delete=False):
        self.yaml_path = yaml_path
        self.endpoint = endpoint
        self.token = token
        self.fiware_service = fiware_service
        self.fiware_servicepath = fiware_servicepath
        self.entity_type = entity_type
        self.entity_id = entity_id
        self.delete = delete


class MockResponse:
    def __init__(self, json_data, status_code, *args, **kwargs):
        self.json_data = json_data
        self.status_code = status_code

    def json(self):
        return self.json_data


TEST_YAML = '''
apiVersion: v1
kind: Service
metadata:
  name: my-service
  labels:
    app: my-service
spec:
  type: ClusterIP
  selector:
    app: my-service
  ports:
  - name: my-service-port
    protocol: TCP
    port: 65432
    targetPort: 65432
'''

QUATED_DATA = urllib.parse.quote(json.dumps(yaml.load(TEST_YAML)))


@pytest.mark.parametrize('yaml_path', ['', 'dummy'])
def test_main_no_yaml(yaml_path):
    args = MockArgs(yaml_path=yaml_path)
    with pytest.raises(FileNotFoundError):
        main(args)


@pytest.mark.parametrize('delete', [True, False])
@patch('deploy_yaml.requests')
def test_main_apply(mocked_requests, delete):
    mocked_requests.post.return_value = MockResponse({'result': 'success'}, 200)

    args = MockArgs()
    args.delete = delete

    with tempfile.NamedTemporaryFile(mode='w+t', encoding='utf-8') as f:
        f.write(TEST_YAML)
        f.seek(0)

        args.yaml_path = f.name
        main(args)

    mocked_requests.post.assert_called_once()
    called_args = mocked_requests.post.call_args_list[0]
    assert called_args[0] == ('https://api.example.com/orion/v1/updateContext', )
    assert called_args[1]['headers'] == {
        'Content-Type': 'application/json',
        'Authorization': 'bearer dummytoken',
        'Fiware-Service': 'fiware_service',
        'Fiware-Servicepath': 'fiware_servicepath',
    }
    assert called_args[1]['json'] == {
        'contextElements': [
            {
                'id': 'entity_id',
                'isPattern': False,
                'type': 'entity_type',
                'attributes': [
                    {
                        'name': 'delete' if args.delete else 'apply',
                        'value': QUATED_DATA,
                    }
                ]
            }
        ],
        'updateAction': 'UPDATE'
    }
