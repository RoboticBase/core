#!/usr/bin/env python
import json
import jmespath

path = '../secrets/auth-tokens.json'
q = '[?contains(host, `kibana`)].settings' \
    '.basic_auths[0].{username: username, password: password}'

try:
    with open(path) as f:
        auth_tokens = json.load(f)
        credentials = jmespath.search(q, auth_tokens)
        print(credentials)
except FileNotFoundError:
    print('auth-tokens.json does not exist')
    exit(1)
