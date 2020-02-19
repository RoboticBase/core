import yaml

from ansible.module_utils.basic import AnsibleModule


def replace_metricsBindAddress(path):
    with open(path, mode='r+') as f:
        config = yaml.load(f, Loader=yaml.SafeLoader)
        config['metricsBindAddress'] = '0.0.0.0'
        f.write(yaml.dump(config, default_flow_style=False))

def main():
    module = AnsibleModule(
        argument_spec=dict(
            path=dict(type='str', required=True),
        ),
        supports_check_mode=True
    )
    if module.check_mode:
        module.exit_json(changed=False)

    replace_metricsBindAddress(module.params['path'])
    module.exit_json(changed=True, path=module.params['path'])


if __name__ == '__main__':
    main()

