import string
import random

from ansible.module_utils.basic import AnsibleModule


def get_random_str(n):
    return ''.join(random.choices(string.ascii_letters + string.digits, k=n))


def main():
    module = AnsibleModule(
        argument_spec=dict(
            length=dict(type='int', required=True),
        ),
        supports_check_mode=True
    )
    if module.check_mode:
        module.exit_json(changed=False)

    randomstr = get_random_str(module.params['length'])
    module.exit_json(changed=False, randomstr=randomstr)


if __name__ == '__main__':
    main()
