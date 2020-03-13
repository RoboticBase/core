import ipaddress

from ansible.module_utils.basic import AnsibleModule

import netifaces


def get_network_info(nwname):
    nwinfo = netifaces.ifaddresses(nwname)[netifaces.AF_INET][0]
    host_addr = ipaddress.ip_address(nwinfo['addr'])
    bcst_addr = ipaddress.ip_address(nwinfo['broadcast'])
    netmask = 32 - len(format(int(host_addr)^int(bcst_addr), 'b'))

    return str(host_addr), netmask


def main():
    module = AnsibleModule(
        argument_spec=dict(
            nwname=dict(type='str', required=True),
        ),
        supports_check_mode=True
    )
    if module.check_mode:
        module.exit_json(changed=False)

    host_addr, netmask = get_network_info(module.params['nwname'])
    module.exit_json(changed=False, host_ipaddr=host_addr, netmask=netmask)


if __name__ == '__main__':
    main()
