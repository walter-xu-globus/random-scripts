"""
Requires PS module
`Install-Module powershell-yaml`

See documentation at https://docs.globusmedical.com/confluence/x/ehHTCQ
"""

# Std libs
import sys
import os
import subprocess as sp
from pathlib import Path
import json

# Add the path to local libraries
#sys.path.append('c:/epic/scripts/pylib')

# Local libs
#import config_helpers as config

from ruamel.yaml import YAML, comments

def get_system_data_path():
    "Return the value of the GM_DATA environment variable"
    return Path(os.environ.get('GM_DATA').strip())

def get_install_path():
    "Return the value of the GM_HOME environment variable"
    return Path(os.environ.get('GM_HOME').strip())

def get_platform_config_path():
    "Assemble the path to system config file"
    base_path = get_system_data_path()
    path = Path.joinpath(base_path, "config", "config_platform.json")
    return path

def get_platform_settings_path():
    "Assemble the path to system config file"
    base_path = get_system_data_path()
    path = Path.joinpath(base_path, "config", "settings_platform.json")
    return path

def read_platform_settings():
    "Read the contents of system settings file"
    data = json.load(open(get_platform_settings_path(), 'r'))
    return data

def read_platform_config():
    "Read the contents of system settings file"
    data = json.load(open(get_platform_config_path(), 'r'))
    return data

def write_platform_config(settings):
    "Write out system settings with the contents 'settings'"
    settingsFile = get_platform_config_path()
    with open(settingsFile, 'w') as jf:
        json.dump(settings, jf, indent=4)
        jf.close()

def write_platform_settings(settings):
    "Write out system settings with the contents 'settings'"
    settingsFile = get_platform_settings_path()
    with open(settingsFile, 'w') as jf:
        json.dump(settings, jf, indent=4)
        jf.close()

def main():
    "Runtime entry point"

    # Read in sysConfig file
    sysconfig = read_platform_settings()
    platform = read_platform_config()
    #system_type = platform['system_type']
    system_type = ""
    networking = sysconfig["networking"]
    adapter_roles = sysconfig["networking"]["role_binding"]

    adapters = query_adapter_information()

    # Based on the adapters can we determine what system we are on?
    for adapter in adapters:
        if "Intel(R) Ethernet Connection I217-LM" == adapter["InterfaceDescription"] or "Realtek PCIe GBE Family Controller" == adapter["InterfaceDescription"]:
            system_type = 'egps'
        elif "Intel(R) I350 Gigabit Network Connection" == adapter["InterfaceDescription"]:
            system_type = 'ehub'

    # Check to see if we are in a known system type
    if system_type == "egps" or system_type == "ehub":
        for adapter in adapters:
            if system_type == "egps":
                # GPS v1
                if "Intel(R) Ethernet Connection I217-LM" == adapter["InterfaceDescription"] or "Realtek PCIe GBE Family Controller" == adapter["InterfaceDescription"]:
                    networking["role_binding"]["internet"] = networking["role_binding"]["glink"] = adapter["Name"]
                elif "Intel(R) I210 Gigabit Network Connection" == adapter["InterfaceDescription"] or "Realtek PCIe GBE Family Controller #2" == adapter["InterfaceDescription"]:
                    networking["role_binding"]["motion"] = adapter["Name"]
                    pscommand = ["powershell", "-NoLogo", "-NoProfile", "-ExecutionPolicy", "bypass",
                     "-Command", "netsh interface ipv4 add address '" + adapter["Name"] + "' 192.168.1.10 255.255.255.0"]
                    sp.run(pscommand)
                    pscommand = ["powershell", "-NoLogo", "-NoProfile", "-ExecutionPolicy", "bypass",
                     "-Command", "netsh interface ipv4 add address '" + adapter["Name"] + "' 169.254.1.100 255.255.255.0"]
                    sp.run(pscommand)
            elif system_type == "ehub":
                if "Intel(R) I350 Gigabit Network Connection" == adapter["InterfaceDescription"]:
                    networking["role_binding"]["camera"] = adapter["Name"]
                    pscommand = ["powershell", "-NoLogo", "-NoProfile", "-ExecutionPolicy", "bypass",
                     "-Command", "netsh interface ipv4 add address '" + adapter["Name"] + "' 169.254.5.100 255.255.0.0 169.254.5.1"]
                    sp.run(pscommand)
                    pscommand = ["powershell", "-NoLogo", "-NoProfile", "-ExecutionPolicy", "bypass",
                     "-Command", "netsh interface ipv4 add dnsservers '" + adapter["Name"] + "' 169.254.5.1"]
                    sp.run(pscommand)
                elif "Intel(R) I350 Gigabit Network Connection #2" == adapter["InterfaceDescription"]:
                    networking["role_binding"]["internet"] = networking["role_binding"]["glink"] = networking["role_binding"]["router"] = adapter["Name"]
                    pscommand = ["powershell", "-NoLogo", "-NoProfile", "-ExecutionPolicy", "bypass",
                     "-Command", "netsh interface ipv4 add address '" + adapter["Name"] + "' 169.254.5.2 255.255.255.0 169.254.5.1"]
                    sp.run(pscommand)
                    pscommand = ["powershell", "-NoLogo", "-NoProfile", "-ExecutionPolicy", "bypass",
                     "-Command", "netsh interface ipv4 add dnsservers '" + adapter["Name"] + "' 169.254.5.1"]
                    sp.run(pscommand)

        # Update the other misc network settings
        if system_type == "egps":
            networking["ip_address"] = ""
            networking["netmask"] = ""
            networking["gateway"] = ""
            networking["ip_protocol"] = "DHCP"
            networking["preferred_dns"] = ""
            networking["alternate_dns"] = ""
            networking["role_binding"]["router"] = ""
            networking["role_binding"]["camera"] = ""
            networking["system_router"]["ip_address"] = ""
            networking["system_router"]["netmask"] = ""
            networking["system_router"]["gateway"] = ""
            networking["system_router"]["ip_protocol"] = "DHCP"
        else:
            networking["ip_address"] = "169.254.5.2"
            networking["netmask"] = "255.255.255.0"
            networking["gateway"] = "169.254.5.1"
            networking["ip_protocol"] = "Static"
            networking["preferred_dns"] = "169.254.5.1"
            networking["alternate_dns"] = "169.254.5.1"
            networking["role_binding"]["motion"] = ""
            networking["system_router"]["ip_address"] = ""
            networking["system_router"]["netmask"] = ""
            networking["system_router"]["gateway"] = ""
            networking["system_router"]["ip_protocol"] = "DHCP"

        # Write the information out to the config files
        platform["system_type"] = system_type
        write_platform_config(platform)
        sysconfig["networking"] = networking
        write_platform_settings(sysconfig)


def query_adapter_information():
    "Use powershell script to get system network adapter information"

    # Call powershell 'Get-NetAdapter' to get adapter information
    pscommand = ["powershell", "-NoLogo", "-NoProfile", "-ExecutionPolicy", "bypass",
                 "-Command",
                 "Get-NetAdapter", "|",
                 "Select-Object", "-Property", "Name,InterfaceDescription", "|",
                 "ConvertTo-Yaml"]  # Requires PS module powershell-yaml

    # PS call should return a yaml document with a list of Interface names to DeviceId's
    out = sp.run(pscommand, capture_output=True, check=True)

    # Convert from yaml to pydict
    yaml_reader = YAML()
    adapters = yaml_reader.load(out.stdout)

    return adapters


if __name__ == "__main__":
    main()
