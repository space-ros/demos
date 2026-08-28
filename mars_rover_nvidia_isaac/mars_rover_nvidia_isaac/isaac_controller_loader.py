#! /usr/bin/env python3

import yaml
from pathlib import Path

class ControllerLoader():

    def __init__(self, file, pkg_config_file):

        self.config_file_ = file
        self.config_ = self._load_params_from_file()
        self.pkg_config_ = self._load_package_configuration(pkg_config_file)

        self._set_supported_controllers()

    def _load_package_configuration(self, pkg_config_file):
        with open(pkg_config_file, 'r') as f:
            return yaml.safe_load(f)
        

    def _load_params_from_file(self):
        with open(self.config_file_, 'r') as f:
            return yaml.safe_load(f)
        
    def print_params(self):
        #TODO
        pass
    
    def get_loaded_controllers(self):
        return self.config_.keys()
    

    def _set_supported_controllers(self):
        self.supported_controllers_ = self.pkg_config_['supported_controllers']


    def get_joints_for_controller(self, name):

        joints = None
        status = "UNSUPPORTED"
        if name in self.supported_controllers_:
            status = "SUPPORTED"
            if 'joints' in self.config_[name].keys():
                joints_list = self.config_[name]['joints']

                joints = ",".join(joints_list)
        
        return status, joints