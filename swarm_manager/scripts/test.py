#!/usr/bin/env python
import os
import yaml

parentdir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(parentdir, 'conf.yaml')

with open(file_path) as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
    print data
    for group, args_dict in data.items():
        for arg_name, arg_val in args_dict.items():
            print arg_name + ':=' + str(arg_val)
