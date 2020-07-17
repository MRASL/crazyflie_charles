#!/usr/bin/env python

"""
Python script to analyse flight data

Options:
    - Specify data name
    - Plot position : exp pose and goal
        - Specify axis
        - Draw a line
        - Possibility to accelerate/slow
"""

import os
from os.path import isfile, join
import argparse
import numpy as np

class DataAnalyser(object):
    """
    To analyse data
    """
    def __init__(self, data_name, to_plot):
        self.data_name = data_name
        self.to_plot = to_plot
        self.data_path = None
        self.data_base_name = 'flight_data_'
        self.crazyflies = None

        self._load_data()

    def _load_data(self):
        """Load flight data.

        If data_name is empty, load latest data set
        """
        parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.data_path = os.path.join(parentdir, 'flight_data')

        if self.data_name == "":
            data_to_load = self._find_latest_data()

        else:
            data_to_load = self.data_name + '.npy'

        data_path = join(self.data_path, data_to_load)

        self.crazyflies = np.load(data_path, allow_pickle='TRUE').item()

    def _find_latest_data(self):
        """Find latest data name

        Returns:
            str: Name of latest data set
        """
        # List all data files
        data_list = [f for f in os.listdir(self.data_path) if isfile(join(self.data_path, f))]
        latest_id = None

        # Find all unnamed data
        if data_list:
            data_list = [f for f in data_list if f.startswith(self.data_base_name)]

            if data_list:
                data_list = [f.replace(self.data_base_name, '') for f in data_list]
                data_list = [int(f.replace('.npy', '')) for f in data_list]
                latest_id = str(np.max(data_list))

        if latest_id is not None:
            latest_data = self.data_base_name + latest_id + '.npy'
        else:
            raise RuntimeError("Data not found")

        return latest_data

if __name__ == '__main__':
    # pylint: disable=invalid-name
    parser = argparse.ArgumentParser(description='Analyse flight data')
    parser.add_argument('--data-name', '-d', type=str, help='Name of data', default='')
    parser.add_argument('--plot', '-p', action='store_true', help='To plot data', default=False)

    args = parser.parse_args()
    d_name = args.data_name
    plot = args.plot

    analyser = DataAnalyser(d_name, plot)

    # pylint: enable=invalid-name
