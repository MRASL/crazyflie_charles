"""
Test de la librairie bitcraze
"""


import cflib.crtp


class SwarmManager:
    def __init__(self):
        self.uris = []
        self._find_uri()

    def _find_uri(self):
        # Initiate the low level drivers
        cflib.crtp.init_drivers(enable_debug_driver=False)

        print('Scanning interfaces for Crazyflies...')
        n_cf = 2
        base_add = 0xE7E7E7E700

        for i in range(n_cf):
            res = cflib.crtp.scan_interfaces(base_add + i + 1)

            if res: self.uris.append(res[0])

        if len(self.uris) > 0:
            print('Found %d Crazyflies: ' % len(self.uris))
            for each_add in self.uris:
                print('\t', each_add[0])
        else:
            print("Erros: No Crazyflie found")

if __name__ == '__main__':
    swarmManager = SwarmManager()
