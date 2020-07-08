#!/usr/bin/env python

base_address = 0xE7E7E7E700
base_radio = 'radio://0/80/2M/'

# Add n CFs
for i in range(3):
    base_address = base_address + i
    uri = base_radio + hex(base_address).upper()
    print "URI:"
    print uri