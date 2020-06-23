#!/usr/bin/env python

from state_machine import StateMachine

def func():
    print 'allo'

sm = StateMachine(["t1", "t2"])

sm.add_states(["s1", "s2", "s3"])

sm.add_states({'f':func})
sm.set_state('f')
f = sm.run_state()

f()
