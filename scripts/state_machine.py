#!/usr/bin/env python

"""
To act as simple state machine
"""
import rospy

class StateMachine(object):
    """A state machine class.

    It can be in different states. Each state can be link to a function.
    """
    def __init__(self, states=None):
        # Initialize states dict
        if states is None:
            self.states = {}
        elif isinstance(states, list):
            self.states = {state:None for state in states}
        else:
            self.states = states

        self.current_state = ""

    def add_states(self, new_states):
        """Add states to possible state list

        Args:
            new_states (list or dict): states name and, if a dict, their associated function
        """
        if isinstance(new_states, list):
            self.states.update({state:None for state in new_states})
        else:
            self.states.update(new_states)

    def get_states(self):
        """Get names of possible states

        Returns:
            list of str: Possible states
        """
        return self.states.keys()

    def get_state(self):
        """Get name of current state

        Returns:
            str: Current state
        """
        return self.current_state

    def set_state(self, new_state):
        """Set current state

        Before setting, verifies the new state is in the possible states

        Args:
            new_state (str): New state
        """
        if new_state in self.states:
            self.current_state = new_state
        else:
            rospy.logerr("Invalid State: %s" % new_state)

    def in_state(self, state):
        """Check current state

        Args:
            state (str): To verify if currently in this state

        Returns:
            bool: True if SM in it's this state
        """
        return self.current_state == state

    def run_state(self):
        """Return method associated with current state

        Returns:
            function: Function to run
        """
        val = self.states[self.current_state]
        if val is None:
            rospy.logwarn("No function to run")

        return val
