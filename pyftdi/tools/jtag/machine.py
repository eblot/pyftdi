# Copyright (c) 2010-2024, Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2016, Emmanuel Bouaziz <ebouaziz@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""JTAG state machine."""

from logging import getLogger
from typing import Dict, List, Tuple, Union

from ..bits import BitSequence


class JtagState:
    """Test Access Port controller state.

       :param name: the name of the state
       :param modes: categories to which the state belongs
    """

    def __init__(self, name: str, modes: Tuple[str, str]):
        self.name = name
        self.modes = modes
        self.exits = [self, self]  # dummy value before initial configuration

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name

    def setx(self, fstate: 'JtagState', tstate: 'JtagState'):
        """Define the two exit state of a state."""
        self.exits = [fstate, tstate]

    def getx(self, event) -> 'JtagState':
        """Retrieve the exit state of the state.

           :param event: evaluated as a boolean value
           :return: next state
        """
        return self.exits[int(bool(event))]

    def is_of(self, mode: str) -> bool:
        """Report if the state is a member of the specified mode."""
        return mode in self.modes


class JtagStateMachine:
    """Test Access Port controller state machine."""

    def __init__(self):
        self._log = getLogger('jtag.fsm')
        self.states = {}
        for state, modes in [('test_logic_reset', ('reset', ' idle')),
                             ('run_test_idle', ('idle',)),
                             ('select_dr_scan', ('dr',)),
                             ('capture_dr', ('dr', 'shift', 'capture')),
                             ('shift_dr', ('dr', 'shift')),
                             ('exit_1_dr', ('dr', 'update', 'pause')),
                             ('pause_dr', ('dr', 'pause')),
                             ('exit_2_dr', ('dr', 'shift', 'udpate')),
                             ('update_dr', ('dr', 'idle')),
                             ('select_ir_scan', ('ir',)),
                             ('capture_ir', ('ir', 'shift', 'capture')),
                             ('shift_ir', ('ir', 'shift')),
                             ('exit_1_ir', ('ir', 'udpate', 'pause')),
                             ('pause_ir', ('ir', 'pause')),
                             ('exit_2_ir', ('ir', 'shift', 'update')),
                             ('update_ir', ('ir', 'idle'))]:
            self.states[state] = JtagState(state, modes)
        self['test_logic_reset'].setx(self['run_test_idle'],
                                      self['test_logic_reset'])
        self['run_test_idle'].setx(self['run_test_idle'],
                                   self['select_dr_scan'])
        self['select_dr_scan'].setx(self['capture_dr'],
                                    self['select_ir_scan'])
        self['capture_dr'].setx(self['shift_dr'], self['exit_1_dr'])
        self['shift_dr'].setx(self['shift_dr'], self['exit_1_dr'])
        self['exit_1_dr'].setx(self['pause_dr'], self['update_dr'])
        self['pause_dr'].setx(self['pause_dr'], self['exit_2_dr'])
        self['exit_2_dr'].setx(self['shift_dr'], self['update_dr'])
        self['update_dr'].setx(self['run_test_idle'],
                               self['select_dr_scan'])
        self['select_ir_scan'].setx(self['capture_ir'],
                                    self['test_logic_reset'])
        self['capture_ir'].setx(self['shift_ir'], self['exit_1_ir'])
        self['shift_ir'].setx(self['shift_ir'], self['exit_1_ir'])
        self['exit_1_ir'].setx(self['pause_ir'], self['update_ir'])
        self['pause_ir'].setx(self['pause_ir'], self['exit_2_ir'])
        self['exit_2_ir'].setx(self['shift_ir'], self['update_ir'])
        self['update_ir'].setx(self['run_test_idle'], self['select_dr_scan'])
        self._current = self['test_logic_reset']
        self._tr_cache: Dict[Tuple[str,  # current state name
                                   int,  # event length
                                   int],  # event value
                             JtagState] = {}  # new state

    def __getitem__(self, name: str) -> JtagState:
        return self.states[name]

    @property
    def state(self) -> JtagState:
        """Return the current state."""
        return self._current

    def state_of(self, mode: str) -> bool:
        """Report if the current state is of the specified mode."""
        return self._current.is_of(mode)

    def reset(self):
        """Reset the state machine."""
        self._current = self['test_logic_reset']

    def find_path(self, target: Union[JtagState, str],
                  source: Union[JtagState, str, None] = None) \
            -> List[JtagState]:
        """Find the shortest event sequence to move from source state to
           target state. If source state is not specified, used the current
           state.

           :return: the list of states, including source and target states.
        """
        if source is None:
            source = self.state
        if isinstance(source, str):
            source = self[source]
        if isinstance(target, str):
            target = self[target]

        def next_path(state, target, path):
            # this test match the target, path is valid
            if state == target:
                return path+[state]
            # candidate paths
            paths = []
            for xstate in state.exits:
                # next state is self (loop around), kill the path
                if xstate == state:
                    continue
                # next state already in upstream (loop back), kill the path
                if xstate in path:
                    continue
                # try the current path
                npath = next_path(xstate, target, path + [state])
                # downstream is a valid path, store it
                if npath:
                    paths.append(npath)
            # keep the shortest path
            return min(((len(path), path) for path in paths),
                       key=lambda x: x[0])[1] if paths else []
        return next_path(source, target, [])

    @classmethod
    def get_events(cls, path):
        """Build up an event sequence from a state sequence, so that the
           resulting event sequence allows the JTAG state machine to advance
           from the first state to the last one of the input sequence"""
        events = []
        for sstate, dstate in zip(path[:-1], path[1:]):
            for epos, xstate in enumerate(sstate.exits):
                if xstate == dstate:
                    events.append(epos)
        if len(events) != len(path) - 1:
            raise RuntimeError("Invalid path")
        return BitSequence(events)

    def handle_events(self, events: BitSequence) -> None:
        """State machine stepping.

           :param events: a sequence of boolean events to advance the FSM.
        """
        transit = (self._current.name, len(events), int(events))
        if transit in self._tr_cache:
            self._current = self._tr_cache[transit]
            return
        for event in events:
            self._current = self._current.getx(event)
        self._tr_cache[transit] = self._current
