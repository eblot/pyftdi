# Copyright (c) 2010-2024, Emmanuel Blot <emmanuel.blot@free.fr>
# Copyright (c) 2016, Emmanuel Bouaziz <ebouaziz@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""JTAG engine."""

from logging import getLogger
from typing import Dict, Optional, Tuple

from ..bits import BitSequence
from .controller import JtagController
from .machine import JtagStateMachine


class JtagEngine:
    """High-level JTAG engine."""

    def __init__(self, ctrl: 'JtagController'):
        self._ctrl = ctrl
        self._log = getLogger('jtag.eng')
        self._fsm = JtagStateMachine()
        self._tr_cache: Dict[Tuple[str,  # from state
                                   str],  # to state
                             BitSequence] = {}  # TMS sequence
        self._seq = bytearray()

    @property
    def fsm(self) -> JtagStateMachine:
        """Return the state machine."""
        return self._fsm

    @property
    def controller(self) -> 'JtagController':
        """Return the JTAG controller."""
        return self._ctrl

    def reset(self) -> None:
        """Reset the attached TAP controller"""
        self._ctrl.reset()
        self._fsm.reset()

    def get_available_statenames(self):
        """Return a list of supported state name"""
        return [str(s) for s in self._fsm.states]

    def change_state(self, statename: str) -> BitSequence:
        """Advance the TAP controller to the defined state"""
        transition = (self._fsm.state, statename)
        self._log.info('-> %s', statename.upper())
        if transition not in self._tr_cache:
            # find the state machine path to move to the new instruction
            path = self._fsm.find_path(statename)
            self._log.debug('new path: %s',
                            ', '.join((str(s).upper() for s in path[1:])))
            # convert the path into an event sequence
            events = self._fsm.get_events(path)
            self._tr_cache[transition] = events
        else:
            # transition already in cache
            events = self._tr_cache[transition]
        # update the remote device tap controller (write TMS consumes the seq)
        bs = self._ctrl.write_tms(events.copy())
        # update the current state machine's state
        self._fsm.handle_events(events.copy())
        return bs

    def go_idle(self) -> None:
        """Change the current TAP controller to the IDLE state"""
        self.change_state('run_test_idle')

    def run(self) -> None:
        """Change the current TAP controller to the IDLE state"""
        self.change_state('run_test_idle')

    def capture_ir(self) -> None:
        """Capture the current instruction from the TAP controller"""
        self.change_state('capture_ir')

    def write_ir(self, instruction: BitSequence) -> None:
        """Change the current instruction of the TAP controller"""
        self.change_state('shift_ir')
        self._ctrl.write(instruction)
        self.change_state('update_ir')

    def exchange_ir(self, instruction: BitSequence) -> BitSequence:
        """Change the current instruction of the TAP controller and retrieve
           previous IR content.
        """
        self.change_state('shift_ir')
        irbuf = self._ctrl.exchange(instruction)
        bs = self.change_state('update_ir')
        irbuf.push_left(bs)
        return irbuf

    def capture_dr(self) -> None:
        """Capture the current data register from the TAP controller"""
        self.change_state('capture_dr')

    def write_dr(self, data: BitSequence) -> None:
        """Change the data register of the TAP controller"""
        self.change_state('shift_dr')
        self._ctrl.write(data)
        self.change_state('update_dr')

    def exchange_dr(self, data: BitSequence) -> BitSequence:
        """Change the data register of the TAP controller and retrieve
           previous DR content."""
        self.change_state('shift_dr')
        drbuf = self._ctrl.exchange(data)
        bs = self.change_state('update_dr')
        drbuf.push_left(bs)
        return drbuf

    def read_dr(self, length: int, tdi: Optional[bool] = None) -> BitSequence:
        """Read the data register from the TAP controller"""
        self.change_state('shift_dr')
        drbuf = self._ctrl.read(length, tdi)
        bs = self.change_state('update_dr')
        drbuf.push_left(bs)
        return drbuf
