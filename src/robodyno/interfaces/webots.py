#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""webots.py
Time    :   2023/01/05
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Robodyno webots interface

  Typical usage example:

  webots = Webots(time_step = 0)
"""

import threading
from threading import Lock
import time

try:
    from controller import Robot
except:
    raise RuntimeError('Only work with Webots environment.')

class Webots(object):
    """Webots interface holds global robot instance, update nodes automaticly.
    
    Attributes:
        robot: Webots robot instance
        time_step: Webots simulate timestep
    """
    
    def __init__(self, time_step = 0, step_callback = None):
        """Init Webots interface.
        
        Args:
            time_step: time step to update simulation
                       set it to 0 to use value of the basicTimeStep field of the WorldInfo node
            step_callback: called after each simulation step
        """
        self.robot = Robot()
        if time_step == 0:
            self.time_step = self.robot.getBasicTimeStep()
        else:
            self.time_step = time_step

        self._nodes = []
        
        self._step_cb = step_callback
        
        self._wb_thread = threading.Thread(
            target = self._wb_thread_callback,
            name = 'webots.step',
            daemon= True
        )
        self._mutex = Lock()
        self.robot.step(int(self.time_step))
        self._wb_thread.start()
    
    def time(self):
        """Get time in simulation world.
        
        Returns:
            current time in seconds
        """
        return self.robot.getTime()

    def sleep(self, seconds):
        """Time sleep.
        
        Args:
            seconds: sleep duration in seconds
        """
        s = self.time()
        while self.time() - s < seconds:
            pass

    def register(self, node):
        """Simulation will update all nodes registered to interface.
        
        Args:
            node: node with update function
        """
        self._nodes.append(node)
    
    def deregister(self, node):
        """Deregister a registered node.
        
        Args:
            node: node registered
        """
        self._nodes.remove(node)
    
    def _wb_thread_callback(self):
        """Step webots simulation thread callback"""
        while(True):
            self._mutex.acquire()
            if self.robot.stepBegin(int(self.time_step)) == -1:
                self._mutex.release()
                break
            for node in self._nodes:
                node.parallel_update()
            if self.robot.stepEnd() == -1:
                self._mutex.release()
                break
            for node in self._nodes:
                node.update()
            self._mutex.release()
            if self._step_cb:
                self._step_cb()
            time.sleep(0.001)
