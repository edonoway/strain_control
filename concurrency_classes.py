'''
A few convenient classes for multithreading.
'''

from threading import Thread, Lock, Event
import threading
import multiprocessing as mp
import queue

def queue_write(q, message):
    '''
    helper function that takes in a Queue object, clears the contents of the Queue and then rewrites to it in a thread and process safe manner.
    '''
    while not q.empty():
        q.get()
    q.put(message)

def queue_read(q):
    '''
    helper function that takes in a Queue object, reads the contents, and then puts the message back so that future calls can still read the value. Thread and process safe.
    '''
    val = q.get()
    q.put(val)
    return val

class LockedVar:
    '''
    Minimal class to implement a locking variable. Contains two private attributes, a value and a lock, and a few methods for safetly reading writing value via the lock.
    '''
    def __init__(self, val):
        self._value = val
        self._lock = Lock()

    def locked_read(self):
        with self._lock:
            return self._value

    def locked_update(self, val):
        with self._lock:
            self._value = val

class StoppableThread(Thread):
    '''
    Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition.
    '''
    def __init__(self,  *args, **kwargs):
        super(StoppableThread, self).__init__(*args, **kwargs)
        self._stop_event = Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()
