from contextlib import contextmanager
from multiprocessing import Lock

import rospy


class TimeoutLock(object):
    def __init__(self):
        self._lock = Lock()

    def acquire(self, timeout=-1):
        return self._lock.acquire(timeout=timeout)

    @contextmanager
    def acquire_timeout(self, timeout):
        result = self._lock.acquire(timeout=timeout)
        yield result
        if result:
            self._lock.release()

    def release(self):
        self._lock.release()

def print_with_prefix(msg, prefix):
    rospy.loginfo('[{}] {}'.format(prefix, msg))

def warn_with_prefix(msg, prefix):
    rospy.logwarn('[{}] {}'.format(prefix, msg))

def error_with_refix(msg, prefix):
    rospy.logerr('[{}] {}'.format(prefix, msg))