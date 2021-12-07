import traceback
from queue import Queue

from actionlib import SimpleActionServer
from refills_msgs.msg import DetectShelfLayersResult
from py_trees import Blackboard, Status

from refills_perception_interface.MyBehavior import MyBahaviour
from refills_perception_interface.utils import TimeoutLock, print_with_prefix


class ActionServerHandler(object):
    """
    Interface to action server which is more useful for behaviors.
    """
    def __init__(self, action_name, action_type):
        self.goal_queue = Queue(1)
        self.result_queue = Queue(1)
        self.lock = Blackboard().lock
        self.action_name = action_name
        self.cancel_next_goal = False
        self._as = SimpleActionServer(action_name, action_type, execute_cb=self.execute_cb, auto_start=False)
        self._as.register_preempt_callback(self.cancel_cb)
        self._as.start()

    def cancel_cb(self):
        self.cancel_next_goal = self._as.is_new_goal_available()

    def execute_cb(self, goal):
        """
        :type goal: MoveGoal
        """
        with self.lock.acquire_timeout(0) as got_lock:
            if got_lock and not self.cancel_next_goal:
                self.my_state = Status.RUNNING
                self.goal_queue.put(goal)
                self.result_queue.get()()
            else:
                self.my_state = Status.FAILURE
                r = self._as.action_server.ActionResultType()
                r.error = DetectShelfLayersResult.SERVER_BUSY
                print_with_prefix('rejected goal because server busy server busy', self.action_name)
                self._as.set_aborted(r)
                self.cancel_next_goal = False

    def get_goal(self):
        try:
            goal = self.goal_queue.get_nowait()
            return goal
        except Empty:
            return None

    def has_goal(self):
        return not self.goal_queue.empty()

    def send_preempted(self, result=None):
        def call_me_now():
            self._as.set_preempted(result)
        self.result_queue.put(call_me_now)

    def send_aborted(self, result=None):
        def call_me_now():
            self._as.set_aborted(result)
        self.result_queue.put(call_me_now)

    def send_result(self, result=None):
        """
        :type result: MoveResult
        """
        def call_me_now():
            self._as.set_succeeded(result)
        self.result_queue.put(call_me_now)

    def is_preempt_requested(self):
        return self._as.is_preempt_requested()

class ActionServerBehavior(MyBahaviour):
    def __init__(self, name, as_name, action_type=None):
        self.as_handler = None
        self.as_name = as_name
        self.action_type = action_type
        super(ActionServerBehavior, self).__init__(name)

    def setup(self, timeout):
        self.as_handler = Blackboard().get(self.as_name)
        if self.as_handler is None:
            self.as_handler = ActionServerHandler(self.as_name, self.action_type)
            Blackboard().set(self.as_name, self.as_handler)
        return super(ActionServerBehavior, self).setup(timeout)

    def get_as(self):
        """
        :rtype: ActionServerHandler
        """
        return self.as_handler

    def get_goal(self):
        return self.get_as().get_goal()

    def has_goal(self):
        return self.get_as().has_goal()

class GoalReceived(ActionServerBehavior):
    def update(self):
        if self.get_as().has_goal():
            return Status.SUCCESS
        return Status.FAILURE


class PerceptionBehavior(ActionServerBehavior):
    prefix = None
    def set_my_state(self, new_state):
        self.my_state = new_state

    def get_my_state(self):
        return self.my_state

    def __start_perception(self):
        if self.get_my_state() == Status.RUNNING:
            raise Exception('perception already running')
        goal = self.get_goal()
        self.set_my_state(Status.RUNNING)
        return self.start_perception(goal)

    def start_perception(self, goal):
        """
        :param goal: action goal
        :return: action result if the perception should be stopped immediately, None otherwise
        """
        pass

    def __stop_perception(self, interrupted):
        if self.get_my_state() != Status.RUNNING:
            raise Exception('perception not running')
        self.set_my_state(Status.SUCCESS)
        return self.stop_perception(interrupted)

    def stop_perception(self, interrupted):
        """
        :param interrupted: whether or not cancel was called.
        :type interrupted: bool
        :return: action result
        """
        pass

    def is_finished(self):
        """
        :rtype: bool
        """
        finished = self.blackboard.finished
        self.blackboard.finished = False
        return finished

    def setup(self, timeout):
        self.set_my_state(Status.FAILURE)
        self.lock = self.blackboard.lock # type: TimeoutLock
        return super(PerceptionBehavior, self).setup(timeout)

    def initialise(self):
        super(PerceptionBehavior, self).initialise()

    def terminate(self, new_status):
        if self.get_my_state() == Status.RUNNING:
            self.__stop_perception(True)
            self.return_interrupted()
            self.set_my_state(Status.FAILURE)
        super(PerceptionBehavior, self).terminate(new_status)

    def return_interrupted(self):
        self.get_as().send_preempted()

    def canceled(self):
        pass

    def __canceled(self):
        self.__stop_perception(True)
        result = self.canceled()
        self.get_as().send_preempted(result)

    def update(self):
        try:
            self.feedback_message = ''
            if self.has_goal() and self.get_my_state() != Status.RUNNING:
                result = self.__start_perception()
                self.feedback_message = 'started'
                if result is not None:
                    if result.error != 0:
                        self.feedback_message = 'something went wrong'
                        self.get_as().send_aborted(result)
                        self.set_my_state(Status.FAILURE)
                    else:
                        self.feedback_message = 'finished immediately'
                        self.get_as().send_result(result)
                        self.set_my_state(Status.SUCCESS)
            elif self.is_finished():
                self.feedback_message = 'finished'
                self.get_as().send_result(self.__stop_perception(False))
                self.set_my_state(Status.SUCCESS)
            elif self.get_as().is_preempt_requested():
                self.feedback_message = 'canceled'
                self.__canceled()
                self.set_my_state(Status.FAILURE)
        except Exception as e:
            traceback.print_exc()
            self.get_as().send_aborted()
            self.set_my_state(Status.FAILURE)
        return self.get_my_state()
