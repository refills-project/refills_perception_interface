from py_trees import Behaviour, Blackboard

from refills_perception_interface.robosherlock_wrapper import FakeRoboSherlock, RoboSherlock


class MyBahaviour(Behaviour):
    """
    Base Behavior that gives access to robosherlock and knowrob.
    """

    def __init__(self, name=""):
        super(MyBahaviour, self).__init__(name)
        self.blackboard = Blackboard()
        self.param_name = 'exception'

    def get_knowrob(self):
        """
        :rtype: KnowRob
        """
        return self.blackboard.knowrob

    def get_robosherlock(self):
        """
        :rtype: RoboSherlock
        """
        return self.blackboard.robosherlock

    def get_blackboard(self):
        return self.blackboard

    def raise_to_blackboard(self, exception):
        """
        saves an exception on the blackboard
        :type exception: Exception
        """
        self.blackboard.set(self.param_name, exception)

    def get_exception(self):
        return self.blackboard.get(self.param_name)