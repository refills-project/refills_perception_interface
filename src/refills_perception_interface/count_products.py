from refills_msgs.msg import DetectShelfLayersGoal, DetectShelfLayersResult, DetectFacingsResult, DetectFacingsGoal, \
    CountProductsResult, CountProductsGoal

from refills_perception_interface.action_server_behavior import PerceptionBehavior
from refills_perception_interface.utils import print_with_prefix


class CountProductsBehavior(PerceptionBehavior):
    prefix = 'count products'
    def start_perception(self, goal):
        """
        :type goal: CountProductsGoal
        """
        print_with_prefix('started', self.prefix)
        result = CountProductsResult()
        if self.get_knowrob().facing_exists(goal.id):
            result.error = CountProductsResult.SUCCESS
            result.count = 1#self.get_robosherlock().count_product(goal.id)
            self.get_knowrob().add_objects(goal.id, result.count)
            print_with_prefix('counted {} times'.format(result.count), self.prefix)
            print_with_prefix('finished', self.prefix)
            return result
        else:
            result.error = DetectFacingsResult.INVALID_ID
            result.error_msg = 'invalid facing id: {}'.format(goal.id)
            print_with_prefix('invalid id', self.prefix)
            return result

