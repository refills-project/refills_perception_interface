from refills_msgs.msg import DetectShelfLayersGoal, DetectShelfLayersResult

from refills_perception_interface.action_server_behavior import PerceptionBehavior
from refills_perception_interface.utils import print_with_prefix


class DetectShelfLayersBehavior(PerceptionBehavior):
    prefix = 'detect shelf layers'
    def start_perception(self, goal):
        """
        :type goal: DetectShelfLayersGoal
        """
        result = DetectShelfLayersResult()
        print_with_prefix('started', self.prefix)
        if not self.get_knowrob().shelf_system_exists(goal.id):
            result.error = DetectShelfLayersResult.INVALID_ID
            result.error_msg = 'invalid shelf id: {}'.format(goal.id)
            print_with_prefix('invalid id', self.prefix)
            return result
        self.get_robosherlock().start_detect_shelf_layers(goal.id)
        self.current_goal = goal

    def stop_perception(self, interrupted):
        result = DetectShelfLayersResult()
        if interrupted:
            result.error = DetectShelfLayersResult.ABORTED
            print_with_prefix('interrupted', self.prefix)
        else:
            result.error = DetectShelfLayersResult.SUCCESS
            shelf_layer_heights = self.get_robosherlock().stop_detect_shelf_layers(self.current_goal.id)
            self.get_knowrob().add_shelf_layers(self.current_goal.id, shelf_layer_heights)
            result.ids = self.get_knowrob().get_shelf_layer_from_system(self.current_goal.id).keys()
            print_with_prefix('finished', self.prefix)
        return result

    def canceled(self):
        result = DetectShelfLayersResult()
        result.error = DetectShelfLayersResult.ABORTED
        print_with_prefix('canceled', self.prefix)
        return result

