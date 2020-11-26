from refills_msgs.msg import DetectShelfLayersGoal, DetectShelfLayersResult, DetectFacingsResult, DetectFacingsGoal

from refills_perception_interface.action_server_behavior import PerceptionBehavior
from refills_perception_interface.not_hacks import update_shelf_system_pose
from refills_perception_interface.utils import print_with_prefix


class DetectFacingsBehavior(PerceptionBehavior):
    prefix = 'detect facings'
    def start_perception(self, goal):
        """
        :type goal: DetectFacingsGoal
        """
        self.goal_for_hacks = goal
        print_with_prefix('started', self.prefix)
        result = DetectFacingsResult()
        if not self.get_knowrob().shelf_layer_exists(goal.id):
            result.error = DetectFacingsResult.INVALID_ID
            result.error_msg = 'invalid layer id: {}'.format(goal.id)
            return result
        self.get_robosherlock().start_separator_detection(goal.id)
        self.get_robosherlock().start_barcode_detection(goal.id)
        self.current_goal = goal

    def stop_perception(self, interrupted):
        result = DetectFacingsResult()
        if interrupted:
            result.error = DetectFacingsResult.ABORTED
            print_with_prefix('interrupted', self.prefix)
        else:
            result.error = DetectFacingsResult.SUCCESS

            separators = self.get_robosherlock().stop_separator_detection(self.current_goal.id)
            barcodes = self.get_robosherlock().stop_barcode_detection(self.current_goal.id)

            update_shelf_system_pose(self.get_knowrob(), self.current_goal.id, separators)
            self.get_knowrob().update_shelf_layer_position(self.current_goal.id, separators)
            self.get_knowrob().create_unknown_barcodes(barcodes)
            self.get_knowrob().add_separators_and_barcodes(self.current_goal.id, separators, barcodes)

            result.ids = self.get_knowrob().get_facing_ids_from_layer(self.current_goal.id).keys()
            print_with_prefix('finished', self.prefix)
        return result

    def canceled(self):
        result = DetectFacingsResult()
        result.error = DetectShelfLayersResult.ABORTED
        print_with_prefix('canceled', self.prefix)
        return result

