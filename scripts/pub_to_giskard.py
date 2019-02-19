import rospy
from giskard_msgs.msg import WorldBody
from giskard_msgs.srv import UpdateWorldRequest, UpdateWorld

from refills_perception_interface import knowrob_wrapper
from refills_perception_interface.knowrob_wrapper import KnowRob

rospy.init_node('adsfasdf')

s = rospy.ServiceProxy('/giskard/update_world', UpdateWorld)
rospy.sleep(1)
m = UpdateWorldRequest()
m.operation = UpdateWorldRequest.ADD
m.body.type = WorldBody.MESH_BODY

k = KnowRob()
for shelf_id in k.get_shelf_system_ids(False):
    shelf_layer = k.get_shelf_layer_from_system(shelf_id)
    if len(shelf_layer) > 0:
        m.body.name = shelf_id
        q = 'object_information(\'{}\',Type,HasVisual,Color,Mesh,[D,W,H],Pose,StaticTransforms)'.format(shelf_id)
        result = k.once(q)
        m.body.mesh = result['Mesh']
        m.body.mesh = m.body.mesh.replace('\'','')
        if m.body.mesh.endswith('.dae'):
            m.body.mesh = m.body.mesh[:-4] + '.stl'
        m.pose = k.prolog_to_pose_msg(result['Pose'])
        # print(m)
        s.call(m)
        # continue
        for shelf_layer_id in shelf_layer:
            m.body.name = shelf_layer_id
            q = 'object_information(\'{}\',Type,HasVisual,Color,Mesh,[D,W,H],Pose,StaticTransforms)'.format(shelf_layer_id)
            result = k.once(q)
            m.body.mesh = result['Mesh']
            m.body.mesh = m.body.mesh.replace('\'', '')
            if m.body.mesh.endswith('.dae'):
                m.body.mesh = m.body.mesh[:-4] + '.stl'
            m.pose = k.prolog_to_pose_msg(result['Pose'])
            # print(m)
            s.call(m)
            q = 'findall(O, (rdf_has(\'{}\', P, O), rdfs_individual_of(O, dmshop:\'DMShelfSeparator\')), Os).'.format(shelf_layer_id)
            separators = k.once(q)['Os']
            for separator_id in separators:
                m.body.name = separator_id
                q = 'object_information(\'{}\',Type,HasVisual,Color,Mesh,[D,W,H],Pose,StaticTransforms)'.format(
                    separator_id)
                result = k.once(q)
                m.body.mesh = result['Mesh']
                m.body.mesh = m.body.mesh.replace('\'', '')
                if m.body.mesh.endswith('.dae'):
                    m.body.mesh = m.body.mesh[:-4] + '.stl'
                m.pose = k.prolog_to_pose_msg(result['Pose'])
                # print(m)
                s.call(m)
    # break


    # break