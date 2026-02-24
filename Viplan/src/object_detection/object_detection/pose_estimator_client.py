#!/usr/bin/env python3
import rclpy
import re
from rclpy.node import Node
from object_detection_msgs.srv import GetObjectPosition 
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import time
from builtin_interfaces.msg import Duration

def extract_objects_from_pddl_test(pddl_text):
    init_start = pddl_text.index(':init')
    goal_start = pddl_text.index(':goal')

    init_obj_wps = re.findall(r"\(\S+ (\S+) (\S+)\)", pddl_text[init_start:goal_start])
    goal_obj_wps = re.findall(r"\(\S+ (\S+) (\S+)\)", pddl_text[goal_start:])

    object_dictionary = {}
    for objs in init_obj_wps:
        object_dictionary[objs[0]]  = {'wps': objs[1]}

    for objs in goal_obj_wps:
        if objs[0] in object_dictionary:
             object_dictionary[objs[0]]['wpf'] = objs[1]

    return object_dictionary


class ObjectPositionClient(Node):
    def __init__(self):
        super().__init__('object_position_client')
        self.out_location = [0.2, 0.65, 0.5, -0.610608, 0.791903, 0.0, 0.0]    
        self.client = self.create_client(GetObjectPosition, 'get_object_positions')
        self.publisher = self.create_publisher(MarkerArray, 'detected_object_marker_array', 10)
        self.marker_array = MarkerArray()
        self.marker_id = 1
        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.request = GetObjectPosition.Request()

        
    def send_request(self):
        # Create the request
        # Replace with the path to your PDDL file
        with open('problem2.pddl', 'r') as file:
            pddl_text = file.read()
        self.object_dictionary = extract_objects_from_pddl_test(pddl_text) #shouldn't this be in send_request? Would this be read after everycall  
        self.get_logger().info(f'Extracted objects: {list(self.object_dictionary.keys())}')


        for obj in self.object_dictionary:
            msg = String()
            msg.data = obj
            self.request.objects_names.append(msg)   

        ### getting the object location
        return self.client.call_async(self.request)

    def save_location_to_file(self, object_request):
        objects_names = object_request.objects_names
        objects_positions = object_request.objects_positions
        


        delete_markers = MarkerArray()
        for marker in self.marker_array.markers:
            # delete_marker = Marker()
            # delete_marker.header.frame_id = "panda_link0"
            # delete_marker.header.stamp = self.get_clock().now().to_msg()
            # delete_marker.ns = marker.ns  # You might need the specific namespace if different markers have different namespaces
            # delete_marker.id = marker.id
            # delete_marker.action = Marker.DELETE
            # delete_markers.markers.append(delete_marker)
            # # Append the delete marker to the array
            self.marker_array.markers.remove(marker)
        
        #self.marker_array.markers.extend(delete_markers.markers)
        ### Mapping between objects and their location using dictionary
        objects_poses = {}
        for obj_name, obj_pose in zip(objects_names, objects_positions):
            objects_poses[obj_name.data] = [obj_pose.position.x,
                                            obj_pose.position.y,
                                            obj_pose.position.z,
                                            obj_pose.orientation.x,
                                            obj_pose.orientation.y,
                                            obj_pose.orientation.z,
                                            obj_pose.orientation.w]

            
            marker = Marker()
            marker.header.frame_id = "panda_link0"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "markers" + obj_name.data
            marker.id = self.marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.lifetime = Duration(sec=10, nanosec = 0)

            # Set position
            marker.pose.position.x = obj_pose.position.x
            marker.pose.position.y =  obj_pose.position.y
            marker.pose.position.z = obj_pose.position.z

            # Set scale
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # Set color
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Set text (name)
            marker.text = obj_name.data

            self.marker_array.markers.append(marker)
            self.marker_id += 1
 
        self.publisher.publish(self.marker_array)
        self.get_logger().info('Published marker array')


        ### Assigning positions wp1s...., wp1f...., object
        object_dictionary_reversed = {}
        for name, way_pnts in self.object_dictionary.items():
            if name in objects_poses:
                wps = way_pnts['wps'] if 'wps' in way_pnts else None
                wpf = way_pnts['wpf'] if 'wpf' in way_pnts else None
                if (wpf is not None) and (wps is not None):
                    if not (re.search('wp\wf', wpf) is None) and (wps is not None): ### final position is same as initial position wp1f or wp2f etc..
                        object_dictionary_reversed[name] = {wps: objects_poses[name],
                                                            wpf: objects_poses[name]}
                        
                    elif (wpf in objects_poses) and (wps is not None): ### final position is top of object
                        object_dictionary_reversed[name] = {wps: objects_poses[name],
                                                            wpf: objects_poses[wpf]}
                        
                    elif ('out' in wpf) and (wps is not None): ### or its an out location
                        object_dictionary_reversed[name] = {wps: objects_poses[name],
                                                            wpf: self.out_location}
                        
                elif (wps is None):
                    object_dictionary_reversed[name] = {wpf: objects_poses[name]}
                
                elif (wpf is None):
                    object_dictionary_reversed[name] = {wps: objects_poses[name]}

        
        writing_everthing_in_yaml = []
        done = []
        for name, way_pnts in object_dictionary_reversed.items():
            if not (name in done):
                for k, v in self.object_dictionary[name].items():
                    if (k=='wps') and (not (v in done)):
                        pose_str = ','.join([f'{p:0.4f}' for p in way_pnts[v]])
                        writing_everthing_in_yaml.append(name + ': ' + '[' + pose_str + ']')
                        writing_everthing_in_yaml.append(v + ': ' + '[' + pose_str + ']')
                        done.extend([name, v])
                        
                    if (k=='wpf') and (not (v in done)):
                        if not (re.search('wp\wf', v) is None): ### final position is same as initial position wp1f or wp2f etc..
                            pose_str = ','.join([f'{p:0.4f}' for p in way_pnts[v]])
                            writing_everthing_in_yaml.append(v + ': ' + '[' + pose_str + ']')
                            done.append(v)
                            
                        elif v in objects_poses: ### final position is top of object
                            pose_str = ','.join([f'{p:0.4f}' for p in objects_poses[v]])
                            writing_everthing_in_yaml.append(v + ': ' + '[' + pose_str + ']')
                            done.append(v)
                            
                        elif 'out' in v: ### or its an out location
                            pose_str = ','.join([f'{p:0.4f}' for p in self.out_location])
                            writing_everthing_in_yaml.append(v + ': ' + '[' + pose_str + ']')
                            done.append(v)
                        
        with open('way_points.yaml', 'w') as f:
            f.write('\n'.join(writing_everthing_in_yaml))
    

def main(args=None):
    rclpy.init(args=args)
    client = ObjectPositionClient()

    try:
        #while rclpy.ok():
        future = client.send_request()
        rclpy.spin_until_future_complete(client, future)

        response = future.result()
        client.save_location_to_file(response)

        #time.sleep(10)  # Sleep for 10 seconds before sending the next request

    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
