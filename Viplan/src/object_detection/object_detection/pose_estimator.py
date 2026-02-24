#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import open3d as o3d
from std_msgs.msg import String
from dataclasses import dataclass
import cv2
import supervision as sv
from cv_bridge import CvBridge
import numpy as np
import numpy.linalg as la
from scipy.spatial.transform import Rotation
import os
from typing import List
from sklearn.cluster import KMeans
from segment_anything import sam_model_registry, SamPredictor
from groundingdino.util.inference import Model
from PIL import Image as PILImage
from object_detection_msgs.srv import GetObjectPosition
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

HOME = "/home/aregbs/ros2_ws/src/"
GROUNDING_DINO_CONFIG_PATH = os.path.join(HOME, "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py")
GROUNDING_DINO_CHECKPOINT_PATH = os.path.join(HOME, "weights", "groundingdino_swint_ogc.pth")
SAM_CHECKPOINT_PATH = os.path.join(HOME, "weights", "sam_vit_h_4b8939.pth")
SAM_ENCODER_VERSION  = "vit_h"


def exists(var):
    return not (var is None)


@dataclass
class ModelConfig:
    HOME: str 
    GROUNDING_DINO_CONFIG_PATH: str 
    GROUNDING_DINO_CHECKPOINT_PATH: str 
    SAM_CHECKPOINT_PATH: str 
    SAM_ENCODER_VERSION: str


class GibsonSAM:
    BOX_TRESHOLD = 0.40
    TEXT_TRESHOLD = 0.25
    MIN_IMAGE_AREA_PERCENTAGE = 0.002
    MAX_IMAGE_AREA_PERCENTAGE = 0.80
    APPROXIMATION_PERCENTAGE = 0.75
    def __init__(self,model_config, device: str = "cuda"):
        self.device = device
        self.model_config = model_config
        self.model_config = model_config
        self.box_annotator = sv.BoxAnnotator()
        self.mask_annotator = sv.MaskAnnotator()
        self.grounding_dino_model = Model(model_config_path=self.model_config.GROUNDING_DINO_CONFIG_PATH, model_checkpoint_path=self.model_config.GROUNDING_DINO_CHECKPOINT_PATH)
        self.sam = sam_model_registry[self.model_config.SAM_ENCODER_VERSION](checkpoint=self.model_config.SAM_CHECKPOINT_PATH).to(device=self.device)
        self.sam_predictor = SamPredictor(self.sam)


    @staticmethod
    def segment(sam_predictor: SamPredictor, image: np.ndarray, xyxy: np.ndarray) -> np.ndarray:
        sam_predictor.set_image(image)
        result_masks = []
        for box in xyxy:
            masks, scores, logits = sam_predictor.predict(
                box=box,
                multimask_output=True
            )
            index = np.argmax(scores)
            result_masks.append(masks[index])
        return np.array(result_masks)
    

    @staticmethod
    def enhance_class_name(class_names: List[str]) -> List[str]:
        return [
            f"{class_name}"
            for class_name
            in class_names
    ]

    
    def get_segmentation(self, image, class_names): 
        detections = self.grounding_dino_model.predict_with_classes(
                image=image,
                classes=self.enhance_class_name(class_names),
                box_threshold=self.BOX_TRESHOLD,
                text_threshold=self.TEXT_TRESHOLD
            )
        detections = detections[detections.class_id != None]
        
        detections.mask = self.segment(
            self.sam_predictor,
            image=cv2.cvtColor(image, cv2.COLOR_BGR2RGB),
            xyxy=detections.xyxy )
        return detections.mask,  [class_names[class_id]for class_id in detections.class_id]
        

config = ModelConfig(HOME=HOME, 
                     GROUNDING_DINO_CONFIG_PATH=GROUNDING_DINO_CONFIG_PATH,
                     GROUNDING_DINO_CHECKPOINT_PATH=GROUNDING_DINO_CHECKPOINT_PATH,
                     SAM_CHECKPOINT_PATH=SAM_CHECKPOINT_PATH,
                     SAM_ENCODER_VERSION=SAM_ENCODER_VERSION
                    )

class PoseEstimator:
    def __init__(self):
        self.segmentation = GibsonSAM(model_config= config)

        # self.panda_T_camera = panda_T_marker @ np.linalg.inv(camera_T_marker)
        # self.panda_T_camera = np.array([[-0.08529065,  0.44455829, -0.89199354,  0.85327619],
        #                                 [ 0.99646067,  0.02573891, -0.07725551,  0.1630355 ],
        #                                 [-0.00208637, -0.89538337, -0.445517  ,  0.47552947],
        #                                 [ 0.        ,  0.        ,  0.        ,  1.        ]])
       
        # self.width = 640
        # self.height = 480
        # self.cx= 320.5542297363281 
        # self.cy= 239.13539123535156
        # self.fx= 390.49041748046875 
        # self.fy = 390.49041748046875   
        self.panda_T_camera = np.array([[-0.03589593,  0.80345579, -0.59420499,  1.61978536],
                                        [ 0.99868421,  0.03197628, -0.04573568,  0.30729697],
                                        [-0.03593915, -0.59571846, -0.80330425,  1.12545326],
                                        [ 0.        ,  0.        ,  0.        ,  1.        ]])
                                        
        self.width = 1280
        self.height = 720
        self.cx = 640.9237060546875 
        self.cy = 358.5589904785156
        self.fx = 650.8173828125 
        self.fy = 650.8173828125

        self.depth_scale = 0.0010000000474974513

    @staticmethod
    def get_surface_normal_direction(pc):
        ## only in 2D, robot xy plane is parallel to table
        mean = pc[:,:2].mean(axis=0)
        minidx = np.linalg.norm(pc[:,:2]-mean).argmin()
        direction = pc[minidx,:2] - mean
        return direction/np.linalg.norm(direction)
    
    @staticmethod
    def get_grasp_pose(pc): # pc in robot frame 
        dim = pc.max(axis=0)-pc.min(axis=0)
        delta = dim.min()
        
        # Get Coordinate Axis for the object
        median = np.median(pc, axis=0)
        mean = np.mean(pc, axis=0)
        diff = pc[:,:2] - mean[:2] #along x and y axis 
        cov = np.cov(diff.T)
        eigvals, eigvecs = np.linalg.eig(cov)
        sort_idx = np.argsort(eigvals)

        # y-direction to be corrected using surface normal direction
        direction = PoseEstimator.get_surface_normal_direction(pc)
        cos_t = np.dot(direction, eigvecs[:,sort_idx[0]])
        if cos_t<0:
            eigvecs[:,sort_idx[0]] *= -1
        
        rot = np.eye(3)*-1 #grazp z is downwards, robot z is upwards
        rot[:2,:2] = eigvecs[:, [sort_idx[1], sort_idx[0]]] #grasp y is hand closing direction should be along small chnage
        
        mean_rot = rot.T@mean + np.array([0,delta*0.5,0])
        median_rot = rot.T@median + np.array([0,delta*0.5,0])

        # return rotmean, median, rot
        return rot@mean_rot, rot@median_rot, rot
    
    @staticmethod
    def object_prefilter(pc):
        obj_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc))
        cl, ind = obj_pcd.remove_statistical_outlier(nb_neighbors=5, std_ratio=1.0)
        return np.asarray(ind)


    def performObjectDetection(self, color_image, depth_image, object_names):
        depth_image = depth_image.astype(float)*self.depth_scale
        masks, classes = self.segmentation.get_segmentation(color_image, object_names)
        mask_image = np.ones((720,1280, 3), dtype=np.uint8)

        objects_dict = {}
        for mask, cls in zip(masks, classes):
            mask = mask.astype(np.uint8)*255
            kernel_size = 3  # You can change this based on how much reduction you want
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            
            # Perform the erosion operation
            eroded_mask = cv2.erode(mask, kernel, iterations=4)
            mask = eroded_mask==255


            indexes = np.argwhere(mask)
            mask_image = np.where(np.stack((mask, mask, mask), axis=2), color_image, mask_image)

            z = depth_image[indexes[:, 0], indexes[:, 1]]
            x = (indexes[:,1].astype(float) - self.cx)*z/self.fx
            y = (indexes[:,0].astype(float) - self.cy)*z/self.fy

            pc = np.hstack((x[:,None], y[:,None], z[:,None]))[z>0.3]
            pc_color = color_image[indexes[:,0], indexes[:,1], ::-1][z>0.3]


            kmean = KMeans(n_clusters=2).fit(pc)
            unq, counts = np.unique(kmean.labels_, return_counts=True)
            pc = pc[kmean.labels_==counts.argmax()]
            pc_color = pc_color[kmean.labels_==counts.argmax()]

            ind = self.object_prefilter(pc)
            if len(ind):
                pc = pc[ind]
                pc_color = pc_color[ind]

                ### Transform to Panda
                pc_panda = np.einsum("ij, kj->ki", self.panda_T_camera[:3,:3], pc) + self.panda_T_camera[:3,3]

                # Compute the grasp pose estimation
                mean_org, median_org, rot_org = self.get_grasp_pose(pc_panda.copy())

                # panda_hand_position = mean_org-np.array([0,0.0,-0.058])
                # if panda_hand_position[2]<0.53:
                #     panda_hand_position[2] = 0.53

                # objects_dict[cls] = {'translation': panda_hand_position.tolist(), 'quaternion': Rotation.from_matrix(rot_org).as_quat().tolist()}
                objects_dict[cls] = {'translation': mean_org.tolist(), 'quaternion': Rotation.from_matrix(rot_org).as_quat().tolist()}

        for name in object_names:
            if not (name in objects_dict):
                objects_dict[name] = {'translation': [0.0,0.0,0.0], 'quaternion': [0.0,0.0,0.0,1.0]}
                #print(f"\033[31m{name} not detected!\033[0m")

        PILImage.fromarray(mask_image[:,:,::-1]).save("segmentation_mask.png")
        return objects_dict


class PoseEstimatorNode (Node):
    def __init__(self):


        super().__init__('pose_estimator_service_service_node')
        self.pose_estimator = PoseEstimator()
        self.get_logger().info('Model Loaded!')
        self.color_image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_camera_callback, 10)
        self.depth_image_sub = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_camera_callback, 10)
        self.srv = self.create_service(GetObjectPosition, 'get_object_positions', self.pose_estimator_callback)
        self.color_image = None
        self.depth_image = None

    def color_camera_callback(self, msg):
        bridge = CvBridge()
        self.color_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        #cv2.imshow("Camera Image", self.color_image)
        #cv2.waitKey(1) 

    def depth_camera_callback(self, msg):
        bridge = CvBridge()
        self.depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(float)
        #cv2.imshow("Depth Image", depth_image_normalized)
        #cv2.waitKey(1)
            

    def pose_estimator_callback(self, request, response):
        if exists(self.color_image) and exists(self.depth_image):
            object_names = [obj.data for obj in request.objects_names]
            objects_dict = self.pose_estimator.performObjectDetection(self.color_image.copy(), self.depth_image.copy(), object_names)
            valid_objects = {}
            for k, v in objects_dict.items():
                #Get object name
                obj = String()
                obj.data = k
                #Get object position
                pose = Pose()
                x, y, z = v['translation']
                qx, qy, qz, qw = v['quaternion']
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                pose.orientation.x = qx
                pose.orientation.y = qy
                pose.orientation.z = qz
                pose.orientation.w = qw
    
                response.objects_names.append(obj)
                response.objects_positions.append(pose)
                if not (x==0.0 and z==0.0 and y==0):
                    self.get_logger().info('Object %s Position Pose:(%0.2f,%0.2f,%0.2f), Orientation:(%0.2f,%0.2f,%0.2f, %0.2f) ' 
                    % (k, x , y, z, qx , qy, qz,qw ))
                    valid_objects[k] = v
                else:
                    self.get_logger().error('Object %s not found!' %k)
    


            return response
        else:
            self.get_logger().error('Camera Image not available!')
            return response

def main(args=None):
    rclpy.init(args=args)
    pose_estimator_service = PoseEstimatorNode()
    rclpy.spin(pose_estimator_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()