import sys
sys.path.append('/usr/local/python')
from openpose import pyopenpose as op
import numpy as np
import cv2

class OpenPoseWrapper:
    '''OpenPose ROS2 Wrapper'''
    params: dict

    # BODY_25 model keypoints (25 body parts consisting of COCO + foot)
    #     {0,  "Nose"},
    #     {1,  "Neck"},
    #     {2,  "RShoulder"},
    #     {3,  "RElbow"},
    #     {4,  "RWrist"},
    #     {5,  "LShoulder"},
    #     {6,  "LElbow"},
    #     {7,  "LWrist"},
    #     {8,  "MidHip"},
    #     {9,  "RHip"},
    #     {10, "RKnee"},
    #     {11, "RAnkle"},
    #     {12, "LHip"},
    #     {13, "LKnee"},
    #     {14, "LAnkle"},
    #     {15, "REye"},
    #     {16, "LEye"},
    #     {17, "REar"},
    #     {18, "LEar"},
    #     {19, "LBigToe"},
    #     {20, "LSmallToe"},
    #     {21, "LHeel"},
    #     {22, "RBigToe"},
    #     {23, "RSmallToe"},
    #     {24, "RHeel"},
    #     {25, "Background"}
    # };
    
    def __init__(self, openpose_path: str = '/home/walleed/openpose/'):
        self.params = dict()
        self.params['model_folder'] = '/home/walleed/openpose/models'
        self.params['net_resolution'] = '-1x320'

        # Config OpenWrapper
        self.op_wrapper = op.WrapperPython()
        self.op_wrapper.configure(self.params)
        self.op_wrapper.start()
    
    def body_from_image(self, image: np.ndarray) -> op.Datum:
        datum = op.Datum()
        datum.cvInputData = image
        self.op_wrapper.emplaceAndPop(op.VectorDatum([datum]))

        return datum
