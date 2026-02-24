#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import os
import base64
from openai import OpenAI
import pprint
from functools import lru_cache
import re
from cv_bridge import CvBridge
import base64
import requests
import cv2
import numpy as np
from PIL import Image as PILImage
from sensor_msgs.msg import Image
from viplan_msgs.srv import CreateProblemFile
MODEL="gpt-4o"

class Vlm:
    def __init__(self, system_prompt, instruction, output_file = "problem.pddl"):

        self.output_file = output_file
        self.prompt = None
        self.instruction = None
        with open(system_prompt, "r") as file:
            # Read the entire content of the file
            self.prompt = file.read()

        key = "src/viplan/key.txt"
        
        with open(key, "r") as file:
            # Read the entire content of the file
            key_file_content = file.read()
            key_file_content = key_file_content.replace("\n","")

        with open(instruction, "r") as command_file:
            # Read the entire content of the file
            self.instruction = command_file.read()
        
        self.openai_client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY", key_file_content))

    # Function to encode the image
    def encode_image(self, image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    # Path to your image
    @lru_cache()
    def analyze_image(self,img_path, user_prompt, prompt):
        base64_image = self.encode_image(img_path)
        response = self.openai_client.chat.completions.create(
        model=MODEL,
        messages=[
            {"role": "system", "content": prompt},
            {"role": "user", "content": [
            {"type": "text", "text": user_prompt},
            {"type": "image_url", "image_url": {
                "url": f"data:image/png;base64,{base64_image}"}
            }
            ]}
        ],
        temperature=0.0,
         )
        
        response = response.choices[0].message.content
        return response
    
    def setOutputFilename(self, name):
        self.output_file = name
    
    def call_vlm(self,image):
        response = self.analyze_image(image, self.instruction, self.prompt)
        return response
    
    def generate_problem_file(self,response):
        
        parse = self.export_file(response, self.output_file)
        print(parse)
        return self.output_file
    

    def export_file(self,input_text, output_file):
        """
        Extracts the PDDL section from a string where the PDDL code is embedded within other text
        and saves it to a specified file path, replacing any existing content.

        Parameters:
        input_text (str): The string from which to extract the PDDL content.
        output_file (str): The path to the file where the PDDL content should be saved.
        
        Returns:
        str: Confirmation message about saving, or an error message if not found.
        """

        start_index = input_text.find("(define")
        if start_index == -1:
            return "PDDL content not found."

        end_index = input_text.rfind(")") + 1
        if end_index == 0:
            return "PDDL content not found."

        pddl_content = input_text[start_index:end_index]

        ### Creating an image with GPT output
        lines = pddl_content.split('\n')
        text_image = np.ones((1200, 800), dtype=np.uint8)*255
        text_size = 0.65
        text_width = 1
        text_color = (0,0,255)
        text_font = cv2.FONT_HERSHEY_SIMPLEX
        text_line = cv2.LINE_AA
        text_height = 50

        for line in lines:
            text_image = cv2.putText(text_image, line, (5, text_height), text_font, text_size, text_color, text_width, text_line)
            text_height += 25
        PILImage.fromarray(text_image).save("plan_image.png")



        with open(output_file, 'w') as file:
            file.write(pddl_content)

        return f"PDDL content successfully saved to {output_file}"
   

import argparse
class VlmNode(Node):

    def __init__(self):
        super().__init__("vlm")
        
        self.image_count = 201
        self.srv = self.create_service(CreateProblemFile, 'compute_problem_file', self.compute_problem_file_callback)
        self.image = None
        self.image_sub =  self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_listener_callback,
            10)

        self.declare_parameter("generate_problem_file",True)
        
        
        
    def compute_problem_file_callback(self, request, response):
        generate_problem_file = self.get_parameter('generate_problem_file').get_parameter_value().bool_value
        if self.image is None:
            self.get_logger().error(f'Image is not available')
        else: 
            try:
                bridge = CvBridge()
                self.get_logger().info(f'Creating problem')
                # Convert ROS Image message to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
                
                # Create a file name
                image_name = f'problem_{self.image_count}.png'
                outfile = f'./dataset/problems/problem_{self.image_count}.pddl'
                image_path = os.path.join("./dataset/images", image_name)
                image = image_path
                # Save the image
                cv2.imwrite(image_path, cv_image)
                self.vlm = Vlm(request.prompt.data, request.instruction.data, outfile)
                if generate_problem_file:
                    resp = self.vlm.call_vlm(image)
                    start = self.image_count
                    final = self.image_count + 3
                    for i in range(start,final):
                        self.vlm.output_file = f'./dataset/problems/problem_{i}.pddl'
                        self.get_logger().info('Problem %s file created!' %self.vlm.output_file)
                        filename = self.vlm.generate_problem_file(resp)
                    msg = String()
                    msg.data = filename
                    response.ouput_file = msg
                    
                else:
                    self.get_logger().info('File generation skipped as generate_file is set to False')
                self.image_count +=3   
            except Exception as e:
                self.get_logger().error(f'Could not convert image: {e}')



        return response



    def image_listener_callback(self, msg):
        self.image = msg


def main(args=None):
    rclpy.init(args=args)

    vlm = VlmNode()
    try:
        rclpy.spin(vlm)
    except KeyboardInterrupt:
        pass
    
    vlm.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()




# if __name__ == '__main__':
#     prompt = "/home/aregbs/Desktop/LangRobPlan/VLM_LMM/scripts/hanoi_prompt.txt"
#     instr = "/home/aregbs/Desktop/LangRobPlan/VLM_LMM/Prompt_vlm/hanoi/instructions/problem.txt"
#     img = "/home/aregbs/Desktop/LangRobPlan/VLM_LMM/Prompt_vlm/hanoi/observation/problem1.png"
#     viplan = Vlm(prompt, instr)
    
