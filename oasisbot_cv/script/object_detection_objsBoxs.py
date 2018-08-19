#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from object_msgs.msg import ObjectsInBoxes
from cv_bridge import CvBridge
import numpy as np
import os
import tensorflow as tf
import sys

class ObjectDetection():

    def __init__(self):
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        path_to_ckpt = rospy.get_param("~path_to_ckpt", "./ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb")
        path_to_labels = rospy.get_param("~path_to_labels", "./data/mscoco_label_map.pbtxt")
        self.min_score_thresh = rospy.get_param("~min_score_thresh", "0.5")
        self.draw_boxes_and_labels_on_image = rospy.get_param("~draw_boxes_and_labels_on_image", "False")

        input_rgb = rospy.get_param("~input_rgb", "/cv_camera/image_raw")

        self._cv_bridge = CvBridge()

        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        self.PATH_TO_CKPT = path_to_ckpt

        # List of the strings that is used to add correct label for each box.
        PATH_TO_LABELS = os.path.join(path_to_labels)

        NUM_CLASSES = 90

        self._session = tf.Session()
        self.create_graph()

        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        self._sub = rospy.Subscriber(input_rgb,
                                     Image,
                                     self.callback,
                                     queue_size=1)
        self._pub_visualize_boxes_and_labels_on_image_array = rospy.Publisher(
            'visualize_boxes_and_labels_on_image',
            Image,
            queue_size=1)
        self._pub_tfResult_to_objBoxs = rospy.Publisher(
            'objsBoxes',
            ObjectsInBoxes,
            queue_size=1)

    def create_graph(self):
        with tf.gfile.FastGFile(self.PATH_TO_CKPT, 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
            _ = tf.import_graph_def(graph_def, name='')


    def callback(self, image_msg):

         image_np = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
         image_np_expanded = np.expand_dims(image_np, axis=0)

         image_tensor = self._session.graph.get_tensor_by_name('image_tensor:0')
         boxes = self._session.graph.get_tensor_by_name('detection_boxes:0')
         scores = self._session.graph.get_tensor_by_name('detection_scores:0')
         classes = self._session.graph.get_tensor_by_name('detection_classes:0')
         num_detections = self._session.graph.get_tensor_by_name('num_detections:0')

         timer = rospy.Time.now().to_sec()
         # Actual detection.
         (boxes, scores, classes, num_detections) = self._session.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

         inference_time_ms = (rospy.Time.now().to_sec() - timer)*1000

         if self.draw_boxes_and_labels_on_image:
             # Visualization of the results of a detection.
             vis_util.visualize_boxes_and_labels_on_image_array(
                image_np,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                self.category_index,
                use_normalized_coordinates=True,
                min_score_thresh = self.min_score_thresh,
                line_thickness=4)

             ros_image = self._cv_bridge.cv2_to_imgmsg(image_np, "bgr8")
             self._pub_visualize_boxes_and_labels_on_image_array.publish(ros_image)

         # tfResult_to_objBoxs
         image_size = image_np.shape[0:2]
         objects_in_boxes = ObjectsInBoxes()
         FLAG_HAVE_BOX = tfResult_to_objBoxs.tfResult_to_objBoxs(
                                                        image_size,
                                                        np.squeeze(boxes),
                                                        np.squeeze(classes).astype(np.int32),
                                                        np.squeeze(scores),
                                                        self.category_index,
                                                        objects_in_boxes,
                                                        use_normalized_coordinates=True,
                                                        min_score_thresh = self.min_score_thresh,
                                                        imageHeader=image_msg.header,
                                                        inference_time_ms=inference_time_ms)

         if FLAG_HAVE_BOX:
             self._pub_tfResult_to_objBoxs.publish(objects_in_boxes)
         else:
             rospy.loginfo("No target detected.")
             self._pub_tfResult_to_objBoxs.publish(objects_in_boxes)

    def shutdown(self):
        rospy.loginfo("Stopping the tensorflow object detection...")
        rospy.sleep(1)

if __name__ == '__main__':

    rospy.init_node('object_detection_objsBoxes')
    # This is needed since the notebook is stored in the object_detection folder.
    tensorflow_object_detection_dir = rospy.get_param("~tensorflow_object_detection_dir", "./object_detection")
    os.chdir(tensorflow_object_detection_dir)
    sys.path.append("..")
    from object_detection.utils import label_map_util
    from object_detection.utils import visualization_utils as vis_util
    from utils import tfResult_to_objBoxs

    try:
        ObjectDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("object_detection_objsBoxes has stoped.")
