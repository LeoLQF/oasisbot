
import collections
from object_msgs.msg import ObjectInBox
from object_msgs.msg import Object
from sensor_msgs.msg import RegionOfInterest


def converts_to_ObjectInBoxe(image_size,
                               ymin,
                               xmin,
                               ymax,
                               xmax,
                               probability=(),
                               object_name_list=(),
                               use_normalized_coordinates=True,
                               do_rectify=False):
  """Adds a bounding box to an image.

  Bounding box coordinates can be specified in either absolute (pixel) or
  normalized coordinates by setting the use_normalized_coordinates argument.

  Each string in display_str_list is displayed on a separate line above the
  bounding box in black text on a rectangle filled with the input 'color'.
  If the top of the bounding box extends to the edge of the image, the strings
  are displayed below the bounding box.

  Args:
    image_size: image_size.
    ymin: ymin of bounding box.
    xmin: xmin of bounding box.
    ymax: ymax of bounding box.
    xmax: xmax of bounding box.
    object_name_list: list of strings to display in box
                      (each to be shown on its own line).
    use_normalized_coordinates: If True (default), treat coordinates
      ymin, xmin, ymax, xmax as relative to the image.  Otherwise treat
      coordinates as absolute.
  """
  im_height, im_width = image_size

  if use_normalized_coordinates:
    (left, right, top, bottom) = (xmin * im_width, xmax * im_width,
                                  ymin * im_height, ymax * im_height)

  else:
    (left, right, top, bottom) = (xmin, xmax, ymin, ymax)

  object = Object()
  object.object_name = object_name_list[0]
  object.probability = probability[0]

  roi = RegionOfInterest()
  roi.x_offset = int(left)
  roi.y_offset = int(top)
  roi.height = int(bottom-top)
  roi.width = int(right-left)
  roi.do_rectify = do_rectify

  object_in_box = ObjectInBox()
  object_in_box.object = object
  object_in_box.roi = roi

  return object_in_box


def tfResult_to_objBoxs(
    image_size,
    boxes,
    classes,
    scores,
    category_index,
    objects_in_boxes,
    use_normalized_coordinates=False,
    max_boxes_num=20,
    min_score_thresh=.5,
    agnostic_mode=False,
    skip_scores=False,
    skip_labels=False,
    do_rectify=False,
    imageHeader=None,
    inference_time_ms=None):
  """Overlay labeled boxes on an image with formatted scores and label names.

  This function groups boxes that correspond to the same location
  and creates a display string for each detection and overlays these
  on the image. Note that this function modifies the image in place, and returns
  that same image.

  Args:
    image_size: image size
    boxes: a numpy array of shape [N, 4]
    classes: a numpy array of shape [N]. Note that class indices are 1-based,
      and match the keys in the label map.
    scores: a numpy array of shape [N] or None.  If scores=None, then
      this function assumes that the boxes to be plotted are groundtruth
      boxes and plot all boxes as black with no classes or scores.
    category_index: a dict containing category dictionaries (each holding
      category index `id` and category name `name`) keyed by category indices.
    use_normalized_coordinates: whether boxes is to be interpreted as
      normalized coordinates or not.
    max_boxes_num: maximum number of boxes to visualize.  If None, draw
      all boxes.
    min_score_thresh: minimum score threshold for a box to be visualized
    agnostic_mode: boolean (default: False) controlling whether to evaluate in
      class-agnostic mode or not.  This mode will display scores but ignore
      classes.
    skip_scores: whether to skip score when drawing a single detection
    skip_labels: whether to skip label when drawing a single detection
    do_rectify:
    imageHeader:
    inference_time_ms:

  Returns:

  """
  # Create a display string (and color) for every box location, group any boxes
  # that correspond to the same location.
  box_to_object_name_map = collections.defaultdict(list)
  box_to_probability_map = collections.defaultdict(list)


  if not max_boxes_num:
      max_boxes_num = boxes.shape[0]
  for i in range(min(max_boxes_num, boxes.shape[0])):
    if scores is None or scores[i] > min_score_thresh:
      box = tuple(boxes[i].tolist())
      if scores is None:
        pass
      else:

        if not skip_labels:
          if not agnostic_mode:
            if classes[i] in category_index.keys():
              class_name = category_index[classes[i]]['name']
            else:
              class_name = 'N/A'
            box_to_object_name_map[box].append(str(class_name))

        if not skip_scores:
            probability = scores[i]
            box_to_probability_map[box].append(probability)

  FLAG_HAVE_BOX = False

  for box, probability in box_to_probability_map.items():
    ymin, xmin, ymax, xmax = box
    object_in_box = converts_to_ObjectInBoxe(image_size,
                                                ymin,
                                                xmin,
                                                ymax,
                                                xmax,
                                                probability=probability,
                                                object_name_list=box_to_object_name_map[box],
                                                use_normalized_coordinates=use_normalized_coordinates,
                                                do_rectify=do_rectify)

    objects_in_boxes.objects_vector.append(object_in_box)
    FLAG_HAVE_BOX = True

  if FLAG_HAVE_BOX:
      objects_in_boxes.header = imageHeader
      objects_in_boxes.inference_time_ms = inference_time_ms
      return FLAG_HAVE_BOX
  else:
      return FLAG_HAVE_BOX
