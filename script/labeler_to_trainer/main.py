import sys
import LabelMeAnnotation
import TensorBoxAnnotation
import json

annotation_list = LabelMeAnnotation.LabelMeAnnotationList(sys.argv[1])

all_annotation = {}

for annotation in annotation_list.annotation_list:
    image_path = annotation.image_path + '/' + annotation.image_name
    # For red only
    object_in_image = []
    for obj in annotation.object:
        min_x = 1000000
        max_x = -1
        min_y = 10000
        max_y = -1
        for pt in obj.polygon_points:
            if pt[0] < min_x:
                min_x = pt[0]
            if pt[0] > max_x:
                max_x = pt[0]
            if pt[1] < min_y:
                min_y = pt[1]
            if pt[1] > max_y:
                max_y = pt[1]

        object_in_image.append(TensorBoxAnnotation.RectArea(obj.name, min_x, max_x, min_y, max_y))
    img_tensor_box_annotation = TensorBoxAnnotation.TrainingImage(image_path, object_in_image)
    all_annotation[image_path] = img_tensor_box_annotation.toDict()

print json.dumps(all_annotation, sort_keys=True, indent=4, separators=(',', ': '))
