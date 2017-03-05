import xml.etree.cElementTree as ET
from os import listdir
from os.path import isfile, join

# For all files in repository
class LabelMeObjectAnnotation:
    def __init__(self, object):
        self.name = object.find('name').text
        self.occluded = object.find('occluded').text.find('yes') != -1
        #self.type = object.find('type').text
        self.polygon_points = []
        for pt in object.find('polygon').findall('pt'):
            self.polygon_points.append([int(pt.find('x').text),
                                       int(pt.find('y').text)])


class LabelMeImageAnnotation:
    def __init__(self, root):
        self.image_name = root.find('filename').text
        self.image_path = root.find('folder').text
        self.image_size = [int(root.find('imagesize').find('nrows').text),
                           int(root.find('imagesize').find('ncols').text)]
        self.object = []
        object_iter = root.findall('object')
        for obj in object_iter:
            object_name = obj.find('name').text
            if object_name.find("rotated") != -1:
                self.image_is_rotated = True
            else:
                self.object.append(LabelMeObjectAnnotation(obj))


class LabelMeAnnotationList:
    def __init__(self, annotation_path):
        self.directory_path = annotation_path
        self.annotation_list = []
        only_files = [f for f in listdir(self.directory_path) if isfile(join(self.directory_path, f))]
        only_xml_files = [f for f in only_files if f.endswith('.xml')]
        for xml_file in only_xml_files:
            tree = ET.parse(self.directory_path + '/' + xml_file)
            root = tree.getroot()
            self.annotation_list.append(LabelMeImageAnnotation(root))