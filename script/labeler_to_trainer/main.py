import argparse
import sys
import LabelMeAnnotation


annotation_list = LabelMeAnnotation.LabelMeAnnotationList(sys.argv[1])

print annotation_list