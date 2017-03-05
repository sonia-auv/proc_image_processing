import json

# TODO:
# -----

# Class
#	- Read and store (runtime Python obj) json file data
#	- Write json file from class

class RectArea:
	def __init__(self, x1, x2, y1, y2):
		self.x1 = x1
		self.x2 = x2
		self.y1 = y1
		self.y2 = y2

	def toJson(self):
		return json.dumps({'x1': self.x1, 'x2': self.x2, 'y1': self.y1, 'y2': self.y2},
							sort_keys=True, indent=4, separators=(',', ': '))

class TrainingImage:
	def __init__(self, image_path, rects):
		self.path = image_path
		self.rects = rects

	def toJson(self):
		# rects_string = ',\n'.join( [ _.toJson() for _ in self.rects ] )

		rects_string = "[\n"
		
		# for ( _, i ) in zip( self.rects, range(len(self.rects)) ):
		# 	if i:
		# 		rects_string += ",\n"
		# 	rects_string += _.toJson()

		rects_string += ',\n'.join( [ _.toJson() for _ in self.rects ])
		
		rects_string += "\n]"

		# print rects_string

		print json.dumps({'image_path': self.path, 'rects': rects_string },
							sort_keys=True, indent=4, separators=(',', ': '))

if __name__ == "__main__":
	rects = [ RectArea(5, 10, 15, 20), RectArea(25, 30, 35, 40)]
	image = TrainingImage("/usr/local/", rects)

	image.toJson()
	# with open("test_json", 'w') as target:
	# 	target.write( ',\n'.join( [ _.toJson() for _ in rects ] ) )

	# print ',\n'.join( [ _.toJson() for _ in rects ] )