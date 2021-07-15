/**
 * \file	MapObject.h
 * \author	Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date	06/02/2016
 */

#ifndef PROC_IMAGE_PROCESSING_MAP_OBJECT_H
#define PROC_IMAGE_PROCESSING_MAP_OBJECT_H

#include <array>
#include <vector>


class MapObject {
public:

    std::array<float, 3> position_XYZ_;
};

typedef std::vector<MapObject> MapObjectVector;


#endif //PROC_IMAGE_PROCESSING_MAP_OBJECT_H
