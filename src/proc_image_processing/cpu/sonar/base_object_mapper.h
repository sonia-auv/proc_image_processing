/**
 * \file	BaseObjectMapperInterface.cc
 * \author	Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date	06/02/2016
 */


#ifndef PROC_IMAGE_PROCESSING_OBJECT_MAPPER_H
#define PROC_IMAGE_PROCESSING_OBJECT_MAPPER_H

#include "map_object.h"

// Interface for all object mapper (vision, sonar, hydrophones, etc)
class BaseObjectMapperInterface {
public:
    virtual void getMapObject(MapObjectVector &list) = 0;

    virtual void resetMapper() = 0;
};


#endif //PROC_IMAGE_PROCESSING_OBJECT_MAPPER_H
