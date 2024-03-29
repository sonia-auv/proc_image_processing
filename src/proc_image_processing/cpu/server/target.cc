#include "target.h"

namespace proc_image_processing {

    Target::Target() : center_(0, 0), dimension_(0, 0), angle_(0) {}

    Target::Target(
            const std::string &header,
            int x,
            int y,
            float width,
            float height,
            float angle,
            int image_height,
            int image_width,
            const std::string &spec_field_1,
            const std::string &spec_field_2
    ) {
        setTarget(header, x, y, width, height, angle, image_height, image_width,
                  spec_field_1, spec_field_2);
    }
}