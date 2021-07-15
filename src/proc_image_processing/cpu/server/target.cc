/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#include "target.h"

namespace proc_image_processing {

Target::Target()
    : center_(0, 0),
      dimension_(0, 0),
      angle_(0),
      header_(""),
      special_field_1_(""),
      special_field_2_("") {}

Target::Target(const std::string &header, int x, int y, float width,
               float height, float angle, int image_height, int image_width,
               const std::string &spec_field_1,
               const std::string &spec_field_2) {
    setTarget(header, x, y, width, height, angle, image_height, image_width,
              spec_field_1, spec_field_2);
}
}