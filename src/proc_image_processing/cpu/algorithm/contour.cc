/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#include "contour.h"

#include <utility>

namespace proc_image_processing {

    Contour::Contour(std::vector<cv::Point> ctr) : contour_(std::move(ctr)) {}

    Contour::Contour(const cv::RotatedRect &rect) {
        cv::Point2f pts[4];
        rect.points(pts);

        for (auto &pt : pts) {
            contour_.push_back(pt);
        }
    }

}  // namespace proc_image_processing
