/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_EQUALIZE_H_
#define PROVIDER_VISION_FILTERS_EQUALIZE_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class Equalize : public Filter {
  public:
    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    using Ptr = std::shared_ptr<Equalize>;

    //============================================================================
    // P U B L I C   C / D T O R S

    explicit Equalize(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      enable_("enable", false, &parameters_) {
      SetName("Equalize");
    }

    virtual ~Equalize() {}

    //============================================================================
    // P U B L I C   M E T H O D S

    virtual void Execute(cv::Mat& image) {
      if (enable_()) {
        cv::equalizeHist(image, image);
      }
    }


  private:
    //============================================================================
    // P R I V A T E   M E M B E R S

    Parameter<bool> enable_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_EQUALIZE_H_
