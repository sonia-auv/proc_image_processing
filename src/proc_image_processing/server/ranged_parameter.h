/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROVIDER_VISION_RANGED_PARAMETER_H_
#define PROVIDER_VISION_RANGED_PARAMETER_H_

#include <sonia_common/macros.h>
#include <proc_image_processing/server/parameter.h>
#include <iostream>
#include <string>
#include <vector>

namespace proc_image_processing {

  template <typename Tp_>
  class RangedParameter : public Parameter<Tp_> {
  public:
    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    using Ptr = std::shared_ptr<RangedParameter<Tp_>>;

    //============================================================================
    // P U B L I C   C / D T O R S

    explicit RangedParameter(const std::string& name, const Tp_& value,
      const Tp_& min, const Tp_& max,
      std::vector<ParameterInterface*>* vector,
      const std::string& description = "")
      : Parameter<Tp_>(name, value, vector, description),
      min_(min),
      max_(max) {    
}

    virtual ~RangedParameter() = default;

    //============================================================================
    // P U B L I C   M E T H O D S

    const Tp_& GetMin() const { return min_; }

    void SetMin(const Tp_& min) { min_ = min; }

    const Tp_& GetMax() const { return max_; }

    void SetMax(const Tp_& max) { max_ = max; }

    std::string ToString() const override {
      std::stringstream ss;
      ss << Parameter<Tp_>::GetName() << Parameter<Tp_>::SEPARATOR;
      ss << Parameter<Tp_>::GetType() << Parameter<Tp_>::SEPARATOR;
      ss << Parameter<Tp_>::GetStringValue() << Parameter<Tp_>::SEPARATOR;
      ss << details::StringConvertor<Tp_>::GetString(min_)
        << Parameter<Tp_>::SEPARATOR;
      ss << details::StringConvertor<Tp_>::GetString(max_)
        << Parameter<Tp_>::SEPARATOR;
      ss << Parameter<Tp_>::GetDescription();
      return ss.str();
    }

  protected:
    //============================================================================
    // P R O T E C T E D   M E M B E R S

    Tp_ min_;

    Tp_ max_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_RANGED_PARAMETER_H_
