/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_RANGED_PARAMETER_H_
#define PROC_IMAGE_PROCESSING_RANGED_PARAMETER_H_

#include <sonia_common/macros.h>
#include "parameter.h"
#include <iostream>
#include <string>
#include <vector>

namespace proc_image_processing {

    template<typename Tp_>
    class RangedParameter : public Parameter<Tp_> {
    public:
        using Ptr = std::shared_ptr<RangedParameter<Tp_>>;

        explicit RangedParameter(const std::string &name, const Tp_ &value,
                                 const Tp_ &min, const Tp_ &max,
                                 std::vector<ParameterInterface *> *vector,
                                 const std::string &description = "")
                : Parameter<Tp_>(name, value, vector, description),
                  min_(min),
                  max_(max) {
        }

        virtual ~RangedParameter() = default;

        const Tp_ &getMin() const { return min_; }

        void setMin(const Tp_ &min) { min_ = min; }

        const Tp_ &getMax() const { return max_; }

        void setMax(const Tp_ &max) { max_ = max; }

        std::string toString() const override {
            std::stringstream ss;
            ss << Parameter<Tp_>::getName() << Parameter<Tp_>::SEPARATOR;
            ss << Parameter<Tp_>::getType() << Parameter<Tp_>::SEPARATOR;
            ss << Parameter<Tp_>::getStringValue() << Parameter<Tp_>::SEPARATOR;
            ss << details::StringConvertor<Tp_>::getString(min_)
               << Parameter<Tp_>::SEPARATOR;
            ss << details::StringConvertor<Tp_>::getString(max_)
               << Parameter<Tp_>::SEPARATOR;
            ss << Parameter<Tp_>::getDescription();
            return ss.str();
        }

    protected:
        Tp_ min_;
        Tp_ max_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_RANGED_PARAMETER_H_
