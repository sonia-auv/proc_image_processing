
#include <proc_image_processing/filters/filter.h>

namespace proc_image_processing
{

    class AbstractFilter : public IFilter
    {
    public:
        explicit AbstractFilter(const GlobalParamHandler &globalParams)
            : IFilter(globalParams),
              enable_("Enable", false, &parameters_),
        {}

        virtual void Execute(cv::Mat &image)
        {
            if (enable_()) {
                ProcessImage(image);
            }
        }

    private:
        Parameter<bool> enable_;

    };
}  // namespace proc_image_processing