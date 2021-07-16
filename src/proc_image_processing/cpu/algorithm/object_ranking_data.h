/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_RANKING_DATA_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_RANKING_DATA_H_

#include <memory>

namespace proc_image_processing {

    // Basic container class. It holds information about the ranking
    // of certain caracteristic of an object compare to the other.
    // In other word, it contains the position resulting of the sorting
    // by ObjectRanker.
    class ObjectRankingData {
    public:
        using Ptr = std::shared_ptr<ObjectRankingData>;

        ObjectRankingData() : area_rank_(0.0f), length_rank_(0.0f) {};

        virtual ~ObjectRankingData() = default;

        // Rank are grade from 0 to 1, 0 being the last, 1 being the first
        void setAreaRank(float rank);

        void setLengthRank(float rank);

        [[maybe_unused]] float getAreaRank() const;

        [[maybe_unused]] float getLengthRank() const;

    private:
        float area_rank_, length_rank_;
    };

    inline void ObjectRankingData::setAreaRank(float rank) { area_rank_ = rank; }

    inline void ObjectRankingData::setLengthRank(float rank) { length_rank_ = rank; }

    [[maybe_unused]] inline float ObjectRankingData::getAreaRank() const { return area_rank_; }

    [[maybe_unused]] inline float ObjectRankingData::getLengthRank() const { return length_rank_; }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_RANKING_DATA_H_
