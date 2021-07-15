/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_OBJECT_RANKING_DATA_H_
#define PROVIDER_VISION_ALGORITHM_OBJECT_RANKING_DATA_H_

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

    virtual ~ObjectRankingData() {};

    // Rank are grade from 0 to 1, 0 being the last, 1 being the first
    void setAreaRank(float rank);

      void setLengthRank(float rank);

      float getAreaRank();

      float getLengthRank();

  private:
    float area_rank_;
    float length_rank_;
  };

    inline void ObjectRankingData::setAreaRank(float rank) { area_rank_ = rank; }

    inline void ObjectRankingData::setLengthRank(float rank) {
        length_rank_ = rank;
    }

    inline float ObjectRankingData::getAreaRank() { return area_rank_; }

    inline float ObjectRankingData::getLengthRank() { return length_rank_; }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_OBJECT_RANKING_DATA_H_
