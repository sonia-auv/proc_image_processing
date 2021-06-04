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
    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    using Ptr = std::shared_ptr<ObjectRankingData>;

    //============================================================================
    // P U B L I C   C / D T O R S

    ObjectRankingData() : area_rank_(0.0f), length_rank_(0.0f) {};

    virtual ~ObjectRankingData() {};

    //============================================================================
    // P U B L I C   M E T H O D S

    // Rank are grade from 0 to 1, 0 being the last, 1 being the first
    void SetAreaRank(float rank);

    void SetLengthRank(float rank);

    float GetAreaRank();

    float GetLengthRank();

  private:
    //============================================================================
    // P R I V A T E   M E M B E R S

    float area_rank_;
    float length_rank_;
  };

  //==============================================================================
  // I N L I N E   F U N C T I O N S   D E F I N I T I O N S

  //------------------------------------------------------------------------------
  //
  inline void ObjectRankingData::SetAreaRank(float rank) { area_rank_ = rank; }

  //------------------------------------------------------------------------------
  //
  inline void ObjectRankingData::SetLengthRank(float rank) {
    length_rank_ = rank;
  }

  //------------------------------------------------------------------------------
  //
  inline float ObjectRankingData::GetAreaRank() { return area_rank_; }

  //------------------------------------------------------------------------------
  //
  inline float ObjectRankingData::GetLengthRank() { return length_rank_; }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_OBJECT_RANKING_DATA_H_
