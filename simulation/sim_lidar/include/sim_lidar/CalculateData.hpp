#include <vector>

namespace calculate
{

  class Calculatedata
  {
      public:
    Calculatedata(std::vector<double> aTheta, std::vector<double> aDistance);
    /**
     * @brief Load the robot controller plugin, overrides the Load from
     * ModelPlugin
     * @param aParent: parent model
     * @param aSdf: the sdf (xml) in the robot model, within the <plugin>
     * element
     */
    virtual ~Calculatedata() = default;

    void calculateAverage();

      private:
    std::vector<double> mTheta_;
    std::vector<double> mDistance_;
    double mSumDistance_;
    double mSumTheta_;
  };

} // namespace calculate