#ifndef VISUALISER_HPP
#define VISUALISER_HPP

#include <functional>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace visualization {

/**
 * @brief Abstract helper class that can be implemented to visualise data in a
 * structured way
 * @author : Jelle Bouwhuis
 * @tparam T : incoming message type
 * @tparam T2 : outgoing message type
 */
template <typename T, typename T2> class Visualizer {
public:
  using publishFn_t = std::function<void(const T2 &)>;

  virtual ~Visualizer() = default;

  /**
   * @brief Abstract callback method used by the subscriber. implement this to
   * convert, add or remove data to publish
   * @author : Jelle Bouwhuis
   * @param message : Incoming message
   */
  virtual void callback(const T &message) = 0;

  /**
   * @brief Set the publish method of this visualizer
   * @author Bas Cop
   * @param publishFn the publish method to use
   */
  void setPublishFn(const publishFn_t &publishFn);

protected:
  /**
   * @brief Create a new object
   * @author Bas Cop
   * @param frameId name of the message frames
   */
  explicit Visualizer(const std::string &frameId);

  /**
   * @brief This method publishes a message if the publishFn attribute is set
   * but first sets the correct frameId and timestamp in its header.
   * @author : Jelle Bouwhuis
   * @param message : Message to publish
   */
  void publishWithHeader(T2 &message) const;

  /**
   * @brief This method creates a new instance of message to make it non-const
   * and calls publishWithHeader(message).
   * @author : Jelle Bouwhuis
   * @param message : Message to publish
   */
  void publishWithHeader(const T2 &message) const;

  /**
   * @brief set timestamp and frameId of a message
   * @author Bas Cop
   * @param message message to set the header from
   */
  void setHeader(T2 &message) const;

  /**
   * @brief : This method publishes a message without any additional actions if
   * the publishFn attribute is set.
   * @author : Bas Cop
   * @param message : Message to publish
   */
  void publish(const T2 &message) const;

  std::string frameId;

private:
  publishFn_t publishFn;
};

} // namespace visualization

#include "Visualizer.tpp"

#endif // VISUALISER_HPP
