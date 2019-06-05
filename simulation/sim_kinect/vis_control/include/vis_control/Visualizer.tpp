#ifndef VISUALISER_TPP
#define VISUALISER_TPP

namespace visualization {

template <typename T, typename T2>
Visualizer<T, T2>::Visualizer(const std::string &frameId)
    : frameId(frameId), publishFn() {}

template <typename T, typename T2>
void Visualizer<T, T2>::publishWithHeader(const T2 &message) const {
  auto newMessage = message;
  publishWithHeader(newMessage);
}

template <typename T, typename T2>
void Visualizer<T, T2>::publishWithHeader(T2 &message) const {
  setHeader(message);

  if (publishFn) {
    publishFn(message);
  }
}

template <typename T, typename T2>
void Visualizer<T, T2>::publish(const T2 &message) const {
  if (publishFn) {
    publishFn(message);
  }
}

template <typename T, typename T2>
void Visualizer<T, T2>::setHeader(T2 &message) const {
  message.header.frame_id = frameId;
  message.header.stamp = ros::Time::now();
}

template <typename T, typename T2>
void Visualizer<T, T2>::setPublishFn(const Visualizer::publishFn_t &publishFn) {
  this->publishFn = publishFn;
}

} // namespace visualization

#endif // VISUALISER_TPP
