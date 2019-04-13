#ifndef __TZC_PUBLISHER_HPP__
#define __TZC_PUBLISHER_HPP__

#include "ros/ros.h"
#include "tzc_object.hpp"

namespace tzc_transport
{

class Topic;

template < class M >
class Publisher
{
public:
  Publisher() {
  }

  ~Publisher() {
  }

  Publisher(const Publisher & p) {
    *this = p;
  }

  Publisher & operator = (const Publisher & p) {
    pub_ = p.pub_;
    pobj_ = p.pobj_;
    return *this;
  }

  bool allocate(M & msg) const {
    if (!pobj_)
      return false;

    int len = msg.getLength();
#define RETRY 2
    // allocation shm message
    ShmMessage * ptr = NULL;
    // bad_alloc exception may occur if some ros messages are lost
    for (int attempt = 0; attempt < RETRY && ptr == NULL; attempt++) {
      try {
        ptr = (ShmMessage *)pobj_->allocate(sizeof(ShmMessage) + len);
      } catch (boost::interprocess::bad_alloc e) {
        if (!pobj_->releaseOne()) {
          ROS_WARN("failed to release message, abandon this message <%p>...", &msg);
          break;
        }
      }
    }
#undef RETRY
    if (ptr) {
      msg.fillArray(pobj_->convertAddress2Handle(ptr), ptr);
      msg.magic_ = ptr->magic_ = rand();

      //link the head of shm
      msg.pmsg_ = pobj_ -> pmsg_;
      msg.pmsg_ -> take();
      msg.name_ = pobj_ -> getName();
      msg.pshm_ = pobj_ -> pshm_;

      pobj_->addLast(ptr);
    }
    return (ptr != NULL);
  }

  void publish(const M & msg) const {
    if (!pobj_)
      return;
    pub_.publish(msg);
  }

  void publish(const typename M::ConstPtr & msg) const {
    if (!pobj_)
      return;
    pub_.publish(msg);
  }

  void shutdown() {
    pub_.shutdown();
  }

  std::string getTopic() const {
    return pub_.getTopic();
  }

  uint32_t getNumSubscribers() const {
    return pub_.getNumSubscribers();
  }

private:
  Publisher(const ros::Publisher & pub, const std::string & topic, uint32_t mem_size)
      : pub_(pub) {
    // change '/' in topic to '_'
    std::string t = topic;
    for (int i = 0; i < t.length(); i++)
      if (t[i] == '/')
        t[i] = '_';
    ShmManager * pshm = new ShmManager(boost::interprocess::open_or_create, t.c_str(), mem_size);
    pobj_ = ShmObjectPtr(new ShmObject(pshm, t));
  }

  ros::Publisher pub_;
  ShmObjectPtr   pobj_;

friend class Topic;

};

} // namespace tzc_transport

#endif // __TZC_PUBLISHER_HPP__

