#ifndef __TZC_SUBSCRIBER_HPP__
#define __TZC_SUBSCRIBER_HPP__

#include "ros/ros.h"
#include "tzc_object.hpp"

namespace tzc_transport
{

class Topic;
template < class M >
class Subscriber;

template < class M >
class SubscriberCallbackHelper
{
  typedef void (*Func)(const typename M::ConstPtr &);

public:
  ~SubscriberCallbackHelper() {
  }

  void callback(const typename M::Ptr & msg) {
    if (!pobj_) {
      // If subscriber runs first, it will not die due to these code.
      ShmManager * pshm = new ShmManager(boost::interprocess::open_only, name_.c_str());
      pobj_ = ShmObjectPtr(new ShmObject(pshm, name_));
    }

    ShmMessage * ptr = pobj_->convertHandle2Address(msg->handle_);
    // if the magic number don't match, the message has been released by now
    if (ptr->magic_ != msg->magic_)
      return;
    ptr->take();

    msg->fillArray(msg->handle_, ptr);
    // call user callback
    fp_(msg);
  }

private:
  SubscriberCallbackHelper(const std::string &topic, Func fp)
    : pobj_(), name_(topic), fp_(fp) {
    // change '/' in topic to '_'
    for (int i = 0; i < name_.length(); i++)
      if (name_[i] == '/')
        name_[i] = '_';
  }

  ShmObjectPtr pobj_;
  std::string name_;
  Func fp_;

friend class Topic;

};

template < class M >
class Subscriber
{
public:
  Subscriber() {
  }

  ~Subscriber() {
  }

  Subscriber(const Subscriber & s) {
    *this = s;
  }

  Subscriber & operator = (const Subscriber & s) {
    sub_ = s.sub_;
    phlp_ = s.phlp_;
    return *this;
  }

  void shutdown() {
    sub_.shutdown();
  }

  std::string getTopic() const {
    return sub_.getTopic();
  }

  uint32_t getNumPublishers() const {
    return sub_.getNumPublishers();
  }

private:
  Subscriber(const ros::Subscriber & sub, SubscriberCallbackHelper< M > * phlp)
      : sub_(sub), phlp_(phlp) {
  }

  ros::Subscriber sub_;
  boost::shared_ptr< SubscriberCallbackHelper< M > > phlp_;

friend class Topic;

};

} // namespace tzc_transport

#endif // __TZC_SUBSCRIBER_HPP__

