#ifndef __TZC_VECTOR_HPP__
#define __TZC_VECTOR_HPP__

namespace tzc_transport
{

template < class T >
class vector
{
public:
  vector() : size_(0), ptr_(NULL) { }

  vector(const vector & v) {
    *this = v;
  }

  ~vector() { }

  vector & operator = (const vector & v) {
    size_ = v.size_;
    ptr_ = v.ptr_;
    return *this;
  }

  T & operator [] (size_t i) const {
    return ptr_[i];
  }

  T * data() const {
    return ptr_;
  }

  size_t size() const {
    return size_;
  }

  void resize(size_t s) {
    size_ = s;
  }

public:
  size_t  size_;
  T *     ptr_;
}; // class vector

} // tzc_transport

namespace ros
{

namespace serialization
{

  template < class V > struct Serializer< ::tzc_transport::vector < V > >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.size_);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  };

} // namespace serialization

} // namespace ros

#endif // __TZC_VECTOR_HPP__

