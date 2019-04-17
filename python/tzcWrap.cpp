#include <boost/python.hpp>
#include "tzc_transport/tzc_topic.hpp"
#include "sensor_msgs/tzc_Image.hpp"

typedef tzc_transport::sensor_msgs::Image Msg;

static tzc_transport::Topic * topic_ptr = nullptr;
static ros::NodeHandle * NodeHandle_ptr = nullptr;


class RateWrap {

public:
    RateWrap(uint32_t hz) :
            loop_rate(hz) {

    }
    void sleep() {
        loop_rate.sleep();
    }

private:
    ros::Rate loop_rate;
};

class ImageWrap {
    typedef tzc_transport::sensor_msgs::Image Msg;
public:
    void set_width(uint32_t w) {
        img.width = w;
    }
    const uint32_t get_width() const {
        return img.width;
    }
    void set_height(uint32_t h) {
        img.height = h;
    }
    const uint32_t get_height() const {
        return img.height;
    }
    void set_step(uint32_t s) {
        img.step = s;
    }
    const uint32_t get_step() const {
        return img.step;
    }
    void data_resize(size_t s) {
        img.data.resize(s);
    }
    void set_header_stamp(ros::Time& time) {
        img.header.stamp = time;
    }
    const ros::Time get_header_stamp() const {
        return img.header.stamp;
    }
    void set_data(const std::string& data) {
        snprintf((char *) img.data.data(), img.data.size_, "%s", data.c_str());
//        memcpy(img.data.data(), data.c_str(), data.length()*sizeof(char));
    }
    const char* get_data() const {
        return (char *) img.data.data();
    }
    ImageWrap& operator=(const Msg& i) {
        img = i;
        return *this;
    }

    Msg& get_img() {
        return img;
    }

private:
    Msg img;
};

class PublisherWrap {

public:
    PublisherWrap(const std::string& name, uint32_t queue_size = 30, uint32_t mem_size = 100 * 1024 * 1024) :
            topic(name),
            queue_size(queue_size),
            mem_size(mem_size),
            pub_has_init(false),
            pub()
    {
        if (topic_ptr) {
            pub_init();
        }
    }

    bool allocate (ImageWrap& i){
        if (!pub_has_init) {
            pub_init();
        }
        return pub.allocate(i.get_img());
    }

    void publish(ImageWrap& i) {
        if (!pub_has_init) {
            pub_init();
        }
        pub.publish(i.get_img());
    }

private:
    std::string topic;
    uint32_t queue_size;
    uint32_t mem_size;
    bool pub_has_init;
    tzc_transport::Publisher<Msg> pub;
    void pub_init() {
        pub = topic_ptr->advertise<Msg>(topic, queue_size, mem_size);
        pub_has_init = true;
    }
};

class SubscriberWrap {

public:

    SubscriberWrap(boost::python::object& c, const std::string& s, uint32_t queue_size = 30):
    callable_(c),
    sub_(topic_ptr->subscribe<Msg, SubscriberWrap>(s, queue_size, &SubscriberWrap::callback, this))
    {
        PyEval_InitThreads();
    }

    void callback(const Msg::ConstPtr & msg) {
        ImageWrap* img = new ImageWrap;
        *img = *msg;
        PyGILState_STATE state = PyGILState_Ensure();
        boost::python::call<void>(callable_.ptr(), boost::python::ptr(img));
        PyGILState_Release(state);
        delete img;
    }
private:
    boost::python::object& callable_;
    tzc_transport::Subscriber<Msg> sub_;
};

void init_node(int argc,char** argv, const std::string& name, uint32_t options) {
    ros::init(argc, argv, name, options);
    NodeHandle_ptr = new ros::NodeHandle;
    topic_ptr = new tzc_transport::Topic(*NodeHandle_ptr);
}
void init_node_1(const std::string& name, uint32_t options){
    init_node(0, new char*, name, options);
}

bool is_shutdown() {
    if (ros::ok())
        return false;
    else {
        delete topic_ptr;
        topic_ptr = NULL;
        delete NodeHandle_ptr;
        NodeHandle_ptr = NULL;
        return true;
    }
}

void loginfo(const char* s){
    ROS_INFO("%s", s);
}
void spin() {
    ros::spin();
}

void spinOnce(){
    ros::spinOnce();
}

const ros::Time get_rostime() {
    return ros::Time::now();
}

BOOST_PYTHON_MODULE(tzc)
{
    using namespace boost::python;
//boost::shared_ptr<PublisherWrap>
    class_< PublisherWrap >("Publisher",
            "arg1 is topic."
            "arg2 is queue_size,optional,default 30."
            "arg3 is mem_size,optional,default 100 * 1024 * 1024.",
            init< const std::string&, optional<uint32_t, uint32_t> >())
            .def("allocate", &PublisherWrap::allocate)
            .def("publish", &PublisherWrap::publish)
            ;
//,boost::shared_ptr<SubscriberWrap>
    class_< SubscriberWrap >("Subscriber",
            "arg1 is callback."
            "arg2 is topic."
            "arg3 is queue_size,optional,default 30.",
            init< boost::python::object& ,const std::string&, optional<uint32_t> >())
            ;
    class_< RateWrap, boost::shared_ptr<RateWrap> >("Rate", "arg1 is frequency.",init<uint32_t>())
            .def("sleep", &RateWrap::sleep)
            ;
//boost::shared_ptr<Imagewrap>
    class_< ImageWrap >("image")
            .add_property("width", &ImageWrap::get_width, &ImageWrap::set_width)
            .add_property("height", &ImageWrap::get_height, &ImageWrap::set_height)
            .add_property("step", &ImageWrap::get_step, &ImageWrap::set_step)
            .add_property("data", &ImageWrap::get_data, &ImageWrap::set_data)
            .add_property("header_stamp", &ImageWrap::get_header_stamp, &ImageWrap::set_header_stamp)
            .def("data_resize", &ImageWrap::data_resize)
            ;

    class_< ros::Time >("Time", no_init)
            .def("toSec", &ros::Time::toSec)
            .def(self - self)
            .def(self < self)
            .def(self > self)
            ;
    class_< ros::Duration >("Duration", no_init)
            .def("toSec", &ros::Duration::toSec)
            .def(self + self)
            .def(self - self)
            .def(self < self)
            .def(self > self)
                    ;
    def("get_rostime", &get_rostime);


    enum_< ros::init_options::InitOption >("init_options")
        .value("NoSigintHandler", ros::init_options::NoSigintHandler)
        .value("AnonymousName", ros::init_options::AnonymousName)
        .value("NoRosout", ros::init_options::NoRosout)
        ;

    def("init_node", &init_node);
    def("init_node", &init_node_1);
    def("is_shutdown", &is_shutdown);
    def("spin", &spin);
    def("spinOnce", &spinOnce);
    def("loginfo", loginfo);
}

