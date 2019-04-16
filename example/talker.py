#!/usr/bin/env python

from tzc_transport import tzc
def talker():
    pub = tzc.Publisher('chatter', 30, 100*1024*1024)
    tzc.init_node('talker', tzc.init_options.AnonymousName)
    rate = tzc.Rate(30)
    count = 0
    while not tzc.is_shutdown():
        img = tzc.image()  
        img.width  = 1920
        img.height = 1080
        img.step = 3
        img.data_resize(1920 * 1080 * 3)
        if pub.allocate(img):
            img.data = 'image # %5d ...'% count
            
            img.header_stamp = tzc.get_rostime()
            tzc.loginfo('info :[%s]'%img.data)
            pub.publish(img)
        tzc.spinOnce()
        rate.sleep()
        count += 1
if __name__ == '__main__':
    talker()
