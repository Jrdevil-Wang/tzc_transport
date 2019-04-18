#!/usr/bin/env python

from tzc_transport import tzc
def callback(img):
    tzc.loginfo('Image (%dx%dx%d) recieved: [%s] delay: [%5.5fms]'%
                (img.width, img.height, img.step, img.data, (tzc.get_rostime() - img.header_stamp).toSec()*1000))

def listener():
    tzc.init_node('listener', tzc.init_options.AnonymousName)
    lis = tzc.Subscriber(callback,'chatter')
    tzc.spin()

if __name__ == '__main__':
    listener()
