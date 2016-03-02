import rospy
import std_msgs.msg
import genpy
from rosservice import get_service_class_by_name, ROSServiceException
import rosmsg
from rostopic import CallbackEcho, _rostopic_echo
import socket
from restfulROSserver import ROSUnavailable

def call_service_util(service_name, service_args, service_class=None):
    rospy.init_node('rosservice', anonymous=True, disable_signals=True)

    if service_class is None:
        service_class = get_service_class_by_name(service_name)
    request = service_class._request_class()
    try:
        now = rospy.get_rostime()
        keys = {'now': now, 'auto': std_msgs.msg.Header(stamp=now) }
        genpy.message.fill_message_args(request, service_args, keys=keys)
    except genpy.MessageException as e:
        def argsummary(args):
            if type(args) in [tuple, list]:
                return '\n'.join([' * %s (type %s)'%(a, type(a).__name__) for a in args])
            else:
                return ' * %s (type %s)'%(args, type(args).__name__)

        raise ROSServiceException("Incompatible arguments to call service:\n%s\nProvided arguments are:\n%s\n\nService arguments are: [%s]"%(e, argsummary(service_args), genpy.message.get_printable_message_args(request)))
    try:
        return request, rospy.ServiceProxy(service_name, service_class)(request)
    except rospy.ServiceException as e:
        raise ROSServiceException(str(e))
    except genpy.SerializationError as e:
        raise ROSServiceException("Unable to send request. One of the fields has an incorrect type:\n"+\
                                      "  %s\n\nsrv file:\n%s"%(e, rosmsg.get_srv_text(service_class._type)))
    except rospy.ROSSerializationException as e:
        raise ROSServiceException("Unable to send request. One of the fields has an incorrect type:\n"+\
                                      "  %s\n\nsrv file:\n%s"%(e, rosmsg.get_srv_text(service_class._type)))


def echo_publisher(topic, count):
    callback_echo = CallbackEcho(topic, None, plot=False,
                                 filter_fn=None,
                                 echo_clear=False, echo_all_topics=False,
                                 offset_time=False, count=count,
                                 field_filter_fn=None, fixed_numeric_width=None)
    try:
        return _rostopic_echo(topic, callback_echo, bag_file=None)
    except socket.error:
        return ROSUnavailable
