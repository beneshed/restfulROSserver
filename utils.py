import rospy
import std_msgs.msg
import genpy
from rosservice import get_service_class_by_name, ROSServiceException
import rosmsg
from rostopic import CallbackEcho, _rostopic_echo, ROSTopicIOException, _check_master, get_topic_class
import roslib
import socket
import time

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
        return echo_util(topic, callback_echo)
    except socket.error:
        raise ROSTopicIOException("Network communication failed. Most likely failed to communicate with master.\n")


def echo_util(topic, callback_echo):
        _check_master()
        rospy.init_node('rostopic', anonymous=True, disable_signals=True)
        msg_class, real_topic, msg_eval = get_topic_class(topic, blocking=True)
        if msg_class is None:
            # occurs on ctrl-C
            return
        callback_echo.msg_eval = msg_eval

        # extract type information for submessages
        type_information = None
        if len(topic) > len(real_topic):
            subtopic = topic[len(real_topic):]
            subtopic = subtopic.strip('/')
            if subtopic:
                fields = subtopic.split('/')
                submsg_class = msg_class
                while fields:
                    field = fields[0].split('[')[0]
                    del fields[0]
                    index = submsg_class.__slots__.index(field)
                    type_information = submsg_class._slot_types[index]
                    if fields:
                        submsg_class = roslib.message.get_message_class(type_information.split('[', 1)[0])
                        if not submsg_class:
                            raise ROSTopicIOException("Cannot load message class for [%s]. Are your messages built?" % type_information)

        use_sim_time = rospy.get_param('/use_sim_time', False)
        sub = rospy.Subscriber(real_topic, msg_class, callback_echo.callback, {'topic': topic, 'type_information': type_information})

        if use_sim_time:
            # #2950: print warning if nothing received for two seconds

            timeout_t = time.time() + 2.
            while time.time() < timeout_t and \
                    callback_echo.count == 0 and \
                    not rospy.is_shutdown() and \
                    not callback_echo.done:
               rospy.rostime.wallsleep(0.1)

            if callback_echo.count == 0 and \
                    not rospy.is_shutdown() and \
                    not callback_echo.done:
                raise ROSTopicIOException("WARNING: no messages received and simulated time is active.\nIs /clock being published?\n")

        while not rospy.is_shutdown() and not callback_echo.done:
            rospy.rostime.wallsleep(0.1)
