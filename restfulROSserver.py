from flask import request
from flask.ext.api import FlaskAPI, status

from rosapi import proxy, objectutils
import rosgraph
import roslib
from rostopic import _check_master, create_publisher, argv_publish, ROSTopicException, \
    _resource_name_package
import rospy

from flask.ext.api.exceptions import APIException

from json import JSONEncoder
import yaml


class ROSUnavailable(APIException):
    status_code = 500
    detail = 'ROS unavailable, reconfigure.'


app = FlaskAPI(__name__)


@app.route('/', methods=['GET'])
def root():
    urls = {
        'topics':
            {
                'url': '/topics',
                'methods': ['GET']
            }
    }
    return urls


@app.route('/topics', methods=['GET'])
def list_topics():
    """
    List all topics from ROS node
    :return:
    """
    topics = proxy.get_topics()
    type_filter = request.args.get('type', '')
    topic_filter = request.args.get('topic', '')
    if type_filter != '':
        return {'topics': [{'topic': topic,
                            'data': objectutils.get_typedef_recursive(proxy.get_topic_type(topic))} for topic in topics
                           if type_filter in proxy.get_topic_type(topic)]}
    elif type_filter != '':
        return {'topics': [{'topic': topic,
                            'data': objectutils.get_typedef_recursive(proxy.get_topic_type(topic))} for topic in topics
                           if topic_filter in topic]}
    return {'topics': [{'topic': topic,
                        'data': objectutils.get_typedef_recursive(proxy.get_topic_type(topic))} for topic in topics]}


@app.route('/publish', methods=['POST'])
def publish():
    op = request.data.get('op', None)
    topic_type = request.data.get('type', None)
    topic_name = request.data.get('topic', None)
    msg = request.data.get('msg', None)
    if op is None or topic_type is None or topic_name is None or msg is None:
        return status.HTTP_400_BAD_REQUEST
    else:
        _check_master()
        topic_name = rosgraph.names.script_resolve_name('rostopic', topic_name)
        try:
            msg_class = roslib.message.get_message_class(topic_type)
        except:
            raise ROSTopicException("invalid topic type: %s" % topic_type)
        if msg_class is None:
            pkg = _resource_name_package(topic_type)
            raise ROSTopicException(
                "invalid message type: %s.\nIf this is a valid message type, perhaps you need to type 'rosmake %s'" % (
                topic_type, pkg))
        rospy.init_node('rostopic', anonymous=True, disable_rosout=True, disable_rostime=True, disable_signals=True)
        pub = rospy.Publisher(topic_name, msg_class, latch=latch, queue_size=100)
        argv_publish(pub, msg_class, yaml.load(JSONEncoder().encode(msg)), None, True, False)
    return {'status': 'PUBLISHED', 'body': request.data}, status.HTTP_200_OK


@app.route('/type/{<string:name>}/', methods=['GET'])
def get_type(name):
    """

    :param name:
    :return:
    """
    print name
    return 'ok'


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
