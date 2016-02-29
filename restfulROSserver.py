from flask import request
from flask.ext.api import FlaskAPI, status

from rosapi import proxy, objectutils

from flask.ext.api.exceptions import APIException

from fred import get_fred_command

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
    ros_type = request.data.get('type', None)
    topic = request.data.get('topic', None)
    msg = request.data.get('msg', None)
    if op is None or ros_type is None or topic is None or msg is None:
        return status.HTTP_400_BAD_REQUEST
    else:
        print get_fred_command(msg)
    return 'Ok', status.HTTP_200_OK

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
