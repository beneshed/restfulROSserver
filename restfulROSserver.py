from flask import request
from flask.ext.api import FlaskAPI

from rosapi import proxy, objectutils

from flask.ext.api.exceptions import APIException


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
    return {'topics': [{'topic': topic, 'type': proxy.get_topic_type(topic),
                        'data': objectutils.get_typedef(proxy.get_topic_type(topic))} for topic in topics]}


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
