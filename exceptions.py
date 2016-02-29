from flask.ext.api.exceptions import APIException


class ROSUnavailable(APIException):
    status_code = 500
    detail = 'ROS unavailable, reconfigure.'
