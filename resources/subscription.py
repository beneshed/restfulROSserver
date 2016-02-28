from flask_restful import fields, marshal_with, reqparse, Resource


class SubscriptionFields(object):
    resource_fields = {
        'op': fields.String,
        'type': fields.String,
        'topic': fields.String,
    }


class Subscription(Resource):
    @marshal_with(SubscriptionFields.resource_fields)
    def post(self):
        return 200

    def get(self):
        return 200
