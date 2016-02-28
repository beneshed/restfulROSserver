from flask_restful import fields, marshal_with, reqparse, Resource

publisher_fields = {
    'op': fields.String,
    'type': fields.String,
    'topic': fields.String,
    'msg': fields.Raw
}


class Publisher(Resource):
    @marshal_with(publisher_fields)
    def post(self):
        return 200
