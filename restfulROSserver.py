from flask import Flask
from flask_restful import Api
from resources.subscription import Subscription
from resources.publisher import Publisher

app = Flask(__name__)
api = Api(app)

api.add_resource(Subscription, '/subscriptions')
api.add_resource(Publisher, '/publishers')


if __name__ == '__main__':
    app.run()
