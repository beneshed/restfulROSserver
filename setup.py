from distutils.core import setup

setup(
    name='restfulROSserver',
    version='0.0.1',
    url='',
    license='',
    author='Ben Waters',
    author_email='bsawyerwaters@gmail.com',
    description='A RESTful API to access ROS Bridge',
    zip_safe=False,
    install_required=['Flask', 'Flask-API'],
    requires=[
        'rostopic',
        'rosgraph'
    ]
)
