from setuptools import find_packages, setup
import os
import glob

package_name = 'myrobot_sim_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [f for f in glob.glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),

        (os.path.join('share', package_name, 'urdf'), [f for f in glob.glob('urdf/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'urdf/meshes'), [f for f in glob.glob('urdf/meshes/*') if os.path.isfile(f)]),

        (os.path.join('share', package_name, 'xacro'), [f for f in glob.glob('xacro/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'xacro/sensor'), [f for f in glob.glob('xacro/sensor/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'worlds'), [f for f in glob.glob('worlds/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'rviz'), [f for f in glob.glob('rviz/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'meshes'), [f for f in glob.glob('meshes/*') if os.path.isfile(f)]),

        # meshes
        (os.path.join('share', package_name, 'models/turtlebot3_common/meshes/sensors'), [f for f in glob.glob('models/turtlebot3_common/meshes/sensors/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models/turtlebot3_common/meshes/bases'), [f for f in glob.glob('models/turtlebot3_common/meshes/bases/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models/turtlebot3_common/meshes/wheels'), [f for f in glob.glob('models/turtlebot3_common/meshes/wheels/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models/turtlebot3_common/meshes'), [f for f in glob.glob('models/turtlebot3_common/meshes/*') if os.path.isfile(f)]),

        (os.path.join('share', package_name, 'models/turtlebot3_waffle'), [f for f in glob.glob('models/turtlebot3_waffle/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models/myrobot'), [f for f in glob.glob('models/myrobot/*') if os.path.isfile(f)]),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenzhaoqi',
    maintainer_email='zhaoqi.chen@spacemit.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_subscriber = myrobot_sim_gazebo.scan_subscriber:main',
        ],
    },
)
