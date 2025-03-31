from setuptools import find_packages, setup

package_name = 'my_tf_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/config.rviz']),
        ('share/' + package_name + '/launch', ['launch/view.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danil',
    maintainer_email='podkolzindanil@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_broadcaster = my_tf_broadcaster.tf_broadcaster:main',
            'exp = my_tf_broadcaster.exp:main',
            'tf_grasp_listener = my_tf_broadcaster.tf_grasp_listener:main',
            'camera = my_tf_broadcaster.camera:main',
            'test = my_tf_broadcaster.test:main'
        ],
    },
)
