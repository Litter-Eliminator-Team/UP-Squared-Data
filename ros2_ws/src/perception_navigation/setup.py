from setuptools import setup

package_name = 'perception_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='literbotpro',
    maintainer_email='literbotpro@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subber = perception_navigation.image_subber:main',
            'keyboard_servos = perception_navigation.keyboard_servos:main'
        ],
    },
)
