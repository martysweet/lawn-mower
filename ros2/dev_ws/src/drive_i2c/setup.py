from setuptools import setup

package_name = 'drive_i2c'

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
    maintainer='marty',
    maintainer_email='marty.sweet@omnistratus.co.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_control = drive_i2c.drive_control:main',
            'odometry_feedback = drive_i2c.odometry_feedback:main',
        ],
    },
)
