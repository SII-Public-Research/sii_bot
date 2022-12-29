from setuptools import setup

package_name = 'sii_bot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clem-irwt',
    maintainer_email='clement.pene@sii.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'motors_control = sii_bot.motors:main',
        'motors_vel = sii_bot.motors2:main',
        'teleop_twist_keyboard = sii_bot.teleop:main',
        'line_follower_command = sii_bot.line_follower:main',
        'braitenberg = sii_bot.braitenberg:main'
        ],
    },
)
