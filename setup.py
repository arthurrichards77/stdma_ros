from setuptools import setup

package_name = 'stdma_ros'

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
    maintainer='aeagr',
    maintainer_email='arthur.richards@bristol.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = stdma_ros.stdma_talker:main',
                'timer = stdma_ros.stdma_timer:main',
        ],
    },
)
