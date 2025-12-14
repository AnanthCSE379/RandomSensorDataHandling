from setuptools import find_packages, setup

package_name = 'random_data_processing'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anantharamakrishnan',
    maintainer_email='iamanantharamakrishnan.s@gmail.com',
    description='This package will act like a broken sensor sending data to a ROS2 network of nodes.One node generates the random data using random library and pass it onto the ROS2 network to test the robustness of the given ROS2 network',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'sens = random_data_processing.data_gen:main',
        	'microprocessor = random_data_processing.data_processing:main',
        ],
    },
)
