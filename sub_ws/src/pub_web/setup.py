from setuptools import find_packages, setup

package_name = 'pub_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websockets>=12.0'],
    zip_safe=True,
    maintainer='air',
    maintainer_email='songunhee5426@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'web_bridge = pub_web.web_bridge:main',
        ],
    },
)
