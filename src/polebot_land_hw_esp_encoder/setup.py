from setuptools import find_packages, setup

package_name = 'polebot_land_hw_esp_encoder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NJ-Ramadhan',
    maintainer_email='nj.ramadhan.mechatronics@gmail.com',
    description='Bridge package to read Encoder data via ESP for POLEBOT_LAND robots by NJ-Ramadhan',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'polebot_land_hw_esp_encoder = polebot_land_hw_esp_encoder.polebot_land_hw_esp_encoder:main'
        ],
    },
)
