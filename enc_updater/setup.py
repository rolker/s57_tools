from setuptools import find_packages, setup

package_name = 'enc_updater'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/region_example.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roland Arsenault',
    maintainer_email='roland@ccom.unh.edu',
    description='Cron-friendly NOAA ENC chart updater (ADR-0010 D7).',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'enc_updater = enc_updater.__main__:main',
        ],
    },
)
