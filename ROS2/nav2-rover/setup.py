
---

### `setup.py`
```python
from setuptools import setup

package_name = 'nav2_rover'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mapping.launch.py']),
        ('share/' + package_name + '/config', ['config/mapper_params_online_async.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='your@email.com',
    description='Rover navigation package (Day1+2)',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'cmdvel_listener = nav2_rover.cmdvel_listener:main',
        ],
    },
)
