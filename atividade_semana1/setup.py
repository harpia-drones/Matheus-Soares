from setuptools import find_packages, setup

package_name = 'atividade_semana1'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "temperature_publisher = atividade_semana1.temperature_publisher:main",
            "temperature_monitor = atividade_semana1.temperature_monitor:main",
            "average_reset_client = atividade_semana1.average_reset_client:main",
            "temperature_avg_monitor = atividade_seaman1.temperature_avg_monitor:main" 
        ],
    },
)
