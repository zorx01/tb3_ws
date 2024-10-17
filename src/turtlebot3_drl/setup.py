from setuptools import find_packages, setup
import os
import glob


package_name = 'turtlebot3_drl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_ddrl_stage1.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage2.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage3.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage4.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage5.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage6.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zorx',
    maintainer_email='nandagopan.k01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'environment = turtlebot3_drl.drl_environment.drl_environment:main',
            'real_environment = turtlebot3_drl.drl_environment.drl_environment_real:main',
            'gazebo_goals = turtlebot3_drl.drl_gazebo.drl_gazebo:main',
            'train_agent = turtlebot3_drl.drl_agent.drl_agent:main_train',
            'test_agent = turtlebot3_drl.drl_agent.drl_agent:main_test',
            'real_agent = turtlebot3_drl.drl_agent.drl_agent:main_real',
            'test = turtlebot3_drl.drl_agent.test:main_real',
            'real_goal = turtlebot3_drl.drl_gazebo.goal_pub:main'
            
        ],
    },
)
