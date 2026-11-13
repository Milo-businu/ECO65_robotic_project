from glob import glob
import os

from setuptools import setup

package_name = 'arm_grasp'

# scripts/ 里的脚本既作为"可执行"装到 lib/arm_grasp（可用 ros2 run），
# 也复制一份到 share（launch 用 python3 直接跑，clone 下来不编译也能用）
py_scripts = [p for p in glob('scripts/*.py') if not os.path.basename(p).startswith('__')]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    scripts=py_scripts,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/scripts', py_scripts),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='noreply@users.noreply.github.com',
    description='真机整机抓取：夹爪驱动 / 相机感知 / 抓取任务总控 / 一键启动',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
