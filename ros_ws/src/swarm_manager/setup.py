from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['swarm_manager'],
    package_dir={'': 'scripts'}
)
setup(**d)