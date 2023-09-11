from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['localization_informed_planning_sim_libs'],
    package_dir={'': 'src'}
)

setup(**d)