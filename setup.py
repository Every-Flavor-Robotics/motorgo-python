from setuptools import setup, find_packages

setup(
    name='pyplink',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'spidev',
    ],
    entry_points={},
    author='Swapnil Pande',
    author_email='swapnil@everyflavorrobotics.com',
    description='Python API for the Raspberry Pi + MotorGo Plink.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/Every-Flavor-Robotics/pyplink',
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)