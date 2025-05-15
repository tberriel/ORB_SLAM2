# ORB_SLAM2-PythonBindings
A python wrapper for ORB_SLAM2, designed to work with the updated version of ORB_SLAM2, with a couple of minimal API changes to access the system output.
It has been tested on ubuntu 20.04 and built against Python3.10.

- Link with Python 3.10 is done through Boost, based on the API developed by JSkin https://github.com/jskinn/ORB_SLAM2-PythonBindings.
- OpenCV to Numpy conversion is managed by PBCVT (Python-Boost-OpenCV Converter) https://github.com/Algomorph/pyboostcvconverter/tree/master

## Installation

This code is implemented as a child from ORB-SLAM2/CMakeList.txt, and need to be compiled from there.

#### Alternative Python Versions

At the moment, CMakeLists is hard-coded to use python 3.10. If you wish to use a different version, simply change the boost component used (python-310) to the desired version (say, python-39), on lines 6-7 of CMakeLists.txt.
You will also need to change the install location on line 38 of CMakeLists.txt to your desired dist/site packages directory.

## License
This code was adapted by ORB_SLAM2-PythonBindings API developed by [John Skinner](https://github.com/jskinn/ORB_SLAM2-PythonBindings), with contributions from [Dmytro Mishkin](https://github.com/ducha-aiki), and uses pyboostcvconverter (https://github.com/Algomorph/pyboostcvconverter) by Gregory Kramida under the MIT licence (see pyboostcvconverter-LICENSE).

