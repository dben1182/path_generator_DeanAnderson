# B-Spline Generator

This README provides instructions on how to run the examples from scratch.

## Prerequisites

Ensure you have Python, `pip`, and git installed on your system. Then, install the required packages:

### Linux

```bash
pip install numpy
pip install matplotlib
pip install scipy
pip install pyrsistent
```
Clone the following repository

```bash
git clone https://github.com/davidcGIThub/bspline_generator.git
```
Navigate to the working directory and install

```bash
cd bspline_generator
pip install .
```

Now clone this repository

```bash
git clone https://github.com/davidcGIThub/path_generator.git
```

You should now be able to run the examples. If you are having trouble, you might have to rebuild the c++ code. Navigate to

```bash
/path_generator/PathObjectivesAndConstraints
```
and then run the following commands

```bash
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```
