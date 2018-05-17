# Advanced Techniques for Curve Fitting


## Introduction
Curve fitting often occurs in lot of situation. It is possible to simple
linear fitting of the data. The most commonly used way is the linear least squares.
However, it is quite sensitive to outliers. It is important to distinguish
outliers from noise. Both have totally different effects on the quality of the solution.

It is also often the case to fit a complicated non-linear function to the data.
[ceres-solver](http://ceres-solver.org/index.html) is a fantastic library from
google for non-linear least squares. It uses functional programming to elegantly
let you specify your non-linear least squares problem. ceres-solver is a
2nd order solver, which means it is roughly based on the Gauss-Newton method
as opposed to Gradient Descent methods. ceres-solver's main purpose is to
use it for function optimization in SLAM community. The main feature that
distinguish it from other general solvers is its ability to optimize on the
manifold. For example, it is produce solution on the SO(3) manifold (ie.
  spaces of rotation matrix).


## Problems
I am attempting to solve 2 problems. This data will have outliers in addition to noise:
- 1d linear fit (given a Nx2 matrix get equation of a line)
- SO(3) + R^3 fit. Given two point cloud to estimate the translation and rotation between them.

## Methods
I shall explore the following techniques to solve the above problem:
- Basic least squares
- Use of Robust norms (Huber Norm and Cauchy Norm)
- L1 norm and linear-programming
- Use of Switch Variables
- Use of Dynamic Covariance Scalling
- BigM trick.


## Note
I shall use the ceres-solver's C++ interface for this. Plotting tools are implemented
in Python and are in `plotting` directory. Methods using python are found in `py` directory.
