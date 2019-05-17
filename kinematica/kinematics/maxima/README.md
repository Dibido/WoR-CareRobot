# Analytical Jacobian



To run the script execute the following command from the command line

```
./jacobian_to_hpp.py -m analytical_jacobian.mac -o ../gen_inc/kinematics/JacobiMatrix.hpp -c
```

## Requirements

* Maxima v 5.41.0 or higher
* Python 3
* `analytical_jacobian.mac` or similar file

Example for expected file output of `analytical_jacobian.mac` for a 3x3 jacobian matrix.

```
.
|-- J11
|-- J12
|-- J13
|-- J21
|-- J22
|-- J23
|-- J31
|-- J32
|-- J33
```

The script assumes the only parameters are Sin(theta) and Cos(theta) for each degree of freedom in the robotarm
and is configured to work with up to 7 degrees of freedom.

## Suppressing compilation warnings

In order to keep supportability for the analytical version of
the Jacobian managable it is necessary to generate its code using
a script. However due to the nature of this generating it is not
possible to easily create functions that have no unused parameters.

So in order to be able to compile this file without running into unnecessary warnings
that cannot be solved anyway and will only result in cluttering up the compilation
output.

As such a special include directory called `gen_inc` has been created.
This include directory has been added to CMakelists in the following way

```
include_directories(
  include
  SYSTEM
  gen_inc
  ${catkin_INCLUDE_DIRS}
)
```

Using the `SYSTEM` flag suppresses all compiler warnings for files in all following directories.

## Jacobi Matrix Unit Test

In order to test if the conversion from maxima to C++ has been done
correctly `jacobi_matrix_test.mac` has been used
to create jacobi matrices to test against in
a unit test.

This unittest is found at `unitTest/JacobiMatrix_test.cpp`
