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

## Running analytical_jacobian.mac in Maxima

To run the script in maxima without running the python script the following commands can be used:


Run the file and exit, generates all files used by the python script
```
maxima -b analytical_jacobian.mac
```

Load the file into maxima, allowing manual experimentation
```
maxima

load("analytical_jacobian.mac");
```

To then for example get the DenavitHartenberg matrix
```
Denavit;
```

To calculate a single joint and assign it to `Joint`
```
Joint : subst[theta=2%pi/4, alpha=%pi/2, a=2, d=2], Denavit);
Joint;
```

To multiply matrices with each other

```
J1 . J2 . J3;
```
