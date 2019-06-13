# README

This package includes one class and related unittests

In order to include this package in your own package add the following to `package.xml`

```
<depend>matrix</depend>
```

And the following to `CMakeLists.txt`


```
find_package (
  REQUIRED COMPONENTS
  matrix
)

include_directories (
  ${catkin_INCLUDE_DIRS}
)

# For each executable or library ${NODE} using the matrix class
add_dependencies(${NODE} ${catkin_EXPORTED_TARGETS})
```

To include the `Matrix` class in your code use the following header

```
#include <matrix/Matrix.hpp>
```
