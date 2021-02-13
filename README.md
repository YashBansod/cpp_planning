# cpp_robotics
This is a C++ repository containing various robotics related algorithms implemented in C++.

Each directory in the main project directory [cpp_robotics](./) contains a sub-project with its own CMakeLists.txt
file. The general process to run any project is following.
```shell script
# open the sub-project directory
cd ./<sub-project>

# view the CMakeLists.txt file of the sub-project
cat ./CMakeLists.txt

# make sure proper directories are included in the CMakeLists.txt file.
<use code editor of your preference to edit CMakeLists.txt>

# create a build directory
mkdir build

# open the build directory
cd build

# cmake the project in the build directory
cmake .. 

# make the cmake project
make -j4

# execute the generated executible file
./<executible file>
```
