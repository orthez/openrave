if not exist inc\boost cmake -E tar zxf msvc_files.tgz
cd python
if not exist sympy cmake -E tar zxf sympy_0.6.3.tgz
cd ..\..
mkdir build
cd build
cmake -DDOUBLE_PRECISION=ON ..
cd ..