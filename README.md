# Kalman Filter Applied

The Kalman Filter applied to simulated environments.

##Clang
### Clang Tidy

To use `clang-tidy` you have to pass as parameter the path to the `compile_commands.json` file. The CMake is capable of generating it when the flag  `DCMAKE_EXPORT_COMPILE_COMMANDS` is activated.
```shell script
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build
```

Once the `compile_commands.json` is generated, you can run the below command.
Note that `$(catkin locate -b -e kf_applied)` is retrieving the build path of the specified package.

```shell script
./scripts/clang_tidy.sh $(catkin locate -b -e kf_applied)
```

### Clang Format

### Clang Complete

[clang_complete](https://github.com/xavierd/clang_complete) is a Vim plugin that uses clang for accurately completing C and C++ code. 

Look in the its documentation to make it work, in the project it is present the `.clang_complete` file that specify to the compiler options.
