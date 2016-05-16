# COMP110 Worksheet 5

This is the skeleton project for COMP110 worksheet 5. Please refer to the worksheet, available on LearningSpace, for instructions.

## Building and running

You should be able to compile the project immediately after checking it out (if not, email Ed). However when you try to run it, you will get error messages about missing DLLs. To fix this, assuming you are building in *Debug* configuration for *Win32*:

* Copy `SDL2.dll` from `SDL2-2.0.4\lib\x86` to `Debug`
* Copy *all* `*.dll` files from `SDL2_image-2.0.1\lib\x86` to `Debug`

Note that the `Debug` directory will not exist until you have compiled the application for the first time.

For *Release* build, copy the dlls into the `Release` directory. For *x64* (64-bit) builds, copy the dlls from `...\lib\x64` to `x64\Debug` or `x64\Release`.
