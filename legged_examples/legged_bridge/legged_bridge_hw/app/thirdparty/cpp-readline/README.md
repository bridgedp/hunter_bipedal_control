cpp-readline
============

This is a very simple library that wraps GNU readline into a C++ reusable class,
hiding all the global state of the readline library. It is meant to help you
interface really easily with your programs, while waiting for a more powerful
solution (or improving this library itself for your needs).

Features
========

The main features of this library are:

- Easy adding of custom commands
- Automatic completion of commands and filenames.
- Can run files containing lists of commands automatically.
- Multiple separate Consoles can be run at the same time, bypassing the readline
  library global state.
- Currently NOT thread-safe.

Requirements
============

The library currently requires support for C++11, and, of course, the readline
library.

Building
========

This repository includes a very simple makefile to build the provided example,
but since the library is a single class you can simply include it directly into
your project and compile it with the rest, without creating a library file.

Otherwise the repository also has supporto for CMake, if you need to integrate
that with your existing build. To build the project using CMake, just do the 
following in the project root directory:

    mkdir build
    cd build
    cmake ..
    make

The makefile default compiler is g++, if you are using a different compiler
simply change the parameters to suit you (or compile manually, it's really just
three files).

Usage
=====

This library's usage can easily be seen in the file `example/main.cpp`.
