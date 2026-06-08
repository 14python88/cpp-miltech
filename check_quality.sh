#!/usr/bin/env bash

clang-tidy -p build/debug homework_06/src/ballistics.cpp
clang-tidy -p build/debug homework_06/src/main.cpp
clang-tidy -p build/debug homework_06/tests/ballistics_tests.cpp