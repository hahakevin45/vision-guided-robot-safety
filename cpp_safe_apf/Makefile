# SafeApfFilter C++ port — differential test against the Python reference.
# Plain g++, C++17, no colcon / CMake / external dependencies.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O2

.PHONY: all golden test clean

all: safe_apf_test

safe_apf_test: main.cpp safe_apf.hpp
	$(CXX) $(CXXFLAGS) main.cpp -o safe_apf_test

golden:
	python3 gen_golden.py

test: safe_apf_test golden.json
	./safe_apf_test golden.json

clean:
	rm -f safe_apf_test
