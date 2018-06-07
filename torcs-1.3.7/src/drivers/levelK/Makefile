
test: unit_test_framework.cpp tests.cpp
	g++ -Wall -Werror -pedantic -O1 --std=c++11 unit_test_framework.cpp tests.cpp -o LKtest
	./LKtest

# Disable built-in rules
.SUFFIXES:

clean :
	rm -vf LKtest *~ *.out

build: build0 build0_1 build0_2 build0_3 build1 build1_1 build1_2 build1_3 build2 build2_1 build2_2 build2_3 build_human

build0:
	cd ../level0 && $(MAKE) build

build0_1:
	cd ../level0_1 && $(MAKE) build

build0_2:
	cd ../level0_2 && $(MAKE) build

build0_3:
	cd ../level0_3 && $(MAKE) build

build1:
	cd ../level1 && $(MAKE) build

build1_1:
	cd ../level1_1 && $(MAKE) build

build1_2:
	cd ../level1_2 && $(MAKE) build

build1_3:
	cd ../level1_3 && $(MAKE) build

build2:
	cd ../level2 && $(MAKE) build

build2_1:
	cd ../level2_1 && $(MAKE) build

build2_2:
	cd ../level2_2 && $(MAKE) build

build2_3:
	cd ../level2_3 && $(MAKE) build

build_human:
	cd ../human && $(MAKE) build

clean_all:
	cd ../human && $(MAKE) clean
	cd ../level0 && $(MAKE) clean
	cd ../level0_1 && $(MAKE) clean
	cd ../level0_2&& $(MAKE) clean
	cd ../level0_3 && $(MAKE) clean
	cd ../level1 && $(MAKE) clean
	cd ../level1_1 && $(MAKE) clean
	cd ../level1_2 && $(MAKE) clean
	cd ../level1_3 && $(MAKE) clean
	cd ../level2 && $(MAKE) clean
	cd ../level2_1 && $(MAKE) clean
	cd ../level2_2 && $(MAKE) clean
	cd ../level2_3 && $(MAKE) clean

