################################################################################
# RPi_Environmental_Monitor_System tests makefile
#
# Author:   J. Parziale
# Created:  12/24/2021
#
################################################################################

# GCC versions under 6.0 can only use c++ std "c++0x".
# Newer ones can use c++17, c++2a, c++2b, or c++20
# C++20 - avail since GCC 8, still experimental. Use c++2a in GCC 9 and earlier.
# C++17 - avail since GCC 5, default in GCC 11
# C++14 - avail since GCC 6.1, default in GCC 10
# C++11 - avail since GCC 4.8.1, using C++0x or C++11 as follows:
#         GCC 4.3 thru 4.6 use c++0x
#         GCC 4.7 thru 4.8 use c++11
GCC_VER=$(shell $(CXX) -dumpversion | cut -f1-2 -d. | sed 's/\.//g')
MIN_GCC_VER = 60

# gcc 8.x just shows up as 8 instead of 8x
ifeq ($(GCC_VER),8)
  GCC_VER=800
endif
# gcc 10.x just shows up as 10 instead of 10x
ifeq ($(GCC_VER),10)
  GCC_VER=100
endif

GCCVERSIONGT60 := $(shell expr $(GCC_VER) \>= $(MIN_GCC_VER))
OSTYPE := $(shell uname)
ifeq "$(GCCVERSIONGT60)" "1"
#    GCC_STD=-std=c++20
    GCC_STD=-std=c++2a
else
    GCC_STD=-std=c++0x
endif

COMPILER = g++
CXX      = g++
CC       = g++
LINK     = g++
INCLUDES = -I. -I../hardware/include
THREADS  = -lpthread -lrt

OBJECTS  = Debug.o

DEFINES  = -D DEBUG_CONSOLE

LIBDIRS = -L ../hardware
LIBS = $(THREADS) -lhardware

LDFLAGS  = $(LIBDIRS) $(LIBS)

CXXFLAGS = -O2 -g -Wall -fmessage-length=0 $(GCC_STD) $(INCLUDES) $(DEFINES)

################################################################################

# If these change, then re-link all targets
LIBFILES = ../hardware/libhardware.a ../hardware/libalgobsec.a

TARGETS= \
	test_ADS1115_single \
	test_ADS1115_differential \
	test_bme280 \
	test_bsec_rpi

################################################################################

.cpp:.o
	$(CXX) -c $(CXXFLAGS) $< -o $@

################################################################################

all: begin $(TARGETS)
#	@echo "--------------------------------------------------------------------------------"
	@echo "All targets up-to-date."
	@echo

# All target executables depend on these library files.
$(TARGETS) : $(LIBFILES)

test_ADS1115_single: test_ADS1115_single.cpp $(OBJECTS)
	${CXX} ${CXXFLAGS} -o $@ $< $(OBJECTS) ${LDFLAGS}
	@echo

test_ADS1115_differential: test_ADS1115_differential.cpp $(OBJECTS)
	${CXX} ${CXXFLAGS} -o $@ $< $(OBJECTS) ${LDFLAGS}
	@echo

test_bme280: test_bme280.cpp $(OBJECTS)
	${CXX} ${CXXFLAGS} -o $@ $< $(OBJECTS) ${LDFLAGS} -lwiringPi
	@echo

test_bsec_rpi: test_bsec_rpi.cpp $(OBJECTS)
	${CXX} ${CXXFLAGS} -o $@ $< $(OBJECTS) -lalgobsec ${LDFLAGS} -lwiringPi -lm
	@echo

################################################################################

begin:
	@echo "--------------------------------------------------------------------------------"
	@echo "Building targets: $(TARGETS)..."
	@echo

hwlib:
	@echo "--------------------------------------------------------------------------------"
	@echo "Forcing build of hardware library..."
	@echo
	( cd ../build; pwd; cmake ..; make clean; make; cd $$OLDPWD; pwd; )
	@echo

################################################################################

tidy:
	$(RM) -vf *.d *.map *.log _sample.xml
	$(RM) -rf *.dSYM

clean: tidy
	rm -f *.o

clobber: clean
	$(RM) -vf $(TARGETS)
	@echo

echo:
	@echo
	@echo "OSTYPE    = $(OSTYPE)"
	@echo "IS_DARWIN = $(IS_DARWIN)"
	@echo "GCC_VER   = $(GCC_VER)"
	@echo "GCCVERSIONGT60 = $(GCCVERSIONGT60)"
	@echo "GCC_STD   = $(GCC_STD)"
	@echo "INCLUDES  = $(INCLUDES)"
	@echo "CXXFLAGS  = $(CXXFLAGS)"
	@echo "LIBDIRS   = $(LIBDIRS)"
	@echo "LIBS      = $(LIBS)"
	@echo "LDFLAGS   = $(LDFLAGS)"
	@echo "LIBFILES   = $(LIBFILES)"
	@echo "TARGETS   = $(TARGETS)"
	@echo

################################################################################
