#These variables control the compiler and linker flags. Change them as
# appropriate.
DEBUG_MODE = 1

GNU_TOOLS_DIR = ../WindRiver/gnu/3.4.4-vxworks-6.3/x86-win32/bin
SETUP_TOOLS_DIR = ../WindRiver/setup/x86-win32/bin
WIND_BASE = ../WindRiver/vxworks-6.3

ADDED_CFLAGS =

ifeq ($(DEBUG_MODE),1)
OBJ_DIR := build/PPC603gnu_DEBUG
CFLAGS = -g -mlongcall
else
OBJ_DIR := build/PPC603gnu
CFLAGS = -Os -fstrength-reduce -fno-builtin -fno-strict-aliasing -mlongcall
endif

ifndef ROBOT
	ROBOT = harlie
	MESSAGE = "You need to set the ROBOT environment variable before running make. ROBOT defaulted to harlie\nSample line: ROBOT=harlie make\n"
endif

LINKFLAGS = $(CFLAGS)
LIBPATH =

DIST_DIR := dist/$(ROBOT)/cRIO

# List all the *compiled* object files here, under the OBJ_DIR
# directory. Make will automatically locate the source file and
# compile it.
OBJECTS := \
	$(OBJ_DIR)/minigzip.o \
	$(OBJ_DIR)/matrixLibStack.o \
	$(OBJ_DIR)/matrixlibtestppc.o \
	$(OBJ_DIR)/NiFpga.o \
	$(OBJ_DIR)/dictionary.o \
	$(OBJ_DIR)/iniparser.o \
	$(OBJ_DIR)/sender.o \
	$(OBJ_DIR)/receiver.o \
	$(OBJ_DIR)/differentialSteering.o \
	$(OBJ_DIR)/harliealloc.o \
	$(OBJ_DIR)/harlielog.o \
	$(OBJ_DIR)/dkalman.o \
	$(OBJ_DIR)/fpga.o \
	$(OBJ_DIR)/sonar.o \
	$(OBJ_DIR)/odometry.o \
	$(OBJ_DIR)/yaw.o \
	$(OBJ_DIR)/PSOmain.o \
	$(OBJ_DIR)/sensor.o \
	$(OBJ_DIR)/sensorfusion.o \
	$(OBJ_DIR)/statevariable.o \
	$(OBJ_DIR)/stringextra.o \
	$(OBJ_DIR)/sysutils.o \
	$(OBJ_DIR)/timerutils.o \
	$(OBJ_DIR)/sharedVariables.o \
	$(OBJ_DIR)/wagonSteering.o \
	$(OBJ_DIR)/vector_math.o \
	$(OBJ_DIR)/hardwarediagnostics.o \
	$(OBJ_DIR)/MainCRIO.o \
	$(OBJ_DIR)/compass.o \
	$(OBJ_DIR)/microprocessorinterface.o \
	$(OBJ_DIR)/gps.o 

MAINRUNNER_OBJECTS := \
	$(OBJ_DIR)/mainRunner.o

# This is the name of the output shared library.
MAINCRIO := $(DIST_DIR)/MainCRIO.out
MAINRUNNER := $(DIST_DIR)/mainRunner.out

# If you have other VxWorks .a files to reference, list them here.
LIBS = 
LIBPATH = 

# Everything after this line should not need to be modified for
# basic compilation. However, significant changes to the build structure
# will probably involve modifying these lines.
WIND_BASE := $(subst \,/,$(WIND_BASE))

CPU = PPC603
TOOL_FAMILY = gnu
TOOL = gnu
CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp

IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 

# This basic rule compiles a .c file into a .o file. It can be adapted to
# all other source files that gcc can compile, including assembly (.s) and
# C++ (.cpp, .cc, .C, .cxx) files. To enable support for those extensions,
# copy this rule and modify its extension and compile flags for the
# required source file type.
$(OBJ_DIR)/%.o : src/%.c
	$(GNU_TOOLS_DIR)/ccppc $(CFLAGS) $(CC_ARCH_SPEC) -std=c99  -Wall  -MD -MP $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"

# Adapted rule for .cpp files
$(OBJ_DIR)/%.o : src/%.cpp
	$(GNU_TOOLS_DIR)/c++ppc $(CFLAGS) $(CC_ARCH_SPEC) -std=c99  -Wall  -MD -MP $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"

all : check_objectdir check_distdir copy_config $(MAINCRIO) $(MAINRUNNER)
	@echo $(MESSAGE)

$(MAINCRIO) : $(OBJECTS)
	rm -f "$@" ctdt.c;$(GNU_TOOLS_DIR)/nmppc $(OBJECTS) | $(SETUP_TOOLS_DIR)/tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > ctdt.c
	$(GNU_TOOLS_DIR)/ccppc $(LINKFLAGS) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o ctdt.o -c ctdt.c
	$(GNU_TOOLS_DIR)/ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJECTS) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) ctdt.o
	rm -f ctdt.c ctdt.o

$(MAINRUNNER) : $(MAINRUNNER_OBJECTS)
	rm -f "$@" ctdt.c;$(GNU_TOOLS_DIR)/nmppc $(MAINRUNNER_OBJECTS) | $(SETUP_TOOLS_DIR)/tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > ctdt.c
	$(GNU_TOOLS_DIR)/ccppc $(LINKFLAGS) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o ctdt.o -c ctdt.c
	$(GNU_TOOLS_DIR)/ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(MAINRUNNER_OBJECTS) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) ctdt.o
	rm -f ctdt.c ctdt.o

# Deploys the build onto the cRIO device
deploy: copy_deployment_ftps copy_config copy_ni_fpga_libs $(MAINCRIO) $(MAINRUNNER)
	cd $(DIST_DIR); \
	ftp -n < deploy_crio_ftp_commands; \
	echo "Deployment complete"

check_objectdir :
	@if [ ! -d "$(OBJ_DIR)" ]; then\
		mkdir -p $(OBJ_DIR);\
	fi

check_distdir :
	@if [ ! -d "$(DIST_DIR)" ]; then\
		mkdir -p $(DIST_DIR);\
	fi

copy_ni_fpga_libs : check_distdir
	cp fpgaInfo/NiFpga_mainFPGA.lvbitx $(DIST_DIR)
	cp fpgaInfo/NiFpga.out $(DIST_DIR)

copy_config : check_distdir
	@if [ ! -d "$(DIST_DIR)/config" ]; then\
		mkdir -p $(DIST_DIR)/config;\
	fi
	cp -R config/$(ROBOT)/* $(DIST_DIR)/config

copy_deployment_ftps : check_distdir
	cp src/deploy_crio_ftp_commands $(DIST_DIR)/deploy_crio_ftp_commands

clean :
	rm -f $(OBJECTS) $(MAINCRIO) $(MAINRUNNER) $(MAINRUNNER_OBJECTS) $(wildcard $(OBJ_DIR)/*.unstripped)
	rm -rf $(DIST_DIR)

.DUMMY: check_objectdir clean

matrixTest: src/matrixLibStack.c src/MatrixLibTest.c
	rm -rf MatrixTest
	mkdir MatrixTest
#	gcc -D MATRIXLIBTEST -c src/harlielog.c -o MatrixTest/matrixLibStack.o
	gcc -D MATRIXLIBTEST -c src/matrixLibStack.c -o MatrixTest/matrixLibStack.o
	gcc -D MATRIXLIBTEST MatrixTest/matrixLibStack.o src/MatrixLibTest.c -o MatrixTest/test
	MatrixTest/test
	rm -rf MatrixTest
