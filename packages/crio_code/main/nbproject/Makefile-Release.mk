#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_CONF=Release
CND_DISTDIR=dist

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=build/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/src/wagonSteering.o \
	${OBJECTDIR}/src/matrixLib.o \
	${OBJECTDIR}/src/sonar.o \
	${OBJECTDIR}/src/vector_math.o \
	${OBJECTDIR}/src/differentialSteering.o \
	${OBJECTDIR}/src/receiver.o \
	${OBJECTDIR}/src/mainRunner.o \
	${OBJECTDIR}/src/kalman.o \
	${OBJECTDIR}/src/harliealloc.o \
	${OBJECTDIR}/src/sensor.o \
	${OBJECTDIR}/src/timerutils.o \
	${OBJECTDIR}/src/sender.o \
	${OBJECTDIR}/src/sysutils.o \
	${OBJECTDIR}/src/NiFpga.o \
	${OBJECTDIR}/src/minigzip.o \
	${OBJECTDIR}/src/odometry.o \
	${OBJECTDIR}/src/statevariable.o \
	${OBJECTDIR}/src/harlielog.o \
	${OBJECTDIR}/src/basicfusion.o \
	${OBJECTDIR}/src/sensorfusion.o \
	${OBJECTDIR}/src/MainCRIO.o \
	${OBJECTDIR}/src/MatrixLibTest.o \
	${OBJECTDIR}/src/dkalman.o \
	${OBJECTDIR}/src/yaw.o \
	${OBJECTDIR}/src/gps.o \
	${OBJECTDIR}/src/PSOmain.o \
	${OBJECTDIR}/src/iniparser.o \
	${OBJECTDIR}/src/matrixLibStack.o \
	${OBJECTDIR}/src/hardwarediagnostics.o \
	${OBJECTDIR}/src/sharedVariables.o \
	${OBJECTDIR}/src/stringextra.o \
	${OBJECTDIR}/src/fpga.o \
	${OBJECTDIR}/src/matrixlibtestppc.o \
	${OBJECTDIR}/src/dictionary.o

# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-Release.mk dist/Release/GNU-Linux-x86/main

dist/Release/GNU-Linux-x86/main: ${OBJECTFILES}
	${MKDIR} -p dist/Release/GNU-Linux-x86
	${LINK.c} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/main ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/src/wagonSteering.o: nbproject/Makefile-${CND_CONF}.mk src/wagonSteering.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/wagonSteering.o src/wagonSteering.c

${OBJECTDIR}/src/matrixLib.o: nbproject/Makefile-${CND_CONF}.mk src/matrixLib.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/matrixLib.o src/matrixLib.c

${OBJECTDIR}/src/sonar.o: nbproject/Makefile-${CND_CONF}.mk src/sonar.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/sonar.o src/sonar.c

${OBJECTDIR}/src/vector_math.o: nbproject/Makefile-${CND_CONF}.mk src/vector_math.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/vector_math.o src/vector_math.c

${OBJECTDIR}/src/differentialSteering.o: nbproject/Makefile-${CND_CONF}.mk src/differentialSteering.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/differentialSteering.o src/differentialSteering.c

${OBJECTDIR}/src/receiver.o: nbproject/Makefile-${CND_CONF}.mk src/receiver.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/receiver.o src/receiver.c

${OBJECTDIR}/src/mainRunner.o: nbproject/Makefile-${CND_CONF}.mk src/mainRunner.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/mainRunner.o src/mainRunner.c

${OBJECTDIR}/src/kalman.o: nbproject/Makefile-${CND_CONF}.mk src/kalman.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/kalman.o src/kalman.c

${OBJECTDIR}/src/harliealloc.o: nbproject/Makefile-${CND_CONF}.mk src/harliealloc.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/harliealloc.o src/harliealloc.c

${OBJECTDIR}/src/sensor.o: nbproject/Makefile-${CND_CONF}.mk src/sensor.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/sensor.o src/sensor.c

${OBJECTDIR}/src/timerutils.o: nbproject/Makefile-${CND_CONF}.mk src/timerutils.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/timerutils.o src/timerutils.c

${OBJECTDIR}/src/sender.o: nbproject/Makefile-${CND_CONF}.mk src/sender.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/sender.o src/sender.c

${OBJECTDIR}/src/sysutils.o: nbproject/Makefile-${CND_CONF}.mk src/sysutils.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/sysutils.o src/sysutils.c

${OBJECTDIR}/src/NiFpga.o: nbproject/Makefile-${CND_CONF}.mk src/NiFpga.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/NiFpga.o src/NiFpga.c

${OBJECTDIR}/src/minigzip.o: nbproject/Makefile-${CND_CONF}.mk src/minigzip.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/minigzip.o src/minigzip.c

${OBJECTDIR}/src/odometry.o: nbproject/Makefile-${CND_CONF}.mk src/odometry.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/odometry.o src/odometry.c

${OBJECTDIR}/src/statevariable.o: nbproject/Makefile-${CND_CONF}.mk src/statevariable.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/statevariable.o src/statevariable.c

${OBJECTDIR}/src/harlielog.o: nbproject/Makefile-${CND_CONF}.mk src/harlielog.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/harlielog.o src/harlielog.c

${OBJECTDIR}/src/basicfusion.o: nbproject/Makefile-${CND_CONF}.mk src/basicfusion.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/basicfusion.o src/basicfusion.c

${OBJECTDIR}/src/sensorfusion.o: nbproject/Makefile-${CND_CONF}.mk src/sensorfusion.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/sensorfusion.o src/sensorfusion.c

${OBJECTDIR}/src/MainCRIO.o: nbproject/Makefile-${CND_CONF}.mk src/MainCRIO.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/MainCRIO.o src/MainCRIO.c

${OBJECTDIR}/src/MatrixLibTest.o: nbproject/Makefile-${CND_CONF}.mk src/MatrixLibTest.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/MatrixLibTest.o src/MatrixLibTest.c

${OBJECTDIR}/src/dkalman.o: nbproject/Makefile-${CND_CONF}.mk src/dkalman.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/dkalman.o src/dkalman.c

${OBJECTDIR}/src/yaw.o: nbproject/Makefile-${CND_CONF}.mk src/yaw.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/yaw.o src/yaw.c

${OBJECTDIR}/src/gps.o: nbproject/Makefile-${CND_CONF}.mk src/gps.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/gps.o src/gps.c

${OBJECTDIR}/src/PSOmain.o: nbproject/Makefile-${CND_CONF}.mk src/PSOmain.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/PSOmain.o src/PSOmain.c

${OBJECTDIR}/src/iniparser.o: nbproject/Makefile-${CND_CONF}.mk src/iniparser.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/iniparser.o src/iniparser.c

${OBJECTDIR}/src/matrixLibStack.o: nbproject/Makefile-${CND_CONF}.mk src/matrixLibStack.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/matrixLibStack.o src/matrixLibStack.c

${OBJECTDIR}/src/hardwarediagnostics.o: nbproject/Makefile-${CND_CONF}.mk src/hardwarediagnostics.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/hardwarediagnostics.o src/hardwarediagnostics.c

${OBJECTDIR}/src/sharedVariables.o: nbproject/Makefile-${CND_CONF}.mk src/sharedVariables.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/sharedVariables.o src/sharedVariables.c

${OBJECTDIR}/src/stringextra.o: nbproject/Makefile-${CND_CONF}.mk src/stringextra.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/stringextra.o src/stringextra.c

${OBJECTDIR}/src/fpga.o: nbproject/Makefile-${CND_CONF}.mk src/fpga.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/fpga.o src/fpga.c

${OBJECTDIR}/src/matrixlibtestppc.o: nbproject/Makefile-${CND_CONF}.mk src/matrixlibtestppc.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/matrixlibtestppc.o src/matrixlibtestppc.c

${OBJECTDIR}/src/dictionary.o: nbproject/Makefile-${CND_CONF}.mk src/dictionary.c 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/dictionary.o src/dictionary.c

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/Release
	${RM} dist/Release/GNU-Linux-x86/main

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
