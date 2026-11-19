##############################################################################
#	Makefile of the controller of the Indy7 (paper, Fig. 1)
#
#	make                  bin/indy7_ctrl
#	make MUJOCO=0|1       without / with the simulation (-s, -b); default 1
#	                      if $(MUJOCO_DIR) has the C library of MuJoCo
#	make ROS=0|1          without / with the bridge of the camera
#	                      application; default 1 if ROS 2 ($(ROS_DIR)) and
#	                      the messages of ~/ros2_ws (colcon build) are there
#	make MALLOC_COUNT=1   the test build that counts the heap allocations of
#	                      the real-time tasks (make clean first)
#	make tools            the offline checks: bin/model_check (MUJOCO=1),
#	                      bin/hl_check, bin/goal_filter_check, bin/slot_check,
#	                      bin/ipc_slot_check, bin/obs_mailbox_check,
#	                      bin/obs_receiver_check, bin/obs_estimate_check,
#	                      bin/obs_goal_check, bin/obs_status_check,
#	                      bin/r5_link_check;
#	                      and bin/fw_cfg_header, the constants of the R5
#	                      firmware from the configuration
#	make clean
#
#	The dependencies (paper, Table 1): the IgH EtherCAT master 1.6
#	(IGH_DIR), RBDL with its URDF reader (RBDL_DIR), Ruckig (RUCKIG_DIR),
#	Eigen 3, MuJoCo (MUJOCO_DIR), ROS 2 Jazzy (ROS_DIR). RT-POSIX is built
#	from ../third_party/rt-posix.
#
#	The defaults below are those of the KV260 (aarch64): ROS 2 Humble, the
#	messages of the camera pipeline in ~/ros2_ws, MuJoCo 3.14.0 and Ruckig
#	0.19.4 in ~/opt (no root), RBDL 3.1.3 and the IgH master 1.6.10 of the
#	system.
##############################################################################

ROOT_DIR    := ..
SRC_DIR     := src
OBJ_DIR     := obj
BIN_DIR     := bin
RTPOSIX_DIR := $(ROOT_DIR)/third_party/rt-posix
TOOLS_DIR   := $(ROOT_DIR)/tools

IGH_DIR     ?= /opt/etherlab
RBDL_DIR    ?= /usr/local
RUCKIG_DIR  ?= $(HOME)/opt/ruckig
EIGEN_DIR   ?= /usr/include/eigen3
MUJOCO_DIR  ?= $(HOME)/opt/mujoco
ROS_DIR     ?= /opt/ros/humble
MY_IFACE_DIR ?= $(HOME)/ros2_ws/install/my_interfaces

CXX ?= g++
CC  ?= gcc
# Cortex-A53 r0p4 (KV260): erratum 835769, which gcc does not work around by default
ARCH_FLAGS := $(if $(filter aarch64,$(shell uname -m)),-mfix-cortex-a53-835769)
CXXFLAGS := -O2 -std=gnu++17 -Wall -fexceptions -D_GNU_SOURCE $(ARCH_FLAGS)
CFLAGS   := -O2 -Wall -D_GNU_SOURCE $(ARCH_FLAGS)
INCLUDES := -I$(SRC_DIR) -I$(RTPOSIX_DIR)/include -I$(IGH_DIR)/include \
            -isystem $(EIGEN_DIR) -isystem $(RBDL_DIR)/include -isystem $(RUCKIG_DIR)/include
LDLIBS   := -L$(IGH_DIR)/lib -lethercat -Wl,-rpath,$(IGH_DIR)/lib \
            -L$(RBDL_DIR)/lib -lrbdl -lrbdl_urdfreader -Wl,-rpath,$(RBDL_DIR)/lib \
            -L$(RUCKIG_DIR)/lib -lruckig -Wl,-rpath,$(RUCKIG_DIR)/lib \
            -lm -lrt -lpthread

# the malloc-count test build (base/MallocCount.h)
MALLOC_COUNT ?= 0
ifeq ($(MALLOC_COUNT),1)
CXXFLAGS += -DG2T_MALLOC_COUNT
endif

# MuJoCo, the simulated arm (sim/SimArm.h)
MUJOCO ?= $(if $(wildcard $(MUJOCO_DIR)/include/mujoco/mujoco.h),1,0)
ifeq ($(MUJOCO),1)
CXXFLAGS += -DG2T_WITH_MUJOCO
INCLUDES += -isystem $(MUJOCO_DIR)/include
LDLIBS   += -L$(MUJOCO_DIR)/lib -lmujoco -Wl,-rpath,$(MUJOCO_DIR)/lib
endif

# ROS 2 (Humble here, Jazzy in the paper), the bridge of the camera
# application (ROS2PickBridge.h). Run a build with ROS=1 after sourcing
# /opt/ros/humble/setup.bash and ~/ros2_ws/install/setup.bash (run.sh does
# it): the type support of the messages is loaded at run time from
# LD_LIBRARY_PATH. Always the my_interfaces of ~/ros2_ws, never a copy:
# Humble has no type hash, a different .msg would be read as garbage.
ROS ?= $(if $(and $(wildcard $(ROS_DIR)/include/rclcpp/rclcpp/rclcpp.hpp),$(wildcard $(MY_IFACE_DIR)/include/my_interfaces/my_interfaces/msg/pick_target3_d.hpp)),1,0)
ifeq ($(ROS),1)
# -I for every ROS package in $(ROS_DIR)/include, i.e. every directory <pkg>
# that holds a directory <pkg>; this leaves out vendored libraries whose
# headers would hide those of the C library
ROS_INC_DIRS := $(foreach d,$(wildcard $(ROS_DIR)/include/*),$(if $(wildcard $(d)/$(notdir $(d))/.),$(d)))
INCLUDES += -isystem $(ROS_DIR)/include $(addprefix -isystem ,$(ROS_INC_DIRS)) -isystem $(MY_IFACE_DIR)/include/my_interfaces
ROS_CORE_LIBS = rclcpp rcl rcutils rmw rmw_implementation rcpputils rcl_yaml_param_parser rcl_logging_interface \
                tracetools libstatistics_collector ament_index_cpp rosidl_runtime_c rosidl_typesupport_c \
                rosidl_typesupport_cpp rosidl_dynamic_typesupport
ROS_MSG_PKGS = sensor_msgs std_msgs builtin_interfaces rcl_interfaces statistics_msgs rosgraph_msgs \
               service_msgs type_description_interfaces
ROS_TS = rosidl_generator_c rosidl_typesupport_c rosidl_typesupport_cpp rosidl_typesupport_fastrtps_c \
         rosidl_typesupport_fastrtps_cpp rosidl_typesupport_introspection_c rosidl_typesupport_introspection_cpp
LDLIBS += $(wildcard $(foreach t,$(ROS_TS),$(MY_IFACE_DIR)/lib/libmy_interfaces__$(t).so)) \
          $(wildcard $(foreach p,$(ROS_MSG_PKGS),$(foreach t,$(ROS_TS),$(ROS_DIR)/lib/lib$(p)__$(t).so))) \
          $(wildcard $(foreach l,$(ROS_CORE_LIBS),$(ROS_DIR)/lib/lib$(l).so)) \
          -Wl,-rpath,$(MY_IFACE_DIR)/lib -Wl,-rpath,$(ROS_DIR)/lib
BRIDGE_SOURCE := ROS2PickBridge.cpp
else
CXXFLAGS += -DG2T_NO_ROS
BRIDGE_SOURCE := ROS2PickBridgeNoRos.cpp
endif
$(info [indy7] MUJOCO=$(MUJOCO) ROS=$(ROS) MALLOC_COUNT=$(MALLOC_COUNT))

SOURCES := base/Config.cpp base/RtTask.cpp base/ViewStream.cpp base/MallocCount.cpp base/RtConsole.cpp \
           ecat/EcatMaster.cpp ecat/Indy7Joint.cpp ecat/Indy7EndTool.cpp \
           sim/SimArm.cpp sim/SimJoint.cpp sim/SimCamera.cpp sim/SimR5.cpp r5link/R5Link.cpp \
           Controller.cpp FullDynControllerRT.cpp HighLevelRT.cpp TrajectoryRT.cpp shadow/ShadowControl.cpp \
           shadow/ControlCycle.cpp \
           JointTestLog.cpp HighLevelLog.cpp obs/ObsLog.cpp TimingLog.cpp CalibCapture.cpp $(BRIDGE_SOURCE) \
           Indy7Ctrl.cpp TaskEtherCAT.cpp TaskTrajectory.cpp TaskDynamics.cpp TaskPlanner.cpp main.cpp
RTPOSIX_SOURCES := posix_rt.c commons.c

OBJECTS := $(addprefix $(OBJ_DIR)/,$(SOURCES:.cpp=.o)) \
           $(addprefix $(OBJ_DIR)/rt-posix/,$(RTPOSIX_SOURCES:.c=.o))

all: $(BIN_DIR)/indy7_ctrl

# the console output of the real-time tasks without the lock of stdio
# (base/RtConsole.h): the stdio calls of the controller go through wrappers
comma := ,
WRAP_STDIO := $(addprefix -Wl$(comma)--wrap=,__printf_chk __fprintf_chk puts putchar fputs fputc fwrite fflush)

$(BIN_DIR)/indy7_ctrl: $(OBJECTS)
	@mkdir -p $(BIN_DIR)
	@! nm -u $(filter-out %/RtConsole.o,$(OBJECTS)) | grep -wE 'printf|fprintf|vprintf|vfprintf|__vprintf_chk|__vfprintf_chk|perror|putc|_IO_putc' || \
	    { echo "stdio calls that WRAP_STDIO does not wrap (build with -O2 and _FORTIFY_SOURCE, or wrap them)"; exit 1; }
	$(CXX) -o $@ $(OBJECTS) $(WRAP_STDIO) $(LDLIBS)

# the objects depend on the options: a stamp per combination
STAMP := $(OBJ_DIR)/.build_$(MUJOCO)$(ROS)$(MALLOC_COUNT)
$(STAMP):
	@mkdir -p $(OBJ_DIR); rm -f $(OBJ_DIR)/.build_*; touch $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(STAMP)
	@mkdir -p $(dir $@)
	$(CXX) -MD -MP $(CXXFLAGS) $(INCLUDES) -c -o $@ $<

$(OBJ_DIR)/rt-posix/%.o: $(RTPOSIX_DIR)/src/%.c $(STAMP)
	@mkdir -p $(dir $@)
	$(CC) -MD -MP $(CFLAGS) -I$(RTPOSIX_DIR)/include -c -o $@ $<

# the offline checks link the same objects as the controller
TOOL_OBJS := $(OBJ_DIR)/FullDynControllerRT.o $(OBJ_DIR)/Controller.o $(OBJ_DIR)/HighLevelRT.o \
             $(OBJ_DIR)/base/MallocCount.o $(OBJ_DIR)/rt-posix/posix_rt.o $(OBJ_DIR)/rt-posix/commons.o

tools: $(BIN_DIR)/hl_check $(BIN_DIR)/goal_filter_check $(BIN_DIR)/slot_check $(BIN_DIR)/ipc_slot_check \
       $(BIN_DIR)/obs_mailbox_check $(BIN_DIR)/obs_receiver_check $(BIN_DIR)/obs_estimate_check \
       $(BIN_DIR)/obs_goal_check $(BIN_DIR)/obs_status_check $(BIN_DIR)/r5_link_check \
       $(BIN_DIR)/fw_cfg_header $(BIN_DIR)/rbdl_model_code $(BIN_DIR)/ctrl_numbers_header $(BIN_DIR)/shadow_check \
       $(BIN_DIR)/control_check \
       $(BIN_DIR)/console_check \
       $(if $(filter 1,$(MUJOCO)),$(BIN_DIR)/model_check)

# the RBDL model of the controller against the MuJoCo model (models/indy7_mujoco.xml)
$(BIN_DIR)/model_check: $(TOOLS_DIR)/model_check.cpp $(TOOL_OBJS)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(TOOL_OBJS) $(LDLIBS)

# the high-level controller on an ideal arm: the reference contract in every period
$(BIN_DIR)/hl_check: $(TOOLS_DIR)/hl_check.cpp $(TOOL_OBJS) $(SRC_DIR)/HighLevelRT.h $(SRC_DIR)/HighLevelLink.h \
                     $(SRC_DIR)/base/AtomicSlot.h
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(TOOL_OBJS) $(LDLIBS)

# the goal checks and the smoothing of the bridge (GoalFilter.h), without ROS 2
$(BIN_DIR)/goal_filter_check: $(TOOLS_DIR)/goal_filter_check.cpp $(SRC_DIR)/GoalFilter.h
	@mkdir -p $(BIN_DIR)
	$(CXX) -O2 -std=gnu++17 -Wall -I$(SRC_DIR) -o $@ $<

# the concurrent-reader test of the atomic slot (paper, Sec. II-A)
$(BIN_DIR)/slot_check: $(TOOLS_DIR)/slot_check.cpp $(SRC_DIR)/base/AtomicSlot.h
	@mkdir -p $(BIN_DIR)
	$(CXX) -O2 -std=gnu++17 -Wall -I$(SRC_DIR) -o $@ $< -lpthread

# the slots between the APU and the R5 (src/ipc): the payloads, the faults, concurrent readers
$(BIN_DIR)/ipc_slot_check: $(TOOLS_DIR)/ipc_slot_check.cpp $(SRC_DIR)/ipc/ipc_layout.h $(SRC_DIR)/ipc/ipc_slot.h
	@mkdir -p $(BIN_DIR)
	$(CXX) -O2 -std=gnu++17 -Wall -I$(SRC_DIR) -o $@ $< -lpthread

# the mailbox of the camera results (src/obs): the result, the faults, one writer and one reader
$(BIN_DIR)/obs_mailbox_check: $(TOOLS_DIR)/obs_mailbox_check.cpp $(SRC_DIR)/obs/obs_mailbox.h $(SRC_DIR)/ipc/ipc_slot.h \
                              $(SRC_DIR)/ipc/ipc_layout.h
	@mkdir -p $(BIN_DIR)
	$(CXX) -O2 -std=gnu++17 -Wall -I$(SRC_DIR) -o $@ $< -lpthread

# the control task's side of the camera results (src/obs): the states, the verdicts, random results, one writer
$(BIN_DIR)/obs_receiver_check: $(TOOLS_DIR)/obs_receiver_check.cpp $(SRC_DIR)/obs/obs_receiver.h $(SRC_DIR)/obs/obs_mailbox.h \
                               $(SRC_DIR)/ipc/ipc_slot.h $(SRC_DIR)/ipc/ipc_layout.h
	@mkdir -p $(BIN_DIR)
	$(CXX) -O2 -std=gnu++17 -Wall -I$(SRC_DIR) -o $@ $< -lpthread

# the object estimate of the camera application (obs/ObsEstimate.h), against the smoothing of the bridge
$(BIN_DIR)/obs_estimate_check: $(TOOLS_DIR)/obs_estimate_check.cpp $(SRC_DIR)/obs/ObsEstimate.h $(SRC_DIR)/GoalFilter.h
	@mkdir -p $(BIN_DIR)
	$(CXX) -O2 -std=gnu++17 -Wall -I$(SRC_DIR) -o $@ $<

# the two ends of the mailbox in the control application (obs/ObsWriter.h, obs/ObsGoal.h), against the bridge of M1
$(BIN_DIR)/obs_goal_check: $(TOOLS_DIR)/obs_goal_check.cpp $(SRC_DIR)/obs/ObsGoal.h $(SRC_DIR)/obs/ObsWriter.h \
                           $(SRC_DIR)/obs/ObsEstimate.h $(SRC_DIR)/obs/obs_receiver.h $(SRC_DIR)/obs/obs_mailbox.h \
                           $(SRC_DIR)/obs/obs_status.h \
                           $(SRC_DIR)/GoalFilter.h $(SRC_DIR)/GoalSlot.h $(SRC_DIR)/base/AtomicSlot.h
	@mkdir -p $(BIN_DIR)
	$(CXX) -O2 -std=gnu++17 -Wall -I$(SRC_DIR) -o $@ $< -lpthread

# what the control task tells the camera side (obs/obs_status.h) and the check value of the contract (obs/ObsContract.h)
$(BIN_DIR)/obs_status_check: $(TOOLS_DIR)/obs_status_check.cpp $(SRC_DIR)/obs/obs_status.h $(SRC_DIR)/obs/ObsContract.h \
                             $(SRC_DIR)/ipc/FwCfg.h $(SRC_DIR)/ipc/ipc_slot.h $(OBJ_DIR)/base/Config.o
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -I$(SRC_DIR) -I$(RTPOSIX_DIR)/include -o $@ $< $(OBJ_DIR)/base/Config.o -lpthread

# the exchange with the R5 (--arch S1): the APU side, following its period, and the R5 side of the simulation
$(BIN_DIR)/r5_link_check: $(TOOLS_DIR)/r5_link_check.cpp $(SRC_DIR)/r5link/R5Link.h $(SRC_DIR)/r5link/R5Follow.h \
                          $(SRC_DIR)/r5link/LinkClock.h $(SRC_DIR)/sim/SimR5Slots.h $(SRC_DIR)/ipc/ipc_slot.h \
                          $(SRC_DIR)/ipc/ipc_layout.h $(OBJ_DIR)/r5link/R5Link.o $(OBJ_DIR)/rt-posix/posix_rt.o \
                          $(OBJ_DIR)/rt-posix/commons.o
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -I$(SRC_DIR) -I$(RTPOSIX_DIR)/include -o $@ $< $(OBJ_DIR)/r5link/R5Link.o \
	    $(OBJ_DIR)/rt-posix/posix_rt.o $(OBJ_DIR)/rt-posix/commons.o -lpthread

# the constants of the R5 firmware and their check value (ipc/FwCfg.h) from the configuration; --check tests it
$(BIN_DIR)/fw_cfg_header: $(TOOLS_DIR)/fw_cfg_header.cpp $(SRC_DIR)/ipc/FwCfg.h $(OBJ_DIR)/base/Config.o
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -I$(SRC_DIR) -I$(RTPOSIX_DIR)/include -o $@ $< $(OBJ_DIR)/base/Config.o

# the RBDL model of the controller as numbers for the firmware of the R5 (rpu_work/r5_freertos/src/ctrl)
$(BIN_DIR)/rbdl_model_code: $(TOOLS_DIR)/rbdl_model_code.cpp $(SRC_DIR)/shadow/ModelCode.h $(SRC_DIR)/ipc/FwCfg.h
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L$(RBDL_DIR)/lib -lrbdl -lrbdl_urdfreader -Wl,-rpath,$(RBDL_DIR)/lib

# the numbers of the controller as a header for the firmware of the R5 (rpu_work/r5_freertos/src/ctrl)
$(BIN_DIR)/ctrl_numbers_header: $(TOOLS_DIR)/ctrl_numbers_header.cpp $(TOOL_OBJS) $(OBJ_DIR)/base/Config.o \
                                $(SRC_DIR)/shadow/ShadowParams.h $(SRC_DIR)/shadow/ShadowControl.h
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(TOOL_OBJS) $(OBJ_DIR)/base/Config.o $(LDLIBS)

# the second controller of --shadow (src/shadow) against the controller it follows; -o writes its periods for
# the controller as the firmware of the R5 builds it
SHADOW_OBJS := $(OBJ_DIR)/shadow/ShadowControl.o $(OBJ_DIR)/TrajectoryRT.o $(OBJ_DIR)/base/Config.o $(OBJ_DIR)/r5link/R5Link.o
$(BIN_DIR)/shadow_check: $(TOOLS_DIR)/shadow_check.cpp $(TOOL_OBJS) $(SHADOW_OBJS) $(SRC_DIR)/shadow/ShadowWriter.h \
                         $(SRC_DIR)/TrajectoryRT.h $(SRC_DIR)/HighLevelLink.h \
                         $(SRC_DIR)/shadow/ShadowTask.h $(SRC_DIR)/shadow/ShadowParams.h \
                         $(SRC_DIR)/shadow/shadow_vectors.h $(SRC_DIR)/sim/SimR5Slots.h
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(TOOL_OBJS) $(SHADOW_OBJS) $(LDLIBS)

# the control task without its operating system (src/shadow/ControlCycle.h; S2), with a second controller on its reports
CONTROL_OBJS := $(OBJ_DIR)/shadow/ControlCycle.o $(OBJ_DIR)/shadow/ShadowControl.o $(OBJ_DIR)/TrajectoryRT.o $(OBJ_DIR)/base/Config.o
$(BIN_DIR)/control_check: $(TOOLS_DIR)/control_check.cpp $(TOOL_OBJS) $(CONTROL_OBJS) $(SRC_DIR)/shadow/ShadowWriter.h \
                          $(SRC_DIR)/shadow/ShadowParams.h $(SRC_DIR)/shadow/control_vectors.h $(SRC_DIR)/ipc/ipc_layout.h
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(TOOL_OBJS) $(CONTROL_OBJS) $(LDLIBS)

# the console of the real-time tasks (base/RtConsole.h), with the stdio wrappers of indy7_ctrl
$(BIN_DIR)/console_check: $(TOOLS_DIR)/console_check.cpp $(OBJ_DIR)/base/RtConsole.o
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(OBJ_DIR)/base/RtConsole.o $(WRAP_STDIO) -lpthread

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

.PHONY: all tools clean
-include $(shell find $(OBJ_DIR) -name '*.d' 2>/dev/null)
