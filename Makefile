CC = gcc
CXX = g++
CFLAGS = -O -g -I -Wall -Wno-unused-result
CXXFLAGS= -std=c++17 $(CFLAGS)

SUFFIX = .cpp
SRCDIR_SRC = ./src
# SRCDIR_CONTROLLER = ./controller/lipm_test_generator
SRCDIR_CONTROLLER = ./controller/lipm_controller

SRCS_CONTROLLER = main.cpp
SRCS_SRC = Link.cpp Kinematics.cpp WalkingParameters.cpp WalkingPatternGenerator.cpp Serial.cpp Log.cpp dynamixel.cpp dx2lib.cpp dx2lib_intuitive.cpp dxmisc.cpp State_estimation.cpp robot.cpp

OBJDIR = ./obj
OBJS_CONTROLLER = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(basename $(SRCS_CONTROLLER))))
OBJS_SRC = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(basename $(SRCS_SRC))))
OBJS = $(OBJS_CONTROLLER) $(OBJS_SRC)

LIBDIR = .lib
LIBS = #-l pthread -l m -L. -l dx2lib_x64

# TARGET = lipm_test_generator
TARGET = lipm_test

INCDIR = .

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(LIBS) $^

$(OBJDIR)/%.o: $(SRCDIR_SRC)/%.cpp
	@[ -d $(OBJDIR) ] || mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) -I$(INCDIR) -I$(SRCDIR_SRC) -c $< -o $@

$(OBJDIR)/%.o: $(SRCDIR_CONTROLLER)/%.cpp
	@[ -d $(OBJDIR) ] || mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) -I$(INCDIR) -I$(SRCDIR_SRC) -c $< -o $@

.PHONY: all

all: clean $(OBJDIR) $(TARGET)

.PHONY: clean

clean:
	rm -rf ./*.o $(OBJDIR)/*.o $(TARGET) *.exe