CC = gcc
CXX = g++
CFLAGS = -O -g -I -Wall -Wno-unused-result
CXXFLAGS= -std=c++11 $(CFLAGS)

SUFFIX = .cpp
SRCDIR = ./src
SRCS = main.cpp Link.cpp Kinematics.cpp WalkingParameters.cpp WalkingPatternGenerator.cpp Serial.cpp RSservo.cpp Log.cpp dynamixel.cpp dx2lib.cpp dx2lib_intuitive.cpp dxmisc.cpp

OBJDIR = ./obj
OBJS = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(basename $(SRCS))))

LIBDIR = ./lib
LIBS = #-l pthread -l m -L. -l dx2lib_x64

TARGET = run

INCDIR = .

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(LIBS) $^ 

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp 
	@[ -d $(OBJDIR) ]
	$(CXX) -o $@ $(LIBS) -c $<

.PHONY: all

all: clean $(OBJDIR) $(TARGET)

.PHONY: clean 
 
clean:
	rm -rf ./*.o $(OBJDIR)/*.o $(TARGET) *.exe
