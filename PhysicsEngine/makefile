CXX = g++
FLAGS = -Wall -std=c++11
CFLAGS = -c -Wall -std=c++11

OBJ = PhysicsEngine.o

%.o : %.cpp makefile
	$(CXX) $(CFLAGS) $< -o $@

Driver : $(OBJ) Driver.cpp makefile
	$(CXX) $(FLAGS) $(OBJ) Driver.cpp
