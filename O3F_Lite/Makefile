CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall
OBJS = main.o env.o option_executor.o option_planner.o

o3f_lite: main.o env.o option_executor.o
	$(CXX) $(CXXFLAGS) -o $@ main.o env.o option_executor.o

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	rm -f *.o o3f_lite
