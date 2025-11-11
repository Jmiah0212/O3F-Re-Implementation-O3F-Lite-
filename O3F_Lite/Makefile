CXX = g++
CXXFLAGS = -std=c++17 -O3 -march=native -DNDEBUG -Wall

OBJS = main.o env.o option_executor.o option_planner.o

all: o3f_lite

o3f_lite: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

main.o: main.cpp env.hpp option_executor.hpp option_planner.hpp utils.hpp
	$(CXX) $(CXXFLAGS) -c main.cpp

env.o: env.cpp env.hpp utils.hpp
	$(CXX) $(CXXFLAGS) -c env.cpp

option_executor.o: option_executor.cpp option_executor.hpp env.hpp utils.hpp
	$(CXX) $(CXXFLAGS) -c option_executor.cpp

option_planner.o: option_planner.cpp option_planner.hpp env.hpp option_executor.hpp utils.hpp
	$(CXX) $(CXXFLAGS) -c option_planner.cpp

clean:
	rm -f *.o o3f_lite o3f_train.csv
