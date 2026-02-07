CXX = g++
CXXFLAGS = -std=c++17 -O3 -Wall

TARGET = run_sim
SRCS = main.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)


run: $(TARGET)
	@mkdir -p results figures
	@echo "Running Simulation..."
	./$(TARGET)
	@echo "Generating Figures..."
	python3 visualize.py
	@echo "Done."

clean:
	rm -f $(TARGET) $(OBJS)
	rm -rf results figures write

plot: visualize.py
	@echo "Generating Figures..."
	python3 visualize.py
	@echo "Done."