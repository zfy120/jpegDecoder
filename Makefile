TARGET = jpegDecoder

SRC = $(wildcard *.cpp)
OBJ = $(SRC:.cpp=.o)

CXX = g++
CPPFLAGS = -std=c++2a -DDEBUG

LOGFILE = log.txt

.PHONY: all clean

all: $(TARGET)
$(TARGET): $(OBJ)
	$(CXX) $^ -o $@
	$(RM) $(OBJ)

%.o: %.cpp
	$(CXX) $(CPPFLAGS) -c $^ -o $@

clean:
	$(RM) $(TARGET) $(OBJ) $(LOGFILE)

run:
	./$(TARGET) gig-sn08.jpg 
