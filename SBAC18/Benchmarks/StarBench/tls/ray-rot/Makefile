CC = clang++
CFLAGS  =  -mrtm -fopenmp=libiomp5 -pthread  -O2
SOURCES = image.cpp rotation_engine.cpp ray_engine.cpp program.cpp
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = ray-rot

all: $(SOURCES) $(EXECUTABLE)
 
$(EXECUTABLE): $(OBJECTS)  
	$(CC) $(OBJECTS) -o $@ $(CFLAGS)

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@ 

clean:
	rm -rf *.o $(EXECUTABLE) out.ppm output.ppm

