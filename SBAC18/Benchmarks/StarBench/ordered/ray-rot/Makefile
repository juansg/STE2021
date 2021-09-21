CXX = clang++
OPT = opt
CFLAGS  = -O3 -fopenmp
EXECUTABLE = ray-rot

all: $(EXECUTABLE)
 
$(EXECUTABLE): program.o image.cpp rotation_engine.cpp ray_engine.cpp
	$(CXX) $(CFLAGS) $^ -o $@ $(LDFLAGS)

program.o: program.bc
	$(CXX) $(CFLAGS) -c $^ -o $@

program.bc: program.ll
	$(OPT) -use-transforms $^ -o $@

program.ll: program.cpp
	$(CXX) $(CFLAGS) -S -emit-llvm $^ -o $@

clean:
	rm -rf *.o *.ll *.bc  $(EXECUTABLE) out.ppm output.ppm

