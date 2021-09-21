CXX = clang++
OPT = opt
CFLAGS  = -O3

all: serial.out ordered.out bdx.out tls.out

serial.out: program.cpp image.cpp rotation_engine.cpp ray_engine.cpp
	$(CXX) $(CFLAGS) $^ -o $@ $(LDFLAGS)

ordered.out: program.cpp image.cpp rotation_engine.cpp ray_engine.cpp
	$(CXX) $(CFLAGS) -fopenmp -D__ORDERED__ $^ -o $@ $(LDFLAGS)

tls.out: program.cpp image.cpp rotation_engine.cpp ray_engine.cpp
	$(CXX) $(CFLAGS) -fopenmp -D__USE_TLS__ -mrtm  $^ -o $@ $(LDFLAGS)
 
bdx.out: program.cpp image.cpp rotation_engine.cpp ray_engine.cpp
	$(CXX) $(CFLAGS) -fopenmp -fopenmp-use-dependence-analysis -D__USE_BDX__ $^ -o $@ $(LDFLAGS)

run-serial: serial.out
	cp serial.out a.out;
	./run.sh serial;
	@echo "Finished!"

run-ordered: ordered.out
	cp ordered.out a.out;
	./run.sh ordered;
	@echo "Finished!"

run-bdx: bdx.out
	cp bdx.out a.out;
	./run.sh bdx;
	@echo "Finished!"

run-tls: tls.out
	cp tls.out a.out;
	./run.sh tls;
	@echo "Finished!"

clean:
	rm -rf *.o *.ll *.bc *.out *.ppm *.ppm

