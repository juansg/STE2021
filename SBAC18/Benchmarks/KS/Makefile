CC		= clang
CFLAGS	= -Wno-format -fopenmp -O3 -g
LDFLAGS = -lomp
SOURCES = KS-1.c KS-2.c
OBJS	= $(SOURCES:%.c=%.o)
EXE		= KS

SMALL_INPUT_PATH	= inputs/KL-4.in
SMALL_OUTPUT_PATH	= outputs/KL-4.OUT
MEDIUM_INPUT_PATH	= inputs/KL-5.in
MEDIUM_OUTPUT_PATH	= outputs/KL-5.OUT


all: $(OBJS)
	$(CC) $(CFLAGS) $^ -o $(EXE) $(LDFLAGS)
	@echo "*** "$<" linked successfully!"

$(OBJS): %.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@
	@echo "*** "$<" compiled successfully!"
	@echo ""

clean:
	rm -rf $(OBJS) $(EXE) outputs/*.tmp

run-small:
	time ./$(EXE) $(SMALL_INPUT_PATH) > $(SMALL_OUTPUT_PATH).tmp
	diff $(SMALL_OUTPUT_PATH).tmp $(SMALL_OUTPUT_PATH)

run-medium:
	time ./$(EXE) $(MEDIUM_INPUT_PATH) > $(MEDIUM_OUTPUT_PATH).tmp
	diff $(MEDIUM_OUTPUT_PATH).tmp $(MEDIUM_OUTPUT_PATH)
