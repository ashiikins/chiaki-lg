# Host build of the input-layer tests: src/input.c is compiled unchanged against the mock in
# mock/ (see mock/redirect.h). Needs libchiaki's sources for controller.c and orientation.c.
#   make -C tests CHIAKI_LIB=/path/to/chiaki-ng/lib
#   make -C tests test

CHIAKI_LIB ?= ../../chiaki-ng/lib
CC ?= cc
SRC := ../src
BUILD := build

CFLAGS ?= -g -O1
CFLAGS += -std=gnu11 -Wall -Wextra -Wno-unused-parameter
INC := -Imock/include -I$(CHIAKI_LIB)/include -I$(SRC) -Imock

.PHONY: all test clean
all: $(BUILD)/test_motion

$(BUILD)/input.o: $(SRC)/input.c mock/redirect.h mock/mock.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) -include mock/redirect.h -c -o $@ $<

$(BUILD)/mock.o: mock/mock.c mock/mock.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<

$(BUILD)/test_motion.o: test_motion.c mock/mock.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<

$(BUILD)/haptics_stub.o: mock/haptics_stub.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<

$(BUILD)/%.o: $(SRC)/%.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<

$(BUILD)/lib_%.o: $(CHIAKI_LIB)/src/%.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<

OBJS := $(BUILD)/test_motion.o $(BUILD)/mock.o $(BUILD)/input.o $(BUILD)/ds5_in.o $(BUILD)/ds5_out.o \
        $(BUILD)/haptics_stub.o $(BUILD)/lib_controller.o $(BUILD)/lib_orientation.o

$(BUILD)/test_motion: $(OBJS)
	$(CC) -o $@ $^ -lm -lpthread

test: $(BUILD)/test_motion
	$(BUILD)/test_motion

clean:
	rm -rf $(BUILD)
