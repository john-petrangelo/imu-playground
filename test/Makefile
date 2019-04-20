OBJ_FILES = attitude.o quaternion.o utils.o vector.o
TEST_SRC_FILES = $(wildcard *.cc)
TEST_OBJ_FILES = $(TEST_SRC_FILES:.cc=.o)
TEST_DEP_FILES = $(TEST_SRC_FILES:.cc=.d)

CFLAGS = -g -std=c++11 -x c++ -I..
LDFLAGS = -L/usr/local/lib -lgtest_main -lgtest

all: unit_tests

-include $(TEST_DEP_FILES)

unit_tests: $(OBJ_FILES) $(TEST_OBJ_FILES)
	@echo "Linking unit test executable"
	@g++ -o $@ $(LDFLAGS) $^

%.o: ../%.ino
	@echo "Compiling arduino file $<"
	@g++ -c $(CFLAGS) -o $@ $<	

%.o: %.cc
	@echo "Compiling tests file $<"
	@g++ -c $(CFLAGS) -o $@ $<

%.o: ../common.h

%.d: %.cc
	@echo "Generating dependencies for $<"
	@g++ $(CFLAGS) $< -MM -MT $(@:.d=.o) > $@

.PHONY: clean test

test: unit_tests
	@./unit_tests

clean:
	@echo "Removing generated files"
	@rm -f *.o *.d unit_tests