OBJ_FILES = vector.o
TEST_SRC_FILES = $(wildcard *.cc)
TEST_OBJ_FILES = $(TEST_SRC_FILES:.cc=.o)
TEST_DEP_FILES = $(TEST_SRC_FILES:.cc=.d)

CFLAGS = -std=c++11 -x c++ -I..
LDFLAGS = -L/usr/local/lib -lgtest_main -lgtest

all: unit_tests

-include $(TEST_DEP_FILES)

unit_tests: $(OBJ_FILES) $(TEST_OBJ_FILES)
	g++ -o $@ $(LDFLAGS) $^

vector.o: ../vector.ino ../common.h
	g++ -c $(CFLAGS) -o $@ $<	

%.d: %.cc
	g++ $(CFLAGS) $< -MM -MT $(@:.d=.o) >$@

%.o: %.cc
	g++ -c $(CFLAGS) -o $@ $<

test: unit_tests
	./unit_tests

.PHONY: clean
clean:
	@rm -f *.o *.d unit_tests