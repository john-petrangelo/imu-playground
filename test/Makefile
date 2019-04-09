OBJ_FILES = simplemath.o

.cpp.o:
	g++ -c -std=c++11 -o $@ $<

unit_tests: $(OBJ_FILES)
	g++ -L/usr/local/lib -lgtest_main -lgtest $(OBJ_FILES) -o unit_tests

test: unit_tests
	./unit_tests

clean:
	@rm -f $(OBJ_FILES) unit_tests