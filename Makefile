.PHONY: build test
build:
	gcc *.c -o out/bno055.o
test:
	build
lint:
	pip install cpplint
	cpplint --filter=-legal/copyright,-build/include_subdir `ls *.c` `ls *.h`
