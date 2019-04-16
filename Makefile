.PHONY: build test
build:
	gcc *.c -o out/bno055.o
run:
	./out/bno055.o
test:
	build
	run
lint:
	cpplint --filter=-legal/copyright,-build/include_subdir `ls *.c` `ls *.h`
