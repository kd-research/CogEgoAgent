default: build/Makefile
	@$(MAKE) -C build VERBOSE=1

build/Makefile: CMakeLists.txt
	mkdir -p build
	cd build && cmake ..

clean:
	rm -rf build/*

.PHONY: default clean
