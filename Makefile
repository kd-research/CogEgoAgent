default: build/Makefile
	@$(MAKE) -C build

build/Makefile: CMakeLists.txt
	mkdir -p build
	cd build && cmake ..

clean:
	rm -rf build/*

.PHONY: default clean
