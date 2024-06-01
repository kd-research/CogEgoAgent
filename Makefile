default: build/Makefile
	@$(MAKE) -C build

build/Makefile: CMakeLists.txt
	mkdir -p build
	cd build && cmake ..

doxygen:
	cd doc && doxygen doxyfile

clean:
	rm -rf build/*

install:
	@$(MAKE) -C build install

.PHONY: default clean
