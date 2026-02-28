.PHONY: help debug release asan coverage bench bench-baseline bench-compare bench-hotspots bench-perfstat bench-profile static-analysis format format-check cpd ci \
        configure build test clean clean-debug clean-release clean-asan clean-coverage clean-static-analysis clean-all clean-fast distclean

help:
	@echo "Common commands:"
	@echo "  make debug            - Configure/build/test clang-debug"
	@echo "  make release          - Configure/build/test clang-release"
	@echo "  make asan             - Configure/build/test clang-debug-asan-ubsan"
	@echo "  make static-analysis  - Configure/build static-analysis preset (clang-tidy/cppcheck/IWYU)"
	@echo "  make bench            - Configure and run spherical-geometry benchmarks (local)"
	@echo "  make bench-baseline   - Record local spherical-geometry benchmark baseline"
	@echo "  make bench-compare    - Run + compare spherical-geometry benchmarks vs local baseline"
	@echo "  make bench-hotspots   - Generate hotspot summary data from longer benchmark runs"
	@echo "  make bench-perfstat   - Collect Linux perf hardware-counter data per benchmark"
	@echo "  make bench-profile    - Capture Linux perf call-stack profile for hotspot analysis"
	@echo "  make format-check     - Run formatting check"
	@echo "  make format           - Apply formatting"
	@echo "  make cpd              - Run PMD CPD duplicate-code scan (report-only by default)"
	@echo "  make coverage         - Configure/build/test coverage preset"
	@echo "  make ci               - Run full local CI target"
	@echo "  make clean            - Clean clang-debug build tree"
	@echo "  make clean-all        - Clean debug/release/asan/coverage/static-analysis build trees"
	@echo "  make clean-fast       - Fast cleanup (same semantics as distclean)"
	@echo "  make distclean        - Remove build directories and coverage outputs"

debug:
	cmake --workflow --preset debug

release:
	cmake --workflow --preset release

asan:
	cmake --workflow --preset asan

coverage:
	cmake --workflow --preset coverage

bench:
	cmake --workflow --preset benchmark

bench-baseline:
	cmake --workflow --preset benchmark-baseline

bench-compare:
	cmake --workflow --preset benchmark-compare

bench-hotspots:
	cmake --workflow --preset benchmark-hotspots

bench-perfstat:
	cmake --workflow --preset benchmark-perfstat

bench-profile:
	cmake --workflow --preset benchmark-profile

static-analysis:
	cmake --workflow --preset static-analysis

format-check:
	cmake --workflow --preset format-check

format:
	cmake --workflow --preset format

cpd:
	cmake --workflow --preset cpd

ci:
	cmake --workflow --preset ci-local

configure:
	cmake --preset clang-debug

build:
	cmake --build --preset build-clang-debug

test:
	ctest --preset test-clang-debug

clean: clean-debug

clean-debug:
	cmake --preset clang-debug
	cmake --build --preset build-clang-debug --target clean

clean-release:
	cmake --preset clang-release
	cmake --build --preset build-clang-release --target clean

clean-asan:
	cmake --preset clang-debug-asan-ubsan
	cmake --build --preset build-clang-debug-asan-ubsan --target clean

clean-coverage:
	cmake --preset gcc-coverage
	cmake --build --preset build-gcc-coverage --target clean

clean-static-analysis:
	cmake --preset clang-static-analysis
	cmake --build --preset build-clang-static-analysis --target clean

clean-all: clean-debug clean-release clean-asan clean-coverage clean-static-analysis

clean-fast: distclean

distclean:
	cmake -E rm -rf build
	cmake -E rm -f coverage.xml
