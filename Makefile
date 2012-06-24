###
# fermat
# -------
# Copyright (c)2010-2012 Daniel Fiser <danfis@danfis.cz>
#
#  This file is part of fermat.
#
#  Distributed under the OSI-approved BSD License (the "License");
#  see accompanying file BDS-LICENSE for details or see
#  <http://www.opensource.org/licenses/bsd-license.php>.
#
#  This software is distributed WITHOUT ANY WARRANTY; without even the
#  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See the License for more information.
##

-include Makefile.local
-include Makefile.include

CFLAGS += -I.
CXXFLAGS += -I.
LDFLAGS += -L. -lsvoboda -lm -lrt

TARGETS = libsvoboda.a
OBJS  = gng.o gng-eu.o gsrm.o
OBJS += gng-t.o
OBJS += prm.o rrt.o
OBJS += nnbp.o
OBJS += kohonen.o
OBJS += gnnp.o
OBJS += gpc.o gpc-tree.o
OBJS += ga.o


BIN_TARGETS  = fer-gsrm
BIN_TARGETS += fer-plan

EXAMPLE_TARGETS += nnbp-simple
ifeq '$(USE_SDL)' 'yes'
  ifeq '$(USE_SDL_IMAGE)' 'yes'
    EXAMPLE_TARGETS += nnbp-img
  endif
endif

EXAMPLE_TARGETS += ga-knapsack
EXAMPLE_TARGETS += ga-knapsack2
EXAMPLE_TARGETS += kohonen-simple
EXAMPLE_TARGETS += gng-eu
EXAMPLE_TARGETS += gng-t
EXAMPLE_TARGETS += prm-6d
EXAMPLE_TARGETS += gpc


OBJS 		    := $(foreach obj,$(OBJS),.objs/$(obj))
BIN_TARGETS     := $(foreach target,$(BIN_TARGETS),bin/$(target))
EXAMPLE_TARGETS := $(foreach target,$(EXAMPLE_TARGETS),examples/$(target))


ifeq '$(EXAMPLES)' 'yes'
  TARGETS += $(EXAMPLE_TARGETS)
endif
ifeq '$(BINS)' 'yes'
  TARGETS += $(BIN_TARGETS)
endif

all: $(TARGETS)

libsvoboda.a: $(OBJS)
	ar cr $@ $(OBJS)
	ranlib $@

bin/fer-%: bin/%-main.c libsvoboda.a
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)
bin/fer-plan: bin/plan-main.o bin/cfg-map.o libsvoboda.a
	$(CC) $(CFLAGS) $(OPENCL_CFLAGS) -o $@ $< bin/cfg-map.o $(LDFLAGS) $(OPENCL_LDFLAGS)
bin/%.o: bin/%.c bin/%.h
	$(CC) $(CFLAGS) -c -o $@ $<

examples/%: examples/%.c libsvoboda.a
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)
examples/nnbp-img: examples/nnbp-img.c libsvoboda.a
	$(CC) $(CFLAGS) $(SDL_CFLAGS) $(SDL_IMAGE_CFLAGS) -o $@ $< $(LDFLAGS) $(SDL_LDFLAGS) $(SDL_IMAGE_LDFLAGS)


.objs/%.o: src/%.c svoboda/%.h
	$(CC) $(CFLAGS) -c -o $@ $<
.objs/%.o: src/%.c
	$(CC) $(CFLAGS) -c -o $@ $<
.objs/cd-sap.o: src/cd-sap.c src/cd-sap-1.c src/cd-sap-threads.c src/cd-sap-gpu.c svoboda/cd-sap.h
	$(CC) $(CFLAGS) -c -o $@ $<


install:
	mkdir -p $(PREFIX)/$(INCLUDEDIR)/svoboda
	mkdir -p $(PREFIX)/$(LIBDIR)
	cp -r svoboda/* $(PREFIX)/$(INCLUDEDIR)/svoboda/
	cp libsvoboda.a $(PREFIX)/$(LIBDIR)

clean:
	rm -f $(OBJS)
	rm -f src/cfg-lexer.c src/cfg-lexer-gen.h
	rm -f .objs/*.o
	rm -f $(TARGETS)
	rm -f $(BIN_TARGETS)
	if [ -d testsuites ]; then $(MAKE) -C testsuites clean; fi;
	if [ -d doc ]; then $(MAKE) -C doc clean; fi;
	
check:
	$(MAKE) -C testsuites check
check-valgrind:
	$(MAKE) -C testsuites check-valgrind

doc:
	$(MAKE) -C doc

analyze: clean
	$(SCAN_BUILD) $(MAKE)

help:
	@echo "Targets:"
	@echo "    all            - Build library"
	@echo "    doc            - Build documentation"
	@echo "    check          - Build & Run automated tests"
	@echo "    check-valgrind - Build & Run automated tests in valgrind(1)"
	@echo "    clean          - Remove all generated files"
	@echo "    install        - Install library into system"
	@echo "    analyze        - Performs static analysis using Clang Static Analyzer"
	@echo ""
	@echo "Options:"
	@echo "    CC         - Path to C compiler          (=$(CC))"
	@echo "    CXX        - Path to C++ compiler        (=$(CXX))"
	@echo "    M4         - Path to m4 macro processor  (=$(M4))"
	@echo "    SED        - Path to sed(1)              (=$(SED))"
	@echo "    PYTHON     - Path to python interpret    (=$(PYTHON))"
	@echo "    PYTHON2    - Path to python interpret v2 (=$(PYTHON2))"
	@echo "    PYTHON3    - Path to python interpret v3 (=$(PYTHON3))"
	@echo "    SCAN_BUILD - Path to scan-build          (=$(SCAN_BUILD))"
	@echo ""
	@echo "    EXAMPLES  'yes'/'no' - Set to 'yes' if examples should be build (=$(EXAMPLES))"
	@echo "    BINS      'yes'/'no' - Set to 'yes' if binaries should be build (=$(BINS))"
	@echo ""
	@echo "    CC_NOT_GCC 'yes'/'no' - If set to 'yes' no gcc specific options will be used (=$(CC_NOT_GCC))"
	@echo ""
	@echo "    DEBUG      'yes'/'no' - Turn on/off debugging          (=$(DEBUG))"
	@echo "    PROFIL     'yes'/'no' - Compiles profiling info        (=$(PROFIL))"
	@echo "    NOWALL     'yes'/'no' - Turns off -Wall gcc option     (=$(NOWALL))"
	@echo "    NOPEDANTIC 'yes'/'no' - Turns off -pedantic gcc option (=$(NOPEDANTIC))"
	@echo ""
	@echo "    USE_SDL      'yes'/'no'  - Use SDL library       (=$(USE_SDL))"
	@echo "                               By default, auto detection is used."
	@echo "    USE_SDL_IMAGE 'yes'/'no' - Use SDL_image library (=$(USE_SDL_IMAGE))"
	@echo "                               By default, auto detection is used."
	@echo ""
	@echo "    PREFIX     - Prefix where library will be installed                             (=$(PREFIX))"
	@echo "    INCLUDEDIR - Directory where header files will be installed (PREFIX/INCLUDEDIR) (=$(INCLUDEDIR))"
	@echo "    LIBDIR     - Directory where library will be installed (PREFIX/LIBDIR)          (=$(LIBDIR))"
	@echo ""
	@echo "Variables:"
	@echo "  Note that most of can be preset or changed by user"
	@echo "    SYSTEM            = $(SYSTEM)"
	@echo "    CFLAGS            = $(CFLAGS)"
	@echo "    CXXFLAGS          = $(CXXFLAGS)"
	@echo "    LDFLAGS           = $(LDFLAGS)"
	@echo "    CONFIG_FLAGS      = $(CONFIG_FLAGS)"
	@echo "    SDL_CFLAGS        = $(SDL_CFLAGS)"
	@echo "    SDL_LDFLAGS       = $(SDL_LDFLAGS)"
	@echo "    SDL_IMAGE_CFLAGS  = $(SDL_IMAGE_CFLAGS)"
	@echo "    SDL_IMAGE_LDFLAGS = $(SDL_IMAGE_LDFLAGS)"

.PHONY: all clean check check-valgrind help doc install analyze examples
