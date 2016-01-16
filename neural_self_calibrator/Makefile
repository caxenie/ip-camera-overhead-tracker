# this defines the binary name, and version (the version will be built further below)
TARGET     = cam_track
MAJOR      = 1
MINOR      = 0.0
 
CC         = g++
# usually, you will need only the following warnings for high safety. -pedantic may be 
# called -Wpedantic on different compilers (tcc, icc)
WARNINGS   = -Wall -Wextra -pedantic
# if you want to have even more warnings about doubtfull stuff, add the next few to the
# WARNINGS variable:
#     -Wundef -Wswitch-default -Wcast-align -Wcast-qual
# you will then have almost all warnings available in gcc.
 
 
# change the path to where your .h files are located at. (for example, you can but
# them into the source directory or have a separate one)
INCLUDES   = -I./src
LIBS       =
VERSION    = ${MAJOR}.${MINOR}
CPPFLAGS   = -DVERSION=\"${VERSION}\" 
CFLAGS     = ${INCLUDES} ${WARNINGS} ${CPPFLAGS} `pkg-config --cflags opencv` -D_REENTRANT
LDFLAGS    = ${LIBS} `pkg-config --libs opencv` -lm -lpthread -ldc1394  -lfann -lconfig
 
# differences when compiling for release and debug 
CFLAGS_DBG = -DDEBUG -ggdb 
CFLAGS_RLS = -O3 -ffast-math 
 
# source directory, source file extension (e.g. change to cpp for C++), 
SRCDIR     = src
SRCEXT     = cpp
 
# the following looks for all source files in the directory SRCDIR and extension SRCEXT
SRC        = $(shell find $(SRCDIR) -name \*.$(SRCEXT) -type f -print)
 
# object files and dependency graphs will be placed into BUILDDIR (debug and release
# get separate directories to avoid linking objects of different builds!)
BUILDDIR   = build
OBJDIR_DBG = ${BUILDDIR}/debug
OBJDIR_RLS = ${BUILDDIR}/release
OBJ_DBG    = $(patsubst $(SRCDIR)/%,$(OBJDIR_DBG)/%,$(patsubst %.$(SRCEXT),%.o,$(SRC)))
OBJ_RLS    = $(patsubst $(SRCDIR)/%,$(OBJDIR_RLS)/%,$(patsubst %.$(SRCEXT),%.o,$(SRC)))
DIRTREE_DBG= $(OBJDIR_DBG) \
	     $(patsubst $(SRCDIR)/%,$(OBJDIR_DBG)/%,\
	     $(shell find $(SRCDIR)/* -type d -print))
DIRTREE_RLS= $(OBJDIR_RLS) \
	     $(patsubst $(SRCDIR)/%,$(OBJDIR_RLS)/%,\
	     $(shell find $(SRCDIR)/* -type d -print))
 
all: debug
 
release: makedirs ${OBJ_RLS}
	@echo ' [LD] '${TARGET}
	@${CC} ${OBJ_RLS} ${LDFLAGS} -o ${TARGET}
 
debug: makedirs ${OBJ_DBG}
	@echo ' [LD] '${TARGET}
	@${CC} ${OBJ_DBG} ${LDFLAGS} -o ${TARGET}
 
-include ${OBJ_DBG:.o=.d}
 
${OBJDIR_DBG}/%.o: ${SRCDIR}/%.$(SRCEXT)
	@echo ' [CC] '$<
	@${CC} ${CFLAGS} ${CFLAGS_DBG} -c -o $@ $<
	@${CC} -MM $< > $(patsubst %.o,%.d,$@)
 
${OBJDIR_RLS}/%.o: ${SRCDIR}/%.$(SRCEXT)
	@echo ' [CC] '$<
	@${CC} ${CFLAGS} ${CFLAGS_RLS} -c -o $@ $<
	@${CC} -MM $< > $(patsubst $(OBJDIR_RLS)/%,$(OBJDIR_DBG)/%,\
			$(patsubst %.o,%.d,$@))
 
makedirs:
	@mkdir -p ${DIRTREE_DBG}
	@mkdir -p ${DIRTREE_RLS}
 
clean:
	@rm -rf ${BUILDDIR}
	@rm -f ${TARGET}
	@rm -f ${SRCDIR}/*~
