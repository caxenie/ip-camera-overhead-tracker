# Overhead ip cam tracker - Main makefile
TARGET     = cam_track
CC         = gcc
WARNINGS   = -Wall -Wextra -pedantic
INCLUDES   = -I. -I/usr/local/include/opencv -I/usr/local/include/opencv2
LIBS       = -L/usr/local/lib/ -lrt -lm -lpthread
CFLAGS     = -std=gnu99 -g ${INCLUDES} ${WARNINGS}
LDFLAGS    = ${LIBS} `pkg-config --cflags --libs opencv`
CFLAGS_DBG = -DDEBUG -ggdb
SRCDIR     = .
SRCEXT     = c
SRC        = $(shell find $(SRCDIR) -name \*.$(SRCEXT) -type f -print)
BUILDDIR   = build
OBJDIR_DBG = ${BUILDDIR}/debug
OBJ_DBG    = $(patsubst $(SRCDIR)/%,$(OBJDIR_DBG)/%,$(patsubst %.$(SRCEXT),%.o,$(SRC)))
DIRTREE_DBG= $(OBJDIR_DBG) \
	     	$(patsubst $(SRCDIR)/%,$(OBJDIR_DBG)/%,\
	     $(shell find $(SRCDIR)/* -type d -print))
 
all: debug
  
debug: makedirs ${OBJ_DBG}
	echo ' [LD] '${TARGET}
	${CC} ${OBJ_DBG} ${LDFLAGS} -o ${TARGET}
 
-include ${OBJ_DBG:.o=.d}
 
${OBJDIR_DBG}/%.o: ${SRCDIR}/%.$(SRCEXT)
	echo ' [CC] '$<
	${CC} ${CFLAGS} ${CFLAGS_DBG} -c -o $@ $<
	${CC} -MM $< > $(patsubst %.o,%.d,$@)
 
makedirs:
	mkdir -p ${DIRTREE_DBG}
 
clean:
	rm -rf ${BUILDDIR}
	rm -f ${TARGET}
