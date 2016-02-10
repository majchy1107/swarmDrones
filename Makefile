EXECNAME = ebee_stix

# all local source files
SOURCES += $(wildcard *.cpp) $(wildcard *.c)

CPPFLAGS += -Wall -I. -Ifwng/ap -Isipp 
CFLAGS +=
CXXFLAGS +=
LDFLAGS += -lm -L. -lgapi

# detect OS
UNAME := $(shell uname)

# Mac OS X specific
ifeq ($(UNAME), Darwin)
	CPPFLAGS += -arch i386
	LDFLAGS += -arch i386
	SOURCES += dashel/poll_emu.c
	LDFLAGS += -framework IOKit -framework CoreFoundation
endif




#########################
### make it happen

BUILDDIR = build

OBJECTS := $(addsuffix .o, $(addprefix ${BUILDDIR}/, ${SOURCES}))
DEPSFILES := $(addsuffix .o.d, $(addprefix ${BUILDDIR}/, ${SOURCES}))


all: ${EXECNAME}
	
${EXECNAME}: ${OBJECTS}
	${CXX} ${OBJECTS} ${LDFLAGS} -lpthread -o ${EXECNAME}

${BUILDDIR}/%.c.o: %.c
	@mkdir -p ${dir $@}
	@${CC} ${CPPFLAGS} -MM -MT ${BUILDDIR}/$(addsuffix .o, $<) $< | sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' > $(addsuffix .d, $@)
	${CC} ${CPPFLAGS} ${CFLAGS} -c $< -o $@
	
${BUILDDIR}/%.cpp.o: %.cpp
	@mkdir -p ${dir $@}
	@${CXX} ${CPPFLAGS} -MM -MT ${BUILDDIR}/$(addsuffix .o, $<) $< | sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' > $(addsuffix .d, $@)
	${CXX} ${CPPFLAGS} ${CXXFLAGS} -c $< -o $@

clean:
	rm -rf ${BUILDDIR} ${EXECNAME}	
	
-include ${DEPSFILES}
