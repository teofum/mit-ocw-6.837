UNAME := $(shell uname)

ifeq ($(UNAME),Darwin)
INCFLAGS = -I ../../lib/vecmath

FRAMEWORKS = -framework Glut
FRAMEWORKS += -framework OpenGL

LINKFLAGS += -L ../../lib -lvecmath
else
INCFLAGS = -I /usr/include/GL
INCFLAGS += -I ../../lib/vecmath

LINKFLAGS = -lglut -lGL -lGLU
LINKFLAGS += -L ../../lib -lvecmath
endif

CFLAGS    = -O2 -Wall -Wno-deprecated-declarations -DSOLN
CC        = $(CXX)
SRCS      = main.cpp parse.cpp curve.cpp surf.cpp camera.cpp
OBJS      = $(SRCS:.cpp=.o)
PROG      = a1

all: $(SRCS) $(PROG)

$(PROG): $(OBJS)
	$(CC) $(FRAMEWORKS) $(CFLAGS) $(OBJS) -o $@ $(INCFLAGS) $(LINKFLAGS)

.cpp.o:
	$(CC) $(CFLAGS) $< -c -o $@ $(INCFLAGS)

depend:
	makedepend $(INCFLAGS) -Y $(SRCS)

clean:
	rm $(OBJS) $(PROG)
