INCFLAGS  = -I /usr/include/GL
INCFLAGS += -I ../../lib/vecmath

LINKFLAGS  = -framework GLUT -framework OpenGL -framework Cocoa
LINKFLAGS += -L ../../lib -lvecmath

CFLAGS    = -O2
CC        = $(CXX)
SRCS      = main.cpp obj.cpp
OBJS      = $(SRCS:.cpp=.o)
PROG      = a0

all: $(SRCS) $(PROG)

$(PROG): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(INCFLAGS) $(LINKFLAGS)

.cpp.o:
	$(CC) $(CFLAGS) $< -c -o $@ $(INCFLAGS)

depend:
	makedepend $(INCFLAGS) -Y $(SRCS)

clean:
	rm $(OBJS) $(PROG)

