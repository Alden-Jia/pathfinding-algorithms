CC := g++
TARGET := search

ifeq ($(OS),Windows_NT)
    CFLAGS := -O2 -std=c++17 -Wall -c
    LFLAGS := -lgdi32 -luser32 -lcomdlg32 -luuid
    EXTENSION := .exe
    CLEANUP := del
    CLEANUP_OBJS := del *.o
else
    CFLAGS := -O2 -std=c++17 -Wall -c
    LFLAGS :=
    EXTENSION :=
    CLEANUP := rm -f
    CLEANUP_OBJS := rm -f *.o
endif

SRCS := $(wildcard *.cpp) $(wildcard */*.cpp)
HDRS := $(wildcard *.h)  $(wildcard */*.h)
OBJS := $(SRCS:.cpp=.o)

$(TARGET)$(EXTENSION): $(OBJS)
	$(CC) -O2 -std=c++17 -o $@ $(OBJS) $(LFLAGS)

%.o: %.cpp $(HDRS)
	$(CC) $(CFLAGS) $< -o $@

clean:
	-$(CLEANUP) $(TARGET)$(EXTENSION)
	-$(CLEANUP_OBJS)
