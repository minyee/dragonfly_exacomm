TARGET := libdfly_exploration.so 
SRC := exacomm_dragonfly_topology.cc dragonfly_switch.cc  exacomm_dragonfly_ugal_router.cc exacomm_dragonfly_ugalG_router.cc exacomm_dragonfly_valiant_router.cc exacomm_dragonfly_par_router.cc exacomm_dragonfly_minimal_router.cc

CXX :=    libsst++
CC :=     libsstcc
CXXFLAGS := -fPIC -std=c++11
CPPFLAGS := -I. -I$(HOME)/Programs/install/sst-core/clang/include
#CPPFLAGS := -I. -I$(HOME)/install/sstcore-7.1.0/include
LIBDIR :=  
PREFIX := 
LDFLAGS :=  -Wl,-rpath,$(PREFIX)/lib

OBJ := $(SRC:.cc=.o) 
OBJ := $(OBJ:.cpp=.o)
OBJ := $(OBJ:.c=.o)

.PHONY: clean install 

all: $(TARGET)

$(TARGET): $(OBJ) 
	$(CXX) -o $@ $+ $(LDFLAGS) $(LIBS)  $(CXXFLAGS)

%.o: %.cc 
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

clean: 
	rm -f $(TARGET) $(OBJ) 

install: $(TARGET)
	cp $< $(PREFIX)/bin

