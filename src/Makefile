CXX=g++
CXXFLAGS=-c -Wall -pedantic -std=c++14
INCFLAGS=-I/home/bence/Documents/Workspace/PointCloudFramework/inc -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I../inc
SOURCES=Packet.cpp PacketData.cpp PacketHeader.cpp PcapReader.cpp VelodyneStreamer.cpp
OBJECTS=$(SOURCES:.cpp=.o)

all: $(OBJECTS)
	ar rvs ../lib/libvpcap.a $(patsubst %.o,../build/%.o,$(OBJECTS))

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCFLAGS) $< -o ../build/$@

clean:
	rm ../build/*.o
	rm ../lib/*.a