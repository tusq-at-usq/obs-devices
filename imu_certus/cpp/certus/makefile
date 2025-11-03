LIBRARIES=
ifeq ($(OS),Windows_NT)
	LIBRARIES += -lWs2_32
endif
LIBRARIES += -ljansson

all: stream_packets

stream_packets: rs232/rs232.c rs232/rs232.h an_packet_protocol.c an_packet_protocol.h ins_packets.c ins_packets.h stream_packets.c
	gcc -pedantic -Wall -o stream_packets rs232/rs232.c an_packet_protocol.c ins_packets.c stream_packets.c $(LIBRARIES)
	
clean:
	rm stream_packets
