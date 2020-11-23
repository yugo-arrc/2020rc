Main: main.o 
	g++ main.cpp `pkg-config --cflags --libs opencv` -o Main -lrealsense2 TCP.o
TCP.o: TCP/tcpSend.cpp
	g++ -c TCP/tcpSend.cpp -o TCP.o
clean:
	rm -f *.o Main
