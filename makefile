default:
	g++ src/simulacion14.cpp -o bin/14.exe
	g++ src/simulacion15.cpp -o bin/15.exe
	g++ src/simulacion16.cpp -o bin/16.exe

build14:
	g++ src/simulacion14.cpp -o bin/14.exe

build15:
	g++ src/simulacion15.cpp -o bin/15.exe

build16:
	g++ src/simulacion16.cpp -o bin/16.exe

run:
	./bin/14.exe
	./bin/15.exe
	./bin/16.exe

run14:
	./bin/14.exe

run15:
	./bin/15.exe

run16:
	./bin/16.exe

test:
	$(RM) bin/*.exe
	$(RM) bin/*.plt
	g++ src/simulacion14.cpp -o bin/14.exe
	g++ src/simulacion15.cpp -o bin/15.exe
	g++ src/simulacion16.cpp -o bin/16.exe
	./bin/14.exe
	./bin/15.exe
	./bin/16.exe

clean:
	$(RM) bin/*.exe
	$(RM) bin/*.plt
