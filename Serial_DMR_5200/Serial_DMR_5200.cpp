#include "SerialPort.h"
#include <iostream>

int main(int argc, char* argv[])
{
	bool rv;

  char* wComPort;
  char* wTime;
  char* wFile;

  if (argc < 4)
  {
    std::cout << "Serial_DMR_5200 <COM#> <Time to run in second> <Destination File>";
    return -1;
  }

  wComPort = argv[1];
  wTime = argv[2];
  wFile = argv[3];

	SerialPort serial;
  serial.SetDestinationFile(wFile);
	rv = serial.start(argv[1], 1200);
	if (rv == false) {
		return -1;
	}

	// initialize
	serial.end_of_line_char(0x0d);

  char wEnter[2] = { 27,0 };
  serial.write_some(wEnter, 1);


	// wait
	std::cout << "Ready for data" << std::endl;

	Sleep(std::atoi(wTime) * 1000);

	serial.stop();

	return 0;
}