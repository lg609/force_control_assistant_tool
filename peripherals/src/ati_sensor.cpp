#include "../include/ATISensor.h"

double ATISensor::s_range[SENSOR_DIMENSION] = {300,300,800,10,10,15};

 ATISensor::ATISensor(std::string IPAddress)
 {
    ipAddress = /*IPAddress*/"192.168.1.1";
    memset(&ftResponse, 0, sizeof(ATIResponse));
 }

 void ATISensor::getFTSensorRange(double range[])
 {
     memcpy(range, s_range, sizeof(double)*SENSOR_DIMENSION);
 }

/* Sleep ms milliseconds */
void ATISensor::mySleep(unsigned long ms)
{
	usleep(ms * 1000);
}

int ATISensor::connectSensor(SOCKET_HANDLE * handle, uint16 port)
{
	struct sockaddr_in addr;
	struct hostent *he;
	int err;

    *handle = socket(AF_INET, SOCK_DGRAM, /*IPPROTO_TCP*/0);

    if (*handle == -1)
    {
		fprintf(stderr, "Socket could not be opened.\n");
		return -2;
	}
    he = gethostbyname(ipAddress.c_str());
    if(he == 0)
        return -4;
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
    fprintf(stdout, "Connecting to ATI\r\n");
	fflush(stdout);
	err = connect(*handle, (struct sockaddr *)&addr, sizeof(addr));
	if (err < 0) {
		return -3;
	}
	return 0;
}

void ATISensor::closeSensor(SOCKET_HANDLE * handle)
{
	close(*handle);
}

int ATISensor::readSensor(SOCKET_HANDLE * socket, ATIResponse &resp)
{
    byte request[8];			/* The request data sent to the Net F/T. */
    byte response[36];			/* The raw response data received from the Net F/T. */
    int readSuccess;
    int sendSuccess;
    *(uint16*)&request[0] = htons(0x1234); /* standard header. */
    *(uint16*)&request[2] = htons(ATI_COMMAND); /* per table 9.1 in Net F/T user manual. */
    *(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
    try
    {
        sendSuccess = send(*socket, &request, 8, 0);
    }
    catch(...)
    {
        return 3;

    }
	if (sendSuccess < 0) {
		return sendSuccess;
	}
    readSuccess = recv(*socket, response, 36, 0);
    resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
    resp.ft_sequence = ntohl(*(uint32*)&response[4]);
    resp.status = ntohl(*(uint32*)&response[8]);
    for(int i = 0; i < 6; i++)
    {
        resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
    }

    if (readSuccess != 36) {
        return 1;
	}

	return 0;
}


void ATISensor::showResponse(ATIResponse * r, float ft[])
{

    double Fx = (double)r->FTData[0] / 1000000;
    double Fy = (double)r->FTData[1] / 1000000;
    double Fz = (double)r->FTData[2] / 1000000;
    double Tx = (double)r->FTData[3] / 1000000;
    double Ty = (double)r->FTData[4] / 1000000;
    double Tz = (double)r->FTData[5] / 1000000;
    ft[0] = Fx; ft[1] = Fy; ft[2] = Fz; ft[3] = Tx; ft[4] = Ty; ft[5] = Tz;
}


bool ATISensor::initialFTSensor()
{
    if (connectSensor(&socketHandle, ATI_PORT) != 0)
    {
		fprintf(stderr, "Could not connect to device...\r\n");
        return false;
	}
    else
    {
		fprintf(stdout, "Connected to Ethernet DAQ\r\n");
        fflush(stdout);
        return true;
	}
}

bool ATISensor::obtainFTSensorData(float m_ftData[SENSOR_DIMENSION])
{
    int readSuccess;
    try
    {
        readSuccess = readSensor(&socketHandle, ftResponse);
    }
    catch(...)
    {

    }

    if (readSuccess == 0)
    {
        showResponse(&ftResponse, m_ftData);
    }
    else
    {
        fprintf(stderr, "Could not read F/T data, error code: %d\r\n", readSuccess);
    }

    return readSuccess==0;
}

bool ATISensor::uninitialFTSensor()
{
    close(socketHandle);
    return true;
}

