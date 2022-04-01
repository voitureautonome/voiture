#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <opencv2/opencv.hpp>
#include "pca9685.h"
#include <src/CYdLidar.h>
#include "time.h"
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 60
#define M_PI 3.14159265358979323846

#define AVANCER 1
#define DEFAULT 0
#define EVITEMENT 2

int etat = DEFAULT;
bool obstacleProche(LaserPoint p);
int decision(LaserScan scan);

/**
 * Calculate the number of ticks the signal should be high for the required amount of time
 */
int calcTicks(float impulseMs, int hertz)
{
	float cycleMs = 1000.0f / hertz;
	return (int)(MAX_PWM * impulseMs / cycleMs + 0.5f);
}

/**
 * input is [0..1]
 * output is [min..max]
 */
float map(float input, float min, float max)
{
	return (input * max) + (1 - input) * min;
}

void arreter() {
	pwmWrite(PIN_BASE + 4, 0);
	pwmWrite(PIN_BASE + 5, 0);
}

void avancer() {
	pwmWrite(PIN_BASE + 4, calcTicks(12, HERTZ));
	pwmWrite(PIN_BASE + 5, calcTicks(12, HERTZ));
}


void tourner(float value){
	float r=(value+45.f)/100;
	float millis = map(r, 1, 2);
    int tick = calcTicks(millis, HERTZ);
    pwmWrite(PIN_BASE + 0, tick);
}
void toutDroit(){
    tourner(12);
}
void tournerGauche() {
	tourner(-45.f);
}
void tournerDroite(){
	tourner(45.f);
}
void evitement(int val){
	if (val == 1)// obstacle à gauche
	{
		tournerDroite();
	} else {
		tournerGauche();
	}
}

using namespace cv;
using namespace ydlidar;

void lidar();
float radToDeg(float rad);
int main()
{
	printf("Bonjour de la part de %s !\n", "VoitureAutonome");
	wiringPiSetup();
	pinMode(23, OUTPUT);
	pinMode(30, OUTPUT);
	pinMode(1, OUTPUT);
	digitalWrite(30, 2);
	digitalWrite(1, 1);
	puts("Wiring ready");

	int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
	if (fd < 0)
	{
		printf("Error in setup\n");
		return fd;
	}
	tournerGauche();
	delay(500);
	tournerDroite();
	delay(500);
	toutDroit();
	lidar();
	// Reset all output
	pca9685PWMReset(fd);

	float millis = 1.5;
	int tick = calcTicks(millis, HERTZ);
	pwmWrite(PIN_BASE + 16, tick);
	delay(2000);
	// wiringPiI2CSetup(0x48);
	// wiringPiI2CWriteReg8(0x48, 0x40, 1);
	int i, j = 1;
	int pin;
	while (j)
	{
		printf("Enter servo pin [0-16]: ");
		scanf("%d", &pin);

		if (pin >= 0 && pin <= 16)
		{
			millis = 1.5f;
			i = 1;

			pwmWrite(PIN_BASE + pin, calcTicks(millis, HERTZ));
			printf("Servo %d is centered at %1.2f ms\n", pin, millis);

			while (i)
			{
				printf("Enter milliseconds: ");
				scanf("%f", &millis);

				if (millis > 0 && millis <= 15)
				{
					pwmWrite(PIN_BASE + pin, calcTicks(millis, HERTZ));
					delay(1000);
				}
				else
					i = 0;
			}
		}
		else
			j = 0;
	}

	/*cv::Mat image = cv::Mat::zeros(300, 600, CV_8UC3);
	circle(image, Point(250, 150), 100, Scalar(0, 255, 128), -100);
	circle(image, Point(350, 150), 100, Scalar(255, 255, 255), -100);
	imshow("Display Window", image);*/
	// waitKey(0);
	getc(stdin);
	return 0;
}

void lidar()
{
	ydlidar::os_init();

	CYdLidar laser;
	//////////////////////string property/////////////////
	/// Lidar ports
	std::map<std::string, std::string> ports = ydlidar::lidarPortList();
	std::string port = "/dev/ydlidar";
	if (!ports.empty())
	{
		port = ports.begin()->second;
	}
	/// lidar port
	laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
	/// ignore array
	std::string ignore_array;
	ignore_array.clear();
	laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
					  ignore_array.size());

	//////////////////////int property/////////////////
	/// lidar baudrate
	int optval = 115200;
	laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
	/// tof lidar
	optval = TYPE_TRIANGLE;
	laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
	/// device type
	optval = YDLIDAR_TYPE_SERIAL;
	laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
	/// sample rate
	optval = 4;
	laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
	/// abnormal count
	optval = 4;
	laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

	//////////////////////bool property/////////////////
	/// fixed angle resolution
	bool b_optvalue = false;
	laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
	/// rotate 180
	laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
	/// Counterclockwise
	laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
	b_optvalue = true;
	laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
	/// one-way communication
	b_optvalue = true;
	laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
	/// intensity
	b_optvalue = false;
	laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
	/// Motor DTR
	b_optvalue = false;
	laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
	/// HeartBeat
	b_optvalue = false;
	laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

	//////////////////////float property/////////////////
	/// unit: °
	float f_optvalue = 180.0f;
	laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
	f_optvalue = -180.0f;
	laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
	/// unit: m
	f_optvalue = 16.f;
	laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
	f_optvalue = 0.1f;
	laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
	/// unit: Hz
	f_optvalue = 10.f;
	laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

	// initialize SDK and LiDAR
	bool ret = laser.initialize();
	if (ret)
	{ // success
		// Start the device scanning routine which runs on a separate thread and enable motor.
		ret = laser.turnOn();
	}
	else
	{
		fprintf(stderr, "%s\n", laser.DescribeError());
		fflush(stderr);
	}

	float periode = 0.5;
	time_t lastime;
	time(&lastime);
	


	// Turn On success and loop
	while (ret && ydlidar::os_isOk())
	{
		if(etat != 0) {
			time_t now;
			time(&now);
			if(now - lastime < periode) continue;
			else etat = 0;
		}


		LaserScan scan;
		if (laser.doProcessSimple(scan))
		{
			int d = decision(scan);
			std::cout<<"decision "<<d<<std::endl;
			if (etat == EVITEMENT)
			{
				switch (d)
				{
				case 0:
					arreter();
					ret = false;
					break;
				case -1:
					toutDroit();
					etat = AVANCER;
				default:
					break;
				}
			}
			else {
				switch (d)
				{
				case 0:
					arreter();
					ret = false;
					break;
				case 1:// obstacle à gauche
					evitement(1);
					etat = EVITEMENT;
					break;
				case 2:// obstacle à droite
					evitement(2);
					etat = EVITEMENT;
					break;
				default:
					toutDroit();
					avancer();
					etat = AVANCER;
				}
			}
			



			// fprintf(stdout, "Scan received[%llu]: %u ranges is [%f]Hz\n",
			// 		scan.stamp,
			// 		(unsigned int)scan.points.size(), 1.0 / scan.config.scan_time);
			// std::cout << scan.points.at(scan.points.size() - 1).range << std::endl;
			for (size_t i = 0; i < scan.points.size(); i++)
			{
				LaserPoint point = scan.points.at(i);
				float angle = radToDeg(point.angle);
				// printf("angle : %f distance : %f \n",radToDeg(point.angle),point.range);
				if (angle >= 0.001f && angle <= 0.1f)
				{
					printf("angle %f", angle);
				}
			}

			fflush(stdout);
		}
		else
		{
			fprintf(stderr, "Failed to get Lidar Data\n");
			fflush(stderr);
		}
	}
}

float radToDeg(float rad)
{
	return rad * (180 / M_PI);
}

bool obstacleProche(LaserPoint p) {
	if (p.range == 0) return false;
	if (p.range < 0.15f) return true;
	else return false;
}

bool obstacleMiDist(LaserPoint p) {
	if (p.range == 0) return false;
	if (p.range > 0.15f && p.range < 0.40f) return true;
	else return false;
}

bool droiteLibre(LaserScan scan) {
	float angleMax = 20.f;
	float debut = 0.f;
	bool res = true;

	for(LaserPoint p : scan.points) {
		float ang = radToDeg(p.angle);
		if(p.range > 0.3) break;
		if((ang > debut) && (ang > debut + angleMax/2) && (ang < debut + 2*angleMax)) return false;
	}
	return res;
}

bool gaucheLibre(LaserScan scan) {
	float angleMax = 20.f;
	float debut = 0.f;
	bool res = true;

	for(LaserPoint p : scan.points) {
		float ang = radToDeg(p.angle);
		if(p.range > 0.3) break;
		if((ang > debut) && (ang < debut - angleMax/2) && (ang < debut - 2*angleMax)) return false;
	}
	return res;
}

int decision(LaserScan scan) {

	float angleMax = 20.f;
	float debut = 0.f;
	float finDroite = 180.f;
	float finGauche = 180.f;

	
	for(LaserPoint p : scan.points)
	{
		float ang = radToDeg(p.angle);
		
		// Obstacle devant le véhicule qui est proche
		bool arret = (ang < debut + angleMax / 2 || ang > debut - angleMax / 2) && obstacleProche(p);
		if (arret) {
			printf("angle du point %f\n", ang);
			printf("distance du point %f\n", ang);

			return 0;

		}


		bool obstGauche = (ang < debut && (ang < debut - angleMax / 2) && (ang > debut - 2*angleMax)) && obstacleMiDist(p);
		if (obstGauche && droiteLibre(scan))
		{
			printf("angle du point %f\n", ang);
			printf("distance du point %f\n", ang);
			return 1;
		}

		bool obstDroite = (ang > debut && (ang > debut + 2*angleMax / 2) && (ang < debut + 2*angleMax)) && obstacleMiDist(p);
		if (obstDroite && gaucheLibre(scan))
		{
			printf("angle du point %f\n", ang);
			printf("distance du point %f\n", ang);
			return 2;
		}

		

	}
	return -1;
}