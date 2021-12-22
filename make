echo "build librairie pca9685"
g++ -c pca9685.c -o obj/pca9685.o
echo "build main"
g++ -c main.cpp -I./include/ -o obj/main.o
echo "assemblage"
g++ -Wall -o ./bin/main -I./include/ obj/pca9685.o obj/main.o -lwiringPi -lpthread -lydlidar_sdk
