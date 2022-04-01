#include <src/CYdLidar.h>


boolean obstacleProche(LaserPoint p) {
	if (p.range == 0) return false;
	if (p.range < 20) return true;
	else return false;
}

boolean obstacleMiDist(LaserPoint p) {
	if (p.range == 0) return false;
	if (p.range > 20 et p.range < 40) return true;
	else return false;
}

int decision(LaserScan scan) {

	float angleMax = 30.f;
	float moitie = 180.f;
	float debut = 0.f;
	float fin = 359.99f;

	for each (LaserPoint p in scan.points)
	{
		float ang = radToDeg(p.angle);
		
		// Obstacle devant le véhicule qui est proche
		bool arret = (p.angle < debut + angleMax / 2 || p.angle > fin - angleMax / 2) && obstacleProche(p);
		if (arret)
			return 0;


		bool obstGauche = (p.angle > moitie && p.angle < fin) && (p.angle < angleMax / 2) && (p.angle > angleMax);
		if (obstGauche)
		{
			return 1;
		}

		bool obstDroite = (p.angle < moitie && p.angle > debut) && (p.angle > angleMax / 2) && (p.angle < angleMax);
		if (obstGauche)
		{
			return 1;
		}

	}
}
