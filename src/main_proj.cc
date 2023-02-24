#include "main_header.h"

/* CAAM 282 Final Project Main File
 *
 * Author: Noah Dohrmann
 *
 * Date: Winter 2023
 *
 * Note: In order to run this, the user needs the "Eigen" header files
 * found in eigen/Eigen from:
 * http://eigen.tuxfamily.org/index.php?title=Main_Page#Download
 */

int main(int argc, char** argv) {


	Vector4d v(0,0,1,1);
	Vector2d e(0,0);

	int id = 1;

	// create a new pedestrian p
	ped p(id, v, e, 10, 10, 120);

	/* p.print(); */

	point p1(1,0);
	point p2(0,1);

	line l(p1, p2);

	l.print();
	
	cout << endl;

	double dist = nearest_point(p, l);

	cout << "The distance is: " << dist << endl;

	return 0;

}
