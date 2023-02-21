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


	Vector4d v(1,1,1,1);
	Vector4d w(2,2,2,2);

	Vector2d e(0,0);

	int id = 1;

	// create a new pedestrian p
	ped p(id, v, e, 10, 10, 120);

	p.print();


	return 0;

}
