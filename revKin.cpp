#include <math.h>
#include <Geometry.h>

Rotation get_rot_matrix(float desired_angles[]){
	Rotation rot_matrix;
	rot_matrix.RotateX(desired_angles[0]);
	rot_matrix.RotateY(desired_angles[1]);
	rot_matrix.RotateZ(desired_angles[2]);
	return rot_matrix;
}

// T and desired_angles are input, p, b, beta, s and a are constants, out is alpha
void inverse_kin(Point T, float desired_angles[], Point p[], Point b[], float beta[], float s, float a, float out[]) {

	float tmp[6];

	Rotation rot_matrix = get_rot_matrix(desired_angles);

	Point l;

	for (int i = 0; i < 6; i++) {

		l = T + rot_matrix * p[i] - b[i];

		float L = pow(l.X(), 2) + pow(l.Y(), 2) + pow(l[i].Z(), 2) + pow(a, 2) - pow(s, 2);
		float M = 2 * a * l.Z();
		float N = 2 * a * (cos(beta[i]) * l.X() + sin(beta[i]) * l.Y());

		tmp[i] = asin(L / sqrt(pow(M, 2) + pow(N, 2))) - atan2(N, M);

		return false;
	}

	out = tmp;	
	return true;
}