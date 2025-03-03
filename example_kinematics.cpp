// robot geometry
// (look at pics above for explanation)
// TODO: CITE THIShttps://hypertriangle.com/~alex/delta-robot-tutorial/

#include <stdio.h>
#include <math.h>
#include <vector>

typedef struct {
  float x, y, z;
} Point;

typedef struct {
  float theta1, theta2, theta3;
} DeltaJoints;

constexpr float e = 73.481; // Platform equilateral triangle side length [mm]
constexpr float f = 261.306; // Base equilateral triangle side length [mm]
constexpr float re = 174.0; // Passive Link Length [mm]
constexpr float rf = 108.0; // Active Link Length [mm]

// trigonometric constants
const float sqrt3 = sqrt(3.0);
constexpr float pi = 3.141592653;    // PI
const float sin120 = sqrt3 / 2.0;
constexpr float cos120 = -0.5;
const float tan60 = sqrt3;
constexpr float sin30 = 0.5;
const float tan30 = 1 / sqrt3;

// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int delta_fk(float theta1, float theta2, float theta3, float& x0, float& y0, float& z0) {
  float t = (f - e) * tan30 / 2;

  float y1 = -(t + rf * cos(theta1));
  float z1 = -rf * sin(theta1);

  float y2 = (t + rf * cos(theta2)) * sin30;
  float x2 = y2 * tan60;
  float z2 = -rf * sin(theta2);

  float y3 = (t + rf * cos(theta3)) * sin30;
  float x3 = -y3 * tan60;
  float z3 = -rf * sin(theta3);

  float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

  float w1 = y1 * y1 + z1 * z1;
  float w2 = x2 * x2 + y2 * y2 + z2 * z2;
  float w3 = x3 * x3 + y3 * y3 + z3 * z3;

  // x = (a1*z + b1)/dnm
  float a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

  // y = (a2*z + b2)/dnm;
  float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

  // a*z^2 + b*z + c = 0
  float a = a1 * a1 + a2 * a2 + dnm * dnm;
  float b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
  float c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);

  // discriminant
  float d = b * b - (float)4.0 * a * c;
  if (d < 0) return -1; // non-existing point

  z0 = -(float)0.5 * (b + sqrt(d)) / a;
  x0 = (a1 * z0 + b1) / dnm;
  y0 = (a2 * z0 + b2) / dnm;
  return 0;
}

// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
int delta_calcAngleYZ(float x0, float y0, float z0, float& theta) {
  float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
  y0 -= 0.5 * 0.57735 * e;    // shift center to edge
  // z = a + b*y
  float a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
  float b = (y1 - y0) / z0;
  // discriminant
  float d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
  if (d < 0) return -1; // non-existing point
  float yj = (y1 - a * b - sqrt(d)) / (b * b + 1); // choosing outer point
  float zj = a + b * yj;
  theta = 180.0 * atan(-zj / (y1 - yj)) / pi + ((yj > y1) ? 180.0 : 0.0);
  return 0;
}

// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
int delta_ik(float x0, float y0, float z0, float& theta1, float& theta2, float& theta3) {
  theta1 = 0;
  theta2 = 0;
  theta3 = 0;
  int status = delta_calcAngleYZ(x0, y0, z0, theta1);
  if (status == 0) status = delta_calcAngleYZ(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0, theta2);  // rotate coords to +120 deg
  if (status == 0) status = delta_calcAngleYZ(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0, theta3);  // rotate coords to -120 deg
  return status;
}

std::vector<Point> pringleTrajectory() {
  // Circle Trajectory in XY plane while Z coordinate goes through 2 cycles of a sine wave
  const int num_points = 100;
  const float circle_center_z = -180.0;
  const float amplitude = 25.0;

  std::vector<float> t(num_points);
  float step = (2 * M_PI) / (num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    t[i] = i * step;
  }

  std::vector<float> x_circle(num_points);
  std::vector<float> y_circle(num_points);
  std::vector<float> z_circle(num_points);
  for (int i = 0; i < num_points; ++i) {
    x_circle[i] = (2.0 * amplitude) * cos(t[i]);
    y_circle[i] = (2.0 * amplitude) * sin(t[i]);
    z_circle[i] = circle_center_z + amplitude * sin(2 * t[i]);
  }

  // Create trajectory
  std::vector<Point> trajectory(num_points);
  for (int i = 0; i < num_points; ++i) {
    trajectory[i].x = x_circle[i];
    trajectory[i].y = y_circle[i];
    trajectory[i].z = z_circle[i];
  }

  // Log the created trajectory
  printf("EE Trajectory created with %ld points\n", trajectory.size());
  for (int i = 0; i < num_points; i++) {
    Point p = trajectory[i];
    printf("\t EE Point %d: (%.2f, %.2f, %.2f)\n", i + 1, p.x, p.y, p.z);
  }

  return trajectory;
}

void testing() {
  // Test some values
  float x0, y0, z0;
  float theta1, theta2, theta3;

  // Test forward kinematics
  int status = delta_fk(15, 15, 15, x0, y0, z0);
  // int status = delta_fk(5, 5, 5, x0, y0, z0);
  if (status == 0) {
    printf("x0=%f, y0=%f, z0=%f\n", x0, y0, z0);
  } else {
    printf("non-existing point\n");
  }

  // Test inverse kinematics
  status = delta_ik(0, 0, -100, theta1, theta2, theta3);
  if (status == 0) {
    // convert degrees to radians
    float dtr = pi / (float)180.0;
    theta1 *= dtr;
    theta2 *= dtr;
    theta3 *= dtr;
    printf("theta1=%f, theta2=%f, theta3=%f\n", theta1, theta2, theta3);
  } else {
    printf("non-existing point\n");
  }
}

int main() {
  std::vector<Point> trajectory = pringleTrajectory();
  int num_points = trajectory.size();

  std::vector<DeltaJoints> joints(num_points);
  for (int i = 0; i < num_points; i++) {
    float x0 = trajectory[i].x;
    float y0 = trajectory[i].y;
    float z0 = trajectory[i].z;

    float theta1, theta2, theta3;
    int status = delta_ik(x0, y0, z0, theta1, theta2, theta3);
    if (status == 0) {
      joints[i].theta1 = theta1;
      joints[i].theta2 = theta2;
      joints[i].theta3 = theta3;
    } else {
      printf("Point %d is non-existing (%.3f, %.3f, %.3f) \n", i + 1, x0, y0, z0);
    }
  }

  // Print the resulting Joint trajectory
  printf("Trajectory created with %ld points:\n", joints.size());
  for (int i = 0; i < num_points; i++) {
    printf("Joint %d: theta1=%.3f, theta2=%.3f, theta3=%.3f\n", i + 1, joints[i].theta1, joints[i].theta2, joints[i].theta3);
  }

  return 0;
}