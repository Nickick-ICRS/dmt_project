#include <pose_calculator.hpp>

float mag(cv::Vec3f v) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

float dot(cv::Vec3f a, cv::Vec3f b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

cv::Vec3f cross(cv::Vec3f a, cv::Vec3f b) {
    cv::Vec3f res;
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
    return res;
}

float calc_angle(cv::Vec3f axis, cv::Vec3f A, cv::Vec3f B) {
    // Find the rotation about axis to get B to A

    // Using Rodriquez' formula
    // A = B*cos(theta) + (axis x B)*sin(theta)
    //   + axis*dot(axis, B)*(1-cos(theta));
    // Rearrange for tan(theta) and we have a vector equation for which
    // we want a scalar solution. Each equation has two solutions, so
    // Find which solution is duplicated
    bool solution_found = false;
    double a, b, c;
    double res[9];

    // normalise
    axis /= mag(axis);
    A /= mag(A);
    B /= mag(B);
    
    for(short i = 0; i < 3; i++) {
        a = A[i] - 2 * axis[i] * dot(axis, B) + B[i];
        b = -2 * cross(axis, B)[i];
        c = A[i] - B[i];

        if(b*b - 4*a*c >= 0) {
            solution_found = true;

            res[3*i] = -b + sqrt(b*b - 4*a*c);
            res[3*i+1] = -b - sqrt(b*b - 4*a*c);
            res[3*i+2] = 2*a;
        }
        else {
            res[3*i+2] = 0;
        }
    }

    if(!solution_found) {
        std::cout << "No rotational solution could be found\n";
        return 0;
    }

    // Calculate angle
    float theta;
    bool theta_found = false;
    // Check for a result which matches with another
    for(char i = 0; i < 3 && !theta_found; i++) {
        if(i == 3) {
            std::cout << "Theta not found" << std::endl;
            return 0;
        }
        if(res[3*i+2] != 0) {
            for(char j = i+1; j < 3 && !theta_found; j++) {
                if(res[3*j+2] != 0) {
                    if(fabs(
                        res[3*i] / res[3*i+2] 
                      - res[3*j] / res[3*j+2]
                       ) < 1e-2) {
                        theta = 2*atan2(res[3*i], res[3*i+2]);
                        theta_found = true;
                    }
                    else if(fabs(
                        res[3*i+1] / res[3*i+2] 
                      - res[3*j+1] / res[3*j+2]
                       ) < 1e-2) {
                        theta = 2*atan2(res[3*i+1], res[3*i+2]);
                        theta_found = true;
                    }
                }
            }
        }
    }
    return theta;
}

void get_rotation_matrix(cv::Matx33f &rot, 
                         Triangle old_pos, 
                         Triangle new_pos) {
    // Step 1 Align point A at the origin for both triangles
    old_pos.B -= old_pos.A;
    old_pos.C -= old_pos.A;
    old_pos.A -= old_pos.A;
    
    new_pos.B -= new_pos.A;
    new_pos.C -= new_pos.A;
    new_pos.A -= new_pos.A;

    // Step 2, find a rotation to get AB aligned, by rotating around 
    // the cross product of the two vectors
    cv::Vec3f AB_old = old_pos.A - old_pos.B;
    cv::Vec3f AB_new = new_pos.A - new_pos.B;

    // Normalise
    AB_old /= mag(AB_old);
    AB_new /= mag(AB_new);

    // Find the rotation matrix by using Rodriguez' formula
    // theta = pi, axis = (AB_old + AB_new) / 2
    
    cv::Vec3f r = AB_old + AB_new;
    cv::Matx13f r_t;
    cv::transpose(r, r_t);
    
    cv::Matx33f I{1, 0, 0,
                  0, 1, 0,
                  0, 0, 1};
    
    cv::Matx33f rot1 = (2 / (r_t * r)[0]) * (r * r_t) - I;
    
    // Rotate new triangle
    AB_new = rot1 * AB_new;

    new_pos.A = rot1 * new_pos.A;
    new_pos.B = rot1 * new_pos.B;
    new_pos.C = rot1 * new_pos.C;
    
    // Step 3 find rotation about AB to get AC_new to AC_old

    // Find new vectors
    cv::Vec3f AC_old = old_pos.A - old_pos.C;
    cv::Vec3f AC_new = new_pos.A - new_pos.C;
    AB_new = new_pos.A - new_pos.B;

    // Normalise
    AC_old /= mag(AC_old);
    AC_new /= mag(AC_new);
    AB_new /= mag(AB_new);

    float theta = calc_angle((AB_old+AB_new)/2.f, AC_old, AC_new);
    
    // Calculate the rotation matrix
    float si = sin(theta);
    float co = cos(theta);
    float t = 1 - co;
    float x = AB_old[0];
    float y = AB_old[1];
    float z = AB_old[2];
    cv::Matx33f rot2 = { t*x*x + co  ,  t*x*y - z*si, t*x*z + y*si,
                         t*x*y + z*si, t*y*y + co   , t*y*z - x*si,
                         t*x*z - y*si, t*y*z + x*si , t*z*z + co };

    // Rotate the new triangle
    new_pos.A = rot2 * new_pos.A;
    new_pos.B = rot2 * new_pos.B;
    new_pos.C = rot2 * new_pos.C;
    
    // Update the output rotation matrix as the product of the two rotations
    rot = rot2*rot1;
}
