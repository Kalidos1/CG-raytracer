#ifndef MAT3_H
#define MAT3_H
#include "vec3.h"

class vec3;

class mat3 {
public:
    double m[3][3];

    mat3() {
        // Initialize the matrix as an identity matrix
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] = i == j ? 1.0f : 0.0f;
            }
        }
    }

    mat3(const vec3&row_1, const vec3&row_2, const vec3&row_3) {
        //Build the matrix out of 3 vectors -> maybe can be done with for loop not sure
        m[0][0] = row_1.x();
        m[0][1] = row_1.y();
        m[0][2] = row_1.z();
        m[1][0] = row_2.x();
        m[1][1] = row_2.y();
        m[1][2] = row_2.z();
        m[2][0] = row_3.x();
        m[2][1] = row_3.y();
        m[2][2] = row_3.z();
    }

    // Function to create a rotation matrix around the given axis (normalized vector) and angle (in radians)
    // Might be better to convert this into one single matrix that governs all different axes -> Because of multiple axes rotation
    static mat3 rotation_matrix(const vec3&axis, const double angle) {
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        const double t = 1.0f - c;

        const double x = axis.x();
        const double y = axis.y();
        const double z = axis.z();

        return mat3{
            {t * x * x + c, t * x * y - s * z, t * x * z + s * y},
            {t * x * y + s * z, t * y * y + c, t * y * z - s * x},
            {t * x * z - s * y, t * y * z + s * x, t * z * z + c}
        };
    }

    // Function to create a translation matrix
    static mat3 translate(const vec3&translation) {
        return mat3{
            {1.0f, 0.0f, translation.x()},
            {0.0f, 1.0f, translation.y()},
            {0.0f, 0.0f, translation.z()}
        };
    }

    // Function to calculate the inverse of the matrix
    mat3 inverse() const {
        const double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

        const double invDet = 1.0f / det;

        mat3 result;
        result.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
        result.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
        result.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;
        result.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
        result.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
        result.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet;
        result.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
        result.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet;
        result.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;

        return result;
    }

    [[nodiscard]] vec3 operator*(const vec3&v) const {
        return {
            m[0][0] * v.x() + m[0][1] * v.y() + m[0][2] * v.z(),
            m[1][0] * v.x() + m[1][1] * v.y() + m[1][2] * v.z(),
            m[2][0] * v.x() + m[2][1] * v.y() + m[2][2] * v.z()
        };
    }
};

#endif //MAT3_H
