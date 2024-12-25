#include "Quaternion.h"
#include <stdlib.h>
#include <assert.h>
#include <math.h>
// #include "usart.h"
#include "wit_task.h"

namespace YXWestimation {

#ifndef M_PI
    #define M_PI (3.14159265358979323846)
#endif

//int a = 0;

void Quaternion_set(double w, double v1, double v2, double v3, Quaternion* output)
{
    assert(output != NULL);
    output->w = w;
    output->v[0] = v1;
    output->v[1] = v2;
    output->v[2] = v3;
}

void Quaternion_setIdentity(Quaternion* q)
{
    assert(q != NULL);
    Quaternion_set(1, 0, 0, 0, q);
}

void Quaternion_copy(Quaternion* q, Quaternion* output)
{
    Quaternion_set(q->w, q->v[0], q->v[1], q->v[2], output);
}

bool Quaternion_equal(Quaternion* q_1, Quaternion* q_2)
{
    bool equalW  = fabs(q_1->w - q_2->w) <= QUATERNION_EPS;
    bool equalV0 = fabs(q_1->v[0] - q_2->v[0]) <= QUATERNION_EPS;
    bool equalV1 = fabs(q_1->v[1] - q_2->v[1]) <= QUATERNION_EPS;
    bool equalV2 = fabs(q_1->v[2] - q_2->v[2]) <= QUATERNION_EPS;
    return equalW && equalV0 && equalV1 && equalV2;
}

void Quaternion_fprint(FILE* file, Quaternion* q)
{
    fprintf(file, "(%.3f, %.3f, %.3f, %.3f)",
        q->w, q->v[0], q->v[1], q->v[2]);
}


void Quaternion_fromAxisAngle(double axis[3], double angle, Quaternion* output)
{
    assert(output != NULL);
    // Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
    output->w = cos(angle / 2.0);
    double c = sin(angle / 2.0);
    output->v[0] = c * axis[0];
    output->v[1] = c * axis[1];
    output->v[2] = c * axis[2];
}

double Quaternion_toAxisAngle(Quaternion* q, double output[3])
{
    assert(output != NULL);
    // Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/
    double angle = 2.0 * acos(q->w);
    double divider = sqrt(1.0 - q->w * q->w);

    if(divider != 0.0) {
        // Calculate the axis
        output[0] = q->v[0] / divider;
        output[1] = q->v[1] / divider;
        output[2] = q->v[2] / divider;
    } else {
        // Arbitrary normalized axis
        output[0] = 1;
        output[1] = 0;
        output[2] = 0;
    }
    return angle;
}

void Quaternion_fromXRotation(double angle, Quaternion* output)
{
    assert(output != NULL);
    double axis[3] = {1.0, 0, 0};
    Quaternion_fromAxisAngle(axis, angle, output);
}

void Quaternion_fromYRotation(double angle, Quaternion* output)
{
    assert(output != NULL);
    double axis[3] = {0, 1.0, 0};
    Quaternion_fromAxisAngle(axis, angle, output);
}

void Quaternion_fromZRotation(double angle, Quaternion* output)
{
    assert(output != NULL);
    double axis[3] = {0, 0, 1.0};
    Quaternion_fromAxisAngle(axis, angle, output);
}

void Quaternion_fromEulerZYX(double eulerZYX[3], Quaternion* output)
{
    assert(output != NULL);
    // Based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    double cy = cos(eulerZYX[2] * 0.5);
    double sy = sin(eulerZYX[2] * 0.5);
    double cr = cos(eulerZYX[0] * 0.5);
    double sr = sin(eulerZYX[0] * 0.5);
    double cp = cos(eulerZYX[1] * 0.5);
    double sp = sin(eulerZYX[1] * 0.5);

    output->w = cy * cr * cp + sy * sr * sp;
    output->v[0] = cy * sr * cp - sy * cr * sp;
    output->v[1] = cy * cr * sp + sy * sr * cp;
    output->v[2] = sy * cr * cp - cy * sr * sp;
}

void Quaternion_toEulerZYX(Quaternion* q, double output[3])
{
    assert(output != NULL);

    // Roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q->w * q->v[0] + q->v[1] * q->v[2]);
    double cosr_cosp = +1.0 - 2.0 * (q->v[0] * q->v[0] + q->v[1] * q->v[1]);
    output[0] = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = +2.0 * (q->w * q->v[1] - q->v[2] * q->v[0]);
    if (fabs(sinp) >= 1)
        output[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        output[1] = asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q->w * q->v[2] + q->v[0] * q->v[1]);
    double cosy_cosp = +1.0 - 2.0 * (q->v[1] * q->v[1] + q->v[2] * q->v[2]);
    output[2] = atan2(siny_cosp, cosy_cosp);
}

void Quaternion_toEulerXYZ(Quaternion* q, double output[3])
{
    assert(output != NULL);

    double m13 = 2*(q->w * q->v[1] + q->v[0] * q->v[2]);
    double m23 = 2*(q->v[1] * q->v[2] - q->v[0] * q->w);
    double m33 = 1 - 2*(q->v[0] * q->v[0] + q->v[1] * q->v[1]);
    double m12 = 2*(q->v[0] * q->v[1] - q->v[2] * q->w);
    double m11 = 1 - 2*(q->v[1] * q->v[1] + q->v[2] * q->v[2]);
    double m32 = 2*(q->w * q->v[0] + q->v[1] * q->v[2]);
    double m22 = 1 - 2*(q->v[0] * q->v[0] + q->v[2] * q->v[2]);

    output[1] = asin(limit_angle(m13, -1, 1));
    if (fabs(m13) < 0.9999999)
    {
        output[0] = atan2(-m23, m33);
        output[2] = atan2(-m12, m11);
    }
    else
    {
        output[0] = atan2(m32, m22);
        output[2] = 0;
    }

    // this._y = Math.asin( clamp( m13, - 1, 1 ) );
    // if ( Math.abs( m13 ) < 0.9999999 ) {
    //     this._x = Math.atan2( - m23, m33 );
    //     this._z = Math.atan2( - m12, m11 );
    // } else {
    //     this._x = Math.atan2( m32, m22 );
    //     this._z = 0;
    // }
}

void Quaternion_conjugate(Quaternion* q, Quaternion* output)
{
    assert(output != NULL);
    output->w = q->w;
    output->v[0] = -q->v[0];
    output->v[1] = -q->v[1];
    output->v[2] = -q->v[2];
}

void Quaternion_inverse(Quaternion* q, Quaternion* output)
{
		assert(output != NULL);
		double len = Quaternion_norm(q);
		Quaternion quat_conj;
		Quaternion_conjugate(q, &quat_conj);
		Quaternion_set(
        quat_conj.w / len,
        quat_conj.v[0] / len,
        quat_conj.v[1] / len,
        quat_conj.v[2] / len,
        output);
}

double Quaternion_norm(Quaternion* q)
{
    assert(q != NULL);
    return sqrt(q->w*q->w + q->v[0]*q->v[0] + q->v[1]*q->v[1] + q->v[2]*q->v[2]);
}

void Quaternion_normalize(Quaternion* q, Quaternion* output)
{
    assert(output != NULL);
    double len = Quaternion_norm(q);
    Quaternion_set(
        q->w / len,
        q->v[0] / len,
        q->v[1] / len,
        q->v[2] / len,
        output);
}

void Quaternion_multiply(Quaternion* q_1, Quaternion* q_2, Quaternion* output)
{
    assert(output != NULL);
    Quaternion result;

    /*
    Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
             a*e - b*f - c*g - d*h
        + i (b*e + a*f + c*h- d*g)
        + j (a*g - b*h + c*e + d*f)
        + k (a*h + b*g - c*f + d*e)
    */
    result.w =    q_1->w   *q_2->w    - q_1->v[0]*q_2->v[0] - q_1->v[1]*q_2->v[1] - q_1->v[2]*q_2->v[2];
    result.v[0] = q_1->v[0]*q_2->w    + q_1->w   *q_2->v[0] + q_1->v[1]*q_2->v[2] - q_1->v[2]*q_2->v[1];
    result.v[1] = q_1->w   *q_2->v[1] - q_1->v[0]*q_2->v[2] + q_1->v[1]*q_2->w    + q_1->v[2]*q_2->v[0];
    result.v[2] = q_1->w   *q_2->v[2] + q_1->v[0]*q_2->v[1] - q_1->v[1]*q_2->v[0] + q_1->v[2]*q_2->w   ;

    *output = result;
}

void Quaternion_rotate(Quaternion* q, double v[3], double output[3])
{
    assert(output != NULL);
    double result[3];

    double ww = q->w * q->w;
    double xx = q->v[0] * q->v[0];
    double yy = q->v[1] * q->v[1];
    double zz = q->v[2] * q->v[2];
    double wx = q->w * q->v[0];
    double wy = q->w * q->v[1];
    double wz = q->w * q->v[2];
    double xy = q->v[0] * q->v[1];
    double xz = q->v[0] * q->v[2];
    double yz = q->v[1] * q->v[2];

    // Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x;
    // p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y;
    // p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z;

    result[0] = ww*v[0] + 2*wy*v[2] - 2*wz*v[1] +
                xx*v[0] + 2*xy*v[1] + 2*xz*v[2] -
                zz*v[0] - yy*v[0];
    result[1] = 2*xy*v[0] + yy*v[1] + 2*yz*v[2] +
                2*wz*v[0] - zz*v[1] + ww*v[1] -
                2*wx*v[2] - xx*v[1];
    result[2] = 2*xz*v[0] + 2*yz*v[1] + zz*v[2] -
                2*wy*v[0] - yy*v[2] + 2*wx*v[1] -
                xx*v[2] + ww*v[2];

    // Copy result to output
    output[0] = result[0];
    output[1] = result[1];
    output[2] = result[2];
}

void Quaternion_slerp(Quaternion* q_1, Quaternion* q_2, double t, Quaternion* output)
{
//		uart_printf("%d\r\n", a); // 0-799
//		a++;
    Quaternion result, q_2_new;

//		uart_printf("%0.3f\r\n", q_1->w); // nan
//		uart_printf("%0.3f\r\n", q_2->w); // have value and nan(first)
    // Based on http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
    double cosHalfTheta = q_1->w*q_2->w + q_1->v[0]*q_2->v[0] + q_1->v[1]*q_2->v[1] + q_1->v[2]*q_2->v[2];
//		uart_printf("%0.3f\r\n", cosHalfTheta); //nan
			
		// If the cosHalfTheta is negative, invert one of the quaternions to ensure the selection of the shortest interpolation path
    if (cosHalfTheta < 0.0) {
        cosHalfTheta = -cosHalfTheta;
        q_2_new.w = -q_2->w;
        q_2_new.v[0] = -q_2->v[0];
        q_2_new.v[1] = -q_2->v[1];
        q_2_new.v[2] = -q_2->v[2];
    } else {
        q_2_new.w = q_2->w;
        q_2_new.v[0] = q_2->v[0];
        q_2_new.v[1] = q_2->v[1];
        q_2_new.v[2] = q_2->v[2];
    }

    // if q1=q2 or qa=-q2 then theta = 0 and we can return qa
    if (fabs(cosHalfTheta) >= 1.0) {
//				uart_printf("%d\r\n", 111);
        Quaternion_copy(q_1, output);
        return;
    }

    double halfTheta = acos(cosHalfTheta);
    double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
//		uart_printf("%0.3f, %0.3f\r\n", halfTheta, sinHalfTheta); // nan
    // If theta = 180 degrees then result is not fully defined
    // We could rotate around any axis normal to q1 or q2
    if (fabs(sinHalfTheta) < QUATERNION_EPS) {
//				uart_printf("%d\r\n", 222);
        result.w = (q_1->w * 0.5 + q_2_new.w * 0.5);
        result.v[0] = (q_1->v[0] * 0.5 + q_2_new.v[0] * 0.5);
        result.v[1] = (q_1->v[1] * 0.5 + q_2_new.v[1] * 0.5);
        result.v[2] = (q_1->v[2] * 0.5 + q_2_new.v[2] * 0.5);
    } else {
//				uart_printf("%d\r\n", 333); // go to this
        // Default quaternion calculation
        double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
        double ratioB = sin(t * halfTheta) / sinHalfTheta;
//				uart_printf("%0.3f, %0.3f\r\n", ratioA, ratioB); // nan
        result.w = (q_1->w * ratioA + q_2_new.w * ratioB);
        result.v[0] = (q_1->v[0] * ratioA + q_2_new.v[0] * ratioB);
        result.v[1] = (q_1->v[1] * ratioA + q_2_new.v[1] * ratioB);
        result.v[2] = (q_1->v[2] * ratioA + q_2_new.v[2] * ratioB);
//				uart_printf("%0.3f\r\n", result.w); // nan
    }
    *output = result;
}

void Quaternion_average(Quaternion *quaternions, int count, Quaternion *average)
{
    average->w = 0.0;
    average->v[0] = 0.0;
    average->v[1] = 0.0;
    average->v[2] = 0.0;

    for (int i = 0; i < count; i++) {
        Quaternion temp;
        Quaternion_slerp(average, &quaternions[i], 1.0 / (i + 1), &temp);
//				uart_printf("%0.3f\r\n", average->w); // 0.000(first) and nan, because of the quaternions[0] is nan
//				uart_printf("%0.3f\r\n", temp.w); // nan
        *average = temp;
    }
}


}
