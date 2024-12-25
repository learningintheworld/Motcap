#pragma once
#ifdef NATIVEUTILS_EXPORTS
#define EXPORT_API __declspec(dllexport)   
#else  
#define EXPORT_API __declspec(dllimport)   
#endif

namespace Hexacercle
{
	struct Quaternion
	{
		float x;
		float y;
		float z;
		float w;
#ifdef __cplusplus

		//Quaternion(const Quaternion& q1)
		//	:x(q1.x), y(q1.y), z(q1.z), w(q1.w)
		//{}

		//Quaternion() :
		//	x(0), y(0), z(0), w(0)
		//{}

		bool IsValid()
		{
			return x * x + y * y + z * z + w * w > (float)0.8
				&& x * x + y * y + z * z + w * w < (float)1.2;
		}

		// 将构造体改为方法, 防止extern "C"调用时报错
		static Quaternion MakeQuat(float X, float Y, float Z, float W)
		{
			Quaternion q;
			q.x = X;
			q.y = Y;
			q.z = Z;
			q.w = W;
			return q;
		}

		Quaternion Inverse()
		{
			return MakeQuat(-x, -y, -z, w);
		}

		static float Dot(Quaternion lhs, Quaternion rhs)
		{
			return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
		}

		Quaternion operator* (Quaternion rhs);
#endif // __cplusplus
	};

	extern "C" EXPORT_API void ToEulerAngles(Quaternion QuatIn, float* EulsArrayOut);
	extern "C" EXPORT_API Quaternion Euler(float Pitch, float Yaw, float Roll);
	extern "C" EXPORT_API void Normalize(Quaternion * in);
	extern "C" EXPORT_API Quaternion Multiply(Quaternion lhs, Quaternion rhs);
}