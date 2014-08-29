#ifndef SE306P1_UPSTAGE_QUATERNION_HPP_DEFINED
#define SE306P1_UPSTAGE_QUATERNION_HPP_DEFINED

#include <cmath>

namespace ups
{
	class Quaternion
	{
	public:
		typedef float qUnit;
		
		Quaternion();
		Quaternion(qUnit w, qUnit x, qUnit y, qUnit z);
		
		void getMatrix(qUnit output[16]) const;
		
		static Quaternion fromEuler(qUnit yaw, qUnit pitch, qUnit roll);
		static Quaternion fromEulerDegrees(qUnit yaw, qUnit pitch, qUnit roll);
	private:
		qUnit _w, _x, _y, _z;
	};
	
	inline Quaternion::Quaternion():
		_w(static_cast<qUnit>(1.0)),
		_x(static_cast<qUnit>(0.0)),
		_y(static_cast<qUnit>(0.0)),
		_z(static_cast<qUnit>(0.0))
	{
	}
	
	inline Quaternion::Quaternion(qUnit w, qUnit x, qUnit y, qUnit z):
		_w(w),
		_x(x),
		_y(y),
		_z(z)
	{
	}
	
	inline void Quaternion::getMatrix(qUnit output[16]) const
	{
		// Based on information from http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
		qUnit ww = _w * _w; qUnit wx = _w * _x; qUnit wy = _w * _y; qUnit wz = _w * _z;
		qUnit xx = _x * _x; qUnit xy = _x * _y; qUnit xz = _x * _z;
		qUnit yy = _y * _y; qUnit yz = _y * _z;
		qUnit zz = _z * _z;
		
		// Column 0
		output[ 0] = 1.f - 2.f * (yy + zz);
		output[ 1] = 2.f * (xy + wz);
		output[ 2] = 2.f * (xz - wy);
		output[ 3] = 0.f;
		
		// Column 1
		output[ 4] = 2.f * (xy - wz);
		output[ 5] = 1.f - 2.f * (xx + zz);
		output[ 6] = 2.f * (wx + yz);
		output[ 7] = 0.f;
		
		// Column 2
		output[ 8] = 2.f * (wy + xz);
		output[ 9] = 2.f * (yz - wx);
		output[10] = 1.f - 2.f * (xx + yy);
		output[11] = 0.f;
		
		// Column 3
		output[12] = 0.f;
		output[13] = 0.f;
		output[14] = 0.f;
		output[15] = 1.f;
	}
	
	inline Quaternion Quaternion::fromEuler(qUnit yaw, qUnit pitch, qUnit roll)
	{
		// Half-angles
		qUnit y = yaw / 2;
		qUnit p = pitch / 2;
		qUnit r = roll / 2;
		
		// Cosines and sines for the half-angles
		qUnit cY = std::cos(y);	qUnit sY = std::sin(y);
		qUnit cP = std::cos(p);	qUnit sP = std::sin(p);
		qUnit cR = std::cos(r);	qUnit sR = std::sin(r);
		
		// Roll-pitch intermediate quaternion
		qUnit rpW = cP*cR;		qUnit rpX = cP*sR;
		qUnit rpY = sP*cR;		qUnit rpZ =-sP*sR;
		
		// Roll-pitch-yaw final quaternion
		return Quaternion(
		cY*rpW - sY*rpZ,
		cY*rpX - sY*rpY,
		cY*rpY + sY*rpX,
		cY*rpZ + sY*rpW);
	}
	
	inline Quaternion Quaternion::fromEulerDegrees(qUnit yaw, qUnit pitch, qUnit roll)
	{
		const qUnit multValue = 3.1415926535897932384626433832795 / 180;
		return fromEuler(yaw * multValue, pitch * multValue, roll * multValue);
	}
}

#endif // #ifndef SE306P1_UPSTAGE_QUATERNION_HPP_DEFINED