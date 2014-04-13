// Vector3.h - definition of the Vector3 class.
//
// Copyright (2005) Gideon Pertzov
//
// This source code is provided "AS IS" without express or implied warranties.
// You may use this source code in FREEWARE applications you distribute, 
// Provided that credit is given to the original author.
// You MAY NOT use the source code in ANY COMMERCIAL APPLICATION
// Without the written consent of the original author.
//
// http://www.gpdev.net
//
//////////////////////////////////////////////////////////////////////
#ifndef VECTOR3_H
#define VECTOR3_H

class Vector3
{
public:
	//~Vector3() // generated by compiler
	Vector3(); // default c-tor
	Vector3(double);
	Vector3(double, double, double);

	void set(double);
	void set(double, double, double);

	double x() const {return vx;}
	double y() const {return vy;}
	double z() const {return vz;}
	void   get(double&, double&, double&) const;

	void setX(double x) { vx = x; }
	void setY(double y) { vy = y; }
	void setZ(double z) { vz = z; }

	Vector3 operator+(const Vector3&) const;
	Vector3 operator-(const Vector3&) const;

	Vector3& operator+=(const Vector3&);
	Vector3& operator-=(const Vector3&);

	Vector3 operator+(double d) const;
	Vector3 operator-(double d) const;
	Vector3 operator*(double d) const;
	Vector3 operator/(double d) const;

	Vector3& operator+=(double d) {add(d); ; return *this;}
	Vector3& operator-=(double d) {sub(d); ; return *this;}
	Vector3& operator*=(double d) {mul(d); ; return *this;}
	Vector3& operator/=(double d) {div(d); ; return *this;}

	bool operator==(const Vector3&);
	
	void mul(double d);
	void div(double d);
	void add(double d);
	void sub(double d);
	
	double   dot  (const Vector3&) const;
	Vector3  cross(const Vector3&) const;
	double	 mag(); // magnitude
	void     normalize();
	

private:
	double vx, vy, vz;
};


#endif


