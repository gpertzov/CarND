// Vector3.cpp - implementation of the Vector3 class.
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
#include "stdafx.h"
#include <math.h>
#include "Vector3.h"


// default c-tor
Vector3::Vector3()
{
	set(0.0);
}


// c-tor with one value for all elements
Vector3::Vector3(double val)
{
	set(val);
}


// vector with value for each element
Vector3::Vector3(double _x, double _y, double _z)
{
	set(_x, _y, _z);
}


// set all elements to same value
void Vector3::set(double val)
{
	vx = val;
	vy = val;
	vz = val;
}


// set value of each element
void Vector3::set(double _x, double _y, double _z)
{
	vx = _x;
	vy = _y;
	vz = _z;
}



// return the 3 elements as reference
void Vector3::get(double& _x, double& _y, double& _z) const
{
	_x = vx;
	_y = vy;
	_z = vz;
}



void Vector3::normalize()
{
  double D =  sqrt(vx * vx + vy * vy + vz * vz);
  double invd = 0.0;

  if (D != 0)
  {
      invd = 1.0 / D;
      vx *= invd;
      vy *= invd;
      vz *= invd;
  }
}
	

// dot product with another vector
double Vector3::dot(const Vector3& other) const
{
   return (vx * other.vx + vy * other.vy + vz * other.vz);	
}


// cross-product with another vector
Vector3 Vector3::cross(const Vector3& other) const
{
  double  cx, cy, cz;
  
  cx = vy * other.vz - other.vy * vz;
  cy = vz * other.vx - other.vz * vx;
  cz = vx * other.vy - other.vx * vy;
  
  return(Vector3(cx, cy ,cz));
}


// test for equality between two vectors
bool Vector3::operator==(const Vector3& v)
{
	if (vx == v.vx && 
		vy == v.vy &&
		vz == v.vz   )
	  return (true);
    else
	  return (false);
}


// multiply by scalar
void Vector3::mul(double d)
{
	vx *= d;
	vy *= d;
	vz *= d;
}



// divide by scalar
void Vector3::div(double d)
{
    if (d != 0)
	   mul( (1.0/d) );
}


// add scalar
void Vector3::add(double d)
{
	vx += d;
	vy += d;
	vz += d;
}



// subtract scalar
void Vector3::sub(double d)
{
	add(-d);
}


// magnitude of vector
double Vector3::mag()
{
	return sqrt(vx * vx + vy * vy + vz * vz);
}


// add two vectors
Vector3 Vector3::operator+(const Vector3& other) const
{
	Vector3 product(vx + other.vx, vy + other.vy, vz + other.vz);
	return(product);
}


// subtract two vectors
Vector3 Vector3::operator-(const Vector3& other) const
{
	Vector3 product(vx - other.vx, vy - other.vy, vz - other.vz);
	return(product);
}



Vector3& Vector3::operator+=(const Vector3& other)
{
	vx += other.vx;
	vy += other.vy;
	vz += other.vz;

	return (*this);
}


Vector3& Vector3::operator-=(const Vector3& other)
{
	vx -= other.vx;
	vy -= other.vy;
	vz -= other.vz;

	return (*this);
}




Vector3 Vector3::operator+(double d) const
{
	Vector3 product(*this);
	product.add(d);
	return(product);
}



Vector3 Vector3::operator-(double d) const
{
	Vector3 product(*this);
	product.sub(d);
	return(product);
}


Vector3 Vector3::operator*(double d) const
{
	Vector3 product(*this);
	product.mul(d);
	return(product);

}



Vector3 Vector3::operator/(double d) const
{
	Vector3 product(*this);
	product.div(d);
	return(product);

}
