// smatrix.h: simple 4x4 matrix structure
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

#ifndef SMATRIX_H
#define SMATRIX_H

struct sMATRIX
{
	union
	{
		struct
		{
			float        _11, _12, _13, _14;
			float        _21, _22, _23, _24;
			float        _31, _32, _33, _34;
			float        _41, _42, _43, _44;

		};
		float m[4][4];
	};
};

#endif