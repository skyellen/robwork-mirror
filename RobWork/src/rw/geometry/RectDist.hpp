#pragma once

#include "rw/math.hpp"


using namespace rw::math;

typedef rw::math::Rotation3D<double>	Rotation;
typedef rw::math::Vector3D<double>		Vector;

double const EPS = 1e-7;

void closestPointsBetweenTwoEdges(		const Vector& endpoint1OfEdge1, 
										const Vector& endpoint2OfEdge1,
										const Vector& endpoint1OfEdge2,
										const Vector& endpoint2OfEdge2,
										Vector& closestPointOnEdge1,
										Vector& closestPointOnEdge2)
{
	Vector directionEdge1 = endpoint2OfEdge1 - endpoint1OfEdge1;
	Vector directionEdge2 = endpoint2OfEdge2 - endpoint1OfEdge2;

	Vector r = endpoint1OfEdge1 - endpoint1OfEdge2;

	double squaredLengthOfEdge1 = dot(directionEdge1,directionEdge1);
	double squaredLengthOfEdge2 = dot(directionEdge2,directionEdge2);

	double f = dot(directionEdge2,r);

	double s;
	double t;

	if(squaredLengthOfEdge1 <= EPS && squaredLengthOfEdge2 <= EPS)
	{
		closestPointOnEdge1 = endpoint1OfEdge1;
		closestPointOnEdge2 = endpoint1OfEdge2;
	}
	if(squaredLengthOfEdge1 <= EPS)
	{
		s = 0.0;
		t = f/squaredLengthOfEdge2;
		t = Math::clamp(t,0.0,1.0);
	}
	else
	{
		double c = dot(directionEdge1,r);
		if(squaredLengthOfEdge2 <= EPS)
		{
			t = 0.0;
			s = Math::clamp(-c/squaredLengthOfEdge1,0.0,1.0);
		}
		else
		{
			double b = dot(directionEdge1,directionEdge2);
			double denominator = squaredLengthOfEdge1*squaredLengthOfEdge2-b*b;

			if(denominator != 0.0) 
				s = Math::clamp((b*f-c*squaredLengthOfEdge2)/denominator,0,1);
			else
				s = 0.0;

			t = (b*s + f) / squaredLengthOfEdge2;

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(-c/squaredLengthOfEdge1,0.0,1.0);
			}
			else if(t > 1.0)
			{
				t = 1.0;
				s = Math::clamp((b-c)/squaredLengthOfEdge1,0.0,1.0);
			}
		}
	}

	closestPointOnEdge1 = endpoint1OfEdge1 + directionEdge1 * s;
	closestPointOnEdge2 = endpoint1OfEdge2 + directionEdge2 * t;
}

bool isInVoronoiRegion(		const Vector& endpointOfEdge, 
							const Vector& thirdPointOnRectangle,
							const Vector& pointToTest)
{
	Vector voronoiRegionPlaneNormal = endpointOfEdge - thirdPointOnRectangle;

	return dot(pointToTest - endpointOfEdge,voronoiRegionPlaneNormal) > 0.0;
}


double distanceBetweenRectangles2(const Rotation& rotation,const  Vector& position,double size[2][2])
{
	Vector rectangle1Corners[4];
	Vector rectangle2Corners[4];

	rectangle1Corners[0] = Vector(0.0,0.0,0.0);
	rectangle1Corners[1] = Vector(size[0][0],0.0,0.0);
	rectangle1Corners[2] = Vector(size[0][0],size[0][1],0.0);
	rectangle1Corners[3] = Vector(0.0,size[0][1],0.0);

	rectangle2Corners[0] = position;
	rectangle2Corners[1] = rotation * Vector(size[1][0],0.0,0.0);
	rectangle2Corners[3] = rotation * Vector(0.0,size[1][1],0.0);
	rectangle2Corners[2] = rectangle2Corners[1] + rectangle2Corners[3] + position;

	rectangle2Corners[1] += position;
	rectangle2Corners[3] += position;

	Vector closestPointOnEdge1;
	Vector closestPointOnEdge2;
	Vector edge1Edge2;

	for(int i = 0;i<4;i++)
	{
		for(int j = 0;j<4;j++)
		{
			if(		(isInVoronoiRegion(
						rectangle1Corners[(i+1)%4],
						rectangle1Corners[(i+2)%4],
						rectangle2Corners[j])
						||
					isInVoronoiRegion(
						rectangle1Corners[(i+1)%4],
						rectangle1Corners[(i+2)%4],
						rectangle2Corners[(j+1)%4]))
					&&
					(isInVoronoiRegion(
						rectangle2Corners[(j+1)%4],
						rectangle2Corners[(j+2)%4],
						rectangle1Corners[i])
						||
					isInVoronoiRegion(
						rectangle2Corners[(j+1)%4],
						rectangle2Corners[(j+2)%4],
						rectangle1Corners[(i+1)%4]))
					)
			{
				closestPointsBetweenTwoEdges(
					rectangle1Corners[i],
					rectangle1Corners[(i+1)%4],
					rectangle2Corners[j],
					rectangle2Corners[(j+1)%4],
					closestPointOnEdge1,
					closestPointOnEdge2);
				if(
					isInVoronoiRegion(
						rectangle1Corners[(i+1)%4],
						rectangle1Corners[(i+2)%4],
						closestPointOnEdge2) 
					&&
					isInVoronoiRegion(
						rectangle2Corners[(j+1)%4],
						rectangle2Corners[(j+2)%4],
						closestPointOnEdge1))
				{
					edge1Edge2 = closestPointOnEdge2-closestPointOnEdge1;

					return sqrt(dot(edge1Edge2,edge1Edge2));
				}
			}
		}
	}

	double z = rotation(0,2) * position(0) + rotation(1,2) * position(1) + rotation(2,2) * position(2);

	double separation1, separation2;
 
	if (position[2] > 0.0)
	{
		separation1 = position[2];
		if (rotation(2,0) < 0.0) separation1 += size[1][0]*rotation(2,0);
		if (rotation(2,1) < 0.0) separation1 += size[1][1]*rotation(2,1);
	}
	else
	{
		separation1 = -position[2];
		if (rotation(2,0) > 0.0) separation1 -= size[1][0]*rotation(2,0);
		if (rotation(2,1) > 0.0) separation1 -= size[1][1]*rotation(2,1);
	}
  
	if (z < 0)
	{
		separation2 = -z;
		if (rotation(0,2) < 0.0) separation2 += size[0][0]*rotation(0,2);
		if (rotation(1,2) < 0.0) separation2 += size[0][1]*rotation(1,2);
	}
	else
	{
		separation2 = z;
		if (rotation(0,2) > 0.0) separation2 -= size[0][0]*rotation(0,2);
		if (rotation(1,2) > 0.0) separation2 -= size[0][1]*rotation(1,2);
	}

	double separation = (separation1 > separation2? separation1 : separation2);
  
	return (separation > 0? separation : 0);
}

double distanceBetweenRectangles3(const Rotation& R,const  Vector& p,double r[2][2])
{
	//Edge 1 Rectagle A and Edge 1 Rectangle B
	//eA1, eB1

	double R2dotP = dot(R.getCol(1),p);
	double R1dotP = dot(R.getCol(0),p);

	double s,t,den;
	Vector v;

	//A1 in Voronoi of eB1 and A2 in Voronoi of eB1
	if(R2dotP > 0 && R2dotP > r[0][0]*R(0,1))
	{
		//B1 in Voronoi of eA1 and B2 in Voronoi of eA1
		if(p(1) < 0 && p(1) + r[1][0]*R(1,0) < 0)
		{

			//Return closest point btw edges
			den = 1 - R(0,0)*R(0,0);
			s =  (p(0) - R(0,0)*R1dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R1dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,0) * s - R1dotP;
			}
			else
			{
				t = R(0,0) * s - R1dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0),0.0,r[0][0]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(0,0)*t+p(0),0.0,r[0][0]);
			}
			
			v = Vector(p(0)+R(0,0)*t-s,p(1)+R(1,0)*t,p(2)+R(2,0)*t);
			
			return sqrt(dot(v,v));
		}
		else if(p(1) < 0 || p(1) + r[1][0]*R(1,0) < 0)
		{
			//calc closest point
			//check: is closest point in voronoi
			den = 1 - R(0,0)*R(0,0);

			s =  (p(0) - R(0,0)*R1dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R1dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,0) * s - R1dotP;
			}
			else
			{
				t = R(0,0) * s - R1dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0),0.0,r[0][0]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(0,0)*t+p(0),0.0,r[0][0]);
			}

			if(R(1,0) * t < - p(1))
			{
				v = Vector(p(0)+R(0,0)*t-s,p(1)+R(1,0)*t,p(2)+R(2,0)*t);
				return sqrt(dot(v,v));
			}
		}
	}
	//Either A1 or A2 is in Voronoi of eB1 AND either B1 or B2 is in Voronoi of eA1
	else if( (R2dotP > 0 || R2dotP > r[0][0]*R(0,1)) && (p(1) < 0 || r[1][0]*R(1,0) + p(1) < 0) )
	{
			//calc closest points
			//check: is closest point in voronoi
			den = 1 - R(0,0)*R(0,0);

			s =  (p(0) - R(0,0)*R1dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R1dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,0) * s - R1dotP;
			}
			else
			{
				t = R(0,0) * s - R1dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0),0.0,r[0][0]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(0,0)*t+p(0),0.0,r[0][0]);
			}

			//If closest points is in Voronoi
			if(R(1,0) * t < - p(1) && R2dotP > R(0,1) * s)
			{
				v = Vector(p(0)+R(0,0)*t-s,p(1)+R(1,0)*t,p(2)+R(2,0)*t);
				return sqrt(dot(v,v));
			}
	}



	//A1 and A2 in Voronoi of eB3
	if(R2dotP + r[1][1] < 0 && R2dotP + r[1][1] < R(0,1)*r[0][0])
	{
		//B3 and B4 in Voronoi of eA1
		if(p(1) + r[1][0]*R(1,0) + r[1][1]*R(1,1) < 0 && p(1) + r[1][1]*R(1,1) < 0)
		{
			den = 1 - R(0,0)*R(0,0);

			s =  (-R(0,0)*R1dotP + p(0) + r[1][1]*R(0,1))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = -R1dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,0) * s - R1dotP;
			}
			else
			{
				t = R(0,0) * s - R1dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0) + r[1][1]*R(0,1),0.0,r[0][0]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(0,0)*t + p(0) + r[1][1]*R(0,1),0.0,r[0][0]);
			}

			v = Vector(
				p(0)+R(0,0)*t + R(0,1)*r[1][1] - s,
				p(1)+R(1,0)*t + R(1,1)*r[1][1],
				p(2)+R(2,0)*t + R(2,1)*r[1][1]);
			
			return sqrt(dot(v,v));
		}
		//If either B3 or B4 in Voronoi of eA1
		else if(p(1) + r[1][0]*R(1,0) + r[1][1]*R(1,1) < 0 || p(1) + r[1][1]*R(1,1) < 0)
		{
			den = 1 - R(0,0)*R(0,0);

			s =  (-R(0,0)*R1dotP + p(0) + r[1][1]*R(0,1))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = -R1dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,0) * s - R1dotP;
			}
			else
			{
				t = R(0,0) * s - R1dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0) + r[1][1]*R(0,1),0.0,r[0][0]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(0,0)*t + p(0) + r[1][1]*R(0,1),0.0,r[0][0]);
			}

			if(p(1) + r[1][1]*R(1,1) + R(1,0)*t < 0)
			{
				v = Vector(
					p(0)+R(0,0)*t + R(0,1)*r[1][1] - s,
					p(1)+R(1,0)*t + R(1,1)*r[1][1],
					p(2)+R(2,0)*t + R(2,1)*r[1][1]);
			
				return sqrt(dot(v,v));
			}
		}
	}
	//Either A1 or A2 is in Voronoi of eB3 AND either B3 or B4 is in Voronoi of eA1
	else if( 
		(R2dotP + r[1][1] < 0 || R2dotP + r[1][1] < R(0,1)*r[0][0]) 
		&&
		(p(1) + r[1][0]*R(1,0) + r[1][1]*R(1,1) < 0 || p(1) + r[1][1]*R(1,1) < 0))
	{
			den = 1 - R(0,0)*R(0,0);

			s =  (-R(0,0)*R1dotP + p(0) + r[1][1]*R(0,1))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = -R1dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,0) * s - R1dotP;
			}
			else
			{
				t = R(0,0) * s - R1dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0) + r[1][1]*R(0,1),0.0,r[0][0]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(0,0)*t + p(0) + r[1][1]*R(0,1),0.0,r[0][0]);
			}

			//Closest point in Voronoi?
			if(p(1) + r[1][1]*R(1,1) + R(1,0)*t < 0 && R(0,1) * s > R2dotP + r[1][1] )
			{
				v = Vector(
					p(0)+R(0,0)*t + R(0,1)*r[1][1] - s,
					p(1)+R(1,0)*t + R(1,1)*r[1][1],
					p(2)+R(2,0)*t + R(2,1)*r[1][1]);
			
				return sqrt(dot(v,v));
			}
	}

	//A3 and A4 in Voronoi of eB1
	if(R2dotP > r[0][0]*R(0,1) + r[0][1]*R(1,1) && R2dotP > r[0][1]*R(1,1) )
	{
		//B1 and B2 in Voronoi of eA3
		if(p(1) > r[0][1] && p(1) + R(1,0)*r[1][0] > r[0][1])
		{
			den = 1 - R(0,0)*R(0,0);

			s = (R(0,0) * (r[0][1]*R(1,0) - R1dotP) + p(0))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][1]*R(1,0) - R1dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,0) * s + r[0][1]*R(1,0) - R1dotP;
			}
			else
			{
				t = R(0,0) * s + r[0][1]*R(1,0) - R1dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0),0.0,r[0][0]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(0,0)*t + p(0),0.0,r[0][0]);
			}

			v = Vector(
				p(0)+R(0,0)*t - s,
				p(1)+R(1,0)*t - r[0][1],
				p(2)+R(2,0)*t );
			
			return sqrt(dot(v,v));
		}
		//Either B1 or B2 in Voronoi of eA3
		else if(p(1) > r[0][1] || p(1) + R(1,0)*r[1][0] > r[0][1])
		{
			den = 1 - R(0,0)*R(0,0);

			s = (R(0,0) * (r[0][1]*R(1,0) - R1dotP) + p(0))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][1]*R(1,0) - R1dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,0) * s + r[0][1]*R(1,0) - R1dotP;
			}
			else
			{
				t = R(0,0) * s + r[0][1]*R(1,0) - R1dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0),0.0,r[0][0]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(0,0)*t + p(0),0.0,r[0][0]);
			}

			if(p(1) + R(1,0)*t > r[0][1])
			{
				v = Vector(
					p(0)+R(0,0)*t - s,
					p(1)+R(1,0)*t - r[0][1],
					p(2)+R(2,0)*t );
			
				return sqrt(dot(v,v));
			}
		}
	}
	//Either A3 or A4 is in Voronoi of eB1 AND either B1 or B2 is in Voronoi of eA3
	else if(
			(R2dotP > r[0][0]*R(0,1) + r[0][1]*R(1,1) || R2dotP > r[0][1]*R(1,1) )
			&&
			(p(1) > r[0][1] || p(1) + R(1,0)*r[1][0] > r[0][1])
			)
	{
		den = 1 - R(0,0)*R(0,0);

		s = (R(0,0) * (r[0][1]*R(1,0) - R1dotP) + p(0))/den;

		if(s < 0.0)
		{
			s = 0.0;
			t = r[0][1]*R(1,0) - R1dotP;
		}
		else if(s > r[0][0])
		{
			s = r[0][0];
			t = R(0,0) * s + r[0][1]*R(1,0) - R1dotP;
		}
		else
		{
			t = R(0,0) * s + r[0][1]*R(1,0) - R1dotP;
		}


		if(t < 0.0)
		{
			t = 0.0;
			s = Math::clamp(p(0),0.0,r[0][0]);
		}
		else if(t > r[1][0])
		{
			t = r[1][0];
			s = Math::clamp(R(0,0)*t + p(0),0.0,r[0][0]);
		}

		if(p(1) + R(1,0)*t > r[0][1] && R2dotP > R(0,1)*s + R(1,1)*r[0][1])
		{
			v = Vector(
				p(0)+R(0,0)*t - s,
				p(1)+R(1,0)*t - r[0][1],
				p(2)+R(2,0)*t );
			
			return sqrt(dot(v,v));
		}
	}


	//A3 and A4 in Voronoi of eB3
	if(r[0][0] * R(0,1) + r[0][1]*R(1,1) - r[1][1] > R2dotP && r[0][1]*R(1,1) - r[1][1] > R2dotP)
	{
		//B3 and B4 in Voronoi of eA3
		if(p(1) + r[1][1]*R(1,1) + r[1][0]*R(1,0) > r[0][1] && p(1) + r[1][1]*R(1,1) > r[0][1])
		{
			den = 1 - R(0,0)*R(0,0);

			s = (R(0,0) * (r[0][1]*R(1,0) - R1dotP) + p(0) + r[1][1]*R(0,1))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][1]*R(1,0) - R1dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,0) * s + r[0][1]*R(1,0) - R1dotP;
			}
			else
			{
				t = R(0,0) * s + r[0][1]*R(1,0) - R1dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0) + r[1][1]*R(0,1),0.0,r[0][0]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(0,0)*t + p(0) + r[1][1]*R(0,1),0.0,r[0][0]);
			}

			v = Vector(
				p(0)+R(0,1)*r[1][1]+R(0,0)*t - s,
				p(1)+R(1,1)*r[1][1]+R(1,0)*t - r[0][1],
				p(2)+R(2,1)*r[1][1]+R(2,0)*t );
			
			return sqrt(dot(v,v));
		}
		//Either B3 or B4 in Voronoi of eA3
		else if(p(1) + r[1][1]*R(1,1) + r[1][0]*R(1,0) > r[0][1] || p(1) + r[1][1]*R(1,1) > r[0][1])
		{
			den = 1 - R(0,0)*R(0,0);

			s = (R(0,0) * (r[0][1]*R(1,0) - R1dotP) + p(0) + r[1][1]*R(0,1))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][1]*R(1,0) - R1dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,0) * s + r[0][1]*R(1,0) - R1dotP;
			}
			else
			{
				t = R(0,0) * s + r[0][1]*R(1,0) - R1dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0) + r[1][1]*R(0,1),0.0,r[0][0]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(0,0)*t + p(0) + r[1][1]*R(0,1),0.0,r[0][0]);
			}

			if(p(1) + R(1,1)*r[1][1] + R(1,0)*t > r[0][1])
			{
				v = Vector(
					p(0)+R(0,1)*r[1][1]+R(0,0)*t - s,
					p(1)+R(1,1)*r[1][1]+R(1,0)*t - r[0][1],
					p(2)+R(2,1)*r[1][1]+R(2,0)*t );
			
				return sqrt(dot(v,v));
			}
		}
	}
	//Either A3 or A4 is in Voronoi of eB1 AND either B1 or B2 is in Voronoi of eA3
	else if(
			(r[0][0] * R(0,1) + r[0][1]*R(1,1) - r[1][1] > R2dotP || r[0][1]*R(1,1) - r[1][1] > R2dotP)
			&&
			(p(1) + r[1][1]*R(1,1) + r[1][0]*R(1,0) > r[0][1] || p(1) + r[1][1]*R(1,1) > r[0][1])
		)
	{
		den = 1 - R(0,0)*R(0,0);

		s = (R(0,0) * (r[0][1]*R(1,0) - R1dotP) + p(0) + r[1][1]*R(0,1))/den;

		if(s < 0.0)
		{
			s = 0.0;
			t = r[0][1]*R(1,0) - R1dotP;
		}
		else if(s > r[0][0])
		{
			s = r[0][0];
			t = R(0,0) * s + r[0][1]*R(1,0) - R1dotP;
		}
		else
		{
			t = R(0,0) * s + r[0][1]*R(1,0) - R1dotP;
		}


		if(t < 0.0)
		{
			t = 0.0;
			s = Math::clamp(p(0) + r[1][1]*R(0,1),0.0,r[0][0]);
		}
		else if(t > r[1][0])
		{
			t = r[1][0];
			s = Math::clamp(R(0,0)*t + p(0) + r[1][1]*R(0,1),0.0,r[0][0]);
		}

		if(p(1) + R(1,1)*r[1][1] + R(1,0)*t > r[0][1] && s*R(0,1) + r[0][1]*R(1,1) - r[1][1] > R2dotP)
		{
			v = Vector(
				p(0)+R(0,1)*r[1][1]+R(0,0)*t - s,
				p(1)+R(1,1)*r[1][1]+R(1,0)*t - r[0][1],
				p(2)+R(2,1)*r[1][1]+R(2,0)*t );
			
			return sqrt(dot(v,v));
		}
	}



	//A1 and A4 in Voronoi of eB4
	if(R1dotP > 0 && R1dotP > r[0][1]*R(1,0))
	{
		//B1 and B4 in Voronoi of eA4
		if(p(0) < 0 && p(0) + R(0,1)*r[1][1] < 0)
		{
			den = 1 - R(1,1)*R(1,1);

			s = (p(1) - R(1,1)*R2dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = -R2dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,1)*s - R2dotP;
			}
			else
			{
				t = R(1,1)*s - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1),0.0,r[0][1]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(1,1)*t + p(1),0.0,r[0][1]);
			}

			v = Vector(
				p(0)+R(0,1)*t,
				p(1)+R(1,1)*t - s,
				p(2)+R(2,1)*t );
			
			return sqrt(dot(v,v));
		}
		//B1 and B4 in Voronoi of eA4
		else if(p(0) < 0 || p(0) + R(0,1)*r[1][1] < 0)
		{
			den = 1 - R(1,1)*R(1,1);

			s = (p(1) - R(1,1)*R2dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = -R2dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,1)*s - R2dotP;
			}
			else
			{
				t = R(1,1)*s - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1),0.0,r[0][1]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(1,1)*t + p(1),0.0,r[0][1]);
			}

			if(p(0) + R(0,1)*t < 0)
			{
				v = Vector(
					p(0)+R(0,1)*t,
					p(1)+R(1,1)*t - s,
					p(2)+R(2,1)*t );
			
				return sqrt(dot(v,v));
			}
		}
	}
	//Either A1 or A4 is in Voronoi of eB4 AND either B1 or B4 is in Voronoi of eA4
	else if( ( R1dotP > 0 || R1dotP > r[0][1]*R(1,0) )  
		&&
			(p(0) < 0 || p(0) + R(0,1)*r[1][1] < 0)
		)
	{
			den = 1 - R(1,1)*R(1,1);

			s = (p(1) - R(1,1)*R2dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = -R2dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,1)*s - R2dotP;
			}
			else
			{
				t = R(1,1)*s - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1),0.0,r[0][1]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(1,1)*t + p(1),0.0,r[0][1]);
			}

			if(p(0) + R(0,1)*t < 0 && R1dotP > s*R(1,0))
			{
				v = Vector(
					p(0)+R(0,1)*t,
					p(1)+R(1,1)*t - s,
					p(2)+R(2,1)*t );
			
				return sqrt(dot(v,v));
			}
	}



	//B2 and B3 in Voronoi of eA4
	if(p(0) + R(0,0)*r[1][0] < 0 && p(0) + R(0,1)*r[1][1] + R(0,0)*r[1][0] < 0)
	{
		//A1 and A4 in Voronoi of eB2
		if(R1dotP + r[1][0] < 0 && R1dotP + r[1][0] < r[0][1]*R(1,0))
		{
			den = 1 - R(1,1)*R(1,1);

			s = (p(1) + R(1,0)*r[1][0] - R(1,1)*R2dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = -R2dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,1)*s - R2dotP;
			}
			else
			{
				t = R(1,1)*s - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1) + R(1,0)*r[1][0],0.0,r[0][1]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(1,1)*t + p(1) + R(1,0)*r[1][0],0.0,r[0][1]);
			}

			v = Vector(
				p(0) + R(0,0)*r[1][0] + R(0,1)*t,
				p(1) + R(1,0)*r[1][0] + R(1,1)*t - s,
				p(2) + R(2,0)*r[1][0] + R(2,1)*t );
			
			return sqrt(dot(v,v));
		}
		else if(R1dotP + r[1][0] < 0 || R1dotP + r[1][0] < r[0][1]*R(1,0))
		{
			den = 1 - R(1,1)*R(1,1);

			s = (p(1) + R(1,0)*r[1][0] - R(1,1)*R2dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = -R2dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,1)*s - R2dotP;
			}
			else
			{
				t = R(1,1)*s - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1) + R(1,0)*r[1][0],0.0,r[0][1]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(1,1)*t + p(1) + R(1,0)*r[1][0],0.0,r[0][1]);
			}

			if(s*R(1,0) > R1dotP + r[1][0])
			{
				v = Vector(
					p(0) + R(0,0)*r[1][0] + R(0,1)*t,
					p(1) + R(1,0)*r[1][0] + R(1,1)*t - s,
					p(2) + R(2,0)*r[1][0] + R(2,1)*t );
			
				return sqrt(dot(v,v));
			}
		}
	}
	else if( 
		(p(0) + R(0,0)*r[1][0] < 0 || p(0) + R(0,1)*r[1][1] + R(0,0)*r[1][0] < 0)
		&&
		(R1dotP + r[1][0] < 0 || R1dotP + r[1][0] < r[0][1]*R(1,0))
		)
	{
			den = 1 - R(1,1)*R(1,1);

			s = (p(1) + R(1,0)*r[1][0] - R(1,1)*R2dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = -R2dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,1)*s - R2dotP;
			}
			else
			{
				t = R(1,1)*s - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1) + R(1,0)*r[1][0],0.0,r[0][1]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(1,1)*t + p(1) + R(1,0)*r[1][0],0.0,r[0][1]);
			}

			if(s*R(1,0) > R1dotP + r[1][0] && p(0) + R(0,0)*r[1][0] + R(0,1)*t < 0)
			{
				v = Vector(
					p(0) + R(0,0)*r[1][0] + R(0,1)*t,
					p(1) + R(1,0)*r[1][0] + R(1,1)*t - s,
					p(2) + R(2,0)*r[1][0] + R(2,1)*t );
			
				return sqrt(dot(v,v));
			}
	}




	//A2 and A3 in Voronoi of eB4
	if(R1dotP > R(0,0)*r[0][0] && r[0][0]*R(0,0) + r[0][1]*R(1,0) < R1dotP)
	{
		//B1 and B4 in Voronoi of eA2
		if(p(0) > r[0][0] && p(0) + R(0,1)*r[1][1] > r[0][0])
		{
			den = 1 - R(1,1)*R(1,1);

			s = ((R(1,1)*(R(0,1)*r[0][0] - R2dotP)) + p(1))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = R(0,1)*r[0][0] - R2dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,1)*s + R(0,1)*r[0][0]  - R2dotP;
			}
			else
			{
				t = R(1,1)*s + R(0,1)*r[0][0]  - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1),0.0,r[0][1]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(1,1)*t + p(1),0.0,r[0][1]);
			}

			v = Vector(
				p(0) + R(0,1)*t - r[0][0],
				p(1) + R(1,1)*t - s,
				p(2) + R(2,1)*t );
			
			return sqrt(dot(v,v));		
		}
		else if(p(0) > r[0][0] || p(0) + R(0,1)*r[1][1] > r[0][0])
		{
			den = 1 - R(1,1)*R(1,1);

			s = ((R(1,1)*(R(0,1)*r[0][0] - R2dotP)) + p(1))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = R(0,1)*r[0][0] - R2dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,1)*s + R(0,1)*r[0][0]  - R2dotP;
			}
			else
			{
				t = R(1,1)*s + R(0,1)*r[0][0]  - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1),0.0,r[0][1]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(1,1)*t + p(1),0.0,r[0][1]);
			}

			if(p(0) + t*R(0,1) > r[0][0])
			{
				v = Vector(
					p(0) + R(0,1)*t - r[0][0],
					p(1) + R(1,1)*t - s,
					p(2) + R(2,1)*t );
			
				return sqrt(dot(v,v));	
			}
		}
	}
	else if(
		(R1dotP > R(0,0)*r[0][0] || r[0][0]*R(0,0) + r[0][1]*R(1,0) < R1dotP)
		&&
		(p(0) > r[0][0] || p(0) + R(0,1)*r[1][1] > r[0][0])
		)
	{
			den = 1 - R(1,1)*R(1,1);

			s = ((R(1,1)*(R(0,1)*r[0][0] - R2dotP)) + p(1))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = R(0,1)*r[0][0] - R2dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,1)*s + R(0,1)*r[0][0]  - R2dotP;
			}
			else
			{
				t = R(1,1)*s + R(0,1)*r[0][0]  - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1),0.0,r[0][1]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(1,1)*t + p(1),0.0,r[0][1]);
			}

			if(p(0) + t*R(0,1) > r[0][0] && R1dotP > R(0,0)*r[0][0] + R(1,0)*s)
			{
				v = Vector(
					p(0) + R(0,1)*t - r[0][0],
					p(1) + R(1,1)*t - s,
					p(2) + R(2,1)*t );
			
				return sqrt(dot(v,v));	
			}
	}



	//A2 and A3 in Voronoi of eB2
	if(r[0][0]*R(0,0) > R1dotP + r[1][0] && r[0][0]*R(0,0) + r[0][1]*R(1,0) > R1dotP + r[1][0])
	{
		//B2 and B3 in Voronoi of eA2
		if(p(0) + r[1][0]*R(0,0) > r[0][0] && p(0) + r[1][0]*R(0,0) + r[1][1]*R(0,1) > r[0][0] )
		{
			den = 1 - R(1,1)*R(1,1);

			s = (R(1,1)*(R(0,1)*r[0][0] - R2dotP) + p(1) + R(1,0)*r[1][0])/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = R(0,1)*r[0][0] - R2dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,1)*s + R(0,1)*r[0][0]  - R2dotP;
			}
			else
			{
				t = R(1,1)*s + R(0,1)*r[0][0]  - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1) + R(1,0)*r[1][0],0.0,r[0][1]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(1,1)*t + p(1) + R(1,0)*r[1][0],0.0,r[0][1]);
			}

			v = Vector(
				p(0) + R(0,0)*r[1][0] + R(0,1)*t - r[0][0],
				p(1) + R(1,0)*r[1][0] + R(1,1)*t - s,
				p(2) + R(2,0)*r[1][0] + R(2,1)*t );
			
			return sqrt(dot(v,v));	
		}
		else if(p(0) + r[1][0]*R(0,0) > r[0][0] || p(0) + r[1][0]*R(0,0) + r[1][1]*R(0,1) > r[0][0] )
		{
			den = 1 - R(1,1)*R(1,1);

			s = (R(1,1)*(R(0,1)*r[0][0] - R2dotP) + p(1) + R(1,0)*r[1][0])/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = R(0,1)*r[0][0] - R2dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,1)*s + R(0,1)*r[0][0]  - R2dotP;
			}
			else
			{
				t = R(1,1)*s + R(0,1)*r[0][0]  - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1) + R(1,0)*r[1][0],0.0,r[0][1]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(1,1)*t + p(1) + R(1,0)*r[1][0],0.0,r[0][1]);
			}

			if(p(0) + R(0,0)*r[1][0] + R(0,1)*t > r[0][0])
			{
				v = Vector(
					p(0) + R(0,0)*r[1][0] + R(0,1)*t - r[0][0],
					p(1) + R(1,0)*r[1][0] + R(1,1)*t - s,
					p(2) + R(2,0)*r[1][0] + R(2,1)*t );
			
				return sqrt(dot(v,v));	
			}
		}
	}
	else if((r[0][0]*R(0,0) > R1dotP + r[1][0] || r[0][0]*R(0,0) + r[0][1]*R(1,0) > R1dotP + r[1][0])
			&&
			(p(0) + r[1][0]*R(0,0) > r[0][0] || p(0) + r[1][0]*R(0,0) + r[1][1]*R(0,1) > r[0][0] )
			)
	{
			den = 1 - R(1,1)*R(1,1);

			s = (R(1,1)*(R(0,1)*r[0][0] - R2dotP) + p(1) + R(1,0)*r[1][0])/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = R(0,1)*r[0][0] - R2dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,1)*s + R(0,1)*r[0][0]  - R2dotP;
			}
			else
			{
				t = R(1,1)*s + R(0,1)*r[0][0]  - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1) + R(1,0)*r[1][0],0.0,r[0][1]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(1,1)*t + p(1) + R(1,0)*r[1][0],0.0,r[0][1]);
			}

			if(p(0) + R(0,0)*r[1][0] + R(0,1)*t > r[0][0] && r[0][0]*R(0,0) + s*R(1,0) > R1dotP + r[1][0])
			{
				v = Vector(
					p(0) + R(0,0)*r[1][0] + R(0,1)*t - r[0][0],
					p(1) + R(1,0)*r[1][0] + R(1,1)*t - s,
					p(2) + R(2,0)*r[1][0] + R(2,1)*t );
			
				return sqrt(dot(v,v));	
			}
	}




	//A1 and A2 in Voronoi of eB4
	if(R1dotP > 0 && R1dotP > r[0][0]*R(0,0))
	{
		//B1 and B4 in Voronoi of eA1
		if(p(1) < 0 && p(1) + R(1,1)*r[1][1] < 0)
		{
			den = 1 - R(0,1)*R(0,1);

			s = (p(0) - R(0,1)*R2dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R2dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,1)*s - R2dotP;
			}
			else
			{
				t = R(0,1)*s - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0),0.0,r[0][0]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(0,1)*t + p(0),0.0,r[0][0]);
			}

			v = Vector(
				p(0) + R(0,1)*t - s,
				p(1) + R(1,1)*t,
				p(2) + R(2,1)*t );
			
			return sqrt(dot(v,v));
		}
		if(p(1) < 0 || p(1) + R(1,1)*r[1][1] < 0)
		{
			den = 1 - R(0,1)*R(0,1);

			s = (p(0) - R(0,1)*R2dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R2dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,1)*s - R2dotP;
			}
			else
			{
				t = R(0,1)*s - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0),0.0,r[0][0]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(0,1)*t + p(0),0.0,r[0][0]);
			}

			if(p(1) + R(1,1)*t < 0)
			{
				v = Vector(
					p(0) + R(0,1)*t - s,
					p(1) + R(1,1)*t,
					p(2) + R(2,1)*t );
			
				return sqrt(dot(v,v));
			}
		}
	}
	if((R1dotP > 0 || R1dotP > r[0][0]*R(0,0))
		&&
		(p(1) < 0 || p(1) + R(1,1)*r[1][1] < 0)
		)
	{
			den = 1 - R(0,1)*R(0,1);

			s = (p(0) - R(0,1)*R2dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R2dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,1)*s - R2dotP;
			}
			else
			{
				t = R(0,1)*s - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0),0.0,r[0][0]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(0,1)*t + p(0),0.0,r[0][0]);
			}

			if(p(1) + R(1,1)*t < 0 && R1dotP > s*R(0,0))
			{
				v = Vector(
					p(0) + R(0,1)*t - s,
					p(1) + R(1,1)*t,
					p(2) + R(2,1)*t );
			
				return sqrt(dot(v,v));
			}
	}



	//A1 and A2 in V(eB2)
	if(R1dotP + r[1][0] < 0 && R1dotP + r[1][0] < r[0][0]*R(0,0))
	{
		if(p(1) + r[1][0]*R(1,0) < 0 && p(1) + r[1][0]*R(1,0)+r[1][1]*R(1,1) < 0)
		{
			den = 1 - R(0,1)*R(0,1);

			s = (p(0) + R(0,0)*r[1][0] - R(0,1)*R2dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R2dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,1)*s - R2dotP;
			}
			else
			{
				t = R(0,1)*s - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0) + R(0,0)*r[1][0],0.0,r[0][0]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(0,1)*t + p(0) + R(0,0)*r[1][0],0.0,r[0][0]);
			}

			v = Vector(
				p(0) + R(0,0)*r[1][0] + R(0,1)*t - s,
				p(1) + R(1,0)*r[1][0] + R(1,1)*t,
				p(2) + R(2,0)*r[1][0] + R(2,1)*t );
			
			return sqrt(dot(v,v));
		}
		else if(p(1) + r[1][0]*R(1,0) < 0 || p(1) + r[1][0]*R(1,0)+r[1][1]*R(1,1) < 0)
		{
			den = 1 - R(0,1)*R(0,1);

			s = (p(0) + R(0,0)*r[1][0] - R(0,1)*R2dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R2dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,1)*s - R2dotP;
			}
			else
			{
				t = R(0,1)*s - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0) + R(0,0)*r[1][0],0.0,r[0][0]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(0,1)*t + p(0) + R(0,0)*r[1][0],0.0,r[0][0]);
			}

			if(p(1) + r[1][0]*R(1,0) + t*R(1,1) < 0)
			{
				v = Vector(
					p(0) + R(0,0)*r[1][0] + R(0,1)*t - s,
					p(1) + R(1,0)*r[1][0] + R(1,1)*t,
					p(2) + R(2,0)*r[1][0] + R(2,1)*t );
			
				return sqrt(dot(v,v));
			}
		}
	}
	else if(
		(R1dotP + r[1][0] < 0 || R1dotP + r[1][0] < r[0][0]*R(0,0))
		&&
		(p(1) + r[1][0]*R(1,0) < 0 || p(1) + r[1][0]*R(1,0)+r[1][1]*R(1,1) < 0)
		)
	{
			den = 1 - R(0,1)*R(0,1);

			s = (p(0) + R(0,0)*r[1][0] - R(0,1)*R2dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R2dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,1)*s - R2dotP;
			}
			else
			{
				t = R(0,1)*s - R2dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0) + R(0,0)*r[1][0],0.0,r[0][0]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(0,1)*t + p(0) + R(0,0)*r[1][0],0.0,r[0][0]);
			}

			if(p(1) + r[1][0]*R(1,0) + t*R(1,1) < 0 && R1dotP + r[1][0] < s*R(0,0))
			{
				v = Vector(
					p(0) + R(0,0)*r[1][0] + R(0,1)*t - s,
					p(1) + R(1,0)*r[1][0] + R(1,1)*t,
					p(2) + R(2,0)*r[1][0] + R(2,1)*t );
			
				return sqrt(dot(v,v));
			}
	}


	//A1 and A4 in V(eB1)
	if(R2dotP > 0 && R2dotP > r[0][1]*R(1,1))
	{
		//B1 and B2 in V(eB4)
		if(p(0) < 0 && p(0) + R(0,0)*r[1][0] < 0)
		{
			den = 1 - R(1,0)*R(1,0);

			s = (p(1) - R(1,0)*R1dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R1dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,0)*s - R1dotP;
			}
			else
			{
				t = R(1,0)*s - R1dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1),0.0,r[0][1]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(1,0)*t + p(1),0.0,r[0][1]);
			}

			v = Vector(
				p(0) + R(0,0)*t,
				p(1) + R(1,0)*t - s,
				p(2) + R(2,0)*t );

			return sqrt(dot(v,v));
		}
		else if(p(0) < 0 || p(0) + R(0,0)*r[1][0] < 0)
		{
			den = 1 - R(1,0)*R(1,0);

			s = (p(1) - R(1,0)*R1dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R1dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,0)*s - R1dotP;
			}
			else
			{
				t = R(1,0)*s - R1dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1),0.0,r[0][1]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(1,0)*t + p(1),0.0,r[0][1]);
			}


			if(p(0) + R(0,0)*t < 0)
			{
				v = Vector(
					p(0) + R(0,0)*t,
					p(1) + R(1,0)*t - s,
					p(2) + R(2,0)*t );

				return sqrt(dot(v,v));
			}
		}
	}
	else if((R2dotP > 0 || R2dotP > r[0][1]*R(1,1))
		&&
		(p(0) < 0 || p(0) + R(0,0)*r[1][0] < 0)
		)
	{
			den = 1 - R(1,0)*R(1,0);

			s = (p(1) - R(1,0)*R1dotP)/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R1dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,0)*s - R1dotP;
			}
			else
			{
				t = R(1,0)*s - R1dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1),0.0,r[0][1]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(1,0)*t + p(1),0.0,r[0][1]);
			}

			if(p(0) + R(0,0)*t < 0 && R2dotP > s*R(1,1))
			{
				v = Vector(
					p(0) + R(0,0)*t,
					p(1) + R(1,0)*t - s,
					p(2) + R(2,0)*t );

				return sqrt(dot(v,v));
			}
	}

	//A1 and A4 in V(eB3)
	if(R2dotP + r[1][1] < 0 && r[0][1]*R(1,1) > R2dotP + r[1][1])
	{
		//B3 and B4 in V(eA4)
		if(p(0) + R(0,1)*r[1][1] < 0 && p(0) + R(0,1)*r[1][1] + R(0,0)*r[1][0] < 0)
		{
			den = 1 - R(1,0)*R(1,0);

			s = (-R(1,0)*R1dotP + p(1) + R(1,1)*r[1][1])/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R1dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,0)*s - R1dotP;
			}
			else
			{
				t = R(1,0)*s - R1dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1) + r[1][1]*R(1,1),0.0,r[0][1]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(1,0)*t + p(1) + r[1][1]*R(1,1),0.0,r[0][1]);
			}

			v = Vector(
				p(0) + R(0,1)*r[1][1] + R(0,0)*t,
				p(1) + R(1,1)*r[1][1] + R(1,0)*t - s,
				p(2) + R(2,1)*r[1][1] + R(2,0)*t );

			return sqrt(dot(v,v));
		}
		else if(p(0) + R(0,1)*r[1][1] < 0 || p(0) + R(0,1)*r[1][1] + R(0,0)*r[1][0] < 0)
		{
			den = 1 - R(1,0)*R(1,0);

			s = (-R(1,0)*R1dotP + p(1) + R(1,1)*r[1][1])/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R1dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,0)*s - R1dotP;
			}
			else
			{
				t = R(1,0)*s - R1dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1) + r[1][1]*R(1,1),0.0,r[0][1]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(1,0)*t + p(1) + r[1][1]*R(1,1),0.0,r[0][1]);
			}


			if(p(0) + R(0,1)*r[1][1] + R(0,0)*t < 0)
			{
				v = Vector(
					p(0) + R(0,1)*r[1][1] + R(0,0)*t,
					p(1) + R(1,1)*r[1][1] + R(1,0)*t - s,
					p(2) + R(2,1)*r[1][1] + R(2,0)*t );

				return sqrt(dot(v,v));
			}
		}
	}
	else if(
		(R2dotP + r[1][1] < 0 || r[0][1]*R(1,1) > R2dotP + r[1][1])
		&&
		(p(0) + R(0,1)*r[1][1] < 0 || p(0) + R(0,1)*r[1][1] + R(0,0)*r[1][0] < 0)
		)
	{
			den = 1 - R(1,0)*R(1,0);

			s = (-R(1,0)*R1dotP + p(1) + R(1,1)*r[1][1])/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = - R1dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,0)*s - R1dotP;
			}
			else
			{
				t = R(1,0)*s - R1dotP;
			}


			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1) + r[1][1]*R(1,1),0.0,r[0][1]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(1,0)*t + p(1) + r[1][1]*R(1,1),0.0,r[0][1]);
			}


			if(p(0) + R(0,1)*r[1][1] + R(0,0)*t < 0 && s*R(1,1) > R2dotP + r[1][1])
			{
				v = Vector(
					p(0) + R(0,1)*r[1][1] + R(0,0)*t,
					p(1) + R(1,1)*r[1][1] + R(1,0)*t - s,
					p(2) + R(2,1)*r[1][1] + R(2,0)*t );

				return sqrt(dot(v,v));
			}
	}



	//A2 and A3 in V(eB1)
	if(R2dotP > R(0,1)*r[0][0] && R2dotP > R(0,1)*r[0][0] + R(1,1)*r[0][1])
	{
		//B1 and B2 in V(eA2)
		if(p(0) > r[0][0] && p(0) + R(0,0)*r[1][0] > r[0][0])
		{
			den = 1 - R(1,0)*R(1,0);

			s = (R(1,0)*(r[0][0]*R(0,0)-R1dotP) + p(1))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][0]*R(0,0) - R1dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,0)*s + r[0][0]*R(0,0) - R1dotP;
			}
			else
			{
				t = R(1,0)*s + r[0][0]*R(0,0) - R1dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1),0.0,r[0][1]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(1,0)*t + p(1),0.0,r[0][1]);
			}

			v = Vector(
				p(0) + R(0,0)*t - r[0][0],
				p(1) + R(1,0)*t - s,
				p(2) + R(2,0)*t );

			return sqrt(dot(v,v));
		}
		else if(p(0) > r[0][0] || p(0) + R(0,0)*r[1][0] > r[0][0])
		{
			den = 1 - R(1,0)*R(1,0);

			s = (R(1,0)*(r[0][0]*R(0,0)-R1dotP) + p(1))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][0]*R(0,0) - R1dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,0)*s + r[0][0]*R(0,0) - R1dotP;
			}
			else
			{
				t = R(1,0)*s + r[0][0]*R(0,0) - R1dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1),0.0,r[0][1]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(1,0)*t + p(1),0.0,r[0][1]);
			}

			if(p(0) + R(0,0)*t > r[0][0])
			{
				v = Vector(
					p(0) + R(0,0)*t - r[0][0],
					p(1) + R(1,0)*t - s,
					p(2) + R(2,0)*t );

				return sqrt(dot(v,v));
			}
		}
	}
	else if(
		(R2dotP > R(0,1)*r[0][0] || R2dotP > R(0,1)*r[0][0] + R(1,1)*r[0][1])
		&&
		(p(0) > r[0][0] || p(0) + R(0,0)*r[1][0] > r[0][0])
		)
	{
			den = 1 - R(1,0)*R(1,0);

			s = (R(1,0)*(r[0][0]*R(0,0)-R1dotP) + p(1))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][0]*R(0,0) - R1dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,0)*s + r[0][0]*R(0,0) - R1dotP;
			}
			else
			{
				t = R(1,0)*s + r[0][0]*R(0,0) - R1dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1),0.0,r[0][1]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(1,0)*t + p(1),0.0,r[0][1]);
			}

			if(p(0) + R(0,0)*t > r[0][0] && R2dotP > R(1,1)*s + R(0,1)*r[0][0])
			{
				v = Vector(
					p(0) + R(0,0)*t - r[0][0],
					p(1) + R(1,0)*t - s,
					p(2) + R(2,0)*t );

				return sqrt(dot(v,v));
			}
	}



	//A2 and A3 in V(eB3)
	if(r[0][0]*R(0,1)>r[1][1]+R2dotP && r[0][0]*R(0,1) + r[0][1]*R(1,1) > r[1][1]+R2dotP)
	{
		//B3 and B4 in V(eA2)
		if(p(0) + R(0,1)*r[1][1] > r[0][0] && p(0) + R(0,1)*r[1][1] + R(0,0)*r[1][0] > r[0][0])
		{
			den = 1 - R(1,0)*R(1,0);

			s = (R(1,0)*(r[0][0]*R(0,0)-R1dotP) + p(1) + R(1,1)*r[1][1])/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][0]*R(0,0) - R1dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,0)*s + r[0][0]*R(0,0) - R1dotP;
			}
			else
			{
				t = R(1,0)*s + r[0][0]*R(0,0) - R1dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1) + r[1][1]*R(1,1),0.0,r[0][1]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(1,0)*t + p(1) + r[1][1]*R(1,1),0.0,r[0][1]);
			}

			v = Vector(
				p(0) + R(0,1)*r[1][1] + R(0,0)*t - r[0][0],
				p(1) + R(1,1)*r[1][1] + R(1,0)*t - s,
				p(2) + R(2,1)*r[1][1] + R(2,0)*t );

			return sqrt(dot(v,v));
		}
		else if(p(0) + R(0,1)*r[1][1] > r[0][0] || p(0) + R(0,1)*r[1][1] + R(0,0)*r[1][0] > r[0][0])
		{
			den = 1 - R(1,0)*R(1,0);

			s = (R(1,0)*(r[0][0]*R(0,0)-R1dotP) + p(1) + R(1,1)*r[1][1])/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][0]*R(0,0) - R1dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,0)*s + r[0][0]*R(0,0) - R1dotP;
			}
			else
			{
				t = R(1,0)*s + r[0][0]*R(0,0) - R1dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1) + r[1][1]*R(1,1),0.0,r[0][1]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(1,0)*t + p(1) + r[1][1]*R(1,1),0.0,r[0][1]);
			}

			if(p(0) + R(0,1)*r[1][1] + R(0,0)*t > r[0][0])
			{
				v = Vector(
					p(0) + R(0,1)*r[1][1] + R(0,0)*t - r[0][0],
					p(1) + R(1,1)*r[1][1] + R(1,0)*t - s,
					p(2) + R(2,1)*r[1][1] + R(2,0)*t );

				return sqrt(dot(v,v));
			}
		}
	}
	else if(
		(r[0][0]*R(0,1)>r[1][1]+R2dotP || r[0][0]*R(0,1) + r[0][1]*R(1,1) > r[1][1]+R2dotP)
		&&
		(p(0) + R(0,1)*r[1][1] > r[0][0] || p(0) + R(0,1)*r[1][1] + R(0,0)*r[1][0] > r[0][0])

		)
	{
			den = 1 - R(1,0)*R(1,0);

			s = (R(1,0)*(r[0][0]*R(0,0)-R1dotP) + p(1) + R(1,1)*r[1][1])/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][0]*R(0,0) - R1dotP;
			}
			else if(s > r[0][1])
			{
				s = r[0][1];
				t = R(1,0)*s + r[0][0]*R(0,0) - R1dotP;
			}
			else
			{
				t = R(1,0)*s + r[0][0]*R(0,0) - R1dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(1) + r[1][1]*R(1,1),0.0,r[0][1]);
			}
			else if(t > r[1][0])
			{
				t = r[1][0];
				s = Math::clamp(R(1,0)*t + p(1) + r[1][1]*R(1,1),0.0,r[0][1]);
			}

			if(p(0) + R(0,1)*r[1][1] + R(0,0)*t > r[0][0] && r[0][0]*R(0,1) + s*R(1,1) > r[1][1]+R2dotP)
			{
				v = Vector(
					p(0) + R(0,1)*r[1][1] + R(0,0)*t - r[0][0],
					p(1) + R(1,1)*r[1][1] + R(1,0)*t - s,
					p(2) + R(2,1)*r[1][1] + R(2,0)*t );

				return sqrt(dot(v,v));
			}
	}



	//A3 and A4 in V(eB4)
	if(R1dotP > r[0][1]*R(1,0) && R1dotP > r[0][1]*R(1,0) + r[0][0]*R(0,0))
	{
		//B1 and B4 in V(eA3)
		if(p(1) > r[0][1] && p(1) + R(1,1)*r[1][1] > r[0][1])
		{
			den = 1 - R(0,1)*R(0,1);

			s = (R(0,1)*(r[0][1]*R(1,1)-R2dotP) + p(0))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][1]*R(1,1) - R2dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,1)*s + r[0][1]*R(1,1) - R2dotP;
			}
			else
			{
				t = R(0,1)*s + r[0][1]*R(1,1) - R2dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0),0.0,r[0][0]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(0,1)*t + p(0),0.0,r[0][0]);
			}

			v = Vector(
				p(0) + R(0,1)*t - s,
				p(1) + R(1,1)*t - r[0][1],
				p(2) + R(2,1)*t );

			return sqrt(dot(v,v));
		}
		else if(p(1) > r[0][1] || p(1) + R(1,1)*r[1][1] > r[0][1])
		{
			den = 1 - R(0,1)*R(0,1);

			s = (R(0,1)*(r[0][1]*R(1,1)-R2dotP) + p(0))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][1]*R(1,1) - R2dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,1)*s + r[0][1]*R(1,1) - R2dotP;
			}
			else
			{
				t = R(0,1)*s + r[0][1]*R(1,1) - R2dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0),0.0,r[0][0]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(0,1)*t + p(0),0.0,r[0][0]);
			}

			if(p(1) + R(1,1)*t > r[0][1])
			{
				v = Vector(
					p(0) + R(0,1)*t - s,
					p(1) + R(1,1)*t - r[0][1],
					p(2) + R(2,1)*t );

				return sqrt(dot(v,v));
			}
		}
	}
	else if(
		(R1dotP > r[0][1]*R(1,0) || R1dotP > r[0][1]*R(1,0) + r[0][0]*R(0,0))
		&&
		(p(1) > r[0][1] || p(1) + R(1,1)*r[1][1] > r[0][1])
		)
	{
			den = 1 - R(0,1)*R(0,1);
		
			s = (R(0,1)*(r[0][1]*R(1,1)-R2dotP) + p(0))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][1]*R(1,1) - R2dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,1)*s + r[0][1]*R(1,1) - R2dotP;
			}
			else
			{
				t = R(0,1)*s + r[0][1]*R(1,1) - R2dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0),0.0,r[0][0]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(0,1)*t + p(0),0.0,r[0][0]);
			}

			if(p(1) + R(1,1)*t > r[0][1] && R1dotP > r[0][1]*R(1,0) + s*R(0,0))
			{
				v = Vector(
					p(0) + R(0,1)*t - s,
					p(1) + R(1,1)*t - r[0][1],
					p(2) + R(2,1)*t );

				return sqrt(dot(v,v));
			}
	}


	//A3 and A4 in V(eB2)
	if(r[0][1]*R(1,0) > R1dotP + r[1][0] && r[0][1]*R(1,0) + r[0][0]*R(0,0) > R1dotP + r[1][0])
	{
		//B2 and B3 in V(eA3)
		if(p(1) + r[1][0]*R(1,0) > r[0][1] && p(1) + r[1][0]*R(1,0) + r[1][1]*R(1,1)> r[0][1])
		{
			den = 1 - R(0,1)*R(0,1);

			s = (R(0,1)*(r[0][1]*R(1,1)-R2dotP) + p(0) + r[1][0]*R(0,0))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][1]*R(1,1) - R2dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,1)*s + r[0][1]*R(1,1) - R2dotP;
			}
			else
			{
				t = R(0,1)*s + r[0][1]*R(1,1) - R2dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0) + r[1][0]*R(0,0),0.0,r[0][0]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(0,1)*t + p(0) + r[1][0]*R(0,0),0.0,r[0][0]);
			}

			v = Vector(
				p(0) + R(0,0)*r[1][0] + R(0,1)*t - s,
				p(1) + R(1,0)*r[1][0] + R(1,1)*t - r[0][1],
				p(2) + R(2,0)*r[1][0] + R(2,1)*t );

			return sqrt(dot(v,v));
		}
		else if(p(1) + r[1][0]*R(1,0) > r[0][1] || p(1) + r[1][0]*R(1,0) + r[1][1]*R(1,1)> r[0][1])
		{
			den = 1 - R(0,1)*R(0,1);

			s = (R(0,1)*(r[0][1]*R(1,1)-R2dotP) + p(0) + r[1][0]*R(0,0))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][1]*R(1,1) - R2dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,1)*s + r[0][1]*R(1,1) - R2dotP;
			}
			else
			{
				t = R(0,1)*s + r[0][1]*R(1,1) - R2dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0) + r[1][0]*R(0,0),0.0,r[0][0]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(0,1)*t + p(0) + r[1][0]*R(0,0),0.0,r[0][0]);
			}

			if(p(1) + r[1][0]*R(1,0) + t*R(1,1)> r[0][1])
			{
				v = Vector(
					p(0) + R(0,0)*r[1][0] + R(0,1)*t - s,
					p(1) + R(1,0)*r[1][0] + R(1,1)*t - r[0][1],
					p(2) + R(2,0)*r[1][0] + R(2,1)*t );

				return sqrt(dot(v,v));
			}
		}
	}
	else if(
		(r[0][1]*R(1,0) > R1dotP + r[1][0] || r[0][1]*R(1,0) + r[0][0]*R(0,0) > R1dotP + r[1][0])
		&&
		(p(1) + r[1][0]*R(1,0) > r[0][1] || p(1) + r[1][0]*R(1,0) + r[1][1]*R(1,1)> r[0][1])
		)
	{
			den = 1 - R(0,1)*R(0,1);

			s = (R(0,1)*(r[0][1]*R(1,1)-R2dotP) + p(0) + r[1][0]*R(0,0))/den;

			if(s < 0.0)
			{
				s = 0.0;
				t = r[0][1]*R(1,1) - R2dotP;
			}
			else if(s > r[0][0])
			{
				s = r[0][0];
				t = R(0,1)*s + r[0][1]*R(1,1) - R2dotP;
			}
			else
			{
				t = R(0,1)*s + r[0][1]*R(1,1) - R2dotP;
			}

			if(t < 0.0)
			{
				t = 0.0;
				s = Math::clamp(p(0) + r[1][0]*R(0,0),0.0,r[0][0]);
			}
			else if(t > r[1][1])
			{
				t = r[1][1];
				s = Math::clamp(R(0,1)*t + p(0) + r[1][0]*R(0,0),0.0,r[0][0]);
			}

			if(p(1) + r[1][0]*R(1,0) + t*R(1,1)> r[0][1] && r[0][1]*R(1,0) + s*R(0,0) > R1dotP + r[1][0])
			{
				v = Vector(
					p(0) + R(0,0)*r[1][0] + R(0,1)*t - s,
					p(1) + R(1,0)*r[1][0] + R(1,1)*t - r[0][1],
					p(2) + R(2,0)*r[1][0] + R(2,1)*t );

				return sqrt(dot(v,v));
			}
	}

	//No side-pairs has the shortest distance
	double z = R(0,2) * p(0) + R(1,2) * p(1) + R(2,2) * p(2);
	double s1, s2;
 
	if (p[2] > 0.0)
	{
		s1 = p[2];
		if (R(2,0) < 0.0) s1 += r[1][0]*R(2,0);
		if (R(2,1) < 0.0) s1 += r[1][1]*R(2,1);
	}
	else
	{
		s1 = -p[2];
		if (R(2,0) > 0.0) s1 -= r[1][0]*R(2,0);
		if (R(2,1) > 0.0) s1 -= r[1][1]*R(2,1);
	}
  
	if (z < 0)
	{
		s2 = -z;
		if (R(0,2) < 0.0) s2 += r[0][0]*R(0,2);
		if (R(1,2) < 0.0) s2 += r[0][1]*R(1,2);
	}
	else
	{
		s2 = z;
		if (R(0,2) > 0.0) s2 -= r[0][0]*R(0,2);
		if (R(1,2) > 0.0) s2 -= r[0][1]*R(1,2);
	}

	s = (s1 > s2? s1 : s2);
  
	return (s > 0? s : 0);
}