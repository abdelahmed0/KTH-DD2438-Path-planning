using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace PathPlanning
{
    class LocalPlanner
    {
        public float maxSpeed = 25f; // Global speed limit
        public float minSpeed = 5f;  // Speed at minimum turn radius
        public float minTurnRadius = 6f; // Minimum turn radius
        public float maxTurnRadius = 120f; // Maximum turn radius allowed before being considered a straight line
        public float acceleration = 3f; // Acceleration of the vehicle
        public float deceleration = 3f; // Deceleration of the vehicle
        public float turnRadiusUncap = 35f; // Turn radius at which no speed limit is applied

        // Set speed limit based on curvature
        public float MaxSpeedOnCurve(float radius)
        {
            if (radius > turnRadiusUncap)
            {
                return maxSpeed;
            }
            float g = minSpeed*minSpeed / minTurnRadius;
            return Mathf.Sqrt(g * radius);
        }
        
        // Set turn radius based on angle between velocity and next point on the path
        public float TurnRadiusFromAngle(float angle)
        {
            float k = minTurnRadius * 0.5f * Mathf.PI;
            return k / angle;
        }
        public float ShortLineLength()
        {
            return minTurnRadius * 0.5f * Mathf.PI;
        }

        // Find Dubins path between two points with relaxed heading
        // If point is too close to drive to, return an approximate path
        public List<PathSegment> FindPathRelaxed(Vector2 p1, Vector2 p2, Vector2 v0, PathSegment parent=null)
        {
            Vector2 p = p2 - p1;
            float rawAngle = Vector2.Angle(p, v0) * Mathf.PI / 180;
            if (rawAngle > 0.5f*Mathf.PI)
            {
                return null;
            }
            Vector2 vNorm = v0.normalized;

            float rawRadius = TurnRadiusFromAngle(rawAngle);
            float radius = Mathf.Min(rawRadius, maxTurnRadius);

            // Right or left turn
            int sgn = Math.Sign(Cross2d(p, v0));
            // Find center of circle
            Vector2 c = p1 - sgn * radius * Rot90(vNorm);

            float D = (p2 - c).magnitude;

            // Cannot drive to point, approximate
            if (D < radius)
            {
                if (rawRadius > maxTurnRadius) // Line
                {
                    float length = ShortLineLength();
                    var path = new LinePathSegment(this.maxSpeed, length, acceleration, deceleration, parent);
                    path.p1 = p1;
                    path.p2 = p1 + vNorm * length;
                    path.endVel = vNorm;

                    return new List<PathSegment>() { path };
                }
                else
                {
                    float length = rawAngle * radius;
                    float maxSpeed = MaxSpeedOnCurve(radius);
                    
                    var path = new CirclePathSegment(maxSpeed, length, acceleration, deceleration, parent);
                    path.radius = radius;
                    path.sgn = sgn;
                    path.angle = rawAngle;
                    path.c = c;
                    path.p1 = p1;
                    path.p2 = Rotate(p1-c, -rawAngle * sgn) + c;
                    path.endVel = Rot90(-sgn*(path.p2 - c));

                    return new List<PathSegment>() { path };
                }
            }
            else
            {
                // Angle to tangent point
                float co = radius / D;
                float si = sgn * Mathf.Sqrt(1 - co * co);

                Vector2 d = (p2 - c) / D; // Vector towards end point
                Vector2 n = new Vector2(d.x * co - d.y * si, d.x * si + d.y * co); // Rotate that vector to get direction toward tangent point
                Vector2 t = c + n * radius; // Finally calculate tangent point

                float angle = AngleOnCircle(p1 - c, t - c, sgn);
                if (angle > 0.5f*Mathf.PI)
                {
                    return null;
                }

                float circleLength = radius * angle;
                float maxSpeed = MaxSpeedOnCurve(radius);

                var circlePath = new CirclePathSegment(maxSpeed, circleLength, acceleration, deceleration, parent);
                circlePath.radius = radius;
                circlePath.sgn = sgn;
                circlePath.angle = angle;
                circlePath.c = c;
                circlePath.p1 = p1;
                circlePath.p2 = t;
                circlePath.endVel = Rot90(-sgn * (t - c));

                float lineLength = (p2 - t).magnitude;
                var linePath = new LinePathSegment(this.maxSpeed, lineLength, acceleration, deceleration, circlePath);
                linePath.p1 = t;
                linePath.p2 = p2;
                linePath.endVel = (p2 - t).normalized;


                return new List<PathSegment>() { circlePath, linePath };
            }
        }

        // Find Dubins path between two points
        public List<PathSegment> FindDubinsPath(Vector2 p1, Vector2 p2, Vector2 v1, Vector2 v2, PathSegment parent = null)
        {
            Vector2 p = p2 - p1;
            float rawAngle1 = Vector2.Angle(p, v1) * Mathf.PI / 180;
            float rawAngle2 = Vector2.Angle(p, v2) * Mathf.PI / 180;
            if (rawAngle1 > 0.5f * Mathf.PI || rawAngle2 > 0.5f * Mathf.PI)
            {
                return null;
            }
            Vector2 v1Norm = v1.normalized;
            Vector2 v2Norm = v2.normalized;

            float rawRadius1 = TurnRadiusFromAngle(rawAngle1);
            float rawRadius2 = TurnRadiusFromAngle(rawAngle2);
            float r1 = Mathf.Min(rawRadius1, maxTurnRadius);
            float r2 = Mathf.Min(rawRadius2, maxTurnRadius);

            // Right or left turn
            int sgn1 = Math.Sign(Cross2d(p, v1));
            int sgn2 = Math.Sign(Cross2d(-p, v2));

            // Find center of circle
            Vector2 c1 = p1 - sgn1 * r1 * Rot90(v1Norm);
            Vector2 c2 = p2 - sgn2 * r2 * Rot90(v2Norm);

            float D = (c2 - c1).magnitude;

            // Cannot drive to point
            if (D <= r1 + r2 && sgn1 != sgn2)
            {
                return null;
            }
           
            // Angle to tangent point
            float co = (r1 - sgn1*sgn2*r2) / D;
            float si = sgn1 * Mathf.Sqrt(1 - co * co);

            Vector2 d = (c2 - c1) / D; // Vector towards end point
            Vector2 n = new Vector2(d.x * co - d.y * si, d.x * si + d.y * co); // Rotate that vector to get direction toward tangent point
            Vector2 t1 = c1 + n*r1; // Finally calculate tangent point
            Vector2 t2 = c2 + sgn1*sgn2*n*r2;

            float angle1 = AngleOnCircle(p1 - c1, t1 - c1, sgn1);
            float angle2 = AngleOnCircle(t2 - c2, p2 - c2, sgn2);
            if (angle1 > 0.5f * Mathf.PI || angle2 > 0.5f * Mathf.PI)
            {
                return null;
            }

            float circle1Length = r1 * angle1;
            float maxSpeed1 = MaxSpeedOnCurve(r1);

            var circle1 = new CirclePathSegment(maxSpeed1, circle1Length, acceleration, deceleration, parent);
            circle1.radius = r1;
            circle1.sgn = sgn1;
            circle1.angle = angle1;
            circle1.c = c1;
            circle1.p1 = p1;
            circle1.p2 = t1;
            circle1.endVel = Rot90(-sgn1 * (t1 - c1));

            float lineLength = (t2 - t1).magnitude;
            var linePath = new LinePathSegment(this.maxSpeed, lineLength, acceleration, deceleration, circle1);
            linePath.p1 = t1;
            linePath.p2 = t2;
            linePath.endVel = (t2 - t1).normalized;

            float circle2Length = r2 * angle2;
            float maxSpeed2 = MaxSpeedOnCurve(r2);

            var circle2 = new CirclePathSegment(maxSpeed2, circle2Length, acceleration, deceleration, linePath);
            circle2.radius = r2;
            circle2.sgn = sgn2;
            circle2.angle = angle2;
            circle2.c = c2;
            circle2.p1 = t2;
            circle2.p2 = p2;
            circle2.endVel = Rot90(-sgn2 * (t2 - c2));


            return new List<PathSegment>() { circle1, linePath, circle2 };
        }

        float AngleOnCircle(Vector2 p1, Vector2 p2, int sgn)
        {
            // Calculates angle traversed between two points on circle given the driving direction
            float theta = Mathf.Atan2(p2.y, p2.x) - Mathf.Atan2(p1.y, p1.x);
            if (theta < 0 && sgn == -1)
                theta += 2 * Mathf.PI;
            else if (theta > 0 && sgn == 1)
                theta -= 2 * Mathf.PI;

            return Mathf.Abs(theta);
        }
        Vector2 Rotate(Vector2 v, float angle)
        {
            float co = Mathf.Cos(angle);
            float si = Mathf.Sin(angle);
            return new Vector2(v.x * co - v.y * si, v.x * si + v.y * co);
        }
        float Cross2d(Vector2 a, Vector2 b)
        {
            return a.x * b.y - a.y * b.x;
        }
        Vector2 Rot90(Vector2 v)
        {
            return new Vector2(-v.y, v.x);
        }
    }
}
