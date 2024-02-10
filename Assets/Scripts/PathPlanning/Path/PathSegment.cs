using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace PathPlanning
{
    abstract class PathSegment
    {
        public PathSegment parent;
        public float maxInSpeed;
        public float maxSpeed;
        public float length;
        public float minCost;
        public Vector2 p1;
        public Vector2 p2;
        public Vector2 endVel;
        public float speedLimit;
        public float accel;
        public float decel;
        public float totalLength;

        public PathSegment(float maxSpeed, float length, float accel, float decel, PathSegment parent = null)
        {
            this.length = length;
            this.parent = parent;
            this.speedLimit = maxSpeed; //
            this.accel = accel;         // Cached for recalculating costs
            this.decel = decel;         //

            UpdateMinCost();
        }

        // Abstract functions to be implemented by subclasses
        public abstract Vector2 GetPosition(float distance);
        public abstract void DebugDraw(Color color);

        // Updates minimum cost of the path
        public void UpdateMinCost()
        {
            float maxSpeed = speedLimit;
            this.maxInSpeed = parent == null ? 0 : parent.maxSpeed;
            minCost = parent == null ? 0 : parent.Cost(maxSpeed, accel, decel);
            totalLength = parent == null ? length : length + parent.totalLength;
            if (maxInSpeed >= maxSpeed) // ---- No acceleration
            {
                this.maxSpeed = maxSpeed;
                minCost += length / maxSpeed;
            }
            else // /''''' Accelerate to max
            {
                float t1, t2;
                (t1, t2) = ConstrainedAccel(maxInSpeed, length, accel, maxSpeed);

                float v = maxInSpeed + accel * t1;

                this.maxSpeed = Mathf.Min(v, maxSpeed);
                minCost += t1 + t2;
            }
        }
        
        // Calculates the cost of the path with constraint on the outspeed
        public float Cost(float outSpeed, float accel, float decel)
        {
            if (outSpeed >= maxSpeed)
            {
                return minCost;
            }
            else if (outSpeed >= maxInSpeed) // /'''''\ Accelerate then decelerate to match out speed
            {
                float t1, t2, t3;
                (t1, t2, t3) = ConstrainedMatchAccel(maxInSpeed, outSpeed, length, accel, decel, maxSpeed);
                return ParentMinCost() + t1 + t2 + t3;
            }
            else if (maxSpeed <= maxInSpeed) // Outspeed less than inspeed and max speed less than input speed
            {
                float v, t;
                (v, t) = Accel(outSpeed, length, decel); // The initial speed that is necessary to decelerate to end of path

                if (v > maxSpeed)  // Can decelerate enough, '''''\ Pure deceleration
                {
                    float t1, t2;
                    (t1, t2) = ConstrainedAccel(outSpeed, length, accel, maxSpeed);
                    return ParentMinCost() + t1 + t2;
                }
                else // Need to start decelerating from an earlier point
                {
                    return parent.Cost(v, accel, decel) + t;
                }
            }
            else // Outspeed less than inspeed with max speed greater than input speed
            {
                float v, t;
                (v, t) = Accel(outSpeed, length, decel); // The initial speed that is necessary to decelerate to end of path

                if (v > maxInSpeed)  // Can decelerate enough, /'''''\ Accelerate then decelerate to match out speed
                {
                    float t1, t2, t3;
                    (t1, t2, t3) = ConstrainedMatchAccel(maxInSpeed, outSpeed, length, accel, decel, maxSpeed);
                    return ParentMinCost() + t1 + t2 + t3;
                }
                else // Need to start decelerating from an earlier point
                {
                    return parent.Cost(v, accel, decel) + t;
                }
            }
        }

        // Creates a list of positions and timestamps from the path
        public (List<Vector2>, List<float>) Render(float outSpeed, float accel, float decel, float resolution)
        {
            
            if (outSpeed >= maxSpeed)
            {
                (List<Vector2>, List<float>) previous = parent == null ? RenderStart() : parent.Render(maxSpeed, accel, decel, resolution);
                if (maxInSpeed >= maxSpeed)
                {
                    float t = length / maxSpeed;
                    return RenderFlat(0, t, maxSpeed, resolution, previous);
                }
                else
                {
                    float t1, t2;
                    (t1, t2) = ConstrainedAccel(maxInSpeed, length, accel, maxSpeed);

                    RenderAccel(0, t1, maxInSpeed, accel, resolution, previous);
                    if (t2 > 0)
                    {
                        float d1 = maxInSpeed*t1 + 0.5f*accel*t1*t1;
                        RenderFlat(d1, t2, maxSpeed, resolution, previous);
                    }
                    return previous;
                }
            }
            else if (outSpeed >= maxInSpeed) // /'''''\ Accelerate then decelerate to match out speed
            {
                (List<Vector2>, List<float>) previous = parent == null ? RenderStart() : parent.Render(maxInSpeed, accel, decel, resolution);

                float t1, t2, t3;
                (t1, t2, t3) = ConstrainedMatchAccel(maxInSpeed, outSpeed, length, accel, decel, maxSpeed);            
                
                return RenderMatchAccel(0, t1, t2, t3, maxInSpeed, maxSpeed, accel, decel, resolution, previous);
            }
            else if (maxSpeed <= maxInSpeed)
            {
                float v, t;
                (v, t) = Accel(outSpeed, length, decel); // The initial speed that is necessary to decelerate to end of path

                if (v > maxSpeed)  // Can decelerate enough, '''''\ Pure deceleration
                {
                    (List<Vector2>, List<float>) previous = parent.Render(maxSpeed, accel, decel, resolution);

                    float t1, t2;
                    (t1, t2) = ConstrainedAccel(outSpeed, length, decel, maxSpeed);

                    if (t2 > 0)
                    {
                        RenderFlat(0, t2, maxSpeed, resolution, previous);
                    }
                    float d1 = maxSpeed * t2;
                    RenderAccel(d1, t1, maxSpeed, -decel, resolution, previous);
                    
                    return previous;

                }
                else // Need to start decelerating from an earlier point
                {
                    (List<Vector2>, List<float>) previous = parent.Render(v, accel, decel, resolution);

                    return RenderAccel(0, t, v, -decel, resolution, previous);
                }
            }
            else // Outspeed less than inspeed, need to decelerate somehow
            {
                float v, t;
                (v, t) = Accel(outSpeed, length, decel); // The initial speed that is necessary to decelerate to end of path

                if (v > maxInSpeed)  // Can decelerate enough, /'''''\ Accelerate then decelerate to match out speed
                {
                    (List<Vector2>, List<float>) previous = parent.Render(maxInSpeed, accel, decel, resolution);

                    float t1, t2, t3;
                    (t1, t2, t3) = ConstrainedMatchAccel(maxInSpeed, outSpeed, length, accel, decel, maxSpeed);

                    return RenderMatchAccel(0, t1, t2, t3, maxInSpeed, maxSpeed, accel, decel, resolution, previous);
                }
                else // Need to start decelerating from an earlier point
                {
                    (List<Vector2>, List<float>) previous = parent.Render(v, accel, decel, resolution);

                    return RenderAccel(0, t, v, -decel, resolution, previous);
                }
            }
        }

        // Utility functions
        (List<Vector2>, List<float>) RenderFlat(float dStart, float t, float maxSpeed, float resolution, (List<Vector2>, List<float>) previous)
        {
            float tPrev = previous.Item2.Last();
            int subdivisions = (int)Mathf.Ceil(t / resolution);

            for (int i = 1; i <= subdivisions; i++)
            {
                float T = t * i / subdivisions;
                float d = dStart + maxSpeed * T;
                previous.Item1.Add(GetPosition(d));
                previous.Item2.Add(tPrev + T);
            }
            return previous;
        }
        (List<Vector2>, List<float>) RenderAccel(float dStart, float t, float startSpeed, float accel, float resolution, (List<Vector2>, List<float>) previous)
        {
            float tPrev = previous.Item2.Last();
            int subdivisions = (int)Mathf.Ceil(t / resolution);
            for (int i = 1; i <= subdivisions; i++)
            {
                float T = t * i / subdivisions;
                float d = dStart + startSpeed * T + 0.5f * accel * T * T;

                previous.Item1.Add(GetPosition(d));
                previous.Item2.Add(tPrev + T);
            }
            return previous;
        }
        (List<Vector2>, List<float>) RenderMatchAccel(float dStart, float t1, float t2, float t3, float startSpeed, float maxSpeed, float accel, float decel, float resolution, (List<Vector2>, List<float>) previous)
        {
            float peakSpeed = startSpeed + accel * t1;

            RenderAccel(dStart, t1, startSpeed, accel, resolution, previous);
            float d1 = dStart + startSpeed * t1 + 0.5f * accel * t1 * t1;

            if (t2 > 0)
            {
                RenderFlat(d1, t2, maxSpeed, resolution, previous);
                float d2 = d1 + maxSpeed * t2;
                RenderAccel(d2, t3, maxSpeed, -decel, resolution, previous);

                return previous;
            }
            else
            {
                RenderAccel(d1, t3, peakSpeed, -decel, resolution, previous);
                return previous;
            }
        }
        public (List<Vector2>, List<float>) RenderStart()
        {
            var positions = new List<Vector2>() { p1 };
            var times = new List<float>() { 0 };
            return (positions, times);
        }
        public static (float, float) ConstrainedAccel(float v0, float length, float accel, float speedLimit)
        {
            float v, t;
            (v, t) = Accel(v0, length, accel);

            if (v < speedLimit)
            {
                return (t, 0);
            }
            else
            {
                float t1 = (speedLimit - v0) / accel;
                float d1 = v0*t1 + 0.5f*accel*t1*t1;
                float d2 = length - d1;
                float t2 = d2 / speedLimit;

                return (t1, t2);
            }
        }
        public static (float, float) Accel(float v0, float length, float accel)
        {
            float p = 2 * v0 / accel;
            float q = -2 * length / accel;
            float t = -0.5f*p + Mathf.Sqrt(0.25f*p * p - q);
            float v = v0 + accel * t;
            return (v, t);
        }
        public static (float, float, float) ConstrainedMatchAccel(float v1, float v2, float length, float accel, float decel, float speedLimit)
        {
            float t1, t2;
            (t1, t2) = MatchAccel(v1, v2, length, accel, decel);
            // If SpeedAt(t1) > speedLimit => etc
            float vMiddle = v1 + accel * t1;
            if (vMiddle <= speedLimit)
            {
                return (t1, 0, t2);
            }
            else
            {
                t1 = (speedLimit - v1) / accel;
                t2 = (speedLimit - v2) / decel;

                float d1 = v1*t1 + 0.5f*accel*t1*t1;
                float d2 = v2*t2 + 0.5f*decel*t2*t2;

                float dm = length - d1 - d2;
                float tm = dm / speedLimit;

                return (t1, tm, t2);
            }
        }
        public static (float, float) MatchAccel(float v1, float v2, float length, float accel, float decel)
        {
            // WolframAlpha {A*x + 0.5*a*x^2 + B*y+0.5*b*y^2=d, A + ax = B + by}

            float a = accel;
            float b = decel;
            float d = length;
            float t1 = Mathf.Sqrt((a + b)*(2*a*b*d + a*v2*v2 + b*v1*v1)) - (a + b)*v1;
            t1 /= a*(a + b);

            float t2 = (v1 + a*t1 - v2) / b;

            return (t1, t2);
        }
        public float ParentMinCost()
        {
            return parent == null ? 0 : parent.minCost;
        }
    }
}
