using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace PathPlanning
{
    class LinePathSegment : PathSegment
    {

        public LinePathSegment(float maxSpeed, float length, float accel, float decel, PathSegment parent = null) : base(maxSpeed, length, accel, decel, parent)
        {}

        public override Vector2 GetPosition(float distance)
        {
            float tau = distance / length;
            return p1 * (1-tau) + p2*tau;
        }
        public override void DebugDraw(Color color)
        {
            DebugDrawLine(p1, p2, color);
        }
        void DebugDrawLine(Vector2 a, Vector2 b, Color color)
        {
            Debug.DrawLine(new Vector3(a.x, 0.3f, a.y), new Vector3(b.x, 0.3f, b.y), color, 1000f);
        }
    }
}
