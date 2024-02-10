using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace PathPlanning
{
    class CirclePathSegment : PathSegment
    {
        public float radius;
        public int sgn;
        public float angle;
        public Vector2 c;
        public CirclePathSegment(float maxSpeed, float length, float accel, float decel, PathSegment parent = null) : base(maxSpeed, length, accel, decel, parent)
        {}

        public override Vector2 GetPosition(float distance)
        {
            float angle = distance / radius;
            Vector2 p1c = p1 - c;
            Vector2 pc = Rotate(p1c, -angle * sgn);
            return pc + c;
        }
        public override void DebugDraw(Color color)
        {
            int subdivisions = 10;

            Vector2 p1c = p1 - c;

            Vector2 pLast = p1;
            for (int i = 1; i <= subdivisions; i++)
            {
                Vector2 pc = Rotate(p1c, -i * angle * sgn / subdivisions);

                DebugDrawLine(pLast, pc + c, color);
                pLast = pc + c;
            }
        }
        Vector2 Rotate(Vector2 v, float angle)
        {
            float co = Mathf.Cos(angle);
            float si = Mathf.Sin(angle);
            return new Vector2(v.x * co - v.y * si, v.x * si + v.y * co);
        }
        void DebugDrawLine(Vector2 a, Vector2 b, Color color)
        {
            Debug.DrawLine(new Vector3(a.x, 0.3f, a.y), new Vector3(b.x, 0.3f, b.y), color, 1000f);
        }

        
    }
}
