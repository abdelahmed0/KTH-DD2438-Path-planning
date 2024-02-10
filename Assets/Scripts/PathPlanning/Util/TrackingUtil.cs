using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace util
{
    class TrackingUtil
    {
        // Get position on the path a certain distance ahead of a given position
        public static (Vector2, Vector2) LookAheadPositionAndVelocity(Vector2 pos, float lookAhead, List<Vector2> positions, List<float> times)
        {
            float minDistance = float.MaxValue;
            int index = 0;
            for (int i = 0; i < positions.Count; i++)
            {
                float distance = (pos - positions[i]).sqrMagnitude;
                if (distance < minDistance)
                {
                    index = i;
                    minDistance = distance;
                }
            }
            int lookAheadIndex = -1;
            for (int i = index; i < positions.Count; i++)
            {
                float distance = (pos - positions[i]).magnitude;
                if (distance > lookAhead)
                {
                    lookAheadIndex = i - 1;
                    break;
                }
            }

            if (lookAheadIndex != -1)
            {
                Vector2 p1 = positions[lookAheadIndex];
                Vector2 p2 = positions[lookAheadIndex + 1];
                float t1 = times[lookAheadIndex];
                float t2 = times[lookAheadIndex + 1];

                float startDist = (pos - p1).magnitude;
                float endDist = (pos - p2).magnitude;

                float mix = (lookAhead - startDist) / (endDist - startDist);
                Vector2 lookAheadPos = p1 * (1 - mix) + p2 * mix;
                Vector2 velocity = (p2 - p1) / (t2 - t1);

                return (lookAheadPos, velocity);
            }
            else
            {
                int hi = positions.Count - 1;
                Vector2 lookAheadPos = positions[hi];
                Vector2 velocity = (positions[hi] - positions[hi-1]) / (times[hi] - times[hi-1]);

                return (lookAheadPos, velocity);
            }
        }
        public static float CalculateAcceleration(float maxAcceleration, Vector2 currentPos, Vector2 lookAtPos, float currentVelocity, float lookAtVelocity)
        {
            float d = (lookAtPos - currentPos).magnitude;
            float v = currentVelocity;
            float w = lookAtVelocity;
            float accel = 0.5f*(w*w - v*v) / d;
            return Mathf.Clamp(accel / maxAcceleration, -1, 1);
        }

        public static float CalculateAccelerationUnclamped(Vector2 currentPos, Vector2 lookAtPos, float currentVelocity, float lookAtVelocity)
        {
            float d = (lookAtPos - currentPos).magnitude;
            float v = currentVelocity;
            float w = lookAtVelocity;
            float accel = 0.5f * (w * w - v * v) / d;
            return accel;
        }
    }
}
