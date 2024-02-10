// Disclaimer: This code was copied from unity forums
// Source: https://discussions.unity.com/t/how-can-you-visualize-a-boxcast-boxcheck-etc/161101

using UnityEngine;
 
namespace aStar
{
    public static class ExtDebug
    {
        //Draws just the box at where it is currently hitting.
        public static void DrawBoxCastOnHit(Vector3 origin, Vector3 halfExtents, Vector3 direction, Quaternion orientation, float hitInfoDistance, Color color)
        {
            origin = CastCenterOnCollision(origin, direction, hitInfoDistance);
            DrawBox(origin, halfExtents, orientation, color);
        }
        
        //Draws the full box from start of cast to its end distance. Can also pass in hitInfoDistance instead of full distance
        public static void DrawBoxCastBox(Vector3 origin, Vector3 halfExtents, Vector3 direction, Quaternion orientation, float distance, Color color)
        {
            direction.Normalize();
            Box bottomBox = new Box(origin, halfExtents, orientation);
            Box topBox = new Box(origin + (direction * distance), halfExtents, orientation);
                
            Debug.DrawLine(bottomBox.backBottomLeft, topBox.backBottomLeft,	color, 1000f);
            Debug.DrawLine(bottomBox.backBottomRight, topBox.backBottomRight, color, 1000f);
            Debug.DrawLine(bottomBox.backTopLeft, topBox.backTopLeft, color, 1000f);
            Debug.DrawLine(bottomBox.backTopRight, topBox.backTopRight,	color, 1000f);
            Debug.DrawLine(bottomBox.frontTopLeft, topBox.frontTopLeft,	color, 1000f);
            Debug.DrawLine(bottomBox.frontTopRight, topBox.frontTopRight, color, 1000f);
            Debug.DrawLine(bottomBox.frontBottomLeft, topBox.frontBottomLeft, color, 1000f);
            Debug.DrawLine(bottomBox.frontBottomRight, topBox.frontBottomRight,	color, 1000f);
        
            DrawBox(bottomBox, color);
            DrawBox(topBox, color);
        }
        
        public static void DrawBox(Vector3 origin, Vector3 halfExtents, Quaternion orientation, Color color)
        {
            DrawBox(new Box(origin, halfExtents, orientation), color);
        }
        public static void DrawBox(Box box, Color color)
        {
            Debug.DrawLine(box.frontTopLeft,	 box.frontTopRight,	color, 1000f);
            Debug.DrawLine(box.frontTopRight,	 box.frontBottomRight, color, 1000f);
            Debug.DrawLine(box.frontBottomRight, box.frontBottomLeft, color, 1000f);
            Debug.DrawLine(box.frontBottomLeft,	 box.frontTopLeft, color, 1000f);
                                                    
            Debug.DrawLine(box.backTopLeft,		 box.backTopRight, color, 1000f);
            Debug.DrawLine(box.backTopRight,	 box.backBottomRight, color, 1000f);
            Debug.DrawLine(box.backBottomRight,	 box.backBottomLeft, color, 1000f);
            Debug.DrawLine(box.backBottomLeft,	 box.backTopLeft, color, 1000f);
                                                    
            Debug.DrawLine(box.frontTopLeft,	 box.backTopLeft, color, 1000f);
            Debug.DrawLine(box.frontTopRight,	 box.backTopRight, color, 1000f);
            Debug.DrawLine(box.frontBottomRight, box.backBottomRight, color, 1000f);
            Debug.DrawLine(box.frontBottomLeft,	 box.backBottomLeft, color, 1000f);
        }
        
        public struct Box
        {
            public Vector3 localFrontTopLeft     {get; private set;}
            public Vector3 localFrontTopRight    {get; private set;}
            public Vector3 localFrontBottomLeft  {get; private set;}
            public Vector3 localFrontBottomRight {get; private set;}
            public Vector3 localBackTopLeft      {get {return -localFrontBottomRight;}}
            public Vector3 localBackTopRight     {get {return -localFrontBottomLeft;}}
            public Vector3 localBackBottomLeft   {get {return -localFrontTopRight;}}
            public Vector3 localBackBottomRight  {get {return -localFrontTopLeft;}}

            public Vector3 frontTopLeft     {get {return localFrontTopLeft + origin;}}
            public Vector3 frontTopRight    {get {return localFrontTopRight + origin;}}
            public Vector3 frontBottomLeft  {get {return localFrontBottomLeft + origin;}}
            public Vector3 frontBottomRight {get {return localFrontBottomRight + origin;}}
            public Vector3 backTopLeft      {get {return localBackTopLeft + origin;}}
            public Vector3 backTopRight     {get {return localBackTopRight + origin;}}
            public Vector3 backBottomLeft   {get {return localBackBottomLeft + origin;}}
            public Vector3 backBottomRight  {get {return localBackBottomRight + origin;}}

            public Vector3 origin {get; private set;}

            public Box(Vector3 origin, Vector3 halfExtents, Quaternion orientation) : this(origin, halfExtents)
            {
                Rotate(orientation);
            }
            public Box(Vector3 origin, Vector3 halfExtents)
            {
                this.localFrontTopLeft     = new Vector3(-halfExtents.x, halfExtents.y, -halfExtents.z);
                this.localFrontTopRight    = new Vector3(halfExtents.x, halfExtents.y, -halfExtents.z);
                this.localFrontBottomLeft  = new Vector3(-halfExtents.x, -halfExtents.y, -halfExtents.z);
                this.localFrontBottomRight = new Vector3(halfExtents.x, -halfExtents.y, -halfExtents.z);

                this.origin = origin;
            }


            public void Rotate(Quaternion orientation)
            {
                localFrontTopLeft     = RotatePointAroundPivot(localFrontTopLeft    , Vector3.zero, orientation);
                localFrontTopRight    = RotatePointAroundPivot(localFrontTopRight   , Vector3.zero, orientation);
                localFrontBottomLeft  = RotatePointAroundPivot(localFrontBottomLeft , Vector3.zero, orientation);
                localFrontBottomRight = RotatePointAroundPivot(localFrontBottomRight, Vector3.zero, orientation);
            }
        }

        //This should work for all cast types
        static Vector3 CastCenterOnCollision(Vector3 origin, Vector3 direction, float hitInfoDistance)
        {
            return origin + (direction.normalized * hitInfoDistance);
        }
        
        static Vector3 RotatePointAroundPivot(Vector3 point, Vector3 pivot, Quaternion rotation)
        {
            Vector3 direction = point - pivot;
            return pivot + rotation * direction;
        }
    }
}
