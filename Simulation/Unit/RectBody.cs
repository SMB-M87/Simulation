using Vector2 = System.Numerics.Vector2;

namespace Simulation.Unit
{
    /// <summary>
    /// Represents a rectangular body with support for rotation, 
    /// collision detection and spatial transformations.
    /// This class provides methods for detecting collisions between rotated rectangles
    /// using the Separating Axis Theorem (SAT) and updates internal properties
    /// like center position and rotation angle.
    /// </summary>
    internal class RectBody(Vector2 position, Vector2 dimension)
    {
        /// <summary>
        /// Gets or sets the top-left position of the rectangle in world space.
        /// Updates the center whenever set.
        /// </summary>
        internal Vector2 Position
        {
            get => _position;
            set
            {
                _position = value;
                UpdateCenter();
            }
        }
        private Vector2 _position = position;

        /// <summary>
        /// Gets or sets the dimensions (width, height) of the rectangle.
        /// Updates the center whenever set.
        /// </summary>
        internal Vector2 Dimension
        {
            get => _dimension;
            set
            {
                _dimension = value;
                UpdateCenter();
            }
        }
        private Vector2 _dimension = dimension;

        /// <summary>
        /// Gets the center point of the rectangle.
        /// Automatically updated when position or dimension changes.
        /// </summary>
        internal Vector2 Center { get; private set; } = position + dimension / 2;

        /// <summary>
        /// Gets the outer bounding radius of the rectangle.
        /// Used for coarse collision checks.
        /// </summary>
        internal float Radius { get; } = MathF.Sqrt(dimension.X * dimension.X + dimension.Y * dimension.Y) / 2;

        /// <summary>
        /// Gets or sets the rotation angle in degrees applied when calculating corner positions.
        /// </summary>
        internal float RotationAngle { get; set; } = 0.0f;

        internal static Vector2 GetTopLeftPosition(Vector2 center, Vector2 dimension)
        {
            return center - dimension / 2;
        }

        /// <summary>
        /// Checks for collision between two axis-aligned rectangles.
        /// This method does not take rotation into account.
        /// </summary>
        /// <param name="other">The other rectangular body to check.</param>
        /// <returns>True if the rectangles overlap; otherwise, false.</returns>
        internal bool IsAABBColliding(RectBody other)
        {
            return Position.X < other.Position.X + other.Dimension.X &&
                   Position.X + Dimension.X > other.Position.X &&
                   Position.Y < other.Position.Y + other.Dimension.Y &&
                   Position.Y + Dimension.Y > other.Position.Y;
        }

        /// <summary>
        /// Checks for collision between two axis-aligned rectangles at future postions.
        /// This method does not take rotation into account.
        /// </summary>
        /// <param name="other">The other rectangular body to check.</param>
        /// <returns>True if the rectangles overlap; otherwise, false.</returns>
        internal static bool IsAABBColliding(
            Vector2 position,
            Vector2 dimension,
            Vector2 otherPosition,
            Vector2 otherDimension)
        {
            return position.X < otherPosition.X + otherDimension.X &&
                   position.X + dimension.X > otherPosition.X &&
                   position.Y < otherPosition.Y + otherDimension.Y &&
                   position.Y + dimension.Y > otherPosition.Y;
        }

        /// <summary>
        /// Checks if this body is colliding with another body using SAT.
        /// </summary>
        /// <param name="other">The other rectangular body.</param>
        /// <returns>True if colliding; otherwise, false.</returns>
        internal bool IsSATColliding(RectBody other)
        {
            var cornersA = GetCorners(Position);
            var cornersB = other.GetCorners(other.Position);

            Vector2[] axes =
            [
                GetEdgeNormal(cornersA[0], cornersA[1]),
                GetEdgeNormal(cornersA[1], cornersA[2]),
                GetEdgeNormal(cornersB[0], cornersB[1]),
                GetEdgeNormal(cornersB[1], cornersB[2])
            ];

            foreach (var axis in axes)
                if (!OverlapsOnAxis(cornersA, cornersB, axis))
                    return false;

            return true;
        }

        /// <summary>
        /// Checks if this body would collide with another body if it were placed at the given position.
        /// </summary>
        /// <param name="other">The other body to check against.</param>
        /// <param name="position">The hypothetical position of this body.</param>
        /// <returns>True if colliding at that position; otherwise, false.</returns>
        internal bool IsSATColliding(RectBody other, Vector2 position)
        {
            var cornersA = GetCorners(position);
            var cornersB = other.GetCorners(other.Position);

            Vector2[] axes =
            [
                GetEdgeNormal(cornersA[0], cornersA[1]),
                GetEdgeNormal(cornersA[1], cornersA[2]),
                GetEdgeNormal(cornersB[0], cornersB[1]),
                GetEdgeNormal(cornersB[1], cornersB[2])
            ];

            foreach (var axis in axes)
                if (!OverlapsOnAxis(cornersA, cornersB, axis))
                    return false;

            return true;
        }

        /// <summary>
        /// Calculates the rotated corner points of this rectangle based on the given top-left position.
        /// </summary>
        /// <param name="position">The top-left position of the rectangle.</param>
        /// <returns>An array of 4 world-space corner coordinates.</returns>
        private Vector2[] GetCorners(Vector2 position)
        {
            var center = position + _dimension / 2;
            var angle = RotationAngle * (MathF.PI / 180f);

            Vector2[] localCorners =
            [
                new(-_dimension.X / 2, -_dimension.Y / 2),
                new(_dimension.X / 2, -_dimension.Y / 2),
                new(_dimension.X / 2, _dimension.Y / 2),
                new(-_dimension.X / 2, _dimension.Y / 2)
            ];

            var worldCorners = new Vector2[4];
            for (var i = 0; i < 4; i++)
            {
                var x =
                    localCorners[i].X *
                    MathF.Cos(angle)
                    -
                    localCorners[i].Y *
                    MathF.Sin(angle)
                    ;

                var y =
                    localCorners[i].X *
                    MathF.Sin(angle)
                    +
                    localCorners[i].Y *
                    MathF.Cos(angle)
                    ;

                worldCorners[i] = center + new Vector2(x, y);
            }

            return worldCorners;
        }

        /// <summary>
        /// Returns a normalized vector perpendicular to the edge between two points.
        /// </summary>
        /// <param name="point1">Start of the edge.</param>
        /// <param name="point2">End of the edge.</param>
        /// <returns>Normalized perpendicular vector (the normal).</returns>
        private static Vector2 GetEdgeNormal(Vector2 point1, Vector2 point2)
        {
            var edge = point2 - point1;

            if (edge.LengthSquared() < 0.0001f)
                return Vector2.Zero;

            return Vector2.Normalize(new Vector2(-edge.Y, edge.X));
        }

        /// <summary>
        /// Projects all corners onto a given axis and returns the minimum and maximum values.
        /// </summary>
        /// <param name="corners">The corners to project.</param>
        /// <param name="axis">The axis onto which to project.</param>
        /// <returns>The min and max scalar projections.</returns>
        private static (float min, float max) ProjectOntoAxis(Vector2[] corners, Vector2 axis)
        {
            var min = Vector2.Dot(corners[0], axis);
            var max = min;

            for (var i = 1; i < corners.Length; i++)
            {
                var projection = Vector2.Dot(corners[i], axis);

                if (projection < min)
                    min = projection;

                if (projection > max)
                    max = projection;
            }

            return (min, max);
        }

        /// <summary>
        /// Determines whether projections of two rectangles overlap on the given axis.
        /// Used in SAT to detect separating axes.
        /// </summary>
        /// <param name="cornersA">Corners of rectangle A.</param>
        /// <param name="cornersB">Corners of rectangle B.</param>
        /// <param name="axis">Axis to test overlap on.</param>
        /// <returns>True if overlapping; otherwise, false.</returns>
        private static bool OverlapsOnAxis(Vector2[] cornersA, Vector2[] cornersB, Vector2 axis)
        {
            (var minA, var maxA) = ProjectOntoAxis(cornersA, axis);
            (var minB, var maxB) = ProjectOntoAxis(cornersB, axis);

            const float margin = 0.1f;

            var overlap =
                maxA > minB +
                margin
                &&
                maxB > minA +
                margin
                ;

            return overlap;
        }

        /// <summary>
        /// Checks whether a line segment between p1 and p2 intersects with an axis-aligned rectangle.
        /// </summary>
        /// <param name="p1">First endpoint of the line segment.</param>
        /// <param name="p2">Second endpoint of the line segment.</param>
        /// <param name="rectPosition">Top-left corner of the rectangle.</param>
        /// <param name="rectDimension">Dimensions (width, height) of the rectangle.</param>
        /// <returns>True if the line segment intersects the rectangle; otherwise, false.</returns>
        internal static bool IsLineIntersectingRectangle(Vector2 p1, Vector2 p2, Vector2 rectPosition, Vector2 rectDimension)
        {
            if (IsPointInsideRect(p1, rectPosition, rectDimension) ||
                IsPointInsideRect(p2, rectPosition, rectDimension))
            {
                return true;
            }

            var topLeft = rectPosition;
            var topRight = new Vector2(rectPosition.X + rectDimension.X, rectPosition.Y);
            var bottomLeft = new Vector2(rectPosition.X, rectPosition.Y + rectDimension.Y);
            var bottomRight = new Vector2(rectPosition.X + rectDimension.X, rectPosition.Y + rectDimension.Y);

            if (LinesIntersect(p1, p2, topLeft, topRight)) return true;
            if (LinesIntersect(p1, p2, topRight, bottomRight)) return true;
            if (LinesIntersect(p1, p2, bottomRight, bottomLeft)) return true;
            if (LinesIntersect(p1, p2, bottomLeft, topLeft)) return true;

            return false;
        }

        /// <summary>
        /// Checks if a given point is inside an axis-aligned rectangle.
        /// </summary>
        /// <param name="point">The point to check.</param>
        /// <param name="rectPosition">Top-left corner of the rectangle.</param>
        /// <param name="rectDimension">Width and height of the rectangle.</param>
        /// <returns>True if the point is within the rectangle, otherwise false.</returns>
        internal static bool IsPointInsideRect(Vector2 point, Vector2 rectPosition, Vector2 rectDimension)
        {
            return point.X >= rectPosition.X &&
                   point.X <= rectPosition.X + rectDimension.X &&
                   point.Y >= rectPosition.Y &&
                   point.Y <= rectPosition.Y + rectDimension.Y;
        }

        /// <summary>
        /// Checks if two line segments (A-B and C-D) intersect.
        /// </summary>
        private static bool LinesIntersect(Vector2 A, Vector2 B, Vector2 C, Vector2 D)
        {
            return (CCW(A, C, D) != CCW(B, C, D)) && (CCW(A, B, C) != CCW(A, B, D));
        }

        /// <summary>
        /// Returns true if the points A, B and C are listed in a counter-clockwise order.
        /// </summary>
        private static bool CCW(Vector2 A, Vector2 B, Vector2 C)
        {
            return (C.Y - A.Y) * (B.X - A.X) > (B.Y - A.Y) * (C.X - A.X);
        }

        /// <summary>
        /// Finds the closest point on segment AB to point P.
        /// </summary>
        internal static Vector2 ClosestPointOnSegment(Vector2 A, Vector2 B, Vector2 P)
        {
            var AB = B - A;
            var ab2 = AB.LengthSquared();

            if (ab2 == 0)
                return A;

            var t = Vector2.Dot(P - A, AB) / ab2;
            t = Math.Clamp(t, 0f, 1f);

            return A + AB * t;
        }

        /// <summary>
        /// Updates the Center property based on the current position and dimensions.
        /// </summary>
        private void UpdateCenter()
        {
            Center = _position + _dimension / 2;
        }
    }
}
