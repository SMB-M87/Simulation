using Vector2 = System.Numerics.Vector2;

namespace Simulation.Scene
{
    /// <summary>
    /// Represents a navigable grid used for spatial indexing and agent tracking.
    /// Each cell can store the number of agents present and be queried for occupancy or neighborhood density.
    /// </summary>
    internal abstract class NavigableGrid
    {
        /// <summary>
        /// Gets or sets the size of each cell in the grid.
        /// </summary>
        internal Vector2 CellSize { get; set; }

        /// <summary>
        /// Gets or sets the overall dimension (width, height) of the grid.
        /// </summary>
        internal Vector2 Dimension { get; set; }

        /// <summary>
        /// Gets the dictionary mapping grid cells containing the cell weight based on the containing agents corners in the cell.
        /// </summary>
        internal Dictionary<(int X, int Y), uint> Grid { get; private set; } = [];

        /// <summary>
        /// Sets the cell counts for the grid using the provided dictionary of agent counts per cell.
        /// </summary>
        /// <param name="agentsCountCells">A dictionary of cell coordinates and agent counts.</param>
        internal void SetCellWeights(Dictionary<(int X, int Y), uint> agentsCountCells)
        {
            foreach (var key in agentsCountCells.Keys)
                Grid[key] += agentsCountCells[key];
        }

        /// <summary>
        /// Returns the cell coordinates (X, Y) in the grid that the given world-space position maps to.
        /// </summary>
        /// <param name="position">The position in world coordinates.</param>
        /// <returns>A tuple representing the cell (X, Y).</returns>
        internal (int X, int Y) GetCell(Vector2 position)
        {
            var X = Math.Max(0, (int)(position.X / CellSize.X));
            var Y = Math.Max(0, (int)(position.Y / CellSize.Y));

            return (X, Y);
        }

        /// <summary>
        /// Finds the least-crowded neighboring grid cell around a given position—optionally excluding the current cell—
        /// and returns its world‐space center. It weights each candidate by its occupancy and distance, doesn't count
        /// the weight of the requesting agent by excluding corner weights from cell weights.
        /// </summary>
        /// <param name="center">The world‐space center of the object to consider.</param>
        /// <param name="dimension">The width and height of the object in world units.</param>
        /// <param name="cellWeight">The weight value representing one unit of “crowdedness” per full cell coverage.</param>
        /// If <c>true</c>, excludes the cell containing <paramref name="center"/> from consideration.
        /// </param>
        /// <returns>
        /// The world‐space coordinates of the center of the chosen adjacent cell (in 8‐neighborhood) that minimizes
        /// (occupancy − cornerPenalty) + ManhattanDistance. Returns (-½ cell, -½ cell) if no valid cell is found.
        /// </returns>
        internal Vector2 GetLeastCrowdedNearbyPosition(
            Vector2 center,
            Vector2 dimension,
            uint cellWeight,
            uint stepsAway = 0
            )
        {
            var (X, Y) = GetCell(center);

            var min = uint.MaxValue;
            (int X, int Y) minPos = (-1, -1);

            var topLeftCorner = center;
            var topRightCorner = topLeftCorner;
            topRightCorner.X += dimension.X;

            var bottomRightCorner = topLeftCorner + dimension;
            var bottomLeftCorner = bottomRightCorner;
            bottomLeftCorner.X -= dimension.X;

            var cornerCells = new[]
            {
                GetCell(topLeftCorner),
                GetCell(topRightCorner),
                GetCell(bottomRightCorner),
                GetCell(bottomLeftCorner)
            };

            var increment = 1;

            while (minPos.X < 0)
            {
                for (var x = X - increment; x <= X + increment; x++)
                    for (var y = Y - increment; y <= Y + increment; y++)
                    {
                        if (X == x && Y == y || Environment.Instance.Producers.IsProcessingPosition((x, y)))
                            continue;

                        if (Grid.TryGetValue((x, y), out var weight))
                        {
                            var posWeight = X != x ? 1U : 0U;
                            posWeight += Y != y ? 1U : 0U;
                            posWeight += X != x && Y != y ? 2U : 0U;
                            uint delta = 1;

                            foreach (var cell in cornerCells)
                            {
                                if (cell == (x, y))
                                    delta += cellWeight / 4;
                            }

                            var sum = (weight > delta ? weight - delta : 0U) + 1U * posWeight;

                            for (var dx = x - 1; dx <= x + 1; dx++)
                                for (var dy = y - 1; dy <= y + 1; dy++)
                                {
                                    if (dx == x && dy == y || Environment.Instance.Producers.IsProcessingPosition((dx, dy)))
                                        continue;

                                    sum += 1U;
                                }

                            if (sum < min && (Math.Abs(X - x) >= stepsAway && (Math.Abs(Y - y) >= stepsAway)))
                            {
                                min = sum;
                                minPos = (x, y);
                            }
                        }
                    }
                increment++;
            }

            return new Vector2(minPos.X * CellSize.X + CellSize.X / 2, minPos.Y * CellSize.Y + CellSize.Y / 2);
        }

        /// <summary>
        /// Finds the least-crowded neighboring grid cell around a given position—optionally excluding the current cell—
        /// and returns its world‐space center. It weights each candidate by its occupancy and distance, doesn't count
        /// the weight of the requesting agent by excluding corner weights from cell weights.
        /// </summary>
        /// <param name="destination">The world‐space center to consider.</param>
        /// <param name="position">The mover position to consider.</param>
        /// <param name="dimension">The width and height of the object in world units.</param>
        /// <param name="cellWeight">The weight value representing one unit of “crowdedness” per full cell coverage.</param>
        /// If <c>true</c>, excludes the cell containing <paramref name="destination"/> from consideration.
        /// </param>
        /// <returns>
        /// The world‐space coordinates of the center of the chosen adjacent cell (in 8‐neighborhood) that minimizes
        /// (occupancy − cornerPenalty) + ManhattanDistance. Returns (-½ cell, -½ cell) if no valid cell is found.
        /// </returns>
        internal Vector2 GetLeastCrowdedNearbyPosition(
            Vector2 destination,
            Vector2 position,
            Vector2 dimension,
            uint cellWeight
            )
        {
            var (X, Y) = GetCell(destination);

            var min = uint.MaxValue;
            (int X, int Y) minPos = (-1, -1);

            var topLeftCorner = position;
            var topRightCorner = topLeftCorner;
            topRightCorner.X += dimension.X;

            var bottomRightCorner = topLeftCorner + dimension;
            var bottomLeftCorner = bottomRightCorner;
            bottomLeftCorner.X -= dimension.X;

            var cornerCells = new[]
            {
                GetCell(topLeftCorner),
                GetCell(topRightCorner),
                GetCell(bottomRightCorner),
                GetCell(bottomLeftCorner)
            };

            var increment = 1;

            while (minPos.X < 0)
            {
                for (var x = X - increment; x <= X + increment; x++)
                    for (var y = Y - increment; y <= Y + increment; y++)
                    {
                        if (X == x && Y == y || Environment.Instance.Producers.IsProcessingPosition((x, y)))
                            continue;

                        if (Grid.TryGetValue((x, y), out var weight))
                        {
                            var posWeight = X != x ? 1U : 0U;
                            posWeight += Y != y ? 1U : 0U;
                            posWeight += X != x && Y != y ? 1U : 0U;
                            uint delta = 0;

                            foreach (var cell in cornerCells)
                            {
                                if (cell == (x, y))
                                    delta += cellWeight / 4;
                            }

                            var sum = (weight > delta ? weight - delta : 0U) + 1U * posWeight;

                            for (var dx = x - 1; dx <= x + 1; dx++)
                                for (var dy = y - 1; dy <= y + 1; dy++)
                                {
                                    if (dx == x && dy == y || Environment.Instance.Producers.IsProcessingPosition((dx, dy)))
                                        continue;

                                    if (Grid.TryGetValue((dx, dy), out var _))
                                        sum += 1U;
                                }

                            if (sum < min)
                            {
                                min = sum;
                                minPos = (x, y);
                            }
                        }
                    }
                increment++;
            }
            return new Vector2(minPos.X * CellSize.X + CellSize.X / 2, minPos.Y * CellSize.Y + CellSize.Y / 2);
        }

        /// <summary>
        /// Initializes the navigable grid cells by skipping any cells that are marked as forbidden.
        /// </summary>
        /// <param name="forbiddenCells">A set of grid coordinates that should be excluded from initialization.</param>
        internal void GenerateGrid(HashSet<(int, int)> forbiddenCells)
        {
            var width = (int)Math.Floor(Dimension.X / CellSize.X);
            var height = (int)Math.Floor(Dimension.Y / CellSize.Y);

            for (var x = 0; x < width; x++)
                for (var y = 0; y < height; y++)
                    if (!forbiddenCells.Contains((x, y)))
                        Grid[new(x, y)] = 0;
        }
    }
}
