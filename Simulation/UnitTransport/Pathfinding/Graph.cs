using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.Pathfinding
{
    /// <summary>
    /// Represents a graph structure for A* pathfinding, containing vertices and connections between cells.
    /// <list type="bullet">
    ///   <item><description><b>VertexMap:</b> A dictionary mapping grid cells to corresponding vertices for pathfinding.</description></item>
    ///   <item><description><b>Graph Initialization:</b> Converts grid cells into vertices and establishes adjacency (neighbors).</description></item>
    ///   <item><description><b>ClearAll:</b> Resets all vertices in the graph to their initial state.</description></item>
    ///   <item><description><b>ClearAll (with grid):</b> Resets all vertices and updates their weights based on the provided grid.</description></item>
    ///   <item><description><b>AddVertex:</b> Adds a new vertex to the graph if it does not already exist.</description></item>
    ///   <item><description><b>GetVertex:</b> Retrieves a vertex by its grid coordinates, returning a null vertex if not found.</description></item>
    ///   <item><description><b>AStar:</b> Executes the A* pathfinding algorithm from the current cell to the destination.</description></item>
    ///   <item><description><b>CanMoveDiagonally:</b> Checks if diagonal movement is possible between two grid cells.</description></item>
    ///   <item><description><b>GetPath:</b> Reconstructs the path from the destination back to the start, returning a stack of positions forming the path.</description></item>
    ///   <item><description><b>GetPathBezier:</b> Generates a Bezier-smoothed path based on the original path with optional corner smoothing.</description></item>
    /// </list>
    /// </summary>
    internal class Graph
    {
        /// <summary>
        /// The mapping of grid cells to their corresponding vertices.
        /// </summary>
        public Dictionary<(int X, int Y), Vertex> VertexMap;

        /// <summary>
        /// Initializes the graph based on a given grid.
        /// Each grid cell is converted into a vertex and adjacency (neighbors) is established.
        /// </summary>
        /// <param name="grid">The grid cells and their corresponding weights.</param>
        internal Graph(Dictionary<(int X, int Y), uint> grid)
        {
            VertexMap = [];

            foreach (var cell in grid.Keys)
                AddVertex(cell, 0);

            (int dx, int dy)[] directions =
            [
                ( 1,  0),
                (-1,  0),
                ( 0,  1),
                ( 0, -1),
                ( 1,  1),
                ( 1, -1),
                (-1,  1),
                (-1, -1)
            ];

            foreach (var cell in grid.Keys)
            {
                var currentVertex = GetVertex(cell);

                foreach (var (dx, dy) in directions)
                {
                    var neighbor = (cell.X + dx, cell.Y + dy);

                    if (grid.ContainsKey(neighbor))
                        Vertex.AddAdjacent(currentVertex, neighbor);
                }
            }
        }

        /// <summary>
        /// Resets all vertices in the graph to their initial state.
        /// </summary>
        internal void ClearAll()
        {
            foreach (var vertex in VertexMap)
                vertex.Value.Reset();
        }

        /// <summary>
        /// Resets all vertices and updates their weights based on a provided grid.
        /// </summary>
        /// <param name="grid">The grid with cell weights.</param>
        internal void ClearAll(Dictionary<(int X, int Y), uint> grid, Dictionary<(int X, int Y), uint> agentCorners)
        {
            foreach (var kv in VertexMap)
            {
                var cell = kv.Key;
                var vertex = kv.Value;

                grid.TryGetValue(cell, out var gridWeight);
                agentCorners.TryGetValue(cell, out var cornerWeight);

                vertex.Reset(gridWeight, cornerWeight);
            }
        }

        /// <summary>
        /// Adds a vertex to the graph if it does not already exist.
        /// </summary>
        /// <param name="Cell">The grid cell coordinates.</param>
        /// <param name="count">The initial weight of the cell.</param>
        internal void AddVertex((int X, int Y) Cell, int count)
        {
            if (!VertexMap.ContainsKey(Cell))
                VertexMap[Cell] = new(Cell, count);
        }

        /// <summary>
        /// Retrieves a vertex from the graph for a given cell. Returns a null vertex if not found.
        /// </summary>
        /// <param name="Cell">The grid cell coordinates.</param>
        /// <returns>The corresponding vertex or a null vertex.</returns>
        internal Vertex GetVertex((int X, int Y) Cell)
        {
            if (VertexMap.TryGetValue(Cell, out var vertex) && vertex != null)
                return vertex;
            else
                return Vertex.Null;
        }

        /// <summary>
        /// Executes the A* pathfinding algorithm from the current cell to the destination.
        /// </summary>
        /// <param name="destination">The target cell to reach.</param>
        /// <param name="current">The starting cell.</param>
        /// <param name="output">The resulting vertex at the destination if a path is found.</param>
        /// <returns>True if a path is found, otherwise false.</returns>
        internal bool AStar((int X, int Y) destination, (int X, int Y) current, out Vertex? output)
        {
            var queue = new PriorityQueue(destination);
            var vertex = GetVertex(current);

            vertex.G = 0;
            vertex.Heuristic(destination);
            vertex.Steps = 0;

            queue.Push(vertex);

            while (!queue.Empty())
            {
                vertex = queue.Pop();

                if (vertex.Cell.Equals(destination))
                {
                    output = vertex;
                    return true;
                }

                for (var i = 0; i < vertex.StraightIndex; i++)
                {
                    var straightVertex = GetVertex(vertex.Straight[i]);
                    var newG = vertex.G + 1.0f + vertex.CellWeight;

                    if (newG < straightVertex.G && straightVertex.CellWeight > -1)
                    {
                        straightVertex.Prev = vertex.Cell;
                        straightVertex.G = newG;
                        straightVertex.Heuristic(destination);
                        straightVertex.Steps = vertex.Steps + 1;
                        queue.Push(straightVertex);
                    }
                }

                var sqrt2 = (float)Math.Sqrt(2);

                for (var i = 0; i < vertex.DiagonalIndex; i++)
                {
                    var diagCell = vertex.Diagonal[i];

                    if (!CanMoveDiagonally(vertex.Cell, diagCell))
                        continue;

                    var DiagonalVertex = GetVertex(vertex.Diagonal[i]);
                    var newG = vertex.G + sqrt2 + vertex.CellWeight;

                    if (newG < DiagonalVertex.G && DiagonalVertex.CellWeight > -1)
                    {
                        DiagonalVertex.Prev = vertex.Cell;
                        DiagonalVertex.G = newG;
                        DiagonalVertex.Heuristic(destination);
                        DiagonalVertex.Steps = vertex.Steps + 1;
                        queue.Push(DiagonalVertex);
                    }
                }
            }
            output = null;
            return false;
        }

        /// <summary>
        /// Determines if diagonal movement is possible between two grid cells.
        /// <list type="bullet">
        ///   <item><description><b>Inputs:</b> 
        ///     The current cell and the target diagonal cell to check movement validity.
        ///   </description></item>
        ///   <item><description><b>Logic:</b> 
        ///     Checks if both adjacent cardinal cells (horizontal and vertical) exist in the vertex map, which ensures the diagonal movement is valid.
        ///   </description></item>
        ///   <item><description><b>Output:</b> 
        ///     Returns true if both adjacent cardinal cells are valid (i.e., they exist in the graph), indicating that diagonal movement is possible. 
        ///     Otherwise, returns false.
        ///   </description></item>
        /// </list>
        /// </summary>
        /// <param name="current">The current grid cell position as a tuple (X, Y).</param>
        /// <param name="diagonal">The target diagonal grid cell position as a tuple (X, Y).</param>
        /// <returns>True if diagonal movement is possible, otherwise false.</returns>
        private bool CanMoveDiagonally((int X, int Y) current, (int X, int Y) diagonal)
        {
            var dx = diagonal.X - current.X;
            var dy = diagonal.Y - current.Y;

            var cardinal1 = (current.X + dx, current.Y);
            var cardinal2 = (current.X, current.Y + dy);

            return VertexMap.ContainsKey(cardinal1) && VertexMap.ContainsKey(cardinal2);
        }

        /// <summary>
        /// Reconstructs the path from the destination back to the start.
        /// </summary>
        /// <param name="output">The final destination vertex.</param>
        /// <param name="cellSize">The size of each cell in world units.</param>
        /// <param name="destination">The exact destination position.</param>
        /// <returns>A stack of positions forming the path.</returns>
        internal Stack<Vector2> GetPath(
            Vertex output,
            Vector2 cellSize,
            Vector2 destination,
            Vector2 position
            )
        {
            var vertex = output;
            var path = new Stack<Vector2>();
            var end = output.Steps;

            var X = cellSize.X / 2;
            var Y = cellSize.Y / 2;

            var index = 1;

            path.Push(destination);
            vertex = GetVertex(vertex.Prev);

            while (index++ < end && vertex.CellWeight != -1)
            {
                path.Push(
                    new(
                        vertex.Cell.X * cellSize.X + X,
                        vertex.Cell.Y * cellSize.Y + Y
                        )
                    );

                vertex = GetVertex(vertex.Prev);
            }

            if (end > 0)
                path.Push(position);

            return path;
        }

        /// <summary>
        /// Bezier-smoothed path, written in pure stack‐operations.
        /// <list type="bullet">
        ///   <item><description><b>Input:</b> Receives a raw path, destination, current position, and segments per corner for smoothing.</description></item>
        ///   <item><description><b>Logic:</b> Converts the raw path into a Bezier-smooth path using quadratic interpolation, dividing each corner into smaller segments for smoother transitions.</description></item>
        ///   <item><description><b>Output:</b> Returns a stack of Vector2 positions forming a smooth path from the current position to the destination.</description></item>
        /// </list>
        /// </summary>
        /// <param name="output">The final destination vertex.</param>
        /// <param name="cellSize">The size of each cell in world units, used to calculate positions.</param>
        /// <param name="destination">The target destination position for the path.</param>
        /// <param name="position">The current position of the agent or unit.</param>
        /// <param name="segmentsPerCorner">The number of segments to divide each corner into for smoother transitions (default is 5).</param>
        /// <returns>A stack of Vector2 positions representing the smoothed path from the current position to the destination.</returns>
        internal Stack<Vector2> GetPathBezier(
            Vertex output,
            Vector2 cellSize,
            Vector2 destination,
            Vector2 position,
            int segmentsPerCorner = 5)
        {
            var raw = GetPath(output, cellSize, destination, position);
            var result = new Stack<Vector2>();

            if (raw.Count < 3)
            {
                if (raw.Count == 2)
                {
                    var end = raw.Pop();
                    var start = raw.Pop();

                    result.Push(end);

                    for (var s = segmentsPerCorner; s >= 1; s--)
                    {
                        var t = s / (float)segmentsPerCorner;
                        result.Push(Vector2.Lerp(start, end, t));
                    }
                    result.Push(start);
                    return ReverseStack(result);
                }
                return raw;
            }

            var prev = raw.Pop();
            var curr = raw.Pop();
            result.Push(prev);

            while (raw.Count > 0)
            {
                var next = raw.Pop();

                var A = Vector2.Lerp(prev, curr, 0.5f);
                var B = curr;
                var C = Vector2.Lerp(curr, next, 0.5f);

                for (var s = 1; s <= segmentsPerCorner; s++)
                {
                    var t = s / (float)segmentsPerCorner;
                    result.Push(QuadBezier(A, B, C, t));
                }

                prev = curr;
                curr = next;
            }
            result.Push(curr);
            return ReverseStack(result);
        }

        /// <summary>
        /// Performs quadratic Bezier interpolation between three points A, B, and C.
        /// <list type="bullet">
        ///   <item><description><b>Input:</b> Three points A, B, and C, along with a parameter t for interpolation.</description></item>
        ///   <item><description><b>Logic:</b> Uses the quadratic Bezier formula to calculate the interpolated point based on t.</description></item>
        ///   <item><description><b>Output:</b> The interpolated point along the Bezier curve.</description></item>
        /// </list>
        /// </summary>
        /// <param name="A">The first point (start of the curve).</param>
        /// <param name="B">The second point (control point).</param>
        /// <param name="C">The third point (end of the curve).</param>
        /// <param name="t">The interpolation parameter, typically ranging from 0 to 1.</param>
        /// <returns>A Vector2 representing the interpolated point along the Bezier curve.</returns>
        private static Vector2 QuadBezier(Vector2 A, Vector2 B, Vector2 C, float t)
        {
            var u = 1 - t;
            return u * u * A + 2 * u * t * B + t * t * C;
        }

        /// <summary>
        /// Reverses the order of elements in a stack.
        /// <list type="bullet">
        ///   <item><description><b>Input:</b> A stack of elements to reverse.</description></item>
        ///   <item><description><b>Output:</b> A new stack with elements in reverse order.</description></item>
        /// </list>
        /// </summary>
        /// <param name="input">The stack of elements to reverse.</param>
        /// <returns>A new stack with the elements in reverse order.</returns>
        private static Stack<T> ReverseStack<T>(Stack<T> input)
        {
            var path = new Stack<T>();

            while (input.Count > 0)
                path.Push(input.Pop());

            return path;
        }
    }
}
