namespace Simulation.UnitTransport.Pathfinding
{
    /// <summary>
    /// A fixed-capacity binary heap priority queue used for A* pathfinding.
    /// Stores vertices ordered by their heuristic cost to the goal.
    /// <list type="bullet">
    ///   <item><description><b>_maxCapacity:</b> The maximum capacity of the priority queue (set to 250).</description></item>
    ///   <item><description><b>_array:</b> The array used to store the vertices in the priority queue.</description></item>
    ///   <item><description><b>_goal:</b> The goal position used for heuristic comparisons.</description></item>
    ///   <item><description><b>_length:</b> The current number of elements in the queue.</description></item>
    /// </list>
    /// </summary>
    internal class PriorityQueue
    {
        /// <summary>
        /// The maximum capacity of the priority queue.
        /// </summary>
        internal const int _maxCapacity = 250;
        private readonly Vertex[] _array;

        /// <summary>
        /// The goal position used for heuristic comparisons.
        /// </summary>
        private (int X, int Y) _goal;

        /// <summary>
        /// The current number of elements in the queue.
        /// </summary>
        private int _length;

        /// <summary>
        /// Initializes a new instance of the <see cref="PriorityQueue"/> class.
        /// </summary>
        /// <param name="goal">The target goal cell for heuristic comparison.</param>
        internal PriorityQueue((int x, int y) goal)
        {
            _goal = goal;
            _length = 0;

            _array = new Vertex[_maxCapacity + 1];
        }

        /// <summary>
        /// Inserts a vertex into the priority queue.
        /// Maintains heap order based on heuristic cost.
        /// </summary>
        /// <param name="vertex">The vertex to insert.</param>
        internal void Push(Vertex vertex)
        {
            if (_length >= _maxCapacity)
                return;

            var hole = ++_length;
            _array[hole] = vertex;

            while (hole > 1 && _array[hole / 2].Heuristic(_goal) > vertex.Heuristic(_goal))
            {
                _array[hole] = _array[hole / 2];
                hole /= 2;
            }
            _array[hole] = vertex;
        }

        /// <summary>
        /// Removes and returns the vertex with the lowest heuristic cost.
        /// </summary>
        /// <returns>The vertex with the minimum cost, or Vertex.Null if empty.</returns>
        internal Vertex Pop()
        {
            if (_length == 0)
                return Vertex.Null;

            var min = _array[1];
            _array[1] = _array[_length--];
            Percolate(1);

            return min;
        }

        /// <summary>
        /// Checks whether the priority queue is empty.
        /// </summary>
        /// <returns>True if the queue is empty; otherwise, false.</returns>
        internal bool Empty()
        {
            return _length == 0;
        }

        /// <summary>
        /// Gets the current number of elements in the queue.
        /// </summary>
        /// <returns>The number of vertices in the queue.</returns>
        internal int Size()
        {
            return _length;
        }

        /// <summary>
        /// Clears the priority queue.
        /// </summary>
        internal void Clear()
        {
            _length = 0;
        }

        /// <summary>
        /// Restores the heap property after removal or replacement of the root.
        /// </summary>
        /// <param name="hole">The index at which to start percolating down.</param>
        private void Percolate(int hole)
        {
            int child;
            var tmp = _array[hole];

            while (hole * 2 <= _length)
            {
                child = hole * 2;

                if (child != _length && _array[child + 1].Heuristic(_goal) < _array[child].Heuristic(_goal))
                    child++;

                if (_array[child].Heuristic(_goal) < tmp.Heuristic(_goal))
                {
                    _array[hole] = _array[child];
                    hole = child;
                }
                else
                    break;
            }
            _array[hole] = tmp;
        }
    }
}
