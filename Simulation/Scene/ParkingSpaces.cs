using ActorRefs = Akka.Actor.ActorRefs;
using Model = Simulation.Unit.Model;
using Mover = Simulation.UnitTransport.Mover;
using State = Simulation.Unit.State;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.Scene
{
    /// <summary>
    /// Manages and maintains a collection of parking spaces within the simulation.
    /// Parking spaces are represented as rectangular bodies (RectBody).
    /// </summary>
    internal partial class ParkingSpaces
    {
        /// <summary>
        /// The set containing all parking spaces in the simulation.
        /// </summary>
        private readonly Dictionary<Model, List<ParkingSpace>> _parkingSpaces = [];

        /// <summary>
        /// Initializes a new instance of the <see cref="ParkingSpaces"/> class.
        /// Converts the provided set of movers into parking spaces.
        /// </summary>
        /// <param name="movers">A set of movers to register as parking spaces.</param>
        internal ParkingSpaces(HashSet<Mover> movers)
        {
            AddParkings(movers);
        }

        internal bool IsParkingSpace(Vector2 position)
        {
            return _parkingSpaces.Any(kvp => kvp.Value.Any(ps => ps.Position == position));
        }

        /// <summary>
        /// Attempts to assign the specified mover to the first available parking space for its model type,
        /// prioritizing spaces by lowest ID. Returns <c>true</c> if a space was successfully assigned; otherwise, <c>false</c>.
        /// </summary>
        /// <param name="mover">The mover to assign a parking space to.</param>
        /// <returns><c>true</c> if an available space was found and assigned; otherwise, <c>false</c>.</returns>
        internal bool AssignSpace(Mover mover)
        {
            if (!_parkingSpaces.TryGetValue(mover.Model, out var spaces) || spaces.Count <= 0)
                return false;

            var candidate = spaces
                .Where(ps => ps.Mover == null || ps.Mover?.ID == mover.ID)
                .OrderBy(ps => ps.ID)
                .FirstOrDefault();

            if (candidate == null)
                return false;

            candidate.Mover = mover;
            mover.Destination = candidate.Position;
            return true;
        }

        /// <summary>
        /// Attempts to free the parking space currently occupied by the specified mover.
        /// Returns <c>true</c> if a matching reservation was found and cleared; otherwise, <c>false</c>.
        /// </summary>
        /// <param name="mover">The mover vacating its parking space.</param>
        /// <returns><c>true</c> if the mover was occupying a space and it was released; otherwise, <c>false</c>.</returns>
        internal bool LeaveSpace(Mover mover)
        {
            if (!_parkingSpaces.TryGetValue(mover.Model, out var spaces) || spaces.Count <= 0)
                return false;

            var reservation = spaces.FirstOrDefault(ps => ps.Mover?.ID == mover.ID);

            if (reservation == null)
                return false;

            reservation.Mover = null;
            return true;
        }

        /// <summary>
        /// Attempts to relocate the specified mover to a closer available parking space,
        /// if such a space exists and is more optimal than the current one.
        /// Returns <c>true</c> if the mover was reassigned to a new parking space; otherwise, <c>false</c>.
        /// </summary>
        /// <param name="mover">The mover checking for closer parking spaces.</param>
        /// <returns><c>true</c> if a closer space was found and reassignment occurred; otherwise, <c>false</c>.</returns>
        internal bool CheckNeighbor(Mover mover)
        {
            if (mover.State != State.Alive ||
                !_parkingSpaces.TryGetValue(mover.Model, out var spaces) ||
                spaces.Count <= 0)
                return false;

            var sortedSpaces = spaces.OrderBy(ps => ps.ID).ToList();

            var currentSpace = spaces.FirstOrDefault(ps => ps.Mover?.ID == mover.ID);

            if (currentSpace == null)
                return false;

            var neighboringSpacesToFill = sortedSpaces.Where(ps =>
                {
                    if (ps == null || ps.ID >= currentSpace.ID)
                        return false;

                    var parkedMover = ps.Mover;

                    if (parkedMover == null)
                        return true;

                    if (parkedMover.ID == mover.ID ||
                        parkedMover.ServiceRequester != ActorRefs.Nobody)
                        return false;

                    var selfDist = Vector2.DistanceSquared(ps.Position, mover.Center);
                    var parkedDist = Vector2.DistanceSquared(ps.Position, parkedMover.Center);
                    return selfDist < parkedDist;
                }).OrderBy(ps => (ps.Position - mover.Center).LengthSquared())
                 .ToList();

            if (neighboringSpacesToFill.Count <= 0)
                return false;

            var neighbor = neighboringSpacesToFill.First();

            var candidateData = sortedSpaces
                .Where(ps => ps.ID > neighbor.ID && ps.Mover != null)
                .Select(ps => new
                {
                    Space = ps,
                    Center = ps.Position
                })
                .ToList();

            var closeNeighboringSpaces = candidateData
                .OrderBy(x => Vector2.Distance(neighbor.Position, x.Center))
                .Select(x => x.Space)
                .ToList();

            ParkingSpace? closestNeighboringSpace = null;

            if (closeNeighboringSpaces.Count > 0)
                closestNeighboringSpace = closeNeighboringSpaces.First();

            if (closestNeighboringSpace != null && closestNeighboringSpace != currentSpace)
                return false;

            if (neighbor.Mover == null)
                currentSpace.Mover = null;
            else
            {
                currentSpace.Mover = neighbor.Mover;
                currentSpace.Mover.Destination = currentSpace.Position;
                currentSpace.Mover.Reset = true;
            }
            neighbor.Mover = mover;
            mover.Destination = neighbor.Position;
            mover.Reset = true;

            return true;
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>AddParkings</b>: Adds parking spaces for movers based on their ID.</description></item>
        /// <item><description>Splits mover ID to extract a numeric identifier.</description></item>
        /// <item><description>If successful, creates a <b>ParkingSpace</b> at the mover's current position.</description></item>
        /// <item><description>Stores parking in a model-specific list within <b>_parkingSpaces</b>.</description></item>
        /// <item><description>Sets the mover’s destination to its current center position.</description></item>
        /// </list>
        /// </summary>
        private void AddParkings(HashSet<Mover> movers)
        {
            foreach (var mover in movers)
            {
                var split = mover.ID.Split('_');

                if (split.Length > 1 && int.TryParse(split[1], out var number))
                {
                    var parking = new ParkingSpace(number, mover, mover.Center);

                    if (!_parkingSpaces.TryGetValue(mover.Model, out var parkingKeeper))
                    {
                        parkingKeeper = [];
                        _parkingSpaces[mover.Model] = parkingKeeper;
                    }
                    parkingKeeper.Add(parking);
                    mover.Destination = mover.Center;
                }
            }
        }
    }
}
