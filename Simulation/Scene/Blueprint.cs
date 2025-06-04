using JsonConvert = Newtonsoft.Json.JsonConvert;
using JsonProperty = Newtonsoft.Json.JsonPropertyAttribute;
using Mover = Simulation.UnitTransport.Mover;
using Producer = Simulation.UnitProduction.Producer;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.Scene
{
    /// <summary>
    /// Represents the blueprint or configuration used to initialize the simulation environment.
    /// Contains all necessary setup data such as grid size, units, zones and groups.
    /// </summary>
    internal sealed class Blueprint
    {
        /// <summary>The name of the blueprint configuration.</summary>
        [JsonProperty("name")]
        public string Name { get; set; } = null!;

        /// <summary>
        /// The maximum number of update ticks the simulation should run before stopping.
        /// Used for benchmarking consistent tick counts across runs.
        /// Defaults to <see cref="ulong.MaxValue"/> when uncapped to prevent overflow.
        /// </summary>
        [JsonProperty("tickCap")]
        public ulong TickCap { get; private set; }

        /// <summary>The size of each grid cell in the environment.</summary>
        [JsonProperty("cellSize")]
        public Vector2 CellSize { get; set; }

        /// <summary>The maximum length of all mover unit, needed to set the correct producer processing dimensions.</summary>
        [JsonProperty("moverMaxExtent")]
        public float MoverMaxExtent { get; set; }

        /// <summary>The maximum queue of production units.</summary>
        [JsonProperty("producerMaxQueue")]
        public uint ProducerMaxQueue { get; set; }

        /// <summary>The overall dimensions of the simulation's environment.</summary>
        [JsonProperty("dimension")]
        public Vector2 Dimension { get; set; }

        /// <summary>List of individual producer's included in the environment.</summary>
        [JsonProperty("producers")]
        public List<Producer> Producers { get; set; } = [];

        /// <summary>List of each individual mover's included in the environment.</summary>
        [JsonProperty("movers")]
        public List<Mover> Movers { get; set; } = [];

        /// <summary>
        /// List of mover groups for logical grouping of movers, 
        /// allowing to fill the given group dimension with the specified movers.
        /// </summary>
        [JsonProperty("moverGroups")]
        public List<MoverGroup> MoverGroups { get; set; } = [];

        /// <summary>
        /// List of producer groups for logical grouping of producers,
        /// allowing to fill the given group dimension with the specified producers.
        /// </summary>
        [JsonProperty("producerGroups")]
        public List<ProducerGroup> ProducerGroups { get; set; } = [];

        /// <summary>List of forbidden zones, defining no-go areas in the environment.</summary>
        [JsonProperty("forbiddenZones")]
        public List<ForbiddenZone> ForbiddenZones { get; set; } = [];

        /// <summary>
        /// Initializes a new instance of the <see cref="Blueprint"/> class.
        /// Default constructor required for deserialization.
        /// </summary>
        internal Blueprint() { }

        /// <summary>
        /// Loads a <see cref="Blueprint"/> instance from a JSON file in the <c>Blueprint/</c> directory.
        /// Throws an exception if the file is not found or deserialization fails.
        /// </summary>
        /// <param name="fileName">The name of the JSON file (without extension) to load.</param>
        /// <returns>A deserialized <see cref="Blueprint"/> object representing the simulation configuration.</returns>
        /// <exception cref="FileNotFoundException">Thrown if the file does not exist at the expected path.</exception>
        /// <exception cref="InvalidDataException">Thrown if the file contents cannot be deserialized into a <see cref="Blueprint"/>.</exception>
        internal static Blueprint LoadFromFile(string fileName)
        {
            var path = $"Blueprint/{fileName}.json";

            if (!File.Exists(path))
                throw new FileNotFoundException("The specified file was not found.", path);

            var json = File.ReadAllText(path);
            var blueprint = JsonConvert.DeserializeObject<Blueprint>(json)
                            ?? throw new InvalidDataException("Failed to deserialize the Blueprint from the file.");

            return blueprint;
        }
    }
}
