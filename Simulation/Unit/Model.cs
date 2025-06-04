namespace Simulation.Unit
{
    /// <summary>
    /// Represents the available model types for units in the simulation.
    /// Each model corresponds to a specific physical specification defined in <see cref="UnitTransport.MoverModel"/> or <see cref="UnitProduction.ProducerModel"/>.
    /// Used to identify and retrieve properties such as payload capacity and dimensions.
    /// </summary>
    internal enum Model
    {
        Kuka,
        Staubli,
        Viper,
        Manuel,
        APM4220,
        APM4221,
        APM4230,
        APM4330,
        APM4331,
        APM4350,
        APM4550
    }
}
