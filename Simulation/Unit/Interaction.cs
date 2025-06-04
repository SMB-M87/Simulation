namespace Simulation.Unit
{
    /// <summary>
    /// Defines the types of interactions that can occur between a mover and a producer
    /// during the simulation. These interactions represent the specific operations
    /// or placements that a producer can perform as part of the production process.
    /// </summary>
    internal enum Interaction
    {
        PlaceHousing,
        PlaceTrimmerElement,
        PlaceLever,
        PlaceCard,
        PersonalizeCard,
        RemoveAssy,
        SpecialTrick,
        Transport
    }
}
