namespace Simulation.Unit
{
    /// <summary>
    /// Provides extension methods for converting between <see cref="Interaction"/> enum values and their corresponding URL strings.
    /// Includes functionality to map interactions to unique URLs and vice versa for easy integration with Asset Administration Shell (AAS) resources.
    /// </summary>
    internal static class InteractionExtensions
    {
        /// <summary>
        /// A dictionary mapping URL strings to their corresponding <see cref="Interaction"/> enum values.
        /// The dictionary allows quick lookup for a given URL to determine the associated interaction type.
        /// </summary>
        private static readonly Dictionary<string, Interaction> _urlToInteraction =
            new(StringComparer.OrdinalIgnoreCase)
            {
                [Interaction.PlaceHousing.ToUrl()] = Interaction.PlaceHousing,
                [Interaction.PlaceTrimmerElement.ToUrl()] = Interaction.PlaceTrimmerElement,
                [Interaction.PlaceLever.ToUrl()] = Interaction.PlaceLever,
                [Interaction.PlaceCard.ToUrl()] = Interaction.PlaceCard,
                [Interaction.PersonalizeCard.ToUrl()] = Interaction.PersonalizeCard,
                [Interaction.RemoveAssy.ToUrl()] = Interaction.RemoveAssy,
                [Interaction.SpecialTrick.ToUrl()] = Interaction.SpecialTrick,
                [Interaction.Transport.ToUrl()] = Interaction.Transport
            };

        /// <summary>
        /// Converts the specified <see cref="Interaction"/> enum value to its corresponding URL string.
        /// Each interaction maps to a unique AAS (Asset Administration Shell) resource identifier.
        /// </summary>
        /// <param name="interaction">The interaction to convert to a URL.</param>
        /// <returns>A string containing the URL associated with the given interaction.</returns>
        /// <exception cref="ArgumentOutOfRangeException">
        /// Thrown if the interaction does not have a defined URL mapping.
        /// </exception>
        internal static string ToUrl(this Interaction interaction)
        {
            return interaction switch
            {
                Interaction.PlaceHousing => "https://aas.2propel.com/ids/sm/7445_9011_6042_2805",
                Interaction.PlaceTrimmerElement => "https://aas.2propel.com/ids/sm/1555_1111_6042_0142",
                Interaction.PlaceLever => "https://aas.2propel.com/ids/sm/6362_2111_6042_2233",
                Interaction.PlaceCard => "https://aas.2propel.com/ids/sm/3555_1111_6042_9999",
                Interaction.PersonalizeCard => "https://aas.2propel.com/ids/sm/4485_9011_6042_0610",
                Interaction.RemoveAssy => "https://aas.2propel.com/ids/sm/0065_1111_6042_4666",
                Interaction.SpecialTrick => "https://aas.2propel.com/ids/sm/5555_1111_6042_8699",
                Interaction.Transport => "https://aas.2propel.com/ids/sm/0065_1111_6042_46253",
                _ => throw new ArgumentOutOfRangeException(nameof(interaction), interaction, "No URL mapping exists for the given interaction.")
            };
        }

        /// <summary>
        /// Parses a URL back into its corresponding Interaction.
        /// Throws if the URL isn’t in the known set.
        /// </summary>
        public static Interaction FromUrl(this string url)
        {
            if (_urlToInteraction.TryGetValue(url, out var interaction))
                return interaction;

            throw new ArgumentException($"No Interaction mapping exists for URL '{url}'", nameof(url));
        }
    }
}
