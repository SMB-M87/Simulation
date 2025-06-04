namespace Simulation.Util
{
    /// <summary>
    /// Defines a two-segment linear mapping between a domain [0,1] and a range [0,1].
    /// <para/>At <see cref="DomainBreak"/> the slope switches from the “first” to the “second” segment,
    /// producing a controllable “kink” in your mapping curve.
    /// <list type="bullet">
    ///   <item><description><b>DomainBreak</b>: input threshold in [0,1] where slope changes.</description></item>
    ///   <item><description><b>RangeBreak</b>: output value at DomainBreak dividing the two linear pieces.</description></item>
    /// </list>
    /// </summary>
    internal class PiecewiseLinearMap
    {
        /// <summary>
        /// Slope of the first linear segment, such that 
        /// <c>output = _firstSlope × input</c> for all <c>input ≤ DomainBreak</c>.<para/>
        /// Computed as <c>RangeBreak / DomainBreak</c>.
        /// </summary>
        private readonly float _firstSlope;

        /// <summary>
        /// Slope of the second linear segment, such that 
        /// <c>output = RangeBreak + _secondSlope × (input – DomainBreak)</c> for all <c>input > DomainBreak</c>.<para/>
        /// Computed as <c>(1 – RangeBreak) / (1 – DomainBreak)</c>.
        /// </summary>
        private readonly float _secondSlope;

        /// <summary>Indicates whether the map is in identity mode (i.e., <c>y = x</c>).<para/>
        /// This is <c>true</c> when either <see cref="DomainBreak"/> or <see cref="RangeBreak"/>
        /// is ≤ 0 or ≥ 1, disabling the two‐segment behavior.
        /// </summary>
        private readonly bool _isIdentity;

        /// <summary>
        /// The input‐space breakpoint in [0,1] where the mapping switches from the first slope to the second. <para/>
        /// All inputs ≤ this value use the first linear formula.
        /// </summary>
        internal float DomainBreak { get; }

        /// <summary>
        /// The corresponding output value in [0,1] at <see cref="DomainBreak"/>.<para/>
        /// It divides the two linear segments: inputs ≤ DomainBreak map into [0, RangeBreak], 
        /// and inputs above map into [RangeBreak, 1].
        /// </summary>
        internal float RangeBreak { get; }

        /// <summary>
        /// Creates the piecewise map.  
        /// <list type="bullet">
        ///   <item><description>
        ///       If <paramref name="domainBreak"/> or <paramref name="rangeBreak"/> are ≤0 or ≥1,  
        ///       the map is set to identity (y = x).  
        ///     </description></item>
        ///   <item><description>
        ///       Otherwise, the first slope = <c>RangeBreak/DomainBreak</c>,  
        ///       the second slope = <c>(1-RangeBreak)/(1-DomainBreak)</c>.  
        ///   </description></item>
        /// </list>
        /// </summary>
        /// <param name="domainBreak">Normalized input‐fraction at which the first piece ends (0…1).</param>
        /// <param name="rangeBreak">Normalized output‐ratio at the kink (0…1), matching <paramref name="domainBreak"/>.</param>
        internal PiecewiseLinearMap(float domainBreak, float rangeBreak)
        {
            var valid = domainBreak > 0f && domainBreak < 1f
                     && rangeBreak > 0f && rangeBreak < 1f;

            if (!valid)
            {
                _isIdentity = true;
                DomainBreak = 0f;
                RangeBreak = 1f;
                _firstSlope = 1f;
                _secondSlope = 1f;
            }
            else
            {
                _isIdentity = false;
                DomainBreak = domainBreak;
                RangeBreak = rangeBreak;
                _firstSlope = RangeBreak / DomainBreak;
                _secondSlope = (1f - RangeBreak) / (1f - DomainBreak);
            }
        }

        /// <summary>
        /// Maps a normalized interval fraction (<paramref name="X"/>) into a slider ratio in [0,1], identity if breakpoints were invalid.
        /// <list type="bullet">
        ///   <item><description>
        ///       If <c>x ≤ DomainBreak</c>, uses the first segment: <c>y = _firstSlope * x</c>.</description></item>
        ///   <item><description>Otherwise, uses the second segment: <c>y = RangeBreak + _secondSlope * (x - DomainBreak)</c>.</description></item>
        /// </list>
        /// </summary>
        /// <param name="X">Input fraction in [0,1].</param>
        /// <returns>Output ratio in [0,1].</returns>
        public float MapDomainToRange(float X)
        {
            if (_isIdentity)
                return X;

            if (X <= DomainBreak)
                return _firstSlope * X;

            return RangeBreak + _secondSlope * (X - DomainBreak);
        }

        /// <summary>
        /// Inverse of <see cref="MapDomainToRange"/>: maps a slider ratio (<paramref name="Y"/>) back to a fraction.
        /// <list type="bullet">
        ///   <item><description>If <c>y ≤ RangeBreak</c>, inverts the first segment: <c>x = y / _firstSlope</c>.</description></item>
        ///   <item><description>Otherwise, inverts the second: <c>x = DomainBreak + (y - RangeBreak) / _secondSlope</c>.</description></item>
        /// </list>
        /// </summary>
        /// <param name="Y">Input ratio in [0,1].</param>
        /// <returns>Output fraction in [0,1].</returns>
        public float MapRangeToDomain(float Y)
        {
            if (_isIdentity)
                return Y;

            if (Y <= RangeBreak)
                return Y / _firstSlope;

            return DomainBreak + (Y - RangeBreak) / _secondSlope;
        }
    }
}
