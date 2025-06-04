using System.Reflection;
using System.Text.Json.Serialization;
using DynamicallyAccessedMembers = System.Diagnostics.CodeAnalysis.DynamicallyAccessedMembersAttribute;
using DynamicallyAccessedMemberTypes = System.Diagnostics.CodeAnalysis.DynamicallyAccessedMemberTypes;
using EnumMemberAttribute = System.Runtime.Serialization.EnumMemberAttribute;
using JsonNamingPolicy = System.Text.Json.JsonNamingPolicy;
using JsonSerializerOptions = System.Text.Json.JsonSerializerOptions;
using RequiresDynamicCode = System.Diagnostics.CodeAnalysis.RequiresDynamicCodeAttribute;
using RequiresUnreferencedCode = System.Diagnostics.CodeAnalysis.RequiresUnreferencedCodeAttribute;

namespace Simulation.MQTT.Messages
{
    /// <summary>
    /// Provides an AOT-compatible extension for <see cref="JsonStringEnumConverter"/> that adds support for <see cref="EnumMemberAttribute"/>.
    /// https://gist.github.com/eiriktsarpalis/2c11d8dde598eab1b54281bc67a3df41
    /// </summary>
    /// <typeparam name="TEnum">The type of the <see cref="TEnum"/>.</typeparam>
    public sealed class EnumMemberJsonStringEnumConverter<[DynamicallyAccessedMembers(DynamicallyAccessedMemberTypes.PublicFields)] TEnum>
        : JsonStringEnumConverter<TEnum> where TEnum : struct, Enum
    {
        public EnumMemberJsonStringEnumConverter() : base(namingPolicy: ResolveNamingPolicy())
        {
        }

        private static JsonNamingPolicy? ResolveNamingPolicy()
        {
            var map = typeof(TEnum).GetFields(BindingFlags.Public | BindingFlags.Static)
                .Select(f => (f.Name, AttributeName: f.GetCustomAttribute<EnumMemberAttribute>()?.Value))
                .Where(pair => pair.AttributeName != null)
                .ToDictionary();

            return map.Count > 0 ? new EnumMemberNamingPolicy(map!) : null;
        }

        private sealed class EnumMemberNamingPolicy(Dictionary<string, string> map) : JsonNamingPolicy
        {
            public override string ConvertName(string name)
                => map.TryGetValue(name, out var newName) ? newName : name;
        }
    }

    /// <summary>
    /// Provides a non-generic variant of <see cref="EnumMemberJsonStringEnumConverter"/> that is not compatible with Native AOT.
    /// </summary>
    /// <typeparam name="TEnum">The type of the <see cref="TEnum"/>.</typeparam>
    [RequiresUnreferencedCode("EnumMemberAttribute annotations might get trimmed.")]
    [RequiresDynamicCode("Requires dynamic code generation.")]
    public sealed class EnumMemberJsonStringEnumConverter : JsonConverterFactory
    {
        public override bool CanConvert(Type typeToConvert) => typeToConvert.IsEnum;

        public override JsonConverter? CreateConverter(Type typeToConvert, JsonSerializerOptions options)
        {
            var typedFactory = typeof(EnumMemberJsonStringEnumConverter<>).MakeGenericType(typeToConvert);
            var innerFactory = (JsonConverterFactory)Activator.CreateInstance(typedFactory)!;
            return innerFactory.CreateConverter(typeToConvert, options);
        }
    }
}
