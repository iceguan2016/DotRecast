using DotRecast.Core.Numerics;

namespace DotRecast.Recast.Demo.Messages;

public class SelectionEvent : IRecastDemoMessage
{
    public RcVec3f RayStart { get; init; }
    public RcVec3f RayEnd { get; init; }

    public bool IsFinished { get; set; }
}