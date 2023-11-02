// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Silk.NET.OpenGL;

namespace DemoUtilities;

public static class SilkUtils
{
    public static bool CanSetObjectLabel(this GL gl)
        => gl.GetStringS(StringName.Version) is var vString && vString.StartsWith("OpenGL ES")
            ? Version.TryParse
            (
                vString.AsSpan()[9..(vString.AsSpan()[9..].IndexOf(' ') is > 0 and var x ? x : vString.Length - 9)],
                out var esVersion
            ) && esVersion >= new Version(3, 2)
            : gl.IsExtensionPresent("GL_KHR_debug") || Version.TryParse
                (vString.AsSpan()[..vString.IndexOf(' ')], out var version) && version >= new Version(4, 3);
}
