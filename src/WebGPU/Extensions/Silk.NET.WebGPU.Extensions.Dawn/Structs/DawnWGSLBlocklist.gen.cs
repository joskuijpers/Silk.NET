// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.


using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;
using System.Text;
using Silk.NET.Core;
using Silk.NET.Core.Native;
using Silk.NET.Core.Attributes;
using Silk.NET.Core.Contexts;
using Silk.NET.Core.Loader;

#pragma warning disable 1591

namespace Silk.NET.WebGPU.Extensions.Dawn
{
    [NativeName("Name", "WGPUDawnWGSLBlocklist")]
    public unsafe partial struct DawnWGSLBlocklist
    {
        public DawnWGSLBlocklist
        (
            ChainedStruct? chain = null,
            nuint? blocklistedFeatureCount = null,
            byte** blocklistedFeatures = null
        ) : this()
        {
            if (chain is not null)
            {
                Chain = chain.Value;
            }

            if (blocklistedFeatureCount is not null)
            {
                BlocklistedFeatureCount = blocklistedFeatureCount.Value;
            }

            if (blocklistedFeatures is not null)
            {
                BlocklistedFeatures = blocklistedFeatures;
            }
        }


        [NativeName("Type", "WGPUChainedStruct")]
        [NativeName("Type.Name", "WGPUChainedStruct")]
        [NativeName("Name", "chain")]
        public ChainedStruct Chain;

        [NativeName("Type", "size_t")]
        [NativeName("Type.Name", "size_t")]
        [NativeName("Name", "blocklistedFeatureCount")]
        public nuint BlocklistedFeatureCount;

        [NativeName("Type", "const char *const *")]
        [NativeName("Type.Name", "const char *const *")]
        [NativeName("Name", "blocklistedFeatures")]
        public byte** BlocklistedFeatures;
    }
}
