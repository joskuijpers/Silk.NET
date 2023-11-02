using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using DemoUtilities;
using Silk.NET.OpenGL;

namespace DemoRenderer
{
    public class ConstantsBuffer<T> : Disposable where T : unmanaged
    {
        private static readonly int alignedSize;

        static ConstantsBuffer()
        {
            var size = Unsafe.SizeOf<T>();
            alignedSize = (size >> 4) << 4;
            if (alignedSize < size)
                alignedSize += 16;
        }

        private readonly uint buffer;
        private readonly BufferTargetARB target;
        public readonly string DebugName;
        private readonly GL gl;

        /// <summary>
        /// Creates a constants buffer.
        /// </summary>
        /// <param name="target">Target to which the buffer is bound.</param>
        /// <param name="debugName">Name to associate with the buffer.</param>
        public unsafe ConstantsBuffer(GL gl, BufferTargetARB target, string debugName = "UNNAMED")
        {
            this.gl = gl;
            this.target = target;
            this.buffer = gl.GenBuffer();
            gl.BindBuffer(target, buffer);
            //gl.BufferStorage((GLEnum)target, (nuint)alignedSize, null, BufferStorageMask.DynamicStorageBit);
            gl.BufferData((GLEnum)target, (nuint)alignedSize, null, BufferUsageARB.DynamicDraw);
            gl.BindBuffer(target, 0);
            DebugName = debugName;
            if (gl.CanSetObjectLabel())
            {
                gl.ObjectLabel(ObjectIdentifier.Buffer, buffer, (uint)DebugName.Length, DebugName);
            }
        }
        /// <summary>
        /// Updates the buffer with the given data.
        /// </summary>
        /// <param name="bufferData">Data to load into the buffer.</param>
        public void Update(ref T bufferData)
        {
            var prevBound = gl.GetInteger
            (
                target switch
                {
                    BufferTargetARB.ArrayBuffer => GetPName.ArrayBufferBinding,
                    //BufferTargetARB.ParameterBuffer => GetPName.ParameterBufferBinding,
                    BufferTargetARB.ElementArrayBuffer => GetPName.ElementArrayBufferBinding,
                    BufferTargetARB.PixelPackBuffer => GetPName.PixelPackBufferBinding,
                    BufferTargetARB.PixelUnpackBuffer => GetPName.PixelUnpackBufferBinding,
                    BufferTargetARB.UniformBuffer => GetPName.UniformBufferBinding,
                    //BufferTargetARB.TextureBuffer => GetPName.TextureBufferBinding,
                    BufferTargetARB.TransformFeedbackBuffer => GetPName.TransformFeedbackBufferBinding,
                    //BufferTargetARB.CopyReadBuffer => GetPName.CopyReadBufferBinding,
                    //BufferTargetARB.CopyWriteBuffer => GetPName.CopyWriteBufferBinding,
                    //BufferTargetARB.DrawIndirectBuffer => GetPName.DrawIndirectBufferBinding,
                    BufferTargetARB.ShaderStorageBuffer => GetPName.ShaderStorageBufferBinding,
                    BufferTargetARB.DispatchIndirectBuffer => GetPName.DispatchIndirectBufferBinding,
                    //BufferTargetARB.QueryBuffer => GetPName.QueryBufferBinding,
                    //BufferTargetARB.AtomicCounterBuffer => GetPName.AtomicCounterBufferBinding,
                    _ => throw new ArgumentOutOfRangeException()
                }
            );
            gl.BindBuffer(target, buffer);
            gl.BufferSubData(target, 0, (nuint)alignedSize, in bufferData);
            gl.BindBuffer(target, (uint)prevBound);
        }
                
        public void Bind(int index) => gl.BindBufferBase(target, (uint)index, buffer);
        protected override void DoDispose() => gl.DeleteBuffer(buffer);
    }
}
