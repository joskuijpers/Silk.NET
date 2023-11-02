using System;
using System.Runtime.CompilerServices;
using DemoUtilities;
using Silk.NET.OpenGL;

namespace DemoRenderer
{
    public class StructuredBuffer<T> : Disposable where T : unmanaged
    {
        /// <summary>
        /// Gets the size of individual elements within the buffer.
        /// </summary>
        public static readonly int Stride = Unsafe.SizeOf<T>();

        private uint buffer;
        private readonly BufferTargetARB target;
        private readonly GL gl;
        /// <summary>
        /// Gets the capacity of the buffer.
        /// </summary>
        public int Capacity { get; private set; }
        /// <summary>
        /// Gets the debug name associated with the buffer and its views.
        /// </summary>
        public readonly string DebugName;

        private unsafe void Allocate(int capacity)
        {
            buffer = gl.GenBuffer();
            Capacity = capacity;
            gl.BindBuffer(target, buffer);
            gl.BufferData((GLEnum)target, (nuint)(Capacity * Stride), null, BufferUsageARB.DynamicDraw);
            gl.BindBuffer(target, 0);
        }

        /// <summary>
        /// Creates an immutable buffer filled with the given values.
        /// </summary>
        /// <param name="target">Target to which the buffer is bound.</param>
        /// <param name="initialCapacity">Number of elements that the buffer can hold by default.</param>
        /// <param name="debugName">Name to associate with the buffer.</param>
        public StructuredBuffer(GL gl, BufferTargetARB target, int initialCapacity, string debugName = "UNNAMED")
        {
            this.gl = gl;
            this.target = target;
            Allocate(initialCapacity);
            DebugName = debugName;
            if (gl.CanSetObjectLabel())
            {
                gl.ObjectLabel(ObjectIdentifier.Buffer, buffer, (uint)DebugName.Length, DebugName);
            }
        }

        /// <summary>
        /// Recreates the buffer without retaining old data.
        /// </summary>
        /// <param name="newCapacity">New capacity of the buffer.</param>
        public void SetCapacityWithoutCopy(int newCapacity)
        {
            DoDispose();
            Allocate(newCapacity);
        }

        /// <summary>
        /// Updates the buffer with the given values.
        /// </summary>
        /// <param name="newValues">Values to load into the buffer.</param>
        public void Update(Span<T> newValues) => Update(newValues, newValues.Length, 0, 0);

        /// <summary>
        /// Updates the buffer with the given values.
        /// </summary>
        /// <param name="newValues">Values to load into the buffer.</param>
        /// <param name="count">Number of elements from the source array to read.</param>
        /// <param name="sourceOffset">Offset from which to begin reading the new values.</param>
        /// <param name="destinationOffset">Index at which the stored values should start in the buffer.</param>
        public void Update(Span<T> newValues, int count, int sourceOffset = 0, int destinationOffset = 0)
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
            gl.BufferSubData(target, destinationOffset * Stride, (nuint) (count * Stride), in newValues[sourceOffset]);
            gl.BindBuffer(target, (uint)prevBound);
        }

        public void Bind(int index) => gl.BindBufferBase(target, (uint)index, buffer);
        protected override void DoDispose() => gl.DeleteBuffer(buffer);
    }
}
