using DemoContentLoader;
using Silk.NET.OpenGL;

namespace DemoRenderer.PostProcessing
{
    /// <summary>
    /// Applies a gamma curve, anti-banding dithering, and outputs the result into the lower precision target.
    /// </summary>
    public class CompressToSwap(GL gl, ContentArchive content, float gamma = 2.2f) : Shader
    (
        gl,
        content.Load<GLSLContent>(@"PostProcessing\CompressToSwap.glvs").Source,
        content.Load<GLSLContent>(@"PostProcessing\CompressToSwap.glfs").Source
    )
    {
        /// <summary>
        /// Gets or sets the display gamma. This isn't SRGB, but it'll do.
        /// </summary>
        public float Gamma = gamma;

        private readonly ConstantsBuffer<float> constants = new
            (gl, BufferTargetARB.UniformBuffer, debugName: "CompressToSwap Constants"); //alas, lack of root constants

        public void Render(uint source)
        {
            Use();
            constants.Bind(0);
            gl.BindTexture(TextureTarget.Texture2D, source);
            float inverseGamma = 1f / Gamma;
            constants.Update(ref inverseGamma);
            gl.DrawArrays(PrimitiveType.Triangles, 0, 3);
        }

        protected override void DoDispose()
        {
            constants.Dispose();
            base.DoDispose();
        }
    }
}
