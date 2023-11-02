using System.Numerics;
using BepuUtilities;
using DemoContentLoader;
using Silk.NET.OpenGL;

namespace DemoRenderer.Background
{
    public class BackgroundRenderer(GL gl, ContentArchive content) : Shader
    (
        gl,
        content.Load<GLSLContent>(@"Background\RenderBackground.glvs").Source,
        content.Load<GLSLContent>(@"Background\RenderBackground.glfs").Source
    )
    {
        private readonly ConstantsBuffer<Matrix> constants = new
            (gl, BufferTargetARB.UniformBuffer, debugName: "BackgroundRenderer Constants");

        public void Render(Camera camera)
        {
            Use();
            constants.Bind(0);
            var constantsData = Matrix.Invert
            (
                camera.View * Matrix.CreatePerspectiveFieldOfView
                    (camera.FieldOfView, camera.AspectRatio, camera.FarClip, camera.NearClip)
            );
            constants.Update(ref constantsData);
            gl.DrawArrays(PrimitiveType.Triangles, 0, 3);
        }

        protected override void DoDispose()
        {
            constants.Dispose();
            base.DoDispose();
        }
    }
}
