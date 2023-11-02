using DemoContentLoader;
using System;
using System.Diagnostics;
using DemoUtilities;
using Silk.NET.OpenGL;

namespace DemoRenderer.UI
{
    /// <summary>
    /// Runtime type containing GPU-related information necessary to render a specific font type.
    /// </summary>
    public class RenderableImage : Disposable
    {
        public readonly uint Texture;
        public readonly Texture2DContent Content;
        private readonly bool srgb;
        public readonly string DebugName;
        private readonly GL gl;

        public RenderableImage(GL gl, int width, int height, bool srgb = false, string debugName = null)
        {
            Texture = gl.GenTexture();
            Content = new Texture2DContent(width, height, 1, 4);
            this.srgb = srgb;
            this.gl = gl;
            DebugName = debugName;
            if (gl.CanSetObjectLabel())
            {
                gl.ObjectLabel(ObjectIdentifier.Texture, Texture, (uint)DebugName.Length, DebugName);
            }
        }

        public RenderableImage(GL gl, Texture2DContent imageContent, bool srgb = false, string debugName = null)
        {
            Texture = gl.GenTexture();
            if (imageContent.TexelSizeInBytes != 4)
            {
                throw new ArgumentException("The renderable image assumes an R8G8B8A8_UNorm or R8G8B8A8_UNorm_SRgb texture.");
            }
            Debug.Assert(imageContent.MipLevels == 1, "We ignore any mip levels stored in the content; if the content pipeline output them, something's likely mismatched.");
            Content = imageContent;
            this.srgb = srgb;
            this.gl = gl;
            DebugName = debugName;
            UploadContentToTexture();
        }

        /// <summary>
        /// Uploads the mip0 stored in the Content to the Texture2D and generates new mips.
        /// </summary>
        public unsafe void UploadContentToTexture()
        {
            gl.BindTexture(TextureTarget.Texture2D, Texture);
            var data = Content.Pin();
            gl.TexImage2D(GLEnum.Texture2D, 0,
                srgb ? InternalFormat.Rgba8SNorm : InternalFormat.Rgba8,
                (uint)Content.Width, (uint)Content.Height, 0,
                PixelFormat.Rgba, PixelType.UnsignedByte,
                data + Content.GetMipStartIndex(0)
            );
            Content.Unpin();
            gl.GenerateMipmap(TextureTarget.Texture2D);
            gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.LinearMipmapLinear);
            gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
            gl.BindTexture(TextureTarget.Texture2D, 0);
        }

        protected override void DoDispose() => gl.DeleteTexture(Texture);
    }
}
