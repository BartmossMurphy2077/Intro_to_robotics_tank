// Heavily inspired and reverse engineered from https://github.com/GarrettGunnell/AcerolaFX

#include "Includes/BaropeFX_Common.fxh"
#include "Includes/BaropeFX_TempTex1.fxh"
#include "Includes/BaropeFX_TempTex2.fxh"

// ---------------------------------ReShade Settings---------------

// Preprocess settings

uniform float _Zoom <
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_min = 0.0f; ui_max = 5.0f;
    ui_label = "Zoom";
    ui_type = "drag";
    ui_tooltip = "Decrease to zoom in, increase to zoom out.";
> = 1.0f;

uniform float2 _Offset <
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_min = -1.0f; ui_max = 1.0f;
    ui_label = "Offset";
    ui_type = "drag";
    ui_tooltip = "Positional offset of the zoom from the center.";
> = 0.0f;

uniform int _KernelSize <
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_min = 1; ui_max = 10;
    ui_type = "slider";
    ui_label = "Kernel Size";
    ui_tooltip = "Size of the blur kernel.";
    ui_spacing = 4;
> = 2;

uniform float _Sigma <
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_min = 0.0f; ui_max = 5.0f;
    ui_type = "slider";
    ui_label = "Blur Strength";
    ui_tooltip = "Sigma of the gaussian function (used for Gaussian blur).";
> = 2.0f;

uniform float _SigmaScale <
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_min = 0.0f; ui_max = 5.0f;
    ui_type = "slider";
    ui_label = "Deviations Scale";
    ui_tooltip = "Scale between the two Gaussian blurs.";
> = 1.6f;

uniform float _Tau <
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_min = 0.0f; ui_max = 1.1f;
    ui_type = "slider";
    ui_label = "Detail";
> = 1.0f;

uniform float _Threshold <
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_min = 0.001; ui_max = 0.01f;
    ui_type = "slider";
    ui_label = "Threshold";
> = 0.005f;

uniform bool _UseDepth<
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_label = "Use Depth";
    ui_tooltip = "use depth info to inform edges.";
    ui_spacing = 4;
> = true;

uniform float _DepthThreshold <
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_min = 0.0f; ui_max = 5.0f;
    ui_type = "slider";
    ui_label = "Depth Threshold";
    ui_tooltip = "Adjust the Threshold for depth differences to count as an edge.";
> = 0.1f;

uniform bool _UseNormals<
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_label = "Use Normals";
    ui_tooltip = "use normal info to inform edges.";
    ui_spacing = 4;
> = true;

uniform float _NormalThreshold <
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_min = 0.0f; ui_max = 5.0f;
    ui_type = "slider";
    ui_label = "Normal Threshold";
    ui_tooltip = "Adjust the Threshold for normal differences to count as an edge.";
> = 0.1f;

uniform float _DepthCutOff <
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_min = 0.0f; ui_max = 1000.0f;
    ui_type = "slider";
    ui_label = "Depth Cutoff";
    ui_tooltip = "Adjust the distance at which edges are no longer drawn.";
> = 0.1f;

uniform int _EdgeThreshold <
    ui_category = "Preprocess Settings";
    ui_category_closed = true;
    ui_min = 0; ui_max = 64;
    ui_type = "slider";
    ui_label = "Edge Threshold";
    ui_tooltip = "how many pixels in an 8x8 grid need to be detected as an edge for an edge to be filled in";
> = 8;

// Color Settings

uniform bool _Edges<
    ui_category = "Color Settings";
    ui_category_closed = true;
    ui_label = "Draw Edges";
    ui_tooltip = "draw ASCII edges.";
> = true;

uniform bool _Fill<
    ui_category = "Color Settings";
    ui_category_closed = true;
    ui_label = "Draw Fill";
    ui_tooltip = "fill screen with ASCII characters.";
> = true;

uniform float _Exposure <
    ui_category = "Color Settings";
    ui_category_closed = true;
    ui_min = 0.0f; ui_max = 5.0f;
    ui_label = "Luminance Attenuation";
    ui_type = "slider";
    ui_tooltip = "Multiplication on the base luminance of the image to bring up ASCII characters.";
> = 1.0f;

uniform float _Attenuation <
    ui_category = "Color Settings";
    ui_category_closed = true;
    ui_min = 0.0f; ui_max = 5.0f;
    ui_label = "Luminance Attenuation";
    ui_type = "slider";
    ui_tooltip = "Exponent on the base luminance of the image to bring up ASCII characters.";
> = 1.0f;

uniform bool _InvertLuminance <
    ui_category = "Color Settings";
    ui_category_closed = true;
    ui_label = "Invert ASCII";
    ui_tooltip = "invert the ASCII luminance relationship.";
> = false;

uniform float3 _ASCIIColor <
    ui_category = "Color Settings";
    ui_category_closed = true;
    ui_label = "ASCII Color";
    ui_type = "color";
    ui_spacing = 4;
> = 0.0f;

uniform float3 _BackgroundColor <
    ui_category = "Color Settings";
    ui_category_closed = true;
    ui_label = "Background Color";
    ui_type = "color";
> = 0.0f;

uniform float _BlendWithBase <
    ui_category = "Color Settings";
    ui_category_closed = true;
    ui_min = 0.0f; ui_max = 1.0f;
    ui_label = "Base Color Blend";
    ui_type = "slider";
    ui_tooltip = "Blend ASCII characters with underlying color from original render.";
> = 0.0f;

uniform float _DepthFallOff <
    ui_category = "Color Settings";
    ui_category_closed = true;
    ui_min = 0.0f; ui_max = 1.0f;
    ui_label = "Depth Falloff";
    ui_type = "slider";
    ui_tooltip = "How quickly ASCII characters fade into the distance.";
    ui_spacing = 4;
> = 0.0f;

uniform float _DepthOffset <
    ui_category = "Color Settings";
    ui_category_closed = true;
    ui_min = 0.0f; ui_max = 1.0f;
    ui_label = "Depth Offset";
    ui_type = "slider";
    ui_tooltip = "Adjust point at which ASCII characters falloff.";
    ui_spacing = 4;
> = 0.0f;

// Debug Settings

uniform bool _viewDoG <
    ui_category = "Debug Settings";
    ui_category_closed = true;
    ui_label = "View DoG";
    ui_tooltip = "view difference of gaussians preprocess.";
> = false;

uniform bool _viewUncompressed <
    ui_category = "Debug Settings";
    ui_category_closed = true;
    ui_label = "View Uncompressed";
    ui_tooltip = "view uncompressed edge direction data.";
> = false;

uniform bool _viewEdges <
    ui_category = "Debug Settings";
    ui_category_closed = true;
    ui_label = "View Edges";
    ui_tooltip = "View edge direction data.";
> = false;

// ---------------------------------Helper Functions---------------

float gaussian(float sigma, float pos) {
    return (1.0f / sqrt(2.0f * BFX_PI * sigma * sigma)) * exp(-(pos * pos) / (2.0f * sigma * sigma));
}

float2 transformUV(float2 uv) {
    float2 zoomUV = uv * 2 - 1;
    zoomUV += float2(-_Offset.x, _Offset.y) *2;
    zoomUV *= _Zoom;
    zoomUV = zoomUV * 0.5f + 0.5f;

    return zoomUV;
}

// ---------------------------------Textures and Samplers---------------

texture2D BFX_ASCIIEdgesLUT < source = "edgesASCII.png"; > {Width = 40; Height = 8; };
sampler2D EdgesASCII { Texture = BFX_ASCIIEdgesLUT; AddressU = REPEAT; AddressV = REPEAT; };

texture2D BFX_ASCIIFillLUT < source = "fillASCII.png"; > {Width = 80; Height = 8; };
sampler2D FillASCII { Texture = BFX_ASCIIFillLUT; AddressU = REPEAT; AddressV = REPEAT; };

sampler2D Normals { Texture = BFXTemp2::BFX_RenderTex2; MagFilter = POINT; MinFilter = POINT; MipFilter = Point; };

texture2D BFX_LuminanceASCIITex { Width = BUFFER_WIDTH; Height = BUFFER_HEIGHT; Format = R16F; };
sampler2D Luminance { Texture = BFX_LuminanceASCIITex; MagFilter = POINT; MinFilter = POINT; MipFilter = Point; };

texture2D BFX_DownscaleTex { Width = BUFFER_WIDTH / 8; Height = BUFFER_HEIGHT / 8; Format = RGBA16F; };
sampler2D Downscale { Texture = BFX_DownscaleTex; MagFilter = POINT; MinFilter = POINT; MipFilter = POINT; };

texture2D BFX_ASCIIPingTex { Width = BUFFER_WIDTH; Height = BUFFER_HEIGHT; Format = RGBA16F; };
sampler2D ASCIIPing { Texture = BFX_ASCIIPingTex; MagFilter = POINT; MinFilter = POINT; MipFilter = POINT; };

texture2D BFX_ASCIIDoGTex { Width = BUFFER_WIDTH; Height = BUFFER_HEIGHT; Format = R16F; };
sampler2D DoG { Texture = BFX_ASCIIDoGTex; MagFilter = POINT; MinFilter = POINT; MipFilter = POINT; };

texture2D BFX_ASCIIEdgesTex { Width = BUFFER_WIDTH; Height = BUFFER_HEIGHT; Format = R16F; };
sampler2D Edges { Texture = BFX_ASCIIEdgesTex; MagFilter = POINT; MinFilter = POINT; MipFilter = POINT; };

texture2D BFX_ASCIISobelTex { Width = BUFFER_WIDTH; Height = BUFFER_HEIGHT; Format = RG16F; };
sampler2D Sobel { Texture = BFX_ASCIISobelTex; MagFilter = POINT; MinFilter = POINT; MipFilter = POINT; };

sampler2D ASCII { Texture = BFXTemp1::BFX_RenderTex1; MagFilter = POINT; MinFilter = POINT; MipFilter = POINT; };
storage2D s_ASCII { Texture = BFXTemp1::BFX_RenderTex1; };

// ---------------------------------Passes---------------

float4 PS_EndPass(float4 position : SV_POSITION, float2 uv : TEXCOORD) : SV_TARGET { return tex2D(ASCII, uv).rgba; }

float PS_Luminance(float4 position : SV_POSITION, float2 uv : TEXCOORD) : SV_TARGET {
    return Common::Luminance(saturate(tex2D(ReShade::BackBuffer, uv).rgb));
}

float4 PS_Downscale(float4 position : SV_POSITION, float2 uv : TEXCOORD) : SV_TARGET {
    float4 col = saturate(tex2D(ReShade::BackBuffer, uv));

    float lum = Common::Luminance(col.rgb);

    return float4(col.rgb, lum);
}

float4 PS_HorizontalBlur(float4 position : SV_POSITION, float2 uv : TEXCOORD) : SV_TARGET {
    float2 texelSize = float2(BUFFER_RCP_WIDTH, BUFFER_RCP_HEIGHT);
    
    float2 blur = 0;
    float2 kernelSum = 0;

    for (int x = -_KernelSize; x <= _KernelSize; ++x){
        float2 luminance = tex2D(Luminance, uv + float2(x, 0) * texelSize).r;
        float2 gauss = float2(gaussian(_Sigma, x), gaussian(_Sigma * _SigmaScale, x));

        blur += luminance * gauss;
        kernelSum += gauss;
    }

    blur /= kernelSum;

    return float4(blur, 0, 0);
}

float4 PS_VerticalBlurAndDifference(float4 position : SV_POSITION, float2 uv : TEXCOORD) : SV_TARGET {
    float2 texelSize = float2(BUFFER_RCP_WIDTH, BUFFER_RCP_HEIGHT);
    
    float2 blur = 0;
    float2 kernelSum = 0;

    for (int y = -_KernelSize; y <= _KernelSize; ++y){
        float2 luminance = tex2D(ASCIIPing, uv + float2(0, y) * texelSize).rg;
        float2 gauss = float2(gaussian(_Sigma, y), gaussian(_Sigma * _SigmaScale, y));

        blur += luminance * gauss;
        kernelSum += gauss;
    }

    blur /= kernelSum;

    float Difference = (blur.x - _Tau * blur.y);

    Difference = (Difference >= _Threshold) ? 1 : 0;

    return Difference;
}

float4 PS_CalculateNormals(float4 position : SV_POSITION, float2 uv : TEXCOORD) : SV_TARGET {
    float3 texelSize = float3(BUFFER_RCP_WIDTH, BUFFER_RCP_HEIGHT, 0.0);
    float2 posCenter = uv;
    float2 posNorth = posCenter - texelSize.zy;
    float2 posEast = posCenter + texelSize.xz;

    float centerDepth = ReShade::GetLinearizedDepth(transformUV(posCenter));

    float3 vertCenter = float3(posCenter - 0.5, 1) * centerDepth;
    float3 vertNorth = float3(posNorth - 0.5, 1) * ReShade::GetLinearizedDepth(transformUV(posNorth));
    float3 vertEast = float3(posEast - 0.5, 1) * ReShade::GetLinearizedDepth(transformUV(posEast));

    return float4(normalize(cross(vertCenter - vertNorth, vertCenter - vertEast)), centerDepth);
}

float4 PS_EdgeDetect(float4 position : SV_POSITION, float2 uv : TEXCOORD) : SV_TARGET {
    float2 texelSize = float2(BUFFER_RCP_WIDTH, BUFFER_RCP_HEIGHT);

    float4 c = tex2D(Normals, uv + float2(0, 0) * texelSize);
    
    float4 w = tex2D(Normals, uv + float2(-1, 0) * texelSize);
    float4 e = tex2D(Normals, uv + float2(1, 0) * texelSize);
    float4 n = tex2D(Normals, uv + float2(0, -1) * texelSize);
    float4 s = tex2D(Normals, uv + float2(0, 1) * texelSize);

    float4 nw = tex2D(Normals, uv + float2(-1, -1) * texelSize);
    float4 sw = tex2D(Normals, uv + float2(-1, 1) * texelSize);
    float4 ne = tex2D(Normals, uv + float2(1, -1) * texelSize);
    float4 se = tex2D(Normals, uv + float2(1, 1) * texelSize);

    float output = 0.0f;
    
    float depthSum = 0.0f;
    depthSum += abs(w.w - c.w);
    depthSum += abs(e.w - c.w);
    depthSum += abs(n.w - c.w);
    depthSum += abs(s.w - c.w);
    depthSum += abs(nw.w - c.w);
    depthSum += abs(sw.w - c.w);
    depthSum += abs(ne.w - c.w);
    depthSum += abs(se.w - c.w);

    if(_UseDepth && depthSum > _DepthThreshold)
        output = 1.0f;

    float3 normalSum = 0.0f;
    normalSum += abs(w.rgb - c.rgb);
    normalSum += abs(e.rgb - c.rgb);
    normalSum += abs(n.rgb - c.rgb);
    normalSum += abs(s.rgb - c.rgb);
    normalSum += abs(nw.rgb - c.rgb);
    normalSum += abs(sw.rgb - c.rgb);
    normalSum += abs(ne.rgb - c.rgb);
    normalSum += abs(se.rgb - c.rgb);

    if(_UseNormals && dot(normalSum, 1) > _NormalThreshold)
        output = 1.0f;
    
    float D = tex2D(DoG, uv).r;

    return saturate(abs(D - output));
}


float4 PS_HorizontalSobel(float4 position : SV_POSITION, float2 uv : TEXCOORD) : SV_TARGET {
    float2 texelSize = float2(BUFFER_RCP_WIDTH, BUFFER_RCP_HEIGHT);

    float lum1 = tex2D(Edges, uv - float2(1, 0) * texelSize).r;
    float lum2 = tex2D(Edges, uv).r;
    float lum3 = tex2D(Edges, uv + float2(1, 0) * texelSize).r;

    float Gx = 3 * lum1 + 0 * lum2 + -3 * lum3;
    float Gy = 3 * lum1 + 10 * lum2 + 3 * lum3;

    return float4(Gx, Gy, 0, 0);
}

float2 PS_VerticalSobel(float4 position : SV_POSITION, float2 uv : TEXCOORD) : SV_TARGET {
    float2 texelSize = float2(BUFFER_RCP_WIDTH, BUFFER_RCP_HEIGHT);

    float2 grad1 = tex2D(ASCIIPing, uv - float2(0, 1) * texelSize).rg;
    float2 grad2 = tex2D(ASCIIPing, uv).rg;
    float2 grad3 = tex2D(ASCIIPing, uv + float2(0, 1) * texelSize).rg;

    float Gx = 3 * grad1.x + 10 * grad2.x + 3 * grad3.x;
    float Gy = 3 * grad1.y + 0 * grad2.y + -3 * grad3.y;

    float2 G = float2(Gx, Gy);
    G = normalize(G);

    float magnitude = length(float2(Gx, Gy));
    float theta = atan2(G.y, G.x); 

    if (_DepthCutOff > 0.0f) {
        if (ReShade::GetLinearizedDepth(uv) * 1000 > _DepthCutOff)
            theta = 0.0f / 0.0f;
    }

    return float2(theta, 1 - isnan(theta));
}

groupshared int edgeCount[64];
void CS_RenderASCII(uint3 tid : SV_DISPATCHTHREADID, uint3 gid : SV_GROUPTHREADID) {
    float grid = ((gid.y == 0) + (gid.x == 0)) * 0.25f;

    float2 sobel = tex2Dfetch(Sobel, tid.xy).rg;

    float theta = sobel.r;
    float absTheta = abs(theta) / BFX_PI; 

    int direction = -1;

    if (any(sobel.g)) {
        if ((0.0f <= absTheta) && (absTheta < 0.05f)) direction = 0;
        else if ((0.9f < absTheta) && (absTheta <= 1.0f)) direction = 0;
        else if ((0.45f < absTheta) && (absTheta < 0.55f)) direction = 1;
        else if (0.05f < absTheta && absTheta < 0.45f) direction = sign(theta) > 0 ? 3 : 2;
        else if (0.55f < absTheta && absTheta < 0.9f) direction = sign(theta) > 0 ? 3 : 2;
    }

    edgeCount[gid.x + gid.y * 8] = direction;

    barrier();

    int commonEdgeIndex = -1;
    if ((gid.x == 0) && (gid.y == 0)) {
        uint buckets[4] = {0, 0, 0, 0};

        for (int i = 0; i < 64; ++i) {
            buckets[edgeCount[i]] += 1;
        }

        int maxValeu = 0;

        for (int j = 0; j < 4; ++j) {
            if (buckets[j] > maxValeu) {
                commonEdgeIndex = j;
                maxValeu = buckets[j];
            }
        }

        if (maxValeu < _EdgeThreshold) commonEdgeIndex = -1;

        edgeCount[0] = commonEdgeIndex;
    }
    
    barrier();

    commonEdgeIndex = _viewUncompressed ? direction : edgeCount[0];

    float4 quantizedEdge = (commonEdgeIndex + 1) * 8;

    float3 ASCII = 0;

    uint2 downscaleID = tid.xy / 8;
    float4 downscaleInfo = tex2Dfetch(Downscale, downscaleID);

    if (saturate(commonEdgeIndex + 1) && _Edges) {
        float2 localUV;
        localUV.x = ((tid.x % 8)) + quantizedEdge.x;
        localUV.y = 8 - (tid.y % 8);

        ASCII = tex2Dfetch(EdgesASCII, localUV).r;
    } else if (_Fill) {
        float luminance = saturate(pow(downscaleInfo.w * _Exposure, _Attenuation));

        if (_InvertLuminance) luminance = 1 - luminance;

        luminance = max(0, (floor(luminance * 10) - 1)) / 10.0f;

        float2 localUV;
        
        localUV.x = (((tid.x % 8)) + (luminance) * 80);
        localUV.y = (tid.y % 8);

        ASCII = tex2Dfetch(FillASCII, localUV).r;
    }

    ASCII = lerp(_BackgroundColor, lerp(_ASCIIColor, downscaleInfo.rgb, _BlendWithBase), ASCII);

    float depth = tex2Dfetch(Normals, (tid.xy - gid.xy) + 4).w;
    float z = depth * 1000.0f;

    float fogFactor = (_DepthFallOff * 0.005f / sqrt(log(2))) * max(0.0f, z - _DepthOffset);
    fogFactor = exp2(-fogFactor * fogFactor);

    ASCII = lerp(_BackgroundColor, ASCII, fogFactor);

    if (_viewDoG) ASCII = tex2Dfetch(Edges, tid.xy).r;
    
    if(_viewEdges || _viewUncompressed) {
        ASCII = 0;
        if (commonEdgeIndex == 0) ASCII = float3(1, 0, 0);
        else if (commonEdgeIndex == 1) ASCII = float3(0, 1, 0);
        else if (commonEdgeIndex == 2) ASCII = float3(0, 1, 1);
        else if (commonEdgeIndex == 3) ASCII = float3(1, 1, 0);
    }

    tex2Dstore(s_ASCII, tid.xy, float4(ASCII, 1.0f));
}

// ---------------------------------Technique---------------

technique BFX_ASCII < ui_label = "ASCII"; ui_tooltip = "(LDR) Replace the image with text characters."; > {
    pass {
        RenderTarget = BFX_LuminanceASCIITex;
        
        VertexShader = PostProcessVS;
        PixelShader = PS_Luminance;
    }
    pass {
        RenderTarget = BFX_DownscaleTex;
        
        VertexShader = PostProcessVS;
        PixelShader = PS_Downscale;
    }
    pass {
        RenderTarget = BFX_ASCIIPingTex;
        
        VertexShader = PostProcessVS;
        PixelShader = PS_HorizontalBlur;
    }
    pass {
        RenderTarget = BFX_ASCIIDoGTex;
        
        VertexShader = PostProcessVS;
        PixelShader = PS_VerticalBlurAndDifference;
    }
    pass {
        RenderTarget = BFXTemp2::BFX_RenderTex2;
        
        VertexShader = PostProcessVS;
        PixelShader = PS_CalculateNormals;
    }
    pass {
        RenderTarget = BFX_ASCIIEdgesTex;
        
        VertexShader = PostProcessVS;
        PixelShader = PS_EdgeDetect;
    }
    pass {
        RenderTarget = BFX_ASCIIPingTex;
        
        VertexShader = PostProcessVS;
        PixelShader = PS_HorizontalSobel;
    }
    pass {
        RenderTarget = BFX_ASCIISobelTex;
        
        VertexShader = PostProcessVS;
        PixelShader = PS_VerticalSobel;
    }
    pass {
        ComputeShader = CS_RenderASCII<8, 8>;
        DispatchSizeX = BUFFER_WIDTH / 8;
        DispatchSizeY = BUFFER_HEIGHT / 8;
    }
    pass EndPass{

        VertexShader = PostProcessVS;
        PixelShader = PS_EndPass;
    }
}