//! GPU-Renderer (wgpu): MSAA, Tiefenpuffer, PBR-artige Beleuchtung, weiche Bodenschatten.
//! Die Szene wird offscreen gerendert und per Callback in das egui-Fenster eingeblendet.

use eframe::egui;
use eframe::egui_wgpu::{self, wgpu, CallbackTrait, ScreenDescriptor};
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub nrm: [i8; 4],
}

/// Teile: 0 = Basis, 1..=6 = Glieder (folgen Gelenk 1..6); FLOOR = Boden.
pub const PARTS: usize = 7;
pub const FLOOR: u32 = 15;
const MODEL: &[u8] = include_bytes!("../assets/cobot.bin");

/// Uniform-Puffer in vec4: 8 Kamera/Licht, 4 Schattenrichtungen, 64 Matrizen, 16 Farbe+Metall, 16 Rauheit.
pub const UNIFORM_VEC4: usize = 8 + 4 + 64 + 16 + 16;
pub type Uniforms = [[f32; 4]; UNIFORM_VEC4];

pub const SHADOW_PASSES: usize = 4;
const SAMPLES: u32 = 4;
const DEPTH: wgpu::TextureFormat = wgpu::TextureFormat::Depth24PlusStencil8;
const COLOR: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;

const SHADER: &str = r#"
struct U {
  eye: vec4<f32>, right: vec4<f32>, up: vec4<f32>, fwd: vec4<f32>,
  proj: vec4<f32>,   // tan(fov/2), aspect, near, far
  light: vec4<f32>,  // Richtung zum Licht
  bg: vec4<f32>,     // Hintergrund rgb, Raster an/aus
  misc: vec4<f32>,   // Rasterweite, Schatten-Alpha
  sdir: array<vec4<f32>, 4>,
  mats: array<mat4x4<f32>, 16>,
  pcol: array<vec4<f32>, 16>,   // rgb, metallisch (< 0 = Boden)
  prough: array<vec4<f32>, 16>, // x = Rauheit
};
@group(0) @binding(0) var<uniform> u: U;

fn project(p: vec3<f32>) -> vec4<f32> {
  let d = p - u.eye.xyz;
  let x = dot(d, u.right.xyz); let y = dot(d, u.up.xyz); let z = dot(d, u.fwd.xyz);
  let n = u.proj.z; let f = u.proj.w;
  return vec4<f32>(x / (u.proj.x * u.proj.y), y / u.proj.x, f * (z - n) / (f - n), z);
}

struct VOut {
  @builtin(position) pos: vec4<f32>,
  @location(0) world: vec3<f32>,
  @location(1) n: vec3<f32>,
  @location(2) @interpolate(flat) part: u32,
};

@vertex
fn vs_main(@location(0) pos: vec3<f32>, @location(1) n: vec4<f32>, @builtin(instance_index) ii: u32) -> VOut {
  var o: VOut;
  let part = ii & 15u;
  let m = u.mats[part];
  let w = (m * vec4<f32>(pos, 1.0)).xyz;
  o.pos = project(w); o.world = w; o.n = (m * vec4<f32>(n.xyz, 0.0)).xyz; o.part = part;
  return o;
}

fn env_map(d: vec3<f32>, rough: f32) -> vec3<f32> {
  let up = clamp(d.z, -1.0, 1.0);
  let sky = mix(vec3<f32>(0.30, 0.33, 0.38), vec3<f32>(0.85, 0.92, 1.05), smoothstep(0.0, 1.0, up));
  let ground = vec3<f32>(0.07, 0.075, 0.085);
  var c = mix(ground, sky, smoothstep(-0.15, 0.1, up));
  let key = pow(max(dot(d, normalize(u.light.xyz)), 0.0), mix(60.0, 3.0, rough)) * 7.0;
  let strip = smoothstep(0.55, 0.62, up) * (1.0 - smoothstep(0.86, 0.95, up)) * (0.5 + 0.5 * sin(atan2(d.y, d.x) * 2.0 + 1.0));
  c += vec3<f32>(1.0, 0.96, 0.9) * key + vec3<f32>(0.9, 0.95, 1.0) * strip * mix(1.6, 0.3, rough);
  let avg = vec3<f32>(0.42, 0.46, 0.52);
  return mix(c, avg, rough * rough * 0.7);
}

fn ggx(n: vec3<f32>, v: vec3<f32>, l: vec3<f32>, rough: f32, f0: vec3<f32>) -> vec3<f32> {
  let h = normalize(l + v);
  let a = rough * rough; let a2 = a * a;
  let ndh = max(dot(n, h), 0.0); let ndl = max(dot(n, l), 0.0); let ndv = max(dot(n, v), 1e-3);
  let dd = ndh * ndh * (a2 - 1.0) + 1.0;
  let D = a2 / (3.14159 * dd * dd);
  let k = (rough + 1.0) * (rough + 1.0) / 8.0;
  let G = (ndl / (ndl * (1.0 - k) + k)) * (ndv / (ndv * (1.0 - k) + k));
  let F = f0 + (vec3<f32>(1.0) - f0) * pow(1.0 - max(dot(h, v), 0.0), 5.0);
  return D * G * F / (4.0 * ndv * max(ndl, 1e-3));
}

fn light(n: vec3<f32>, v: vec3<f32>, l: vec3<f32>, radiance: vec3<f32>, albedo: vec3<f32>, f0: vec3<f32>, rough: f32) -> vec3<f32> {
  let ndl = max(dot(n, l), 0.0);
  let h = normalize(l + v);
  let F = f0 + (vec3<f32>(1.0) - f0) * pow(1.0 - max(dot(h, v), 0.0), 5.0);
  let kd = (vec3<f32>(1.0) - F);
  return (kd * albedo / 3.14159 + ggx(n, v, l, rough, f0)) * radiance * ndl;
}

fn aces(x: vec3<f32>) -> vec3<f32> {
  return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), vec3<f32>(0.0), vec3<f32>(1.0));
}

fn grid_line(p: vec2<f32>) -> f32 {
  let w = fwidth(p);
  let c = abs(fract(p - 0.5) - 0.5) / max(w, vec2<f32>(1e-4));
  return 1.0 - min(min(c.x, c.y), 1.0);
}

fn floor_color(w: vec3<f32>) -> vec3<f32> {
  let d = length(w.xy);
  let fade = 1.0 - smoothstep(300.0, 1000.0, d);
  var c = u.bg.rgb;
  if (u.bg.w > 0.5) {
    let s = u.misc.x;
    let minor = grid_line(w.xy / s);
    let major = grid_line(w.xy / (s * 5.0));
    c -= vec3<f32>(1.0) * (minor * 0.07 + major * 0.11) * fade;
    let fw = fwidth(w.xy);
    let ax = 1.0 - clamp(abs(w.y) / max(fw.y * 1.2, 1e-4), 0.0, 1.0);
    let ay = 1.0 - clamp(abs(w.x) / max(fw.x * 1.2, 1e-4), 0.0, 1.0);
    c = mix(c, vec3<f32>(0.9, 0.35, 0.35), ax * 0.8 * fade * step(0.0, w.x));
    c = mix(c, vec3<f32>(0.35, 0.78, 0.45), ay * 0.8 * fade * step(0.0, w.y));
  }
  return c;
}

@fragment
fn fs_main(in: VOut) -> @location(0) vec4<f32> {
  let pc = u.pcol[in.part];
  if (pc.w < -0.5) {
    return vec4<f32>(floor_color(in.world), 1.0);
  }
  let v = normalize(u.eye.xyz - in.world);
  var n = normalize(in.n);
  if (dot(n, v) < 0.0) { n = -n; }
  let metal = pc.w; let rough = clamp(u.prough[in.part].x, 0.05, 1.0);
  let base = pc.rgb;
  let f0 = mix(vec3<f32>(0.04), base, metal);
  let albedo = base * (1.0 - metal);
  var lo = vec3<f32>(0.0);
  lo += light(n, v, normalize(u.light.xyz), vec3<f32>(1.0, 0.96, 0.9) * 3.2, albedo, f0, rough);
  lo += light(n, v, normalize(vec3<f32>(-0.7, 0.55, 0.3)), vec3<f32>(0.5, 0.62, 0.9) * 1.1, albedo, f0, rough);
  lo += light(n, v, normalize(vec3<f32>(-0.2, -0.9, 0.35)), vec3<f32>(0.9, 0.95, 1.0) * 0.9, albedo, f0, rough);
  let r = reflect(-v, n);
  let ndv = max(dot(n, v), 0.0);
  let fr = f0 + (max(vec3<f32>(1.0 - rough), f0) - f0) * pow(1.0 - ndv, 5.0);
  var amb = env_map(n, 1.0) * albedo * 0.75 + env_map(r, rough) * fr;
  let ao = mix(0.55, 1.0, smoothstep(0.0, 35.0, in.world.z + 25.0 * max(n.z, 0.0)));
  amb *= ao;
  var col = aces((lo + amb) * 0.95);
  col = pow(col, vec3<f32>(1.0 / 2.2));
  return vec4<f32>(col, 1.0);
}

@vertex
fn vs_shadow(@location(0) pos: vec3<f32>, @builtin(instance_index) ii: u32) -> @builtin(position) vec4<f32> {
  let w = (u.mats[ii & 15u] * vec4<f32>(pos, 1.0)).xyz;
  let l = u.sdir[ii >> 4u].xyz;
  let p = vec3<f32>(w.x - l.x * w.z / l.z, w.y - l.y * w.z / l.z, 0.3);
  return project(p);
}
@fragment
fn fs_shadow() -> @location(0) vec4<f32> {
  return vec4<f32>(0.0, 0.0, 0.0, u.misc.y);
}

struct BOut { @builtin(position) pos: vec4<f32>, @location(0) uv: vec2<f32> };
@vertex
fn vs_blit(@builtin(vertex_index) i: u32) -> BOut {
  var o: BOut;
  let p = vec2<f32>(f32((i << 1u) & 2u), f32(i & 2u));
  o.pos = vec4<f32>(p * 2.0 - 1.0, 0.0, 1.0);
  o.uv = vec2<f32>(p.x, 1.0 - p.y);
  return o;
}
@group(0) @binding(0) var tex: texture_2d<f32>;
@group(0) @binding(1) var samp: sampler;
@fragment
fn fs_blit(in: BOut) -> @location(0) vec4<f32> {
  return textureSample(tex, samp, in.uv);
}
"#;

struct Target {
    size: [u32; 2],
    msaa: wgpu::TextureView,
    depth: wgpu::TextureView,
    resolve: wgpu::TextureView,
    blit_bg: wgpu::BindGroup,
}

pub struct Gpu {
    scene: wgpu::RenderPipeline,
    shadow: Vec<wgpu::RenderPipeline>,
    blit: wgpu::RenderPipeline,
    blit_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    ubuf: wgpu::Buffer,
    ubg: wgpu::BindGroup,
    vbuf: wgpu::Buffer,
    ibuf: wgpu::Buffer,
    /// (erster Index, Anzahl, Basis-Vertex) je Teil; Boden zuletzt
    ranges: Vec<(u32, u32, i32)>,
    target: Option<Target>,
}

// Liest assets/cobot.bin (7 Teile, ~450k Dreiecke) in den Speicher
fn load_model() -> (Vec<Vertex>, Vec<u32>, Vec<(u32, u32, i32)>) {
    let rd = |o: usize| u32::from_le_bytes(MODEL[o..o + 4].try_into().unwrap()) as usize;
    let (mut o, mut verts, mut idx, mut ranges) = (4, vec![], vec![], vec![]);
    for _ in 0..rd(0) {
        let (nv, ni) = (rd(o), rd(o + 4));
        o += 8;
        ranges.push((idx.len() as u32, ni as u32, verts.len() as i32));
        verts.extend(MODEL[o..o + nv * 16].chunks_exact(16).map(|c| bytemuck::pod_read_unaligned::<Vertex>(c)));
        o += nv * 16;
        idx.extend(MODEL[o..o + ni * 4].chunks_exact(4).map(|c| u32::from_le_bytes(c.try_into().unwrap())));
        o += ni * 4;
    }
    // Boden
    let e = 4000.0;
    let v = |x: f32, y: f32| Vertex { pos: [x, y, 0.0], nrm: [0, 0, 127, 0] };
    ranges.push((idx.len() as u32, 6, verts.len() as i32));
    verts.extend([v(-e, -e), v(e, -e), v(e, e), v(-e, e)]);
    idx.extend([0, 1, 2, 0, 2, 3]);
    (verts, idx, ranges)
}

impl Gpu {
    // Pipelines, Buffer, Shader aufsetzen. Einmal beim Start, danach nur noch zeichnen
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("arm-shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let ubl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            }],
        });
        let ubuf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("arm-uniforms"),
            size: (UNIFORM_VEC4 * 16) as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let ubg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &ubl,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: ubuf.as_entire_binding() }],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&ubl)],
            immediate_size: 0,
        });
        let ms = wgpu::MultisampleState { count: SAMPLES, mask: !0, alpha_to_coverage_enabled: false };
        let scene_attrs = wgpu::vertex_attr_array![0 => Float32x3, 1 => Snorm8x4];
        
        let prim = wgpu::PrimitiveState { cull_mode: None, ..Default::default() };

        let scene = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("arm-scene"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &module,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: 16,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &scene_attrs,
                })],
            },
            fragment: Some(wgpu::FragmentState {
                module: &module,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(COLOR.into())],
            }),
            primitive: prim,
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::Less),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: ms,
            multiview_mask: None,
            cache: None,
        });

        // Jeder Schatten-Durchlauf nutzt ein eigenes Stencil-Bit: pro Pixel höchstens einmal blenden.
        let shadow = (0..SHADOW_PASSES)
            .map(|k| {
                let face = wgpu::StencilFaceState {
                    compare: wgpu::CompareFunction::Equal,
                    fail_op: wgpu::StencilOperation::Keep,
                    depth_fail_op: wgpu::StencilOperation::Keep,
                    pass_op: wgpu::StencilOperation::Invert,
                };
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("arm-shadow"),
                    layout: Some(&layout),
                    vertex: wgpu::VertexState {
                        module: &module,
                        entry_point: Some("vs_shadow"),
                        compilation_options: Default::default(),
                        buffers: &[Some(wgpu::VertexBufferLayout {
                            array_stride: 16,
                            step_mode: wgpu::VertexStepMode::Vertex,
                            attributes: &scene_attrs[..1],
                        })],
                    },
                    fragment: Some(wgpu::FragmentState {
                        module: &module,
                        entry_point: Some("fs_shadow"),
                        compilation_options: Default::default(),
                        targets: &[Some(wgpu::ColorTargetState {
                            format: COLOR,
                            blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                            write_mask: wgpu::ColorWrites::ALL,
                        })],
                    }),
                    primitive: prim,
                    depth_stencil: Some(wgpu::DepthStencilState {
                        format: DEPTH,
                        depth_write_enabled: Some(false),
                        depth_compare: Some(wgpu::CompareFunction::LessEqual),
                        stencil: wgpu::StencilState { front: face, back: face, read_mask: 1 << k, write_mask: 1 << k },
                        bias: Default::default(),
                    }),
                    multisample: ms,
                    multiview_mask: None,
                    cache: None,
                })
            })
            .collect();

        let blit_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let blit_pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&blit_layout)],
            immediate_size: 0,
        });
        let blit = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("arm-blit"),
            layout: Some(&blit_pl),
            vertex: wgpu::VertexState { module: &module, entry_point: Some("vs_blit"), compilation_options: Default::default(), buffers: &[] },
            fragment: Some(wgpu::FragmentState {
                module: &module,
                entry_point: Some("fs_blit"),
                compilation_options: Default::default(),
                targets: &[Some(target_format.into())],
            }),
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let (verts, idx, ranges) = load_model();
        let vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("arm-verts"), contents: bytemuck::cast_slice(&verts), usage: wgpu::BufferUsages::VERTEX });
        let ibuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("arm-indices"), contents: bytemuck::cast_slice(&idx), usage: wgpu::BufferUsages::INDEX });
        Self { scene, shadow, blit, blit_layout, sampler, ubuf, ubg, vbuf, ibuf, ranges, target: None }
    }

    // Offscreen-Bild bei Größenänderung neu anlegen
    fn ensure_target(&mut self, device: &wgpu::Device, size: [u32; 2]) {
        if self.target.as_ref().is_some_and(|t| t.size == size) {
            return;
        }
        let tex = |format, samples, usage| {
            device
                .create_texture(&wgpu::TextureDescriptor {
                    label: Some("arm-target"),
                    size: wgpu::Extent3d { width: size[0], height: size[1], depth_or_array_layers: 1 },
                    mip_level_count: 1,
                    sample_count: samples,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage,
                    view_formats: &[],
                })
                .create_view(&Default::default())
        };
        let msaa = tex(COLOR, SAMPLES, wgpu::TextureUsages::RENDER_ATTACHMENT);
        let depth = tex(DEPTH, SAMPLES, wgpu::TextureUsages::RENDER_ATTACHMENT);
        let resolve = tex(COLOR, 1, wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING);
        let blit_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &self.blit_layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&resolve) },
                wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(&self.sampler) },
            ],
        });
        self.target = Some(Target { size, msaa, depth, resolve, blit_bg });
    }
}

/// Alle pro Frame nötigen Daten.
pub struct SceneCallback {
    pub size_points: [f32; 2],
    pub uniforms: Uniforms,
    pub bg: [f32; 3],
    pub shadows: bool,
}

impl CallbackTrait for SceneCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        sd: &ScreenDescriptor,
        encoder: &mut wgpu::CommandEncoder,
        res: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let Some(gpu) = res.get_mut::<Gpu>() else { return vec![] };
        let size = [
            ((self.size_points[0] * sd.pixels_per_point) as u32).max(1),
            ((self.size_points[1] * sd.pixels_per_point) as u32).max(1),
        ];
        gpu.ensure_target(device, size);
        queue.write_buffer(&gpu.ubuf, 0, bytemuck::cast_slice(&self.uniforms));

        let t = gpu.target.as_ref().unwrap();
        let bg = self.bg;
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("arm-scene"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &t.msaa,
                depth_slice: None,
                resolve_target: Some(&t.resolve),
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color { r: bg[0] as f64, g: bg[1] as f64, b: bg[2] as f64, a: 1.0 }),
                    store: wgpu::StoreOp::Discard,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &t.depth,
                depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Discard }),
                stencil_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(0), store: wgpu::StoreOp::Discard }),
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_bind_group(0, &gpu.ubg, &[]);
        pass.set_vertex_buffer(0, gpu.vbuf.slice(..));
        pass.set_index_buffer(gpu.ibuf.slice(..), wgpu::IndexFormat::Uint32);
        let draw = |pass: &mut wgpu::RenderPass, part: usize, inst: u32| {
            let (first, n, base) = gpu.ranges[part];
            pass.draw_indexed(first..first + n, base, inst..inst + 1);
        };
        pass.set_pipeline(&gpu.scene);
        draw(&mut pass, PARTS, FLOOR);
        if self.shadows {
            for k in 0..SHADOW_PASSES {
                pass.set_pipeline(&gpu.shadow[k]);
                pass.set_stencil_reference(0);
                for part in 0..PARTS {
                    draw(&mut pass, part, ((k as u32) << 4) | part as u32);
                }
            }
        }
        pass.set_pipeline(&gpu.scene);
        for part in 0..PARTS {
            draw(&mut pass, part, part as u32);
        }
        drop(pass);
        vec![]
    }

    fn paint(&self, _info: egui::PaintCallbackInfo, pass: &mut wgpu::RenderPass<'static>, res: &egui_wgpu::CallbackResources) {
        let Some(gpu) = res.get::<Gpu>() else { return };
        if let Some(t) = &gpu.target {
            pass.set_pipeline(&gpu.blit);
            pass.set_bind_group(0, &t.blit_bg, &[]);
            pass.draw(0..3, 0..1);
        }
    }
}

