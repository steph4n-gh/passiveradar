use num_complex::Complex;
use std::sync::OnceLock;

static GPU_CONTEXT: OnceLock<Option<GpuContext>> = OnceLock::new();
static GPU_DDC_PIPELINE: OnceLock<Option<GpuDdcPipeline>> = OnceLock::new();
static GPU_CAF_PIPELINE: OnceLock<Option<GpuCafPipeline>> = OnceLock::new();
static GPU_ISAR_PIPELINE: OnceLock<Option<GpuIsarPipeline>> = OnceLock::new();

pub struct GpuContext {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
}

pub fn get_gpu_context() -> Option<&'static GpuContext> {
    GPU_CONTEXT.get_or_init(|| {
        pollster::block_on(init_gpu_context())
    }).as_ref()
}

async fn init_gpu_context() -> Option<GpuContext> {
    if crate::dsp::fft::DISABLE_GPU.load(std::sync::atomic::Ordering::SeqCst) {
        return None;
    }
    let backends = if cfg!(target_os = "macos") {
        wgpu::Backends::METAL
    } else {
        wgpu::Backends::all()
    };
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends,
        ..wgpu::InstanceDescriptor::new_without_display_handle()
    });
    
    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        })
        .await
        .ok()?;
        
    let (device, queue) = adapter
        .request_device(
            &wgpu::DeviceDescriptor {
                label: Some("PR-FIS GPU Device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                memory_hints: wgpu::MemoryHints::default(),
                experimental_features: wgpu::ExperimentalFeatures::default(),
                trace: wgpu::Trace::Off,
            }
        )
        .await
        .ok()?;
        
    Some(GpuContext { device, queue })
}

pub fn get_gpu_ddc_pipeline() -> Option<&'static GpuDdcPipeline> {
    GPU_DDC_PIPELINE.get_or_init(|| {
        let ctx = get_gpu_context()?;
        pollster::block_on(GpuDdcPipeline::new(ctx.device.clone(), ctx.queue.clone())).ok()
    }).as_ref()
}

pub struct GpuDdcPipeline {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub pipeline: wgpu::ComputePipeline,
    pub history_pipeline: wgpu::ComputePipeline,
    pub bind_group_layout: wgpu::BindGroupLayout,
}

impl GpuDdcPipeline {
    pub async fn new(device: wgpu::Device, queue: wgpu::Queue) -> Result<Self, String> {
        let shader_source = include_str!("shaders/ddc.wgsl");
        let shader_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("DDC Shader Module"),
            source: wgpu::ShaderSource::Wgsl(shader_source.into()),
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("DDC Bind Group Layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 4,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 5,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("DDC Pipeline Layout"),
            bind_group_layouts: &[Some(&bind_group_layout)],
            immediate_size: 0,
        });

        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("DDC Decimate Compute Pipeline"),
            layout: Some(&pipeline_layout),
            module: &shader_module,
            entry_point: Some("mix_and_decimate"),
            compilation_options: Default::default(),
            cache: None,
        });

        let history_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("DDC History Copy Compute Pipeline"),
            layout: Some(&pipeline_layout),
            module: &shader_module,
            entry_point: Some("copy_history"),
            compilation_options: Default::default(),
            cache: None,
        });

        Ok(Self {
            device,
            queue,
            pipeline,
            history_pipeline,
            bind_group_layout,
        })
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ShaderParams {
    pub phase: f32,
    pub phase_step: f32,
    pub decimation_factor: u32,
    pub num_taps: u32,
    pub input_len: u32,
    pub counter: u32,
}

pub struct GpuDdcState {
    pub input_buffer: wgpu::Buffer,
    pub taps_buffer: wgpu::Buffer,
    pub history_in_buffer: wgpu::Buffer,
    pub output_buffer: wgpu::Buffer,
    pub history_out_buffer: wgpu::Buffer,
    pub params_buffer: wgpu::Buffer,
    pub bind_group: wgpu::BindGroup,
    pub output_staging_buffer: wgpu::Buffer,
    pub history_staging_buffer: wgpu::Buffer,
    
    pub input_len: usize,
    pub num_taps: usize,
    pub history_len: usize,
    pub output_len: usize,
}

fn cast_complex_slice(slice: &[Complex<f32>]) -> &[u8] {
    unsafe {
        std::slice::from_raw_parts(
            slice.as_ptr() as *const u8,
            slice.len() * std::mem::size_of::<Complex<f32>>(),
        )
    }
}

fn cast_bytes_to_complex(bytes: &[u8]) -> &[Complex<f32>] {
    assert_eq!(bytes.len() % std::mem::size_of::<Complex<f32>>(), 0);
    unsafe {
        std::slice::from_raw_parts(
            bytes.as_ptr() as *const Complex<f32>,
            bytes.len() / std::mem::size_of::<Complex<f32>>(),
        )
    }
}

impl GpuDdcState {
    pub fn new(
        pipeline: &GpuDdcPipeline,
        input_len: usize,
        num_taps: usize,
        decimation_factor: usize,
    ) -> Self {
        let device = &pipeline.device;
        let history_len = num_taps.saturating_sub(1);
        let output_len = (history_len + input_len).saturating_sub(num_taps) / decimation_factor + 1;

        let params_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("DDC Params Buffer"),
            size: std::mem::size_of::<ShaderParams>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let taps_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("DDC Taps Buffer"),
            size: (num_taps * std::mem::size_of::<f32>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let input_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("DDC Input Buffer"),
            size: (input_len * std::mem::size_of::<[f32; 2]>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let history_in_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("DDC History In Buffer"),
            size: (history_len * std::mem::size_of::<[f32; 2]>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let output_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("DDC Output Buffer"),
            size: (output_len * std::mem::size_of::<[f32; 2]>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let history_out_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("DDC History Out Buffer"),
            size: (history_len * std::mem::size_of::<[f32; 2]>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("DDC Bind Group"),
            layout: &pipeline.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: taps_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: input_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: history_in_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: output_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: history_out_buffer.as_entire_binding(),
                },
            ],
        });

        let output_staging_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("DDC Output Staging Buffer"),
            size: (output_len * std::mem::size_of::<[f32; 2]>()) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let history_staging_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("DDC History Staging Buffer"),
            size: (history_len * std::mem::size_of::<[f32; 2]>()) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        Self {
            input_buffer,
            taps_buffer,
            history_in_buffer,
            output_buffer,
            history_out_buffer,
            params_buffer,
            bind_group,
            output_staging_buffer,
            history_staging_buffer,
            input_len,
            num_taps,
            history_len,
            output_len,
        }
    }

    pub fn process_block(
        &mut self,
        pipeline: &GpuDdcPipeline,
        phase: &mut f32,
        phase_step: f32,
        decimation_factor: usize,
        counter: &mut usize,
        input_samples: &[Complex<f32>],
        taps: &[f32],
        history: &mut Vec<Complex<f32>>,
        output: &mut Vec<Complex<f32>>,
    ) {
        if input_samples.is_empty() {
            output.clear();
            return;
        }

        if input_samples.len() != self.input_len
            || taps.len() != self.num_taps
            || history.len() != self.history_len
        {
            *self = GpuDdcState::new(
                pipeline,
                input_samples.len(),
                taps.len(),
                decimation_factor,
            );
        }

        let shader_params = ShaderParams {
            phase: *phase,
            phase_step,
            decimation_factor: decimation_factor as u32,
            num_taps: taps.len() as u32,
            input_len: input_samples.len() as u32,
            counter: *counter as u32,
        };

        pipeline.queue.write_buffer(
            &self.params_buffer,
            0,
            bytemuck::bytes_of(&shader_params),
        );

        pipeline.queue.write_buffer(
            &self.taps_buffer,
            0,
            bytemuck::cast_slice(taps),
        );

        pipeline.queue.write_buffer(
            &self.input_buffer,
            0,
            cast_complex_slice(input_samples),
        );

        pipeline.queue.write_buffer(
            &self.history_in_buffer,
            0,
            cast_complex_slice(history),
        );

        let mut encoder = pipeline.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("DDC Command Encoder"),
        });

        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("DDC Compute Pass"),
                timestamp_writes: None,
            });
            compute_pass.set_pipeline(&pipeline.pipeline);
            compute_pass.set_bind_group(0, &self.bind_group, &[]);
            let workgroup_count = (self.output_len + 255) / 256;
            compute_pass.dispatch_workgroups(workgroup_count as u32, 1, 1);
        }

        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("DDC History Compute Pass"),
                timestamp_writes: None,
            });
            compute_pass.set_pipeline(&pipeline.history_pipeline);
            compute_pass.set_bind_group(0, &self.bind_group, &[]);
            let workgroup_count = (self.history_len + 255) / 256;
            compute_pass.dispatch_workgroups(workgroup_count as u32, 1, 1);
        }

        encoder.copy_buffer_to_buffer(
            &self.output_buffer,
            0,
            &self.output_staging_buffer,
            0,
            (self.output_len * std::mem::size_of::<[f32; 2]>()) as u64,
        );

        encoder.copy_buffer_to_buffer(
            &self.history_out_buffer,
            0,
            &self.history_staging_buffer,
            0,
            (self.history_len * std::mem::size_of::<[f32; 2]>()) as u64,
        );

        pipeline.queue.submit(Some(encoder.finish()));

        {
            let buffer_slice = self.output_staging_buffer.slice(..);
            let (tx, rx) = std::sync::mpsc::channel();
            buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
                let _ = tx.send(result);
            });
            let _ = pipeline.device.poll(wgpu::PollType::wait_indefinitely());
            rx.recv().unwrap().unwrap();
            
            let data = buffer_slice.get_mapped_range();
            let result_slice: &[Complex<f32>] = cast_bytes_to_complex(&data);
            output.clear();
            output.extend_from_slice(result_slice);
            drop(data);
            self.output_staging_buffer.unmap();
        }

        {
            let buffer_slice = self.history_staging_buffer.slice(..);
            let (tx, rx) = std::sync::mpsc::channel();
            buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
                let _ = tx.send(result);
            });
            let _ = pipeline.device.poll(wgpu::PollType::wait_indefinitely());
            rx.recv().unwrap().unwrap();
            
            let data = buffer_slice.get_mapped_range();
            let result_slice: &[Complex<f32>] = cast_bytes_to_complex(&data);
            history.clear();
            history.extend_from_slice(result_slice);
            drop(data);
            self.history_staging_buffer.unmap();
        }

        let final_phase = *phase + input_samples.len() as f32 * phase_step;
        *phase = final_phase % (2.0 * std::f32::consts::PI);
        if *phase < 0.0 {
            *phase += 2.0 * std::f32::consts::PI;
        }

        *counter = 0;
    }
}

// ---------------------------------------------------------------------------
// GPU CAF (Cross-Ambiguity Function) pipeline
// ---------------------------------------------------------------------------

pub fn get_gpu_caf_pipeline() -> Option<&'static GpuCafPipeline> {
    GPU_CAF_PIPELINE.get_or_init(|| {
        let ctx = get_gpu_context()?;
        pollster::block_on(GpuCafPipeline::new(ctx.device.clone(), ctx.queue.clone())).ok()
    }).as_ref()
}

pub struct GpuCafPipeline {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub pipeline: wgpu::ComputePipeline,
    pub bind_group_layout: wgpu::BindGroupLayout,
}

impl GpuCafPipeline {
    pub async fn new(device: wgpu::Device, queue: wgpu::Queue) -> Result<Self, String> {
        let shader_source = include_str!("shaders/caf.wgsl");
        let shader_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("CAF Shader Module"),
            source: wgpu::ShaderSource::Wgsl(shader_source.into()),
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("CAF Bind Group Layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("CAF Pipeline Layout"),
            bind_group_layouts: &[Some(&bind_group_layout)],
            immediate_size: 0,
        });

        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("CAF Compute Pipeline"),
            layout: Some(&pipeline_layout),
            module: &shader_module,
            entry_point: Some("caf_compute"),
            compilation_options: Default::default(),
            cache: None,
        });

        Ok(Self {
            device,
            queue,
            pipeline,
            bind_group_layout,
        })
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ShaderCafParams {
    pub num_samples: u32,
    pub max_delay: u32,
    pub num_doppler_bins: u32,
    pub doppler_step: f32,
    pub block_size: u32,
    pub pad0: u32,
}

pub struct GpuCafState {
    pub surv_buffer: wgpu::Buffer,
    pub ref_buffer: wgpu::Buffer,
    pub output_buffer: wgpu::Buffer,
    pub output_staging_buffer: wgpu::Buffer,
    pub params_buffer: wgpu::Buffer,
    pub bind_group: wgpu::BindGroup,

    pub num_samples: usize,
    pub max_delay: usize,
    pub num_doppler_bins: usize,
}

impl GpuCafState {
    pub fn new(
        pipeline: &GpuCafPipeline,
        num_samples: usize,
        max_delay: usize,
        num_doppler_bins: usize,
    ) -> Self {
        let device = &pipeline.device;
        let output_len = max_delay * num_doppler_bins;

        let params_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("CAF Params Buffer"),
            size: std::mem::size_of::<ShaderCafParams>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let surv_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("CAF Surveillance Buffer"),
            size: (num_samples * std::mem::size_of::<[f32; 2]>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let ref_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("CAF Reference Buffer"),
            size: (num_samples * std::mem::size_of::<[f32; 2]>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let output_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("CAF Output Buffer"),
            size: (output_len * std::mem::size_of::<f32>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let output_staging_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("CAF Output Staging Buffer"),
            size: (output_len * std::mem::size_of::<f32>()) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("CAF Bind Group"),
            layout: &pipeline.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: surv_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: ref_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: output_buffer.as_entire_binding(),
                },
            ],
        });

        Self {
            surv_buffer,
            ref_buffer,
            output_buffer,
            output_staging_buffer,
            params_buffer,
            bind_group,
            num_samples,
            max_delay,
            num_doppler_bins,
        }
    }

    pub fn process_caf(
        &mut self,
        pipeline: &GpuCafPipeline,
        surv: &[Complex<f32>],
        reference: &[Complex<f32>],
        max_delay: usize,
        num_doppler_bins: usize,
        doppler_step: f32,
    ) -> Vec<Vec<f32>> {
        let num_samples = surv.len();
        let output_len = max_delay * num_doppler_bins;

        if num_samples == 0 || max_delay == 0 || num_doppler_bins == 0 {
            return vec![vec![0.0; num_doppler_bins]; max_delay];
        }

        if num_samples != self.num_samples
            || max_delay != self.max_delay
            || num_doppler_bins != self.num_doppler_bins
        {
            *self = GpuCafState::new(pipeline, num_samples, max_delay, num_doppler_bins);
        }

        let block_size = num_samples.saturating_sub(max_delay);

        let shader_params = ShaderCafParams {
            num_samples: num_samples as u32,
            max_delay: max_delay as u32,
            num_doppler_bins: num_doppler_bins as u32,
            doppler_step,
            block_size: block_size as u32,
            pad0: 0,
        };

        pipeline.queue.write_buffer(
            &self.params_buffer,
            0,
            bytemuck::bytes_of(&shader_params),
        );

        pipeline.queue.write_buffer(
            &self.surv_buffer,
            0,
            cast_complex_slice(surv),
        );

        pipeline.queue.write_buffer(
            &self.ref_buffer,
            0,
            cast_complex_slice(reference),
        );

        let mut encoder = pipeline.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("CAF Command Encoder"),
        });

        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("CAF Compute Pass"),
                timestamp_writes: None,
            });
            compute_pass.set_pipeline(&pipeline.pipeline);
            compute_pass.set_bind_group(0, &self.bind_group, &[]);
            let workgroup_count = (output_len + 255) / 256;
            compute_pass.dispatch_workgroups(workgroup_count as u32, 1, 1);
        }

        encoder.copy_buffer_to_buffer(
            &self.output_buffer,
            0,
            &self.output_staging_buffer,
            0,
            (output_len * std::mem::size_of::<f32>()) as u64,
        );

        pipeline.queue.submit(Some(encoder.finish()));

        let buffer_slice = self.output_staging_buffer.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = tx.send(result);
        });
        let _ = pipeline.device.poll(wgpu::PollType::wait_indefinitely());
        rx.recv().unwrap().unwrap();

        let data = buffer_slice.get_mapped_range();
        let flat_surface: &[f32] = bytemuck::cast_slice(&data);

        let mut surface = Vec::with_capacity(max_delay);
        for delay_idx in 0..max_delay {
            let start = delay_idx * num_doppler_bins;
            let end = start + num_doppler_bins;
            surface.push(flat_surface[start..end].to_vec());
        }

        drop(data);
        self.output_staging_buffer.unmap();

        surface
    }
}

// ---------------------------------------------------------------------------
// GPU ISAR (Inverse Synthetic Aperture Radar) backprojection pipeline
// ---------------------------------------------------------------------------

pub fn get_gpu_isar_pipeline() -> Option<&'static GpuIsarPipeline> {
    GPU_ISAR_PIPELINE.get_or_init(|| {
        let ctx = get_gpu_context()?;
        pollster::block_on(GpuIsarPipeline::new(ctx.device.clone(), ctx.queue.clone())).ok()
    }).as_ref()
}

pub struct GpuIsarPipeline {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub pipeline: wgpu::ComputePipeline,
    pub bind_group_layout: wgpu::BindGroupLayout,
}

impl GpuIsarPipeline {
    pub async fn new(device: wgpu::Device, queue: wgpu::Queue) -> Result<Self, String> {
        let shader_source = include_str!("shaders/isar_fbp.wgsl");
        let shader_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("ISAR Shader Module"),
            source: wgpu::ShaderSource::Wgsl(shader_source.into()),
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("ISAR Bind Group Layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("ISAR Pipeline Layout"),
            bind_group_layouts: &[Some(&bind_group_layout)],
            immediate_size: 0,
        });

        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("ISAR Compute Pipeline"),
            layout: Some(&pipeline_layout),
            module: &shader_module,
            entry_point: Some("backproject"),
            compilation_options: Default::default(),
            cache: None,
        });

        Ok(Self {
            device,
            queue,
            pipeline,
            bind_group_layout,
        })
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ShaderIsarParams {
    pub grid_size: u32,
    pub num_angles: u32,
    pub num_bins: u32,
    pub pad: u32,
}

pub struct GpuIsarState {
    pub profiles_buffer: wgpu::Buffer,
    pub angles_buffer: wgpu::Buffer,
    pub image_buffer: wgpu::Buffer,
    pub image_staging_buffer: wgpu::Buffer,
    pub params_buffer: wgpu::Buffer,
    pub bind_group: wgpu::BindGroup,

    pub grid_size: usize,
    pub num_angles: usize,
    pub num_bins: usize,
}

impl GpuIsarState {
    pub fn new(
        pipeline: &GpuIsarPipeline,
        grid_size: usize,
        num_angles: usize,
        num_bins: usize,
    ) -> Self {
        let device = &pipeline.device;

        let params_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("ISAR Params Buffer"),
            size: std::mem::size_of::<ShaderIsarParams>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let profiles_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("ISAR Profiles Buffer"),
            size: (num_angles * num_bins * std::mem::size_of::<f32>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let angles_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("ISAR Angles Buffer"),
            size: (num_angles * 2 * std::mem::size_of::<f32>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let image_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("ISAR Image Buffer"),
            size: (grid_size * grid_size * std::mem::size_of::<f32>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let image_staging_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("ISAR Image Staging Buffer"),
            size: (grid_size * grid_size * std::mem::size_of::<f32>()) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("ISAR Bind Group"),
            layout: &pipeline.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: profiles_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: angles_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: image_buffer.as_entire_binding(),
                },
            ],
        });

        Self {
            profiles_buffer,
            angles_buffer,
            image_buffer,
            image_staging_buffer,
            params_buffer,
            bind_group,
            grid_size,
            num_angles,
            num_bins,
        }
    }

    pub fn process_isar(
        &mut self,
        pipeline: &GpuIsarPipeline,
        profiles_flat: &[f32],
        angles_flat: &[f32],
        grid_size: usize,
        num_angles: usize,
        num_bins: usize,
    ) -> Vec<Vec<f32>> {
        if num_angles == 0 || num_bins == 0 || grid_size == 0 {
            return vec![vec![0.0; grid_size]; grid_size];
        }

        if grid_size != self.grid_size
            || num_angles != self.num_angles
            || num_bins != self.num_bins
        {
            *self = GpuIsarState::new(pipeline, grid_size, num_angles, num_bins);
        }

        let shader_params = ShaderIsarParams {
            grid_size: grid_size as u32,
            num_angles: num_angles as u32,
            num_bins: num_bins as u32,
            pad: 0,
        };

        pipeline.queue.write_buffer(
            &self.params_buffer,
            0,
            bytemuck::bytes_of(&shader_params),
        );

        pipeline.queue.write_buffer(
            &self.profiles_buffer,
            0,
            bytemuck::cast_slice(profiles_flat),
        );

        pipeline.queue.write_buffer(
            &self.angles_buffer,
            0,
            bytemuck::cast_slice(angles_flat),
        );

        let mut encoder = pipeline.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("ISAR Command Encoder"),
        });

        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("ISAR Compute Pass"),
                timestamp_writes: None,
            });
            compute_pass.set_pipeline(&pipeline.pipeline);
            compute_pass.set_bind_group(0, &self.bind_group, &[]);
            let x_groups = (grid_size + 15) / 16;
            let y_groups = (grid_size + 15) / 16;
            compute_pass.dispatch_workgroups(x_groups as u32, y_groups as u32, 1);
        }

        encoder.copy_buffer_to_buffer(
            &self.image_buffer,
            0,
            &self.image_staging_buffer,
            0,
            (grid_size * grid_size * std::mem::size_of::<f32>()) as u64,
        );

        pipeline.queue.submit(Some(encoder.finish()));

        let buffer_slice = self.image_staging_buffer.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = tx.send(result);
        });
        let _ = pipeline.device.poll(wgpu::PollType::wait_indefinitely());
        rx.recv().unwrap().unwrap();

        let data = buffer_slice.get_mapped_range();
        let flat_image: &[f32] = bytemuck::cast_slice(&data);

        let mut image = Vec::with_capacity(grid_size);
        for x in 0..grid_size {
            let start = x * grid_size;
            let end = start + grid_size;
            image.push(flat_image[start..end].to_vec());
        }

        drop(data);
        self.image_staging_buffer.unmap();

        image
    }
}
