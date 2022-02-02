
#include "gpu_shader_create_info.hh"

/* -------------------------------------------------------------------- */
/** \name Common
 * \{ */

/* TODO(@fclem): This is a bit out of place at the moment. */
GPU_SHADER_CREATE_INFO(eevee_shared)
    .typedef_source("eevee_defines.hh")
    .typedef_source("eevee_shader_shared.hh");

/** \} */

/* -------------------------------------------------------------------- */
/** \name Surface Mesh Type
 * \{ */

GPU_SHADER_CREATE_INFO(eevee_surface_mesh)
    .vertex_in(0, Type::VEC3, "pos")
    .vertex_in(1, Type::VEC3, "nor")
    .vertex_source("eevee_surface_mesh_vert.glsl")
    .additional_info("draw_mesh");

GPU_SHADER_CREATE_INFO(eevee_surface_gpencil)
    .define("MAT_GEOM_GPENCIL")
    .vertex_source("eevee_surface_gpencil_vert.glsl")
    .additional_info("draw_gpencil");

GPU_SHADER_CREATE_INFO(eevee_surface_hair)
    .define("MAT_GEOM_HAIR")
    .vertex_source("eevee_surface_hair_vert.glsl")
    .additional_info("draw_hair");

/** \} */

/* -------------------------------------------------------------------- */
/** \name Surface
 * \{ */

GPU_SHADER_INTERFACE_INFO(eevee_surface_iface, "interp")
    .smooth(Type::VEC3, "P")
    .smooth(Type::VEC3, "N")
    .smooth(Type::VEC3, "hair_binormal")
    .smooth(Type::FLOAT, "hair_time")
    .smooth(Type::FLOAT, "hair_time_width")
    .smooth(Type::FLOAT, "hair_thickness")
    .flat(Type::INT, "hair_strand_id");

GPU_SHADER_CREATE_INFO(eevee_surface_deferred)
    .vertex_out(eevee_surface_iface)
    /* Diffuse or Transmission Color. */
    .fragment_out(0, Type::VEC3, "out_transmit_color")
    /* RG: Normal (negative if Tranmission), B: SSS ID, A: Min-Thickness */
    .fragment_out(1, Type::VEC4, "out_transmit_normal")
    /* RGB: SSS RGB Radius.
     * or
     * R: Transmission IOR, G: Transmission Roughness, B: Unused. */
    .fragment_out(2, Type::VEC3, "out_transmit_data")
    /* Reflection Color. */
    .fragment_out(3, Type::VEC3, "out_reflection_color")
    /* RG: Normal, B: Roughness X, A: Roughness Y. */
    .fragment_out(4, Type::VEC4, "out_reflection_normal")
    /* Volume Emission, Absorption, Scatter, Phase. */
    .fragment_out(5, Type::UVEC4, "out_volume_data")
    /* Emission. */
    .fragment_out(6, Type::VEC3, "out_emission_data")
    /* Transparent BSDF, Holdout. */
    .fragment_out(7, Type::VEC4, "out_transparency_data")
    .fragment_source("eevee_surface_deferred_frag.glsl")
    .additional_info("eevee_sampling_data", "eevee_utility_texture");

GPU_SHADER_CREATE_INFO(eevee_sampling_data).uniform_buf(0, "SamplingData", "sampling");

GPU_SHADER_CREATE_INFO(eevee_surface_forward)
    .uniform_buf(0, "HiZData", "hiz")
    .sampler(0, ImageType::FLOAT_2D, "hiz_tx")
    .sampler(1, ImageType::FLOAT_2D, "radiance_tx")
    .vertex_out(eevee_surface_iface)
    .fragment_out(0, Type::VEC4, "out_radiance", DualBlend::SRC_0)
    .fragment_out(0, Type::VEC4, "out_transmittance", DualBlend::SRC_1)
    .fragment_source("eevee_surface_forward_frag.glsl")
    .additional_info("eevee_transmittance_data",
                     "eevee_sampling_data",
                     "eevee_lightprobe_data",
                     "eevee_light_data",
                     "eevee_shadow_data");

GPU_SHADER_CREATE_INFO(eevee_surface_depth)
    .vertex_out(eevee_surface_iface)
    .fragment_source("eevee_surface_depth_frag.glsl")
    .additional_info("eevee_sampling_data");

GPU_SHADER_CREATE_INFO(eevee_surface_depth_simple)
    .vertex_out(eevee_surface_iface)
    .fragment_source("eevee_surface_depth_simple_frag.glsl");

/** \} */

/* -------------------------------------------------------------------- */
/** \name Background
 * \{ */

GPU_SHADER_CREATE_INFO(eevee_surface_background)
    .vertex_out(eevee_surface_iface)
    .fragment_out(0, Type::VEC4, "out_background")
    .fragment_source("eevee_surface_background_frag.glsl");

GPU_SHADER_CREATE_INFO(eevee_surface_world)
    .vertex_source("eevee_surface_world_vert.glsl")
    .additional_info("eevee_surface_background");

GPU_SHADER_CREATE_INFO(eevee_surface_lookdev)
    .vertex_in(0, Type::VEC3, "pos")
    .vertex_in(1, Type::VEC3, "nor")
    .vertex_source("eevee_surface_lookdev_vert.glsl")
    .additional_info("eevee_surface_background");

GPU_SHADER_CREATE_INFO(eevee_background_lookdev)
    .uniform_buf(0, "LightProbeInfoData", "probes_info")
    .sampler(0, ImageType::FLOAT_CUBE_ARRAY, "lightprobe_cube_tx")
    .push_constant(Type::FLOAT, "opacity")
    .push_constant(Type::FLOAT, "blur")
    .fragment_out(0, Type::VEC4, "out_background")
    .fragment_source("eevee_lookdev_background_frag.glsl")
    .additional_info("draw_fullscreen");

/** \} */

/* -------------------------------------------------------------------- */
/** \name Volume
 * \{ */

GPU_SHADER_INTERFACE_INFO(eevee_volume_iface, "interp")
    .smooth(Type::VEC3, "P_start")
    .smooth(Type::VEC3, "P_end");

GPU_SHADER_CREATE_INFO(eevee_volume_deferred)
    .sampler(0, ImageType::DEPTH_2D, "depth_max_tx")
    .vertex_in(0, Type::VEC3, "pos")
    .vertex_out(eevee_volume_iface)
    .fragment_out(0, Type::UVEC4, "out_volume_data")
    .fragment_out(1, Type::VEC4, "out_transparency_data")
    .additional_info("eevee_shared")
    .vertex_source("eevee_volume_vert.glsl")
    .fragment_source("eevee_volume_deferred_frag.glsl")
    .additional_info("draw_fullscreen");

/** \} */