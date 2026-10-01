# Source me: route GL work to the NVIDIA dGPU (PRIME on-demand laptop).
#   GUI apps (MuJoCo window, RViz)  -> GLX PRIME render offload
#   Offscreen rendering (cameras, video) -> EGL with the NVIDIA vendor library
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export MUJOCO_GL=egl
