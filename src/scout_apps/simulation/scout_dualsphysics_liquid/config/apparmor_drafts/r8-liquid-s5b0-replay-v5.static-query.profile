# Static syntax fixture only; not an authorized replay profile or runtime instance.
#include <tunables/global>
profile r8-liquid-s5b0-static-query-v5 flags=(attach_disconnected,mediate_deleted) {
  #include <abstractions/base>
  /stage/runtime/candidate rix,
  /stage/runtime/DsphConfig.xml r,
  /runtime/lib/libcuda.so.1 mr,
  /runtime/lib/libnvidia-ptxjitcompiler.so.1 mr,
  /stage/case/ r,
  /stage/case/** r,
  /stage/restart/ r,
  /stage/restart/** r,
  /dev/nvidia0 rw,
  /dev/nvidiactl rw,
  /dev/nvidia-uvm rw,
  /output/ rw,
  /output/** rwk,
  deny network,
  deny mount,
  deny umount,
  deny pivot_root,
  deny /dev/nvidia-uvm-tools rw,
  deny /dev/nvidia-modeset rw,
  deny /dev/dri/** rw,
}
