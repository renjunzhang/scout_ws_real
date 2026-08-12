# v4 fresh-campaign draft: v3 plus the sole audited pre-pivot tmp mkdir/create delta.
abi <abi/4.0>,
include <tunables/global>
profile r8-liquid-motion-gauge-gpu-build-v4 flags=(attach_disconnected,mediate_deleted) {
  /usr/bin/bwrap rix,
  /usr/bin/make rix,
  /usr/bin/x86_64-linux-gnu-g++-11 rix,
  /usr/local/cuda-12.8/bin/nvcc rix,
  /usr/local/cuda-12.8/** r,
  /usr/** r,
  /lib/** r,
  /lib64/** r,
  /dev/null rw,
  /dev/zero rw,
  /newroot/work/ rw,
  /newroot/work/tmp/ rw,
  /newroot/work/tmp/** rw,
  /work/ rw,
  /work/tmp/ rw,
  /work/tmp/** rw,
  capability sys_admin,
}
