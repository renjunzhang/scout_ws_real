# R8 liquid U3 CPU-build profile v1.
# REVIEWED_SINGLE_ATTEMPT_ONLY: transient named profile; never install this
# file under /etc/apparmor.d.  It is valid only for the exact build attempt
# below and must be removed in the same privileged session that loads it.
#
# The only host bind writable by the child is the attempt's new output tree.
# The previously copied source is inside that tree.  No materialized source,
# bare Git repository, workspace, ROS, GPU, precompiled ELF or host network is
# visible.  The profile permits only the fixed GNU Make/G++ toolchain needed
# by the reviewed Makefile wrapper; it does not permit the produced binary.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-u3-cpu-build-v1 flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/env rix,
  /usr/bin/prlimit rix,
  /usr/bin/make rix,
  /usr/bin/dash rix,
  /usr/bin/x86_64-linux-gnu-g++-13 rix,
  /usr/bin/x86_64-linux-gnu-as rix,
  /usr/bin/x86_64-linux-gnu-ld.bfd rix,
  /usr/bin/as rix,
  /usr/bin/ld rix,
  /usr/bin/ld.bfd rix,
  /usr/libexec/gcc/x86_64-linux-gnu/13/cc1plus rix,
  /usr/libexec/gcc/x86_64-linux-gnu/13/collect2 rix,
  /usr/lib/gcc/x86_64-linux-gnu/13/** mr,

  / r,
  /usr/ r,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /usr/libexec/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,
  /etc/ld.so.conf r,
  /etc/ld.so.conf.d/** r,

  /home/ r,
  /home/zrj/ r,
  /home/zrj/scout_liquid_lab/ r,
  /home/zrj/scout_liquid_lab/build/ r,
  /home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260806T165729Z.partial/ rw,
  /home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260806T165729Z.partial/output/ rw,
  /home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260806T165729Z.partial/output/** rw,

  owner /proc/** r,
  owner /proc/*/uid_map w,
  owner /proc/*/gid_map w,
  owner /proc/*/setgroups w,
  /proc/filesystems r,
  /proc/sys/kernel/overflowuid r,
  /proc/sys/kernel/overflowgid r,
  /dev/null rw,
  /dev/zero r,
  /dev/random r,
  /dev/urandom r,

  mount options=(rw, silent, rslave) -> /,
  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /tmp/,
  mount options=(rw, rbind) /tmp/newroot/ -> /tmp/newroot/,
  pivot_root oldroot=/tmp/oldroot/ /tmp/,
  mount options=(rw, silent, rprivate) -> /oldroot/,
  umount /oldroot/,
  pivot_root oldroot=/newroot/ /newroot/,
  umount /,
  mount options=(rw, rbind) /oldroot/usr/ -> /newroot/usr/,
  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/usr/,
  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/work/,
  mount options=(rw, rbind) /oldroot/home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260806T165729Z.partial/output/ -> /newroot/work/output/,
  remount options=(rw, nosuid, nodev, bind, silent, relatime) /newroot/work/output/,

  /tmp/newroot/ rw,
  /tmp/newroot/** rw,
  /tmp/oldroot/ rw,
  /tmp/oldroot/** rw,
  /newroot/usr/ rw,
  /newroot/lib64 wl,
  /newroot/work/ rw,
  /newroot/work/output/ rw,
  /work/output/ rw,
  /work/output/** rw,

  capability sys_admin,
  capability sys_ptrace,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-u3-cpu-build-v1,
  signal (send, receive) peer=r8-liquid-u3-cpu-build-v1,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: compiled-artifact execution, every upstream binary,
  # all source/repository/workspace binds, GPU/DRM devices, broad host paths,
  # generic mount/remount/umount, stream networking, dbus, profile transition
  # and flags=(unconfined).
}
